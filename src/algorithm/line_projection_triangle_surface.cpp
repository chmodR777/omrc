#include "stdafx.h"
#include "line_projection_triangle_surface.h"
#include "map_point3d64_converter.h"
#include "math3d/vector_math.h"
#include "grap_point_algorithm.h"

forceinline MapPoint3D64 boundaryVector(std::vector<MapPoint3D64>& vertexes) {
	MapPoint3D64 vec = {};
	for (int idx = 0; idx < vertexes.size() - 1; idx++) {
		auto& currPt = vertexes[idx];
		auto& nextPt = vertexes[idx + 1];
		MapPoint3D64 tmpVec = nextPt - currPt;
		vec = vec + tmpVec;
	}
	return vec;
}

forceinline bool hasIntersection(MapPoint3D64& pointStart, MapPoint3D64& pointEnd, std::vector<segment_2t>& triEdges2D) {
	linestring_2t tmpExpandLine;
	tmpExpandLine.push_back(point_2t(pointStart.pos.lon, pointStart.pos.lat));
	tmpExpandLine.push_back(point_2t(pointEnd.pos.lon, pointEnd.pos.lat));
	for (size_t i = 0; i < triEdges2D.size(); ++i)
	{
		linestring_2t clipper;
		bg::convert(triEdges2D[i], clipper);
		std::vector<point_2t> intersectPoints;
		bg::intersection(tmpExpandLine, clipper, intersectPoints);
		if (!intersectPoints.empty())
			return true;
	}
	return false;
}

forceinline double linestringAverage(const linestring_t& originLine) {
	if (originLine.size() > 2)
	{
		point_t pStart = originLine.front();
		point_t pEnd = originLine.back();
		vector_t v = V3_N(S3_V3(pStart, pEnd));
		double vSum = 0.0;
		for (size_t i = 1; i < originLine.size(); ++i)
		{
			point_t pa = originLine.at(i - 1);
			point_t pb = originLine.at(i);
			vector_t tmpV = V3_N(S3_V3(pa, pb));
			vSum += bg::dot_product(v, tmpV);
		}
		double vAverage = vSum / (originLine.size() - 1);
		return vAverage;
	}
	return DBL_MAX;
}

void LineProjectionTriangleSurface::projection(
	const std::vector<MapPoint3D64>& leftRdBoundary,
	const std::vector<MapPoint3D64>& rightRdBoundary,
	const std::vector<MapPoint3D64>& line,
	std::vector<MapPoint3D64>& lineOnRoadSurface)
{
	auto getLine2D = [&](const linestring_t& line)-> linestring_2t {
		linestring_2t tmp;
		for (auto& item : line)
			tmp.push_back(point_2t(item.get<0>(), item.get<1>()));
		return tmp;
	};

	auto isPointInPolygonOrNearRoad = [](MapPoint3D64& point, 
		std::vector<MapPoint3D64>& roadFacePoints,
		std::vector<MapPoint3D64>& nearestRdPoints)->bool {
			if (GrapPointAlgorithm::isPointInPolygon(point, roadFacePoints))
				return true;

			size_t si, ei;
			MapPoint3D64 grappedPt;
			if (GrapPointAlgorithm::grapOrMatchNearestPoint(point, nearestRdPoints, grappedPt, si, ei))
				if (point.pos.distance(grappedPt.pos) < 300)
					return true;
			return false;
	};

	auto updateRdBoundaryGap = [](MapPoint3D64& point, 
		const std::vector<MapPoint3D64>& nearestRdPoints,
		const std::vector<MapPoint3D64>& originAllPoints) {
		size_t si, ei;
		MapPoint3D64 grappedPt, nearestPt;
		bool grapped = GrapPointAlgorithm::grapOrMatchNearestPoint(point, nearestRdPoints, grappedPt, nearestPt, si, ei);
		auto grappedPointDistance = grappedPt.pos.distance(point.pos);
		auto nearestPointDistance = nearestPt.pos.distance(point.pos);
		if (grapped && grappedPointDistance < 10) // 修复车道线与道路边界不重合产生的小细缝
			point = grappedPt;
		if (nearestPointDistance < 20)
			point = nearestPt;
		if (!GrapPointAlgorithm::isPointInPolygon(point, originAllPoints)) {
			if (nearestPointDistance < 50) // 车道线在道路边界外5cm内
				point = nearestPt;
		}
	};

	//自相交情况下不进行三角化插值
	auto leftRdBoundary2t = LINESTRING_2T(leftRdBoundary);
	auto rightRdBoundary2t = LINESTRING_2T(rightRdBoundary);
	if (bg::intersects(leftRdBoundary2t) || bg::intersects(rightRdBoundary2t))
	{
		lineOnRoadSurface = line;
		return;
	}

	auto tmpLeftPoints = leftRdBoundary;
	auto tmpRightPoints = rightRdBoundary;
	auto tmpLinePoints = line;
	if (tmpLeftPoints.empty() || tmpRightPoints.empty() || tmpLinePoints.empty())
	{
		lineOnRoadSurface = line;
		return;
	}

	auto originAllPoints = tmpRightPoints;
	std::reverse(tmpLeftPoints.begin(), tmpLeftPoints.end());
	originAllPoints.insert(originAllPoints.end(), tmpLeftPoints.begin(), tmpLeftPoints.end());
	if (!floatEqual(originAllPoints.front().pos.distanceSquare(originAllPoints.back().pos), 0)) {
		originAllPoints.push_back(originAllPoints.front());
	}

	MapPoint3D64Converter mapPointConverter;
	mapPointConverter.setBasePoint(tmpLeftPoints.front());
	mapPointConverter.convert(tmpLeftPoints.data(), tmpLeftPoints.size());
	mapPointConverter.convert(tmpRightPoints.data(), tmpRightPoints.size());
	mapPointConverter.convert(tmpLinePoints.data(), tmpLinePoints.size());
	auto tmpAllPoints = tmpRightPoints;
	tmpAllPoints.insert(tmpAllPoints.end(), tmpLeftPoints.begin(), tmpLeftPoints.end());
	std::vector<std::array<int, 2>> tmpEdgePointIds;
	auto d_left = LINESTRING_2T(tmpLeftPoints);
	auto d_right = LINESTRING_2T(tmpRightPoints);
	auto d_mid = LINESTRING_2T(tmpLinePoints);
	if (!triangularizeStroke(tmpAllPoints.data(), tmpRightPoints.size(), tmpLeftPoints.size(), tmpEdgePointIds))
	{
		lineOnRoadSurface = line;
		return;
	}
	std::vector<segment_t> triEdges3D;
	std::vector<segment_2t> triEdges2D;
	for (auto& index : tmpEdgePointIds)
	{
		triEdges3D.push_back(segment_t(POINT_T(tmpAllPoints[index[0]]), POINT_T(tmpAllPoints[index[1]])));
		triEdges2D.push_back(SEGMENT_2T_EX(segment_2t(POINT_2T(tmpAllPoints[index[0]]), POINT_2T(tmpAllPoints[index[1]])), 1000.0));
	}
	linestring_t originLine3D = LINESTRING_T(tmpLinePoints);
	bg::unique(originLine3D);
	linestring_2t originLine2D = getLine2D(originLine3D);
	size_t tmpSize = originLine2D.size();
	if (tmpSize < 2)
	{
		lineOnRoadSurface = line;
		return;
	}

	if (triEdges2D.size() < 3)
	{
		lineOnRoadSurface = line;
		return;
	}
	bool isStartProjection = true;
	bool isEndProjection = true;
	for (size_t i = 0; i < triEdges2D.size(); ++i)
	{
		linestring_2t clipper;
		bg::convert(triEdges2D[i], clipper);
		std::vector<point_2t> intersectPoints;
		bg::intersection(originLine2D, clipper, intersectPoints);
		if (intersectPoints.empty())
		{
			if (i == 0)
			{
				isStartProjection = false;
				continue;
			}
			else if (i == triEdges2D.size() - 1)
			{
				isEndProjection = false;
				continue;
			}
			else
			{
				continue;
			}
		}
		MapPoint3D64 mapPoint;
		if (!isGetValue(triEdges3D[i], triEdges2D[i], intersectPoints[0], mapPoint))
			continue;
		if (!lineOnRoadSurface.empty()) {
			auto& backPt = lineOnRoadSurface.back();
			if (floatEqual(backPt.pos.distanceSquare(mapPoint.pos), 0)) {
				continue;
			}
		}
		lineOnRoadSurface.push_back(mapPoint);
	}
	// 20158320 both false
	if (!isStartProjection)
	{
		vector_2t dirA = V2_N(vector_2t(
			originLine2D[0].get<0>() - originLine2D[1].get<0>(),
			originLine2D[0].get<1>() - originLine2D[1].get<1>()));
		point_2t tmpA = point_2t(
			originLine2D.front().get<0>() + int64(dirA.get<0>() * 5.0e6),
			originLine2D.front().get<1>() + int64(dirA.get<1>() * 5.0e6));
		linestring_2t tmpExpandLine;
		tmpExpandLine.push_back(tmpA);
		tmpExpandLine.push_back(originLine2D.front());
		std::vector<MapPoint3D64> firstResults;
		for (size_t i = 0; i < triEdges2D.size(); ++i)
		{
			linestring_2t clipper;
			bg::convert(triEdges2D[i], clipper);
			std::vector<point_2t> intersectPoints;
			bg::intersection(tmpExpandLine, clipper, intersectPoints);
			if (intersectPoints.empty())
				continue;
			MapPoint3D64 mapPoint;
			if (!isGetValue(triEdges3D[i], triEdges2D[i], intersectPoints[0], mapPoint))
				continue;
			firstResults.push_back(mapPoint);
		}
		if (!firstResults.empty())
		{
			// 20158337,大圆弧抓到对侧点
			MapPoint3D64* startPoint = nullptr;
			auto& tmpLineFrontPoint = tmpLinePoints.front();
			for (int idx = firstResults.size() - 1; idx >= 0; idx--) 
			{
				auto& firstResultsPt = firstResults[idx];
				if (abs(firstResultsPt.z - tmpLineFrontPoint.z) < 300)
				{
					startPoint = &firstResultsPt;
					break;
				}
			}
			if (startPoint != nullptr)
			{
				MapPoint3D64 mapPoint;
				if (!lineOnRoadSurface.empty())
				{
					if (isGetValue(segment_t(POINT_T((*startPoint)), POINT_T(lineOnRoadSurface.front())), segment_2t(POINT_2T((*startPoint)), POINT_2T(lineOnRoadSurface.front())), originLine2D.front(), mapPoint))
					{
						lineOnRoadSurface.emplace(lineOnRoadSurface.begin(), mapPoint);
					}
				}
				else
				{
					lineOnRoadSurface.emplace(lineOnRoadSurface.begin(), (*startPoint));
				}
			}
		}
	}
	if (!isEndProjection)
	{
		vector_2t dirB = V2_N(vector_2t(
			originLine2D[tmpSize - 1].get<0>() - originLine2D[tmpSize - 2].get<0>(),
			originLine2D[tmpSize - 1].get<1>() - originLine2D[tmpSize - 2].get<1>()));
		point_2t tmpB = point_2t(
			originLine2D.back().get<0>() + int64(dirB.get<0>() * 5.0e6),
			originLine2D.back().get<1>() + int64(dirB.get<1>() * 5.0e6));
		linestring_2t tmpExpandLine;
		tmpExpandLine.push_back(originLine2D.back());
		tmpExpandLine.push_back(tmpB);
		std::vector<MapPoint3D64> secondResults;
		for (size_t i = 0; i < triEdges2D.size(); ++i)
		{
			linestring_2t clipper;
			bg::convert(triEdges2D[i], clipper);
			std::vector<point_2t> intersectPoints;
			bg::intersection(tmpExpandLine, clipper, intersectPoints);
			if (intersectPoints.empty())
				continue;
			MapPoint3D64 mapPoint;
			if (!isGetValue(triEdges3D[i], triEdges2D[i], intersectPoints[0], mapPoint))
				continue;
			secondResults.push_back(mapPoint);
		}
		if (!secondResults.empty())
		{
			MapPoint3D64* endPoint = nullptr;
			auto& tmpLineBackPoint = tmpLinePoints.back();
			for (int idx = 0; idx <= secondResults.size() - 1; idx++) {
				auto& secondResultsPt = secondResults[idx];
				if (abs(secondResultsPt.z - tmpLineBackPoint.z) < 300)
				{
					endPoint = &secondResultsPt;
					break;
				}
			}
			if (endPoint != nullptr)
			{
				MapPoint3D64 mapPoint;
				if (!lineOnRoadSurface.empty())
				{
					if (isGetValue(segment_t(POINT_T(lineOnRoadSurface.back()), POINT_T((*endPoint))), segment_2t(POINT_2T(lineOnRoadSurface.back()), POINT_2T((*endPoint))), originLine2D.back(), mapPoint))
					{
						lineOnRoadSurface.push_back(mapPoint);
					}
				}
				else
				{
					lineOnRoadSurface.push_back((*endPoint));
				}
			}
		}
	}
	if (lineOnRoadSurface.empty())
	{
		lineOnRoadSurface = line;
		return;
	}

	// 闭环
	auto d_res = LINESTRING_2T(lineOnRoadSurface);
	MapPoint3D64& startPt = tmpAllPoints.front();
	MapPoint3D64& endPt = tmpAllPoints.back();
	if (!floatEqual(startPt.pos.distanceSquare(endPt.pos), 0)) {
		tmpAllPoints.push_back(startPt);
	}

	// 最近的道路边界
	auto line2t = LINESTRING_2T(line);
	auto lineLeftRdDis = bg::distance(line2t, leftRdBoundary2t);
	auto lineRightRdDis = bg::distance(line2t, rightRdBoundary2t);
	auto& nearestRdPoints = lineLeftRdDis < lineRightRdDis ? tmpLeftPoints : tmpRightPoints;
	auto& nearestRdBoundary = lineLeftRdDis < lineRightRdDis ? leftRdBoundary : rightRdBoundary;

	// 处理车道线在外面的情况
	polygon_2t tmpAllpolygon = POLYGON_2T(tmpAllPoints);
	if (std::none_of(originLine2D.begin(), originLine2D.end(), [&tmpAllpolygon](auto& pt) {return bg::within(pt, tmpAllpolygon); })) {
		std::vector<point_2t> tmpIntersectPoints;
		bg::intersection(originLine2D, tmpAllpolygon, tmpIntersectPoints);
		auto& originFrontPt = originLine2D.front();
		auto& originBackPt = originLine2D.back();
		for (auto iter = tmpIntersectPoints.begin(); iter != tmpIntersectPoints.end();) {
			if (bg::equals(*iter, originFrontPt) || bg::equals(*iter, originBackPt))
				iter = tmpIntersectPoints.erase(iter);
			else
				iter++;
		}
		if (tmpIntersectPoints.empty()) {
			lineOnRoadSurface = line;
			for (auto& pointOnRoadSurface : lineOnRoadSurface) {
				// 修复使用LA补面时产生的小细缝
				updateRdBoundaryGap(pointOnRoadSurface, nearestRdBoundary, originAllPoints);
			}
			return;
		}
	}

	// 处理车道线坐标被反向的情况
	MapPoint3D64 originLineVec = boundaryVector(tmpLinePoints);
	MapPoint3D64 lineOnRoadVec = boundaryVector(lineOnRoadSurface);
	if (dot(originLineVec, lineOnRoadVec) < 0) {
		std::reverse(lineOnRoadSurface.begin(), lineOnRoadSurface.end());
	}

	// 补充首尾
	int startIdx = -1, endIdx = tmpLinePoints.size();
	for (int idx = 0; idx < tmpLinePoints.size() - 1; idx++) {
		auto& tmpLinePoint = tmpLinePoints[idx];
		if (!isPointInPolygonOrNearRoad(tmpLinePoint, tmpAllPoints, nearestRdPoints))
		{
			if (startIdx != -1 && hasIntersection(tmpLinePoints[startIdx], tmpLinePoint, triEdges2D)) {
				break;
			}
			startIdx = idx;
		}
		else
		{
			break;
		}
	}
	for (int idx = tmpLinePoints.size() - 1; idx > 0; idx--) {
		auto& tmpLinePoint = tmpLinePoints[idx];
		if (!isPointInPolygonOrNearRoad(tmpLinePoint, tmpAllPoints, nearestRdPoints))
		{
			if (endIdx != tmpLinePoints.size() && hasIntersection(tmpLinePoints[endIdx], tmpLinePoint, triEdges2D)) {
				break;
			}
			endIdx = idx;
		}
		else
		{
			break;
		}
	}

	for (int idx = startIdx; idx >= 0; idx--) {
		auto& tmpLinePoint = tmpLinePoints[idx];
		auto iter = lineOnRoadSurface.begin();
		double distance = iter->pos.distance(tmpLinePoint.pos);
		auto zDelta = abs(tmpLinePoint.z - iter->z);
		if (!floatEqualWithEpsilon(distance, 0, 0)) {
			if (distance <= 1000.0 && zDelta > 2)
				tmpLinePoint.z = iter->z;
			lineOnRoadSurface.emplace(iter, tmpLinePoint);
		}
	}
	for (int idx = endIdx; idx < tmpLinePoints.size(); idx++) {
		auto& tmpLinePoint = tmpLinePoints[idx];
		auto iter = lineOnRoadSurface.rbegin();
		double distance = iter->pos.distance(tmpLinePoint.pos);
		auto zDelta = abs(tmpLinePoint.z - iter->z);
		if (!floatEqualWithEpsilon(distance, 0, 0)) {
			if (distance <= 1000.0 && zDelta > 2)
				tmpLinePoint.z = iter->z;
			lineOnRoadSurface.push_back(tmpLinePoint);
		}
	}
	mapPointConverter.invert(lineOnRoadSurface.data(), lineOnRoadSurface.size());

	// 修复插值相关的问题
	double maxProjectionDistance = 0;
	for (auto& point : line)
	{
		size_t si, ei;
		int nearestPtIdx = -1;
		MapPoint3D64 grappedPt;
		bool grapped = GrapPointAlgorithm::grapOrMatchNearestPoint(point, lineOnRoadSurface, grappedPt, nearestPtIdx, si, ei);
		auto& pointOnRoadSurface = lineOnRoadSurface[nearestPtIdx];
		auto grappedPointDistance = grappedPt.pos.distance(point.pos);
		auto nearestPointDistance = pointOnRoadSurface.pos.distance(point.pos);
		if (nearestPointDistance < 10) // 修复invert产生的坐标偏移
		{
			pointOnRoadSurface.pos.lon = point.pos.lon;
			pointOnRoadSurface.pos.lat = point.pos.lat;
			if (abs(pointOnRoadSurface.z - point.z) < 2)
				pointOnRoadSurface.z = point.z;
		}
		if (grappedPointDistance > maxProjectionDistance)
			maxProjectionDistance = grappedPointDistance;
	}

	// 修复弯曲车道线被三角化拉直的问题,arccos(0.998) = 3.6 degree
	if (maxProjectionDistance > 300 && linestringAverage(originLine3D) < 0.998) {
		lineOnRoadSurface = line; // 20597513,20158071
		return;
	}

	//合并序号错乱且离得较近的点
	if (lineOnRoadSurface.size() > 2)
	{
		Vector2 vRef = vec2(
			lineOnRoadSurface[1].pos.lon - lineOnRoadSurface[0].pos.lon, 
			lineOnRoadSurface[1].pos.lat - lineOnRoadSurface[0].pos.lat);
		vRef.normalize();
		for (size_t i = 1; i < lineOnRoadSurface.size() - 1; ++i)
		{
			auto& pa = lineOnRoadSurface.at(i);
			auto& pb = lineOnRoadSurface.at(i + 1);
			auto dis1 = pa.pos.distance(pb.pos);
			Vector2 va = vec2(pb.pos.lon - pa.pos.lon, pb.pos.lat - pa.pos.lat);
			va.normalize();
			double tmpValue = dot(vRef, va);
			int64 tmpTol = 1200.0;	// 经过全国统计找到的合适的取值
			if (dis1 == 0 || (tmpValue < -0.8 && dis1 < tmpTol)) // -0.8 (145度)
			{
				lineOnRoadSurface.erase(lineOnRoadSurface.begin() + i + 1);
				--i;
			}
			else
			{
				vRef = va;
			}

			// 修复插值点在路面外时产生的小细缝
			updateRdBoundaryGap(pa, nearestRdBoundary, originAllPoints);
		}
		updateRdBoundaryGap(lineOnRoadSurface.front(), nearestRdBoundary, originAllPoints);
		updateRdBoundaryGap(lineOnRoadSurface.back(), nearestRdBoundary, originAllPoints);
	}

}

bool LineProjectionTriangleSurface::isGetValue(
	const segment_t& originSeg,
	const segment_2t& seg,
	const point_2t& pc,
	MapPoint3D64& point)
{
	int64 l = bg::length(seg);
	if (l == 0)
		return false;
	double w1 = bg::distance(pc, seg.first) / l;
	double w2 = 1.0 - w1;
	int64 z1 = originSeg.first.get<2>() * w2;
	int64 z2 = originSeg.second.get<2>() * w1;
	int64 pz = z1 + z2;
	point = MapPoint3D64_make(pc.get<0>(), pc.get<1>(), pz / 10);
	return true;
}

bool LineProjectionTriangleSurface::triangularizeStroke(
	MapPoint3D64* contour,
	int rightN,
	int leftN,
	std::vector<std::array<int, 2>>& out_edges)
{
	if (leftN < 1 || rightN < 1 || leftN + rightN < 3 || leftN + rightN > 0xffff)
		return false;

	MapPoint3D64Converter c;
	c.setBasePoint(contour[0]);
	c.convert(contour, rightN + leftN);

	uint16 leftIdx = leftN + rightN - 1, rightIdx = 0;
	for (; leftIdx > rightN || rightIdx + 1 < rightN;)
	{
		std::array<int, 2> edge;
		edge[0] = leftIdx;
		edge[1] = rightIdx;
		out_edges.emplace_back(edge);

		if (leftIdx == rightN)
			++rightIdx;
		else if (rightIdx + 1 == rightN)
			--leftIdx;
		else //All in range
		{
			double leftWeight = Math_segGeoLengthNdsF(contour[leftIdx - 1].pos.toNdsPoint(), contour[rightIdx].pos.toNdsPoint());
			double rightWeight = Math_segGeoLengthNdsF(contour[rightIdx + 1].pos.toNdsPoint(), contour[leftIdx].pos.toNdsPoint());

			if (leftWeight < rightWeight)
				--leftIdx;
			else
				++rightIdx;
		}
	}
	std::array<int, 2> edge;
	edge[0] = leftIdx;
	edge[1] = rightIdx;
	out_edges.emplace_back(edge);
	c.invert(contour, rightN + leftN);
	return true;
}


