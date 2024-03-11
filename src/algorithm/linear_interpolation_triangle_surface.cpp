#include "stdafx.h"
#include <unordered_map>
#include "grap_point_algorithm.h"
#include "polygon_triangularizer.h"
#include "linear_interpolation_triangle_surface.h"

#define MAX_TRIANGLE_HEIGHT 2500
#define INTERPOLATION_HEIGHT 100

forceinline DVector4 Eigen_Vector4d_DVector4(Eigen::Vector4d ev) {
	DVector4 dv;
	dv.set(ev(0), ev(1), ev(2), ev(3));
	return dv;
}

forceinline Eigen::Vector4d DVector4_Eigen_Vector4d(DVector4 dv) {
	Eigen::Vector4d ev;
	ev(0) = dv.x;
	ev(1) = dv.y;
	ev(2) = dv.z;
	ev(3) = dv.w;
	return ev;
}

void LinearInterpolationTriangleSurface::interpolationLine(
	MapPoint3D64Converter& mapPointConverter, 
	const std::vector<Triangle>& surfaceTriangles, 
	const std::vector<MapPoint3D64>& line, 
	std::vector<MapPoint3D64>& lineOnRoadSurface)
{
	if (line.size() < 2) {
		lineOnRoadSurface = line;
		return;
	}

	parameters surfaceParam;
	std::vector<box_2t> surfaceTriangleBoxes;
	getTriangleBoxes(surfaceTriangles, surfaceTriangleBoxes);
	index_getter_2box surfaceOriginInd(surfaceTriangleBoxes);
	rtree_type_2box box2TRTree(boost::irange<std::size_t>(0lu, surfaceTriangleBoxes.size()), surfaceParam, surfaceOriginInd);
	interpolationLine(mapPointConverter, surfaceTriangles, box2TRTree, line, lineOnRoadSurface);
}

void LinearInterpolationTriangleSurface::interpolationLine(
	MapPoint3D64Converter& mapPointConverter, 
	const std::vector<Triangle>& surfaceTriangles, 
	const rtree_type_2box& surfaceRTree2T, 
	const std::vector<MapPoint3D64>& line, 
	std::vector<MapPoint3D64>& lineOnRoadSurface)
{
	if (line.size() < 2) {
		lineOnRoadSurface = line;
		return;
	}

	auto tmpLinePoints = line;
	auto tmpLineIp = std::unique(tmpLinePoints.begin(), tmpLinePoints.end(), [](auto& a, auto& b) {return a == b; });
	tmpLinePoints.resize(std::distance(tmpLinePoints.begin(), tmpLineIp));

	mapPointConverter.convert(tmpLinePoints.data(), tmpLinePoints.size());
	linestring_t tmpLine3D = LINESTRING_T(tmpLinePoints);
	std::vector<point_t> newLine3D;
	for (auto idx = 0; idx < tmpLine3D.size() - 1; idx++) {
		std::vector<point_t> trianglePts;
		auto& currPt = tmpLine3D[idx];
		calculateTrianglePt(currPt, surfaceRTree2T, surfaceTriangles);
		trianglePts.push_back(currPt);

		auto& nextPt = tmpLine3D[idx + 1];
		calculateTrianglePt(nextPt, surfaceRTree2T, surfaceTriangles);
		trianglePts.push_back(nextPt);

		segment_t segment;
		segment.first = currPt;
		segment.second = nextPt;
		std::vector<point_t> segTrianglePts;
		calculateTrianglePts(segment, surfaceRTree2T, surfaceTriangles, segTrianglePts);
		for (auto& trianglePt : segTrianglePts) {
			if (trianglePt != currPt && trianglePt != nextPt)
				trianglePts.push_back(trianglePt);
		}

		vector_t pabVec = V3_N(S3_V3(segment));
		std::sort(trianglePts.begin(), trianglePts.end(), [&](point_t& pa, point_t& pb) {
			vector_t tmp0V = S3_V3(currPt, pa);
			auto v0Sum = bg::dot_product(pabVec, tmp0V);

			vector_t tmp1V = S3_V3(currPt, pb);
			auto v1Sum = bg::dot_product(pabVec, tmp1V);
			return v0Sum < v1Sum;
		});
		auto ip = std::unique(trianglePts.begin(), trianglePts.end(), isEqualPoint);
		trianglePts.resize(std::distance(trianglePts.begin(), ip));
		newLine3D.insert(newLine3D.end(), trianglePts.begin(), trianglePts.end());
	}

	// lineOnRoadSurface
	auto newLineIp = std::unique(newLine3D.begin(), newLine3D.end(), isEqualPoint);
	newLine3D.resize(std::distance(newLine3D.begin(), newLineIp));
	if (newLine3D.size() < 2) {
		lineOnRoadSurface = line;
		return;
	}
	for (auto& pos : newLine3D) {
		MapPoint3D64 tmpPos = MapPoint3D64_make(pos.get<0>(), pos.get<1>(), pos.get<2>() / 10);
		lineOnRoadSurface.push_back(tmpPos);
	}
	mapPointConverter.invert(lineOnRoadSurface.data(), lineOnRoadSurface.size());

	// 修复插值相关的问题
	for (auto& point : line)
	{
		size_t si, ei;
		int nearestPtIdx = -1;
		MapPoint3D64 grappedPt;
		bool grapped = GrapPointAlgorithm::grapOrMatchNearestPoint(point, lineOnRoadSurface, grappedPt, nearestPtIdx, si, ei);
		auto& pointOnRoadSurface = lineOnRoadSurface[nearestPtIdx];
		// auto grappedPointDistance = grappedPt.pos.distance(point.pos);
		auto nearestPointDistance = pointOnRoadSurface.pos.distance(point.pos);
		if (nearestPointDistance < 10) // 修复invert产生的坐标偏移
		{
			pointOnRoadSurface.pos.lon = point.pos.lon;
			pointOnRoadSurface.pos.lat = point.pos.lat;
		}
	}
}

void LinearInterpolationTriangleSurface::interpolationPolygon(
	MapPoint3D64Converter& mapPointConverter,
	const std::vector<Triangle>& surfaceTriangles,
	const std::vector<Triangle>& polyTriangles,
	const std::vector<MapPoint3D64>& polygon, 
	std::vector<MapPoint3D64>& polygonOnSurface)
{
	if (polygon.size() < 3) {
		polygonOnSurface = polygon;
		return;
	}

	// surfaceRTree
	parameters surfaceParam;
	std::vector<box_2t> surfaceTriangleBoxes;
	getTriangleBoxes(surfaceTriangles, surfaceTriangleBoxes);
	index_getter_2box surfaceOriginInd(surfaceTriangleBoxes);
	rtree_type_2box box2TRTree(boost::irange<std::size_t>(0lu, surfaceTriangleBoxes.size()), surfaceParam, surfaceOriginInd);

	auto tmpPolygonPoints = polygon;
	auto ip = std::unique(tmpPolygonPoints.begin(), tmpPolygonPoints.end(), [](auto& a, auto& b) {return a == b; });
	tmpPolygonPoints.resize(std::distance(tmpPolygonPoints.begin(), ip));
	polygonOnSurface = tmpPolygonPoints;

	mapPointConverter.convert(tmpPolygonPoints.data(), tmpPolygonPoints.size());
	linestring_t tmpPolygon3D = LINESTRING_T(tmpPolygonPoints);

	parameters param;
	index_getter originInd(tmpPolygon3D);
	rtree_type rtree(boost::irange<std::size_t>(0lu, tmpPolygon3D.size()), param, originInd);

	std::vector<ring_2t> dRings;
	for (auto& polyTriangle : polyTriangles) {
		dRings.push_back(polyTriangle.trianglePoly);
	}

	std::unordered_map<point_t*, int32> point2ZValue;
	for (auto triIdx = 0; triIdx < polyTriangles.size(); triIdx++) {
		auto& polyTriangle = polyTriangles[triIdx];
		if (isStraightTriangle(polyTriangle)) {
			// 过滤几乎是直线的三角化面
			continue;
		}

		segment_t triangleSegment;
		std::vector<point_t> trianglePts;
		std::vector<point_t> tmpTrianglePts;
		auto vertexeSize = polyTriangle.vertexes.size();
		for (auto tIdx = 0; tIdx < vertexeSize; tIdx++) {
			auto currTrianglePt = polyTriangle.vertexes[tIdx];
			calculateTrianglePt(currTrianglePt, box2TRTree, surfaceTriangles);
			trianglePts.push_back(currTrianglePt);

			tmpTrianglePts.clear();
			auto& nextTrianglePt = (tIdx != vertexeSize - 1) ?
				polyTriangle.vertexes[tIdx + 1] : polyTriangle.vertexes[0];
			triangleSegment.first = currTrianglePt;
			triangleSegment.second = nextTrianglePt;
			calculateTrianglePts(triangleSegment, box2TRTree, surfaceTriangles, tmpTrianglePts);
			trianglePts.insert(trianglePts.end(), tmpTrianglePts.begin(), tmpTrianglePts.end());
		}

		DVector4 coeffs;
		if (interpolationTriangle(trianglePts, coeffs)) {
			for (auto& currTrianglePt : polyTriangle.vertexes) {
				std::vector<point_t> indexPoints;
				auto indexes = getPointIndexes(currTrianglePt, rtree, tmpPolygon3D);
				for_each(indexes.begin(), indexes.end(), [&](size_t id)->void {indexPoints.push_back(tmpPolygon3D[id]); });
				for (auto iIdx = 0; iIdx < indexes.size(); iIdx++) {
					// 插值Z值
					auto& indexPt = indexPoints[iIdx];
					auto indexPtZ = interpolatePt(coeffs, indexPt);
					auto& currPt = tmpPolygon3D[indexes[iIdx]];
					if (point2ZValue.count(&currPt)) {
						point2ZValue[&currPt] = max(point2ZValue[&currPt], indexPtZ);
					} else {
						point2ZValue[&currPt] = indexPtZ;
					}
				}
			}
		}
	}

	// 加1cm解决深度问题
	for (int i = 0; i < tmpPolygon3D.size(); ++i) {
		auto& currPt = tmpPolygon3D[i];
		if (point2ZValue.count(&currPt)) {
			if (abs(polygonOnSurface[i].z - (point2ZValue[&currPt] / 10 + 1)) < INTERPOLATION_HEIGHT) {
				polygonOnSurface[i].z = point2ZValue[&currPt] / 10 + 1;
			} else {
				polygonOnSurface[i].z = currPt.get<2>() / 10 + 1;
			}
		} else {
			polygonOnSurface[i].z = currPt.get<2>() / 10 + 1;
		}
	}
}

void LinearInterpolationTriangleSurface::getTriangleBoxes(
	const std::vector<Triangle>& polyTriangles, 
	std::vector<box_2t>& triangleBoxes)
{
	triangleBoxes.clear();
	for (auto& polyTriangle : polyTriangles) {
		linestring_2t triangleLine;
		for (auto& vertex : polyTriangle.vertexes)
			triangleLine.push_back(point_2t(vertex.get<0>(), vertex.get<1>()));
		box_2t triangleBox;
		bg::envelope(triangleLine, triangleBox);
		triangleBoxes.push_back(triangleBox);
	}
}

bool LinearInterpolationTriangleSurface::interpolationTriangle(
	std::vector<point_t> trianglePts, DVector4& dVector4)
{
	std::sort(trianglePts.begin(), trianglePts.end(), [](point_t& pa, point_t& pb) {
		return pa < pb;
		});
	auto ip = std::unique(trianglePts.begin(), trianglePts.end(), isEqualPoint);
	trianglePts.resize(std::distance(trianglePts.begin(), ip));
	if (trianglePts.size() < 3)
		return false;

	Eigen::MatrixXd matrix(trianglePts.size(), 3);
	for (int i = 0; i < trianglePts.size(); ++i) {
		matrix(i, 0) = trianglePts[i].get<0>();
		matrix(i, 1) = trianglePts[i].get<1>();
		matrix(i, 2) = trianglePts[i].get<2>();
	}

	// 1、计算质心
	Eigen::RowVector3d centroid = matrix.colwise().mean();
	// 2、去质心
	Eigen::MatrixXd demean = matrix;
	demean.rowwise() -= centroid;
	// 3、SVD分解求解协方差矩阵的特征值特征向量
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(demean, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::Matrix3d V = svd.matrixV();
	Eigen::MatrixXd U = svd.matrixU();
	// 4、平面的法向量a,b,c
	Eigen::RowVector3d normal;
	normal << V(0, 2), V(1, 2), V(2, 2);
	Eigen::RowVector3d zAxis;
	zAxis << 0, 0, 1;
	//if (normal.dot(zAxis) < 0) {
	// normal = -normal;
	//}

	// 过滤法向量是Z轴的面
	if (abs(normal(0, 2)) < 0.6) {
		//normal = zAxis;
		return false;
	}
	// 5、原点到平面的距离d
	double D = -normal * centroid.transpose();

	Eigen::Vector4d coeffs;
	coeffs(0) = normal(0, 0);
	coeffs(1) = normal(0, 1);
	coeffs(2) = normal(0, 2);
	coeffs(3) = D;
	coeffs.normalize(); // 归一化系数向量
	// 6、最高点到平面的高度差
	double delta = DBL_MIN;
	auto tmpCoeffs = Eigen_Vector4d_DVector4(coeffs);
	for (int i = 0; i < trianglePts.size(); ++i) {
		auto z = interpolatePt(tmpCoeffs, trianglePts[i]);
		if (trianglePts[i].get<2>() > z) {
			auto tmpDelta = trianglePts[i].get<2>() - z;
			delta = max(delta, tmpDelta);
		}
	}

	// 7、重新归一化
	coeffs(0) = normal(0, 0);
	coeffs(1) = normal(0, 1);
	coeffs(2) = normal(0, 2);
	coeffs(3) = D - delta;
	coeffs.normalize(); // 归一化系数向量
	dVector4 = Eigen_Vector4d_DVector4(coeffs);
	return true;
}

double LinearInterpolationTriangleSurface::interpolatePt(DVector4& coeffs, const point_t& pt)
{
	auto z = (-coeffs.x * pt.get<0>() - coeffs.y * pt.get<1>() - coeffs.w) / coeffs.z;
	return z;
}

bool LinearInterpolationTriangleSurface::calculateTrianglePt(
	point_t& point, 
	const rtree_type_2box& rtree, 
	const std::vector<Triangle>& polyTriangles)
{
	std::vector<Triangle> triangles;
	if (getTrianglesByPt(point, rtree, polyTriangles, triangles)) {
		auto tmpPoint = P3_P2(point);
		std::unordered_map<point_t*, int32> point2ZValue;
		for (auto& triangle : triangles) {
			if (isStraightTriangle(triangle)) {
				// 过滤几乎是直线的三角化面
				continue;
			}

			for (auto idx = 0; idx < triangle.triangleLine.size(); idx++) {
				auto& triangleLinePt = triangle.triangleLine[idx];
				if (isEqualPoint2T(tmpPoint, triangleLinePt)) {
					auto& trianglePt = triangle.vertexes[idx];
					auto z = trianglePt.get<2>();
					if (point2ZValue.count(&point)) {
						point2ZValue[&point] = max(point2ZValue[&point], z);
					} else {
						point2ZValue[&point] = z;
					}
				}
			}

			double z = calculateZOnTriangle(tmpPoint, triangle);
			if (z != 0)
			{
				if (point2ZValue.count(&point)) {
					point2ZValue[&point] = max(point2ZValue[&point], z);
				} else {
					point2ZValue[&point] = z;
				}
			}
		}
		if (point2ZValue.count(&point)) {
			point.set<2>(point2ZValue[&point]);
		}
	}
	// outer
	return false;
}

bool LinearInterpolationTriangleSurface::calculateTrianglePts(
	const segment_t& segment, 
	const rtree_type_2box& rtree, 
	const std::vector<Triangle>& polyTriangles, 
	std::vector<point_t>& trianglePts)
{
	std::vector<Triangle> triangles;
	if (getTrianglesBySegment(segment, rtree, polyTriangles, triangles)) {
		linestring_2t clipper;
		clipper.push_back(P3_P2(segment.first));
		clipper.push_back(P3_P2(segment.second));

		for (auto& triangle : triangles) {
			if (isStraightTriangle(triangle)) {
				// 过滤几乎是直线的三角化面
				continue;
			}

			for (auto& clipperPt : clipper) {
				if (coveredByTriangle(clipperPt, triangle)) {
					double z = calculateZOnTriangle(clipperPt, triangle);
					if (z != 0) {
						point_t trianglePt;
						trianglePt.set<0>(clipperPt.get<0>());
						trianglePt.set<1>(clipperPt.get<1>());
						trianglePt.set<2>(z);
						trianglePts.push_back(trianglePt);
					}
				}
			}

			std::vector<point_2t> intersectPoints;
			bg::intersection(triangle.trianglePoly, clipper, intersectPoints);
			for (auto& intersectPoint : intersectPoints) {
				double z = calculateZOnTriangle(intersectPoint, triangle);
				if (z != 0) {
					point_t trianglePt;
					trianglePt.set<0>(intersectPoint.get<0>());
					trianglePt.set<1>(intersectPoint.get<1>());
					trianglePt.set<2>(z);
					trianglePts.push_back(trianglePt);
				}
			}
		}
	}

	// 按segment方向排序
	auto& firstPt = segment.first;
	vector_t pabVec = V3_N(S3_V3(segment));
	std::sort(trianglePts.begin(), trianglePts.end(), [&](point_t& pa, point_t& pb) {
		vector_t tmp0V = S3_V3(firstPt, pa);
		auto v0Sum = bg::dot_product(pabVec, tmp0V);

		vector_t tmp1V = S3_V3(firstPt, pb);
		auto v1Sum = bg::dot_product(pabVec, tmp1V);
		return v0Sum < v1Sum;
	});

	// 同一个点可能在多个面内,保留高程最高的
	for (auto iter = trianglePts.begin(); iter != trianglePts.end(); ) {
		auto& currPt = *iter;
		if (iter + 1 != trianglePts.end()) {
			auto& nextPt = *(iter + 1);
			if (isEqualPoint2T(P3_P2(currPt), P3_P2(nextPt))) {
				if (currPt.get<2>() > nextPt.get<2>()) 
				{
					iter = trianglePts.erase(iter + 1);
					iter--;
				}
				else
				{
					iter = trianglePts.erase(iter);
				}
				continue;
			}
		}

		iter++;
	}
	auto ip = std::unique(trianglePts.begin(), trianglePts.end(), isEqualPoint);
	trianglePts.resize(std::distance(trianglePts.begin(), ip));
	return !trianglePts.empty();
}

bool LinearInterpolationTriangleSurface::getTrianglesByPt(
	const point_t& point, 
	const rtree_type_2box& rtree, 
	const std::vector<Triangle>& polyTriangles, 
	std::vector<Triangle>& triangles)
{
	triangles.clear();
	auto tmpPoint = P3_P2(point);
	std::vector<size_t> triangleIds;
	rtree.query(bgi::intersects(tmpPoint),
		boost::make_function_output_iterator([&](size_t const& id) {
			auto& triangle = polyTriangles[id];
			if (coveredByTriangle(tmpPoint, triangle)) {
				point_t nearbyPoint = getClosestPoint(triangle, point);
				if (std::abs(point.get<2>() - nearbyPoint.get<2>()) < MAX_TRIANGLE_HEIGHT) {
					triangleIds.push_back(id);
				}
			}
		}));

	for (auto triangleId : triangleIds) {
		auto& triangle = polyTriangles[triangleId];
		triangles.push_back(triangle);
	}
	return !triangles.empty();
}

bool LinearInterpolationTriangleSurface::getTrianglesBySegment(
	const segment_t& segment, 
	const rtree_type_2box& rtree, 
	const std::vector<Triangle>& polyTriangles, 
	std::vector<Triangle>& triangles)
{
	int offset = 500;
	linestring_2t clipper;
	clipper.push_back(P3_P2(segment.first));
	clipper.push_back(P3_P2(segment.second));
	auto segBox = BOX_2T(segment);
	auto segment2T = S3_S2(segment);
	auto& max_corner = segBox.max_corner();
	auto& min_corner = segBox.min_corner();
	max_corner.set<0>(max_corner.get<0>() + offset);
	max_corner.set<1>(max_corner.get<1>() + offset);
	min_corner.set<0>(min_corner.get<0>() - offset);
	min_corner.set<1>(min_corner.get<1>() - offset);
	std::vector<size_t> triangleIds;
	std::vector<Triangle> tmpTriangles;
	std::vector<size_t> tmpIds;
	rtree.query(bgi::intersects(segBox),
		boost::make_function_output_iterator([&](size_t const& id) {
			auto& triangle = polyTriangles[id];
			tmpTriangles.push_back(triangle);
			tmpIds.push_back(id);
			}));

	for(size_t i = 0; i < tmpIds.size(); ++i)
	{
		auto& triangle = tmpTriangles.at(i);
		std::vector<point_2t> intersectPoints;
		linestring_2t clipper;
		clipper.push_back(P3_P2(segment.first));
		clipper.push_back(P3_P2(segment.second));
		bg::intersection(triangle.trianglePoly, clipper, intersectPoints);
		if (!intersectPoints.empty()) 
		{
			for (auto& pc : intersectPoints) 
			{
				//begin: get z value by segment_t
				auto& seg = segment2T;
				auto& originSeg = segment;
				int64 l = bg::length(seg);
				if (l == 0)
					continue;
				double w1 = bg::distance(pc, seg.first) / l;
				double w2 = 1.0 - w1;
				int64 z1 = originSeg.first.get<2>() * w2;
				int64 z2 = originSeg.second.get<2>() * w1;
				int64 pz = z1 + z2;
				//end: get z value by segment_t

				point_t relPoint = point_t(pc.get<0>(), pc.get<1>(), pz);
				point_t nearbyPoint = getClosestPoint(triangle, relPoint);
				if (std::abs(relPoint.get<2>() - nearbyPoint.get<2>()) < MAX_TRIANGLE_HEIGHT) {
					triangleIds.push_back(tmpIds.at(i));
				}
			}
		}
	}

	for (auto triangleId : triangleIds) {
		auto& triangle = polyTriangles[triangleId];
		triangles.push_back(triangle);
	}
	return !triangles.empty();
}

double LinearInterpolationTriangleSurface::calculateZOnTriangle(const point_2t& pt, const Triangle& triangle)
{
	auto calculateNormal = [](const point_t& v1, const point_t& v2, const point_t& v3, DVector3& vn) {
		//平面方程: na * (x – n1) + nb * (y – n2) + nc * (z – n3) = 0 ;
		vn.x = (v2.get<1>() - v1.get<1>()) * (v3.get<2>() - v1.get<2>()) - (v2.get<2>() - v1.get<2>()) * (v3.get<1>() - v1.get<1>());
		vn.y = (v2.get<2>() - v1.get<2>()) * (v3.get<0>() - v1.get<0>()) - (v2.get<0>() - v1.get<0>()) * (v3.get<2>() - v1.get<2>());
		vn.z = (v2.get<0>() - v1.get<0>()) * (v3.get<1>() - v1.get<1>()) - (v2.get<1>() - v1.get<1>()) * (v3.get<0>() - v1.get<0>());
	};

	DVector3 vn = {};
	//v1(n1,n2,n3);
	auto& v1 = triangle.vertexes[0];
	auto& v2 = triangle.vertexes[1];
	auto& v3 = triangle.vertexes[2];
	calculateNormal(v1, v2, v3, vn);
	if (vn.z != 0) //如果平面平行Z轴
	{
		double z = v1.get<2>() - (vn.x * (pt.get<0>() - v1.get<0>()) + vn.y * (pt.get<1>() - v1.get<1>())) / vn.z;
		return z;
	}
	return 0;
}

void LinearInterpolationTriangleSurface::triangularize(
	MapPoint3D64Converter& mapPointConverter, 
	const std::vector<MapPoint3D64>& poly, 
	const PolygonDirection polygonDir,
	std::vector<Triangle>& polyTriangles)
{
	auto tmpPoly = poly;
	mapPointConverter.convert(tmpPoly.data(), tmpPoly.size());

	std::vector<Vector3> contour;
	for (auto& pt : tmpPoly) {
		contour.push_back(vec3(pt.pos.lon, pt.pos.lat, pt.z));
	}

	PolygonTriangularizer trizer;
	if (!trizer.triangularize((const Vector2*)contour.data(), contour.size(), polygonDir, sizeof(Vector3)))
		return;

	appendTriangulationResult(trizer, contour, polyTriangles);
}

void LinearInterpolationTriangleSurface::triangularizeWMT(MapPoint3D64Converter& mapPointConverter, const std::vector<MapPoint3D64>& poly, std::vector<Triangle>& polyTriangles)
{
	auto tmpPoly = poly;
	mapPointConverter.convert(tmpPoly.data(), tmpPoly.size());

	std::vector<Vector3> contour;
	for (auto& pt : tmpPoly) {
		contour.push_back(vec3(pt.pos.lon, pt.pos.lat, pt.z));
	}

	PolygonTriangularizer trizer;
	if (!trizer.triangularizeMWT((const Vector2*)contour.data(), contour.size(), sizeof(Vector3))
		&& trizer.pointMapping() != NULL)
		return;

	appendTriangulationResult(trizer, contour, polyTriangles);
}

void LinearInterpolationTriangleSurface::triangularizeStroke(
	MapPoint3D64Converter& mapPointConverter, 
	const std::vector<MapPoint3D64>& leftBoundary,
	const std::vector<MapPoint3D64>& rightBoundary,
	std::vector<Triangle>& polyTriangles)
{
	std::vector<Vector3> contour;
	auto tmpLeftPoints = leftBoundary;
	auto tmpRightPoints = rightBoundary;
	mapPointConverter.convert(tmpLeftPoints.data(), tmpLeftPoints.size());
	mapPointConverter.convert(tmpRightPoints.data(), tmpRightPoints.size());
	for_each(tmpRightPoints.begin(), tmpRightPoints.end(), [&contour](MapPoint3D64& pt) {
		contour.push_back(vec3(pt.pos.lon, pt.pos.lat, pt.z));
	});
	for_each(tmpLeftPoints.rbegin(), tmpLeftPoints.rend(), [&contour](MapPoint3D64& pt) {
		contour.push_back(vec3(pt.pos.lon, pt.pos.lat, pt.z));
	});

	PolygonTriangularizer trizer;
	auto rightN = tmpRightPoints.size();
	auto leftN = tmpLeftPoints.size();
	if (!trizer.triangularizeStroke((const Vector2*)contour.data(), rightN, leftN, sizeof(Vector3))
		|| trizer.pointMapping() == NULL)
		return;

	appendTriangulationResult(trizer, contour, polyTriangles);
}

void LinearInterpolationTriangleSurface::appendTriangulationResult(
	PolygonTriangularizer& trizer, 
	const std::vector<Vector3>& contour,
	std::vector<Triangle>& polyTriangles)
{
	std::vector<Vector3> points;
	const int* mapping = trizer.pointMapping();
	for (size_t iPt = 0; iPt < trizer.pointNumber(); iPt++)
	{
		Vector3 xyz;
		if (mapping != NULL)
			xyz = vec3(trizer.pointArray()[iPt], contour[mapping[iPt]].z);
		else
		{
			int index = findNearestPoint(contour.data(), contour.size(), trizer.pointArray()[iPt]);
			xyz = vec3(trizer.pointArray()[iPt], contour[index].z);
		}

		points.push_back(xyz);
	}

	auto triNum = trizer.indexNumber() / 3;
	for (size_t iIt = 0; iIt < trizer.indexNumber(); iIt += 3)
	{
		Triangle triangle{};
		auto a = trizer.indexArray()[iIt + 0];
		auto b = trizer.indexArray()[iIt + 1];
		auto c = trizer.indexArray()[iIt + 2];
		triangle.vertexes[0] = point_t(points[a].x, points[a].y, static_cast<int64>(points[a].z) * static_cast<int64>(10));
		triangle.vertexes[1] = point_t(points[b].x, points[b].y, static_cast<int64>(points[b].z) * static_cast<int64>(10));
		triangle.vertexes[2] = point_t(points[c].x, points[c].y, static_cast<int64>(points[c].z) * static_cast<int64>(10));
		for (auto& vertex : triangle.vertexes) {
			auto tmpPt = P3_P2(vertex);
			triangle.triangleLine.push_back(tmpPt);
			triangle.trianglePoly.push_back(tmpPt);
		}
		bg::correct(triangle.trianglePoly);
		if (triangle.trianglePoly.size() == 4)
			polyTriangles.push_back(triangle);
	}
}

std::vector<size_t> LinearInterpolationTriangleSurface::getPointIndexes(const point_t& point, rtree_type& rtree, std::vector<point_t>& originPoints)
{
	std::vector<size_t> indexes;
	rtree.query(bgi::intersects(point),
		boost::make_function_output_iterator([&](size_t const& id) {
			if (isEqualPoint(point, originPoints[id]))
			{
				indexes.push_back(id);
			}
		}));
	return indexes;
}

point_t LinearInterpolationTriangleSurface::getClosestPoint(const Triangle& triangle, const point_t& originPoint)
{
	linestring_t points;
	for (auto& point : triangle.vertexes)
		points.push_back(point);

	segment_t tmpSeg;
	points.push_back(points.front());
	bg::closest_points(originPoint, points, tmpSeg);
	return tmpSeg.first == originPoint ? tmpSeg.second : tmpSeg.first;
}

int LinearInterpolationTriangleSurface::findNearestPoint(const Vector3* points, int n, Vector2 p)
{
	float minLengthSquared = FLT_MAX;
	int index = 0;
	for (int i = 0; i < n; i++)
	{
		float lengthSquared = (vec2(points[i]) - p).lengthSquared();
		if (floatEqual(lengthSquared, 0))
			return i;
		if (lengthSquared < minLengthSquared)
		{
			index = i;
			minLengthSquared = lengthSquared;
		}
	}

	return index;
}

forceinline double LinearInterpolationTriangleSurface::minimalDegree(point_t& dir1, point_t& dir2)
{
	double angle1 = std::atan2(dir1.get<1>(), dir1.get<0>());
	angle1 = angle1 * 180 / MATH_PI;
	double angle2 = std::atan2(dir2.get<1>(), dir2.get<0>());
	angle2 = angle2 * 180 / MATH_PI;
	double angle = DBL_MAX;
	if (angle1 * angle2 >= 0) {
		angle = abs(angle1 - angle2);
	}
	else {
		angle = abs(angle1) + abs(angle2);
		if (angle > 180) {
			angle = 360 - angle;
		}
	}
	if (angle > 180) {
		angle = 360 - angle;
	}
	if (angle > 90) {
		angle = 180 - angle;
	}
	return angle;
}

