#include "stdafx.h"
#include "IntersectionCompiler.h"
#include <algorithm>
#include "math3d/vector_math.h"
#include "algorithm/grap_point_algorithm.h"
#include "algorithm/poly2tri.h"
#include "algorithm/line_projection_triangle_surface.h"
#include "algorithm/linear_interpolation_triangle_surface.h"

namespace OMDB
{
	void IntersectionCompiler::compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile)
	{
		/***
		调试路口典型网格
		ids.push_back(20597829);
		ids.push_back(20597828);
		ids.push_back(20169162);
		ids.push_back(19927141);
		ids.push_back(19927059);
		ids.push_back(20459268);
		ids.push_back(19962887);
		ids.push_back(20596473);
		***/

		UNREFERENCED_PARAMETER(nearby);
		for (auto obj : pGrid->query(ElementType::HAD_INTERSECTION))
		{
			HadIntersection* hadInts = (HadIntersection*)obj;
			m_hadInts = hadInts;

			//判断是否为普通路
			if (CompileSetting::instance()->isNotCompileUrbanData)
			{
				if (!isProDataLevel(hadInts->refLaneGroups))
					continue;
			}

			if (hadInts->originId == 84206502092750504)
				printInfo("");

			if (hadInts == nullptr || hadInts->refLaneGroups.size() == 0)
				continue;
			clearInfoData();
			bool isNextRun = false;
			for (auto iter = hadInts->refLaneGroups.begin(); iter != hadInts->refLaneGroups.end(); ++iter)
			{
				HadLaneGroup* pGroup = (HadLaneGroup*)(*iter);
				for (auto& d1 : pGroup->lanes)
				{
					if (d1->originId == 196252384386038273)
						printInfo("");
				}
				isNextRun = setLaneAndNodeInfo(pGroup);
			}
			if (!isNextRun)
				continue;
			setIntersectionInfo(hadInts);

			// 路口外的车道组的相邻点也加到NodeInfo
			setOutIntersectionNodeInfo();

			// first
			std::vector<point_2t> tmpOriginPoint2ts;
			std::vector<NodeInfo> originNodes;
			for (auto item : allNodes)
			{
				originNodes.push_back(item.second);
				auto tmpPoint = item.second._originPosition;
				coordinatesTransform.convert(&tmpPoint, 1);
				tmpOriginPoint2ts.push_back(POINT_2T(tmpPoint));
			}
			parameters param;
			index_2getter originInd(tmpOriginPoint2ts);
			rtree_2type rtree(boost::irange<std::size_t>(0lu, tmpOriginPoint2ts.size()), param, originInd);
			linestring_2t tmpLineString2T;
			if (newIntersectionBoundaries.empty())
				continue;
			for (auto item : newIntersectionBoundaries)
				tmpLineString2T.insert(tmpLineString2T.end(), item.begin(), item.end());
			ring_2t tmpRing;
			bg::convex_hull(tmpLineString2T, tmpRing);
			multi_linestring_2t tmpMultiLines;
			for (auto item : newIntersectionBoundaries)
				tmpMultiLines.push_back(item);
			for (auto it = tmpRing.begin(); it != tmpRing.end();)
			{
				point_2t tmpPoint;
				rtree.query(bgi::nearest(*it, 1),
					boost::make_function_output_iterator([&](size_t const& id) {
						tmpPoint = tmpOriginPoint2ts[id];
						}));

				bool isEqual = bg::equals(*it, tmpPoint);
				bool isNearStopLine = bg::distance(*it, tmpMultiLines) < 1000;
				if (!bg::equals(*it, tmpPoint) && !isNearStopLine)
					it = tmpRing.erase(it);
				else
					it++;
			}
			bg::correct(tmpRing);

			// 使用newIntersectionBoundaries重构多边形
			std::vector<point_2t> tmpRingVector;
			for (size_t i = 0; i < tmpRing.size(); ++i)
				tmpRingVector.push_back(tmpRing.at(i));
			for (size_t i = 0; i < newIntersectionBoundaries.size(); ++i)
			{
				for (size_t j = 0; j < newIntersectionBoundaries.at(i).size(); ++j)
				{
					point_2t point = newIntersectionBoundaries.at(i).at(j);
					std::vector<point_2t>::iterator ita, itb;
					getClosestSeg(point, tmpRingVector, ita, itb);
					tmpRingVector.insert(itb, point);
				}
			}
			tmpRing.assign(tmpRingVector.begin(), tmpRingVector.end());
			bg::unique(tmpRing);

			// 路口编译主流程 
			Polygon3d originIntersctPoly;
			for (size_t i = 1; i < tmpRing.size(); i++)
			{
				const auto& startPt = getRingPt(tmpRing, i - 1);
				const auto& endPt = getRingPt(tmpRing, i);
				std::vector<NodeInfo> resultStartNodes = getNearestNodeInfos(startPt, 2, rtree, originNodes);
				std::vector<NodeInfo> resultEndNodes = getNearestNodeInfos(endPt, 2, rtree, originNodes);
				if (resultStartNodes.empty() || resultEndNodes.empty())
					continue;

				auto getOriginRoadResult = [&]()-> bool
				{
					std::vector<LaneInfo> laneInfos;
					for (auto resultStartNode : resultStartNodes)
					{
						for (auto j : resultStartNode._line)
						{
							for (auto resultEndNode : resultEndNodes)
							{
								for (auto k : resultEndNode._line)
								{
									if (j._originLaneId == k._originLaneId)
									{
										laneInfos.push_back(j);
									}
								}
							}
						}
					}
					if (!laneInfos.empty()) {
						LaneInfo laneInfo;
						getNearestLaneInfo(laneInfos, laneInfo);
						auto tmpEdge = laneInfo._originLocation.vertexes;
						if (isEqualPoint2T(startPt, laneInfo._line2d.back()))
							std::reverse(tmpEdge.begin(), tmpEdge.end());
						originIntersctPoly.vertexes.insert(originIntersctPoly.vertexes.end(), tmpEdge.begin(), tmpEdge.end());
						return true;
					}

					return false;
				};

				ring_2t tmpBox;
				double paPbLength = getRingBox(startPt, endPt, tmpBox);

				//两点距离较小情况下，对于停止线和原始情况，都采用直连方式
				if (paPbLength < 1000.0)
				{
					originIntersctPoly.vertexes.push_back(resultStartNodes.front()._originPosition);
					originIntersctPoly.vertexes.push_back(resultEndNodes.front()._originPosition);
					continue;
				}

				bool isStopLine = isBoxStopLine(tmpBox, paPbLength);
				if (isStopLine)
				{
					originIntersctPoly.vertexes.push_back(resultStartNodes.front()._originPosition);
					originIntersctPoly.vertexes.push_back(resultEndNodes.front()._originPosition);
					compilerData.stopLineToOriginIdMaps.emplace(compilerData.gridIntectionStopLines.size(), hadInts->originId);
					compilerData.gridIntectionStopLines.push_back(segment_t(resultStartNodes.front()._point3d, resultEndNodes.front()._point3d));
					continue;
				}

				if (getOriginRoadResult())
					continue;

				auto jNode = resultStartNodes.front();
				auto kNode = resultEndNodes.front();
				originIntersctPoly.vertexes.push_back(jNode._originPosition);
				const auto& prevPt = getRingPt(tmpRing, i - 1, -1);
				const auto& nextPt = getRingPt(tmpRing, i, 1);
				point_2t startPtDir = { startPt.get<1>() - prevPt.get<1>() , startPt.get<0>() - prevPt.get<0>() };
				point_2t endPtDir = { nextPt.get<1>() - endPt.get<1>() , nextPt.get<0>() - endPt.get<0>() };
				segment_2t startSeg = segment_2t(startPt, prevPt);
				segment_2t endSeg = segment_2t(endPt, nextPt);
				double disP = bg::distance(startPt, endPt);
				double disPaEx = bg::distance(startPt, SEGMENT_2T_EX(endSeg, 1.0e6));
				double disPbEx = bg::distance(endPt, SEGMENT_2T_EX(startSeg, 1.0e6));
				double disPa = bg::distance(startPt, endSeg);
				double disPb = bg::distance(endPt, startSeg);
				double kPa = disPaEx / disP;
				double kPb = disPbEx / disP;
				double startEndPtDirDegree = minimalDegree(startPtDir, endPtDir);
				if (startEndPtDirDegree > 30 && kPa > 0.2 && kPb > 0.2 && disPa - disPaEx > 1000.0 && disPb - disPbEx > 1000.0)
				{
					int numSegments = 10;
					auto p0 = jNode._point3d;
					auto p2 = kNode._point3d;
					segment_2t tmpSeg = segment_2t(P3_P2(p0), P3_P2(p2));
					tmpSeg = SEGMENT_2T_IN(tmpSeg, 0.01);
					bool isOut = false;
					for (auto tmpLane : intersectionOnlyLaneLines)
					{
						if (bg::intersects(tmpLane, tmpSeg))
						{
							isOut = true;
							break;
						}
					}
					auto vd = S3_V3(p0, p2);
					vector_t vn = isOut ? bg::cross_product(vector_t(0.0, 0.0, 1.0), vd) : bg::cross_product(vector_t(0.0, 0.0, -1.0), vd);
					double var = isOut ? 0.6 : 0.2;
					auto vnUnit = V3_N(vn);
					double factor = std::abs(bg::distance(p0, p2)) * var;
					point_t p1 = point_t(
						(p0.get<0>() + p2.get<0>()) / 2 + vnUnit.get<0>() * factor,
						(p0.get<1>() + p2.get<1>()) / 2 + vnUnit.get<1>() * factor,
						(p0.get<2>() + p2.get<2>()) / 2);
					for (int j = 1; j < numSegments; j++)
					{
						double t = static_cast<double>(j) / numSegments;
						point_t curvePoint = calculateBezierPoint(p0, p1, p2, t);
						auto curvePointMap = MapPoint3D64_make(curvePoint.get<0>(), curvePoint.get<1>(), curvePoint.get<2>() / 10);
						coordinatesTransform.invert(&curvePointMap, 1);
						originIntersctPoly.vertexes.push_back(curvePointMap);
					}
				}
				originIntersctPoly.vertexes.push_back(kNode._originPosition);
			}

			//debug data
			auto dData = originIntersctPoly;
			coordinatesTransform.convert(dData.vertexes.data(), dData.vertexes.size());
			auto dRing = RING_2T(dData.vertexes);

			//删除相邻重复的点
			for (auto it = originIntersctPoly.vertexes.begin(); it != originIntersctPoly.vertexes.end() - 1;)
			{
				if (mapPoint3D64_compare(*it, *(it + 1)))
					it = originIntersctPoly.vertexes.erase(it);
				else
					it++;
			}

			//判断自相交且打印
			//if (Poly2Tri::isSelfIntersections(originIntersctPoly.vertexes))
			//	std::cout << "intersection is self intersects: " << "(" << originIntersctPoly.vertexes.front().pos.lon / 1000 << "," << originIntersctPoly.vertexes.front().pos.lat / 1000 << ")" << std::endl;

			// 生成Rds数据
			RdsIntersection* pInts = (RdsIntersection*)createObject(pTile, EntityType::RDS_INTERSECTION);
			convert(originIntersctPoly, pInts->contour);
			saveRdsIntersection(pInts, originIntersctPoly, hadInts->originId);
			for (auto laneGroup : hadInts->refLaneGroups)
			{
				RdsGroup* pRdsGroup = queryGroup(laneGroup->originId, pTile);
				if (pRdsGroup)
					pRdsGroup->objects.push_back(pInts);
			}
		}
	}

	void IntersectionCompiler::saveRdsIntersection(RdsIntersection* pIntersection, Polygon3d& originIntersctPoly, int64& originId)
	{
		if (pIntersection->contour.vertexes.empty())
			return;

		if (originIntersctPoly.vertexes.size() > 2)
		{
			std::vector<Triangle> intersectionTriangles;
			LinearInterpolationTriangleSurface::triangularize(coordinatesTransform, originIntersctPoly.vertexes, PolygonDirection_unknown, intersectionTriangles);
			compilerData.m_intersectionTriangles.insert(compilerData.m_intersectionTriangles.end(), intersectionTriangles.begin(), intersectionTriangles.end());
		}

		rdsIntersectionInfo tmp;
		tmp._originId = originId;
		tmp._intersection = pIntersection;
		tmp._originIntersctPoly = originIntersctPoly;
		coordinatesTransform.convert(originIntersctPoly.vertexes.data(), originIntersctPoly.vertexes.size());
		tmp._intersectionBox2T = BOX_2T(originIntersctPoly.vertexes);
		tmp._intersectionPoly2T = RING_2T(originIntersctPoly.vertexes);
		tmp._intersectionPoints = LINESTRING_T(originIntersctPoly.vertexes);
		bg::correct(tmp._intersectionPoly2T);
		compilerData.m_rdsIntersections.push_back(tmp);
		compilerData.m_rdsIntersectionBoxes.push_back(tmp._intersectionBox2T);
	}

	void IntersectionCompiler::combineNearbyStopLine(ring_2t& tmpRing, rtree_2type& rtree, std::vector<NodeInfo>& originNodes)
	{
		auto isSameStopLine = [&](std::pair<segment_2t, bool>& currSeg, std::pair<segment_2t, bool>& nextSeg)->bool {
			if (currSeg.second && nextSeg.second) {
				point_2t currDir = V2_P2(S2_V2(currSeg.first));
				point_2t nextDir = V2_P2(S2_V2(nextSeg.first));
				double currNextDirDegree = minimalDegree(currDir, nextDir);
				if (currNextDirDegree < 10)
					return true;
			}
			return false;
		};

		std::vector<std::pair<segment_2t, bool>> ringSegments;
		for (size_t i = 1; i < tmpRing.size(); i++) {
			const auto& startPt = getRingPt(tmpRing, i - 1);
			const auto& endPt = getRingPt(tmpRing, i);
			std::vector<NodeInfo> resultStartNodes = getNearestNodeInfos(startPt, 2, rtree, originNodes);
			std::vector<NodeInfo> resultEndNodes = getNearestNodeInfos(endPt, 2, rtree, originNodes);
			if (resultStartNodes.empty() || resultEndNodes.empty()) {
				ringSegments.push_back(std::make_pair(segment_2t(startPt, endPt), false));
				continue;
			}

			ring_2t tmpBox;
			double paPbLength = getRingBox(startPt, endPt, tmpBox);
			bool isStopLine = isBoxStopLine(tmpBox, paPbLength);
			if (isStopLine) {
				ringSegments.push_back(std::make_pair(segment_2t(startPt, endPt), true));
			} else {
				ringSegments.push_back(std::make_pair(segment_2t(startPt, endPt), false));
			}
		}

		// merge
		for (auto iter = ringSegments.begin(); iter != ringSegments.end();) {
			auto& currSegment = *iter;
			if (iter != ringSegments.end() - 1) {
				auto& nextSegment = *(iter + 1);
				if (isSameStopLine(currSegment, nextSegment)) {
					currSegment.first.second = nextSegment.first.second;
					iter = ringSegments.erase(iter + 1);
					iter--;
					continue;
				}
			} else {
				auto& nextSegment = *(ringSegments.begin());
				if (isSameStopLine(currSegment, nextSegment)) {
					currSegment.first.second = nextSegment.first.second;
					ringSegments.erase(ringSegments.begin());
					break;
				}
			}
			iter++;
		}

		// combine
		tmpRing.clear();
		for (auto iter = ringSegments.begin(); iter != ringSegments.end(); iter++) {
			auto& item = (*iter).first;
			tmpRing.push_back(item.first);
			if (iter == ringSegments.end() - 1) {
				tmpRing.push_back(item.second);
			}
		}
		bg::correct(tmpRing);
	}

	std::vector<NodeInfo> IntersectionCompiler::getNearestNodeInfos(const point_2t& point, size_t k, rtree_2type& rtree, std::vector<NodeInfo>& originNodes)
	{
		std::vector<NodeInfo> nodes;
		rtree.query(bgi::nearest(point, k),
			boost::make_function_output_iterator([&](size_t const& id) {
				if (isEqualPoint2T(point, originNodes[id]._point2d))
				{
					nodes.push_back(originNodes[id]);
				}
			}));
		return nodes;
	}

	inline bool IntersectionCompiler::isBoxStopLine(ring_2t& outBox, double paPbLength)
	{
		bool isStopLine = false;
		for (auto tmpRefPoint : refLinkNodes)
		{
			if (bg::within(tmpRefPoint, outBox))
			{
				isStopLine = true;
				break;
			}
		}

		auto isStopLineSegment = [&](std::vector<segment_t>& segments)->bool {
			bool intersected = false;
			double maxStopLineLength = DBL_MIN;
			for (auto& tmpStopLine : segments) {
				auto tmpStopLine2t = S3_S2(tmpStopLine);
				if (bg::intersects(tmpStopLine2t, outBox)) {
					double stopLineLength = std::abs(bg::length(tmpStopLine2t));
					if (stopLineLength > maxStopLineLength) {
						maxStopLineLength = stopLineLength;
						intersected = true;
					}
				}
			}
			if (intersected) {
				if (paPbLength / maxStopLineLength < 1)
					return true;
			}
			return false;
		};

		if (!isStopLine) {
			isStopLine = isStopLineSegment(connectStopLineSegments);
		}

		return isStopLine;
	}

	inline double IntersectionCompiler::getRingBox(const point_2t& startPt, const point_2t& endPt, ring_2t& outBox)
	{
		outBox.clear();
		double tmpWidth = std::abs(bg::distance(startPt, endPt));
		tmpWidth *= 0.1;
		double tmpHeight = 500.0;
		tmpWidth = min(tmpWidth, tmpHeight);
		auto va = S2_V2(startPt, endPt);
		auto v_n = V2_N(va);
		auto va_n = vector_2t(v_n.get<1>(), -v_n.get<0>());
		auto vb_n = vector_2t(-v_n.get<1>(), v_n.get<0>());
		auto pa = point_2t(startPt.get<0>() + v_n.get<0>() * tmpWidth, startPt.get<1>() + v_n.get<1>() * tmpWidth);
		auto pb = point_2t(endPt.get<0>() - v_n.get<0>() * tmpWidth, endPt.get<1>() - v_n.get<1>() * tmpWidth);
		outBox.push_back(point_2t(pa.get<0>() + va_n.get<0>() * tmpHeight, pa.get<1>() + va_n.get<1>() * tmpHeight));
		outBox.push_back(point_2t(pb.get<0>() + va_n.get<0>() * tmpHeight, pb.get<1>() + va_n.get<1>() * tmpHeight));
		outBox.push_back(point_2t(pb.get<0>() + vb_n.get<0>() * tmpHeight, pb.get<1>() + vb_n.get<1>() * tmpHeight));
		outBox.push_back(point_2t(pa.get<0>() + vb_n.get<0>() * tmpHeight, pa.get<1>() + vb_n.get<1>() * tmpHeight));
		bg::correct(outBox);
		return std::abs(bg::distance(pa, pb));
	}

	inline point_2t& IntersectionCompiler::getRingPt(ring_2t& ring, int index, int offset/* = 0*/)
	{
		if (offset == 0)
			return ring[index];

		auto oi = index + offset;
		if (index == 0) {
			if (offset > 0)
				return ring[oi];

			// skip back point
			return ring[ring.size() - 1 + offset];
		}
		if (index == ring.size() - 1) {
			if (offset < 0)
				return ring[oi];

			// skip first point
			return ring[offset];
		}
		return ring[oi];
	}

	std::vector<MapPoint3D64> IntersectionCompiler::offsetPointForBoundary(
		const point_t& pa,
		const point_t& pb)
	{
		std::vector<MapPoint3D64> tmpMps;
		segment_t pabSeg = segment_t(pa, pb);
		std::vector<point_t> pts;
		segment_t tmpSeg;
		pts.push_back(pa);
		pts.push_back(pb);
		for (auto& item : pts)
		{
			double dist_min = 1.0e10;
			double tmpDis = 1.0e10;
			segment_t currentSeg;
			for (size_t i = 0; i < stopLineNearbySegments.size(); ++i)
			{
				bg::closest_points(item, stopLineNearbySegments[i], tmpSeg);
				double const dist = bg::length(tmpSeg);
				if (dist == 0.0)
				{
					currentSeg = stopLineNearbySegments[i];
					tmpDis = 0.0;
					break;
				}
				else if (dist < dist_min)
				{
					dist_min = dist;
					tmpDis = dist;
					currentSeg = stopLineNearbySegments[i];
				}
			}

			double extendDis = 3000.0;
			if (tmpDis > extendDis)
			{
				tmpMps.push_back(MapPoint3D64_make(item.get<0>(), item.get<1>(), item.get<2>() / 10));
				continue;
			}
			if (bg::dot_product(S2_V2(S3_S2(pabSeg)), S2_V2(S3_S2(currentSeg))) < 0.0)
			{
				currentSeg = segment_t(currentSeg.second, currentSeg.first);
			}
			bg::closest_points(item, currentSeg, tmpSeg);
			vector_t pabNV = bg::cross_product(vector_t(0.0, 0.0, 1.0), V3_N(S3_V3(pabSeg)));
			tmpMps.push_back(MapPoint3D64_make(
				tmpSeg.second.get<0>() + pabNV.get<0>() * 50.0, 
				tmpSeg.second.get<1>() + pabNV.get<1>() * 50.0, 
				tmpSeg.second.get<2>() / 10));
		}
		coordinatesTransform.invert(tmpMps.data(), tmpMps.size());
		return tmpMps;
	}

	std::vector<MapPoint3D64> IntersectionCompiler::stopLinePointForBoundary(const point_t& pa, const point_t& pb)
	{
		std::vector<MapPoint3D64> tmpMps;
		std::vector<MapPoint3D64> originStopLinePts;
		std::vector<bool> originStopLineNums;
		segment_t pabSeg = segment_t(pa, pb);
		std::vector<point_t> pts;
		segment_t tmpSeg;
		pts.push_back(pa);
		pts.push_back(pb);
		for (auto& item : pts)
		{
			segment_t currentSeg;
			double tmpDis = 1.0e10;
			double ptsDis = 1.0e10;
			std::vector<MapPoint3D64> originStopLine;
			for (size_t i = 0; i < stopLineNearbySegments.size(); ++i)
			{
				auto& stopLineNearbySegment = stopLineNearbySegments[i];
				bg::closest_points(item, stopLineNearbySegment, tmpSeg);
				double const dist = bg::length(tmpSeg);
				if (dist == 0.0)
				{
					segment_t stopLineTmpSeg;
					double stopLinePointDis = 0.0;
					bg::closest_points(stopLineNearbySegment.first, pabSeg, stopLineTmpSeg);
					stopLinePointDis += bg::length(stopLineTmpSeg);
					bg::closest_points(stopLineNearbySegment.second, pabSeg, stopLineTmpSeg);
					stopLinePointDis += bg::length(stopLineTmpSeg);
					if (stopLinePointDis < ptsDis)
					{
						// 处理路口处两条停止线首尾相连的情况
						originStopLine = originStopLineNearbySegments[i];
						currentSeg = stopLineNearbySegment;
						ptsDis = stopLinePointDis;
						tmpDis = 0.0;
					}
				}
				else if (dist < tmpDis)
				{
					tmpDis = dist;
					currentSeg = stopLineNearbySegment;
					originStopLine = originStopLineNearbySegments[i];
				}
			}

			double extendDis = 3000.0;
			if (tmpDis > extendDis)
			{
				tmpMps.push_back(MapPoint3D64_make(item.get<0>(), item.get<1>(), item.get<2>() / 10));
				continue;
			}
			if (bg::dot_product(S2_V2(S3_S2(pabSeg)), S2_V2(S3_S2(currentSeg))) < 0.0)
			{
				currentSeg = segment_t(currentSeg.second, currentSeg.first);
			}
			bg::closest_points(item, currentSeg, tmpSeg);
			vector_t pabNV = bg::cross_product(vector_t(0.0, 0.0, 1.0), V3_N(S3_V3(pabSeg)));
			tmpMps.push_back(MapPoint3D64_make(
				tmpSeg.second.get<0>() + pabNV.get<0>() * 50.0,
				tmpSeg.second.get<1>() + pabNV.get<1>() * 50.0,
				tmpSeg.second.get<2>() / 10));

			if (originStopLine.size() > 1) {
				originStopLineNums.push_back(true);
				originStopLinePts.push_back(originStopLine.front());
				originStopLinePts.push_back(originStopLine.back());
			}
		}

		if (originStopLineNums.size() == pts.size())
		{
			vector_t pabVec = V3_N(S3_V3(pabSeg));
			MapPoint3D64 tmpPos = MapPoint3D64_make(pa.get<0>(), pa.get<1>(), pa.get<2>());
			coordinatesTransform.invert(&tmpPos, 1); tmpPos.z /= 10;
			point_t tmpPosPoint = POINT_T(tmpPos);
			std::sort(originStopLinePts.begin(), originStopLinePts.end(), [&](MapPoint3D64& pt0, MapPoint3D64& pt1)->bool {
				vector_t tmp0V = S3_V3(tmpPosPoint, POINT_T(pt0));
				auto v0Sum = bg::dot_product(pabVec, tmp0V);

				vector_t tmp1V = S3_V3(tmpPosPoint, POINT_T(pt1));
				auto v1Sum = bg::dot_product(pabVec, tmp1V);
				return v0Sum < v1Sum;
			});
			auto ip = std::unique(originStopLinePts.begin(), originStopLinePts.end(), mapPoint3D64_compare);
			originStopLinePts.resize(std::distance(originStopLinePts.begin(), ip));
			return originStopLinePts;
		}
		coordinatesTransform.invert(tmpMps.data(), tmpMps.size());
		return tmpMps;
	}

	bool IntersectionCompiler::setLaneAndNodeInfo(
		HadLaneGroup* const pGroup)
	{
		bool isNextRun = false;
		if (pGroup->roadBoundaries.size() > 1)
		{
			for (size_t i = 0; i < pGroup->roadBoundaries.size(); i++)
			{
				LineString3d leftSide;
				HadRoadBoundary* obj = pGroup->roadBoundaries[i];
				LaneInfo tmpLaneInfo;

				leftSide = obj->location;
				if (directionEqual(obj, pGroup, 3))
					std::reverse(leftSide.vertexes.begin(), leftSide.vertexes.end());

				//删除相邻重复或者离得特别近的点
				for (auto it = leftSide.vertexes.begin(); it != leftSide.vertexes.end() - 1;)
				{
					auto& opa = *it;
					auto& opb = *(it + 1);
					if (std::abs(opa.pos.lon - opb.pos.lon) < 10 && std::abs(opa.pos.lat - opb.pos.lat) < 10)
						it = leftSide.vertexes.erase(it);
					else
						it++;
				}

				tmpLaneInfo._originLocation = leftSide;
				auto tmpLeftSide = leftSide.vertexes;
				coordinatesTransform.convert(tmpLeftSide.data(), tmpLeftSide.size());
				auto d1 = LINESTRING_2T(tmpLeftSide);
				intersectionLaneLines.push_back(d1);
				tmpLaneInfo._type = LaneInfoType::ROAD_BOUNDARY;
				tmpLaneInfo._originLaneId = obj->originId;
				tmpLaneInfo._length = bg::length(d1);
				tmpLaneInfo._line2d = d1;

				auto it = allNodes.find(obj->startNode->originId);
				if (it != allNodes.end())
				{
					it->second._line.push_back(tmpLaneInfo);
				}
				else
				{
					NodeInfo tmpStartNode;
					tmpStartNode._originNodeId = obj->startNode->originId;
					tmpStartNode._originPosition = leftSide.vertexes.front();
					auto _p = tmpStartNode._originPosition;
					coordinatesTransform.convert(&_p, 1);
					originPoints.push_back(tmpStartNode._originPosition);
					tmpStartNode._line.push_back(tmpLaneInfo);
					tmpStartNode._point2d = POINT_2T(_p);
					tmpStartNode._point3d = POINT_T(_p);
					allNodes.emplace(obj->startNode->originId, tmpStartNode);
					roadBoundaryPts.push_back(tmpStartNode._point2d);
				}

				it = allNodes.find(obj->endNode->originId);
				if (it != allNodes.end())
				{
					it->second._line.push_back(tmpLaneInfo);
				}
				else
				{
					NodeInfo tmpStartNode;
					tmpStartNode._originNodeId = obj->endNode->originId;
					tmpStartNode._originPosition = leftSide.vertexes.back();
					auto _p = tmpStartNode._originPosition;
					coordinatesTransform.convert(&_p, 1);
					originPoints.push_back(tmpStartNode._originPosition);
					tmpStartNode._line.push_back(tmpLaneInfo);
					tmpStartNode._point2d = POINT_2T(_p);
					tmpStartNode._point3d = POINT_T(_p);
					allNodes.emplace(obj->endNode->originId, tmpStartNode);
					roadBoundaryPts.push_back(tmpStartNode._point2d);
				}
			}
			isNextRun = true;
		}
		if (pGroup->laneBoundaries.size() > 1)
		{
			for (size_t i = 0; i < pGroup->laneBoundaries.size(); i++)
			{
				LineString3d leftSide;
				HadLaneBoundary* obj = pGroup->laneBoundaries[i];
				LaneInfo tmpLaneInfo;

				leftSide = obj->location;
				if (directionEqual(obj, pGroup, 3))
					std::reverse(leftSide.vertexes.begin(), leftSide.vertexes.end());

				//删除相邻重复或者离得特别近的点
				for (auto it = leftSide.vertexes.begin(); it != leftSide.vertexes.end() - 1;)
				{
					auto& opa = *it;
					auto& opb = *(it + 1);
					if (std::abs(opa.pos.lon - opb.pos.lon) < 10 && std::abs(opa.pos.lat - opb.pos.lat) < 10)
						it = leftSide.vertexes.erase(it);
					else
						it++;
				}

				tmpLaneInfo._originLocation = leftSide;
				auto tmpLeftSide = leftSide.vertexes;
				coordinatesTransform.convert(tmpLeftSide.data(), tmpLeftSide.size());
				auto d1 = LINESTRING_2T(tmpLeftSide);
				intersectionLaneLines.push_back(d1);
				intersectionOnlyLaneLines.push_back(d1);
				tmpLaneInfo._type = LaneInfoType::LANE_BOUNDARY;
				tmpLaneInfo._originLaneId = obj->originId;
				tmpLaneInfo._length = bg::length(d1);
				tmpLaneInfo._line2d = d1;

				auto it = allNodes.find(obj->startNode->originId);
				if (it != allNodes.end())
				{
					it->second._line.push_back(tmpLaneInfo);
				}
				else
				{
					NodeInfo tmpStartNode;
					tmpStartNode._originNodeId = obj->startNode->originId;
					tmpStartNode._originPosition = leftSide.vertexes.front();
					auto _p = tmpStartNode._originPosition;
					coordinatesTransform.convert(&_p, 1);
					originPoints.push_back(tmpStartNode._originPosition);
					tmpStartNode._line.push_back(tmpLaneInfo);
					tmpStartNode._point2d = POINT_2T(_p);
					tmpStartNode._point3d = POINT_T(_p);
					allNodes.emplace(obj->startNode->originId, tmpStartNode);
				}

				it = allNodes.find(obj->endNode->originId);
				if (it != allNodes.end())
				{
					it->second._line.push_back(tmpLaneInfo);
				}
				else
				{
					NodeInfo tmpStartNode;
					tmpStartNode._originNodeId = obj->endNode->originId;
					tmpStartNode._originPosition = leftSide.vertexes.back();
					auto _p = tmpStartNode._originPosition;
					coordinatesTransform.convert(&_p, 1);
					originPoints.push_back(tmpStartNode._originPosition);
					tmpStartNode._line.push_back(tmpLaneInfo);
					tmpStartNode._point2d = POINT_2T(_p);
					tmpStartNode._point3d = POINT_T(_p);
					allNodes.emplace(obj->endNode->originId, tmpStartNode);
				}
			}
			isNextRun = true;
		}
		for (size_t k = 0; k < intersectionLaneLines.size(); ++k)
		{
			for (size_t m = 0; m < intersectionLaneLines.at(k).size(); ++m)
			{
				intersectionLaneLinePoints.push_back(intersectionLaneLines.at(k).at(m));
			}
		}
		return isNextRun;
	}

	void IntersectionCompiler::setOutIntersectionNodeInfo()
	{
		std::vector<point_2t> tmpOriginPoint2ts;
		std::vector<NodeInfo> originNodes;
		int64 maxNodeId = LLONG_MIN;
		for (auto item : allNodes)
		{
			originNodes.push_back(item.second);
			auto tmpPoint = item.second._originPosition;
			coordinatesTransform.convert(&tmpPoint, 1);
			tmpOriginPoint2ts.push_back(POINT_2T(tmpPoint));
			maxNodeId = max(maxNodeId, item.second._originNodeId);
		}

		parameters param;
		index_2getter originInd(tmpOriginPoint2ts);
		rtree_2type rtree(boost::irange<std::size_t>(0lu, tmpOriginPoint2ts.size()), param, originInd);

		for (size_t i = 0; i < outIntersectionPoints.size(); ++i)
		{
			NodeInfo tmpStartNode;
			tmpStartNode._originPosition = outIntersectionPoints.at(i);
			auto _p = tmpStartNode._originPosition;
			coordinatesTransform.convert(&_p, 1);
			tmpStartNode._point2d = POINT_2T(_p);
			tmpStartNode._point3d = POINT_T(_p);
			std::vector<NodeInfo> resultStartNodes = getNearestNodeInfos(tmpStartNode._point2d, 2, rtree, originNodes);
			if (resultStartNodes.empty())
			{
				maxNodeId++;
				tmpStartNode._originNodeId = maxNodeId;
				allNodes.emplace(maxNodeId, tmpStartNode);
				roadBoundaryPts.push_back(tmpStartNode._point2d);
			}
		}
	}

	void IntersectionCompiler::setIntersectionInfo(
		HadIntersection* const pIntersection)
	{
		for (auto iter = pIntersection->refLaneGroups.begin(); iter != pIntersection->refLaneGroups.end(); ++iter)
		{
			HadLaneGroup* pGroup = (HadLaneGroup*)(*iter);
			if (pGroup->lanes.size())
			{
				for (size_t i = 0; i < pGroup->lanes.size(); i++)
				{
					HadLane* obj = pGroup->lanes[i];
					if (!obj->location.vertexes.empty())
					{
						auto sp = obj->location.vertexes.front();
						auto ep = obj->location.vertexes.back();
						coordinatesTransform.convert(&sp, 1);
						coordinatesTransform.convert(&ep, 1);
						refLinkNodes.push_back(POINT_2T(sp));
						refLinkNodes.push_back(POINT_2T(ep));
					}
				}
			}
			updateStopLineData(pGroup);
		}

		std::vector<linestring_2t> tmpFrontPoints, tmpBackPoints;
		for (auto tmp : frontPoints)
		{
			coordinatesTransform.convert(tmp.data(), tmp.size());
			tmpFrontPoints.push_back(LINESTRING_2T(tmp));
		}
		for (auto tmp : backPoints)
		{
			coordinatesTransform.convert(tmp.data(), tmp.size());
			tmpBackPoints.push_back(LINESTRING_2T(tmp));
		}

		// 过滤：线段两点重合 
		for (auto it = connectStopLineSegments.begin(); it != connectStopLineSegments.end();)
		{
			if (bg::length(*it) == 0)
				it = connectStopLineSegments.erase(it);
			else
				it++;
		}

		// 过滤: 线段在多边形里面
		linestring_2t tmpLine;
		for (auto item : connectStopLineSegments)
		{
			tmpLine.push_back(P3_P2(item.first));
			tmpLine.push_back(P3_P2(item.second));
		}
		ring_2t stopLineRing;
		bg::convex_hull(tmpLine, stopLineRing);
		std::vector<point_2t> tmpRingVector;
		for (size_t i = 0; i < stopLineRing.size(); ++i)
			tmpRingVector.push_back(stopLineRing.at(i));
		for (auto item : tmpLine)
		{
			std::vector<point_2t>::iterator ita, itb;
			getClosestSeg(item, tmpRingVector, ita, itb);
			tmpRingVector.insert(itb, item);
		}
		stopLineRing.assign(tmpRingVector.begin(), tmpRingVector.end());
		bg::unique(stopLineRing);
		for (auto ita = connectStopLineSegments.begin(); ita != connectStopLineSegments.end();)
		{
			auto& currentSeg = *ita;
			point_t cp;
			bg::centroid(currentSeg, cp);
			bool isErase = true;
			point_2t tmpCp = P3_P2(cp);
			for (auto itb = bg::segments_begin(stopLineRing); itb != bg::segments_end(stopLineRing); ++itb)
			{
				if (bg::distance(tmpCp, *itb) < 200)
				{
					isErase = false;
					break;
				}
				if (!bg::within(tmpCp, stopLineRing))
				{
					isErase = false;
					break;
				}
			}
			if (isErase)
			{
				segment_2t exCurrentSeg = S3_S2(SEGMENT_EX(currentSeg, 50000));
				std::vector<point_2t> leftPoints;
				std::vector<point_2t> rightPoints;
				for (auto item : stopLineRing)
				{
					if (IS_LEFT_POINT_OF_SEGMENT_2T(exCurrentSeg, item))
						leftPoints.push_back(item);
					else
						rightPoints.push_back(item);
				}
				std::sort(leftPoints.begin(), leftPoints.end(), [&](const point_2t& a, const point_2t& b)->bool {
					return bg::distance(a, exCurrentSeg) < bg::distance(b, exCurrentSeg);
					});
				std::sort(rightPoints.begin(), rightPoints.end(), [&](const point_2t& a, const point_2t& b)->bool {
					return bg::distance(a, exCurrentSeg) < bg::distance(b, exCurrentSeg);
					});
				if (!leftPoints.empty() && !rightPoints.empty())
				{
					int64 disLeft = bg::distance(leftPoints.back(), exCurrentSeg);
					int64 disRight = bg::distance(rightPoints.back(), exCurrentSeg);
					int64 dis;
					if (disLeft < disRight)
						dis = disLeft;
					else
						dis = disRight;
					if (dis < 500)
						isErase = false;
				}
			}

			if (isErase)
				ita = connectStopLineSegments.erase(ita);
			else
				ita++;
		}

		// 过滤: 两条线段重合（首尾点不一定重合）
		for (auto it = connectStopLineSegments.begin(); it != connectStopLineSegments.end();)
		{
			bool isErase = false;
			auto& currentSeg = *it;
			for (auto& tmp : connectStopLineSegments)
			{
				if (&(*it) == &tmp)
					continue;
				auto exSeg = SEGMENT_EX(currentSeg, 50000.0);
				if (bg::distance(tmp.first, exSeg) < 100 &&
					bg::distance(tmp.second, exSeg) < 100 &&
					bg::intersects(tmp.first, currentSeg) &&
					bg::intersects(tmp.second, currentSeg)
					)
				{
					isErase = true;
					break;
				}
			}
			if (isErase)
				it = connectStopLineSegments.erase(it);
			else
				it++;
		}

		// 重构connectStopLineSegments
		std::vector<segment_2t> refactorStopLineSegments;
		for (auto& tmp : connectStopLineSegments)
		{
			int64 tolDis = 1000;
			double tolExDis = 50000.0;
			linestring_2t tmpPoints;
			linestring_2t tmpOtherPoinst;
			segment_2t tmpSeg = S3_S2(tmp);
			segment_2t tmpExSeg = SEGMENT_2T_EX(tmpSeg, tolExDis);
			int frontCount = 0;
			int backCount = 0;
			for (auto& pas : tmpFrontPoints)
			{
				for (auto& pbs : pas)
				{
					if (bg::distance(pbs, tmpSeg) < 300)
						frontCount++;
				}
			}
			for (auto& pas : tmpBackPoints)
			{
				for (auto& pbs : pas)
				{
					if (bg::distance(pbs, tmpSeg) < 300)
						backCount++;
				}
			}

			std::vector<linestring_2t> currentPoints;
			currentPoints = frontCount > backCount ? tmpFrontPoints : tmpBackPoints;
			for (auto& pas : currentPoints)
			{
				int countA = 0;
				int countB = pas.size();
				linestring_2t ps;
				for (auto& pbs : pas)
				{
					auto tmpDisForSeg = bg::distance(pbs, tmpExSeg);
					if (tmpDisForSeg < tolDis)
					{
						ps.push_back(pbs);
						countA++;
					}
					if (tmpDisForSeg > 25000)
					{
						countB--;
					}
				}
				double tmpValue = double(countA) / double(countB);
				if (tmpValue > 0.5)
				{
					tmpPoints = ps;
					break;
				}
			}

			tmpPoints.push_back(tmpSeg.first);
			tmpPoints.push_back(tmpSeg.second);
			vector_2t pabVec = V2_N(S2_V2(tmpSeg));
			std::sort(tmpPoints.begin(), tmpPoints.end(), [&](point_2t& pt0, point_2t& pt1)->bool {
				auto v0Sum = bg::dot_product(pabVec, S2_V2(tmpSeg.first, pt0));
				auto v1Sum = bg::dot_product(pabVec, S2_V2(tmpSeg.first, pt1));
				return v0Sum < v1Sum;
				});
			if (tmpPoints.size() > 1)
			{
				refactorStopLineSegments.push_back(segment_2t(tmpPoints.front(), tmpPoints.back()));
			}
		}
		for (auto& tmp : refactorStopLineSegments)
		{
			linestring_2t tmpL;
			tmpL.push_back(tmp.first);
			tmpL.push_back(tmp.second);
			newIntersectionBoundaries.push_back(tmpL);
		}

		for (auto it = stopLineNearbySegments.begin(); it != stopLineNearbySegments.end();)
		{
			if (bg::length(*it) == 0)
				it = stopLineNearbySegments.erase(it);
			else
				it++;
		}

	}

	void IntersectionCompiler::updateStopLineData(
		HadLaneGroup* const pGroup)
	{
		std::vector<HadLaneGroup*> currentOtherGroups;
		std::vector<std::vector<MapPoint3D64>> nearbySegPoints;
		std::vector<std::vector<MapPoint3D64>> selfSegPoints;
		std::vector<HadLaneGroup*> currentGroups;

		//debug group lines
		std::vector<linestring_2t> debugGroupLines;
		if (!pGroup->laneBoundaries.empty())
		{
			for (size_t i = 0; i < pGroup->laneBoundaries.size(); i++)
			{
				LineString3d leftSide;
				HadLaneBoundary* obj = pGroup->laneBoundaries[i];
				leftSide = obj->location;
				if (directionEqual(obj, pGroup, 3))
					std::reverse(leftSide.vertexes.begin(), leftSide.vertexes.end());
				auto tmpLeftSide = leftSide.vertexes;
				coordinatesTransform.convert(tmpLeftSide.data(), tmpLeftSide.size());
				debugGroupLines.push_back(LINESTRING_2T(tmpLeftSide));
			}
			for (size_t i = 0; i < pGroup->roadBoundaries.size(); i++)
			{
				LineString3d leftSide;
				HadRoadBoundary* obj = pGroup->roadBoundaries[i];
				leftSide = obj->location;
				if (directionEqual(obj, pGroup, 3))
					std::reverse(leftSide.vertexes.begin(), leftSide.vertexes.end());
				auto tmpLeftSide = leftSide.vertexes;
				coordinatesTransform.convert(tmpLeftSide.data(), tmpLeftSide.size());
				debugGroupLines.push_back(LINESTRING_2T(tmpLeftSide));
			}
		}

		//next
		std::vector<MapPoint3D64> tmpGroupBackPoints;
		for (auto itb : pGroup->roadBoundaries)
		{
			directionEqual(itb, pGroup, 3) ?
				tmpGroupBackPoints.push_back(itb->location.vertexes.front()) :
				tmpGroupBackPoints.push_back(itb->location.vertexes.back());
		}
		bool isForkGroup = false;
		if (tmpGroupBackPoints.size() >= 2)
		{
			auto copyNearybySegPoints = tmpGroupBackPoints;
			coordinatesTransform.convert(copyNearybySegPoints.data(), copyNearybySegPoints.size());
			auto copyNearybySegPointsGeo = LINESTRING_2T(copyNearybySegPoints);
			if (bg::length(copyNearybySegPointsGeo) > 30000)
				isForkGroup = true;
		}
		if (tmpGroupBackPoints.size() < 2 || isForkGroup)
		{
			if (tmpGroupBackPoints.size() > 1)
			{
				for (auto itb : pGroup->laneBoundaries)
				{
					directionEqual(itb, pGroup, 3) ?
						tmpGroupBackPoints.insert(tmpGroupBackPoints.end() - 1, itb->location.vertexes.front()) :
						tmpGroupBackPoints.insert(tmpGroupBackPoints.end() - 1, itb->location.vertexes.back());
				}
			}
			else
			{
				tmpGroupBackPoints.clear();
				for (auto itb : pGroup->laneBoundaries)
				{
					directionEqual(itb, pGroup, 3) ?
						tmpGroupBackPoints.push_back(itb->location.vertexes.front()) :
						tmpGroupBackPoints.push_back(itb->location.vertexes.back());
				}
			}
			if (isForkGroup && !tmpGroupBackPoints.empty())
			{
				auto copyTmpGroupBackPoints = tmpGroupBackPoints;
				auto copyNearybySegPoints = tmpGroupBackPoints;
				coordinatesTransform.convert(copyNearybySegPoints.data(), copyNearybySegPoints.size());
				auto copyNearybySegPointsGeo = LINESTRING_2T(copyNearybySegPoints);
				std::vector<size_t> firstIds;
				std::vector<size_t> secondIds;
				std::vector<size_t> thirdIds;
				linestring_2t firstLine;
				linestring_2t thirdLine;
				if (copyNearybySegPointsGeo.size() > 1)
				{
					auto pa = copyNearybySegPointsGeo.front();
					auto pb = copyNearybySegPointsGeo.back();
					firstLine.push_back(pa);
					thirdLine.push_back(pb);
					for (size_t i = 1; i < copyNearybySegPointsGeo.size() - 1; ++i)
					{
						auto p = copyNearybySegPointsGeo[i];
						if (bg::distance(p, firstLine) < 15000)
						{
							firstIds.push_back(i);
							firstLine.push_back(p);
						}
						else if (bg::distance(p, thirdLine) < 15000)
						{
							thirdIds.push_back(i);
							thirdLine.push_back(p);
						}
						else
						{
							secondIds.push_back(i);
						}
					}
				}

				tmpGroupBackPoints.clear();
				if (!secondIds.empty())
				{
					for (auto id : secondIds)
						tmpGroupBackPoints.push_back(copyTmpGroupBackPoints[id]);
				}
				else
				{
					for (auto id : thirdIds)
						tmpGroupBackPoints.push_back(copyTmpGroupBackPoints[id]);
					tmpGroupBackPoints.push_back(copyTmpGroupBackPoints.back());
				}
			}
		}

		//previous
		std::vector<MapPoint3D64> tmpGroupFrontPoints;
		for (auto itb : pGroup->roadBoundaries)
		{
			directionEqual(itb, pGroup, 3) ?
				tmpGroupFrontPoints.push_back(itb->location.vertexes.back()) :
				tmpGroupFrontPoints.push_back(itb->location.vertexes.front());
		}
		if (tmpGroupFrontPoints.size() < 2)
		{
			tmpGroupFrontPoints.clear();
			for (auto itb : pGroup->laneBoundaries)
			{
				directionEqual(itb, pGroup, 3) ?
					tmpGroupFrontPoints.push_back(itb->location.vertexes.back()) :
					tmpGroupFrontPoints.push_back(itb->location.vertexes.front());
			}
		}

		//next
		for (auto ita : pGroup->next)
		{
			HadLaneGroup* pNextGroup = (HadLaneGroup*)ita;
			if (pNextGroup->inIntersection == m_hadInts->originId)
				continue;

			currentOtherGroups.push_back(pNextGroup);
			currentGroups.push_back(pGroup);
			std::vector<MapPoint3D64> tmpNearbySegPoints;
			for (auto itb : pNextGroup->roadBoundaries)
			{
				directionEqual(itb, pNextGroup, 3) ?
					tmpNearbySegPoints.push_back(itb->location.vertexes.back()) :
					tmpNearbySegPoints.push_back(itb->location.vertexes.front());
			}
			if (tmpNearbySegPoints.empty())
			{
				for (auto itb : pNextGroup->laneBoundaries)
				{
					directionEqual(itb, pNextGroup, 3) ?
						tmpNearbySegPoints.push_back(itb->location.vertexes.back()) :
						tmpNearbySegPoints.push_back(itb->location.vertexes.front());
				}
			}
			std::vector<MapPoint3D64> tmpPts;

			//两个路口相邻，解决面缺失问题
			if (pGroup->inIntersection != 0 && pNextGroup->inIntersection != 0)
				tmpPts.insert(tmpPts.end(), tmpGroupBackPoints.begin(), tmpGroupBackPoints.end());

			tmpPts.insert(tmpPts.end(), tmpNearbySegPoints.begin(), tmpNearbySegPoints.end());
			backPoints.push_back(tmpPts);
			outIntersectionPoints.insert(outIntersectionPoints.end(), tmpNearbySegPoints.begin(), tmpNearbySegPoints.end());
			nearbySegPoints.push_back(tmpNearbySegPoints);
			selfSegPoints.push_back(tmpGroupBackPoints);

		}

		//previous
		for (auto ita : pGroup->previous)
		{
			HadLaneGroup* pPrevGroup = (HadLaneGroup*)ita;
			if (pPrevGroup->inIntersection == m_hadInts->originId)
				continue;
			currentOtherGroups.push_back(pPrevGroup);
			currentGroups.push_back(pGroup);
			std::vector<MapPoint3D64> tmpNearbySegPoints;
			for (auto itb : pPrevGroup->roadBoundaries)
			{
				directionEqual(itb, pPrevGroup, 3) ?
					tmpNearbySegPoints.push_back(itb->location.vertexes.front()) :
					tmpNearbySegPoints.push_back(itb->location.vertexes.back());
			}
			if (tmpNearbySegPoints.empty())
			{
				for (auto itb : pPrevGroup->laneBoundaries)
				{
					directionEqual(itb, pPrevGroup, 3) ?
						tmpNearbySegPoints.push_back(itb->location.vertexes.front()) :
						tmpNearbySegPoints.push_back(itb->location.vertexes.back());
				}
			}
			std::vector<MapPoint3D64> tmpPts;

			//两个路口相邻，解决面缺失问题
			if (pGroup->inIntersection != 0 && pPrevGroup->inIntersection != 0)
				tmpPts.insert(tmpPts.end(), tmpGroupFrontPoints.begin(), tmpGroupFrontPoints.end());

			tmpPts.insert(tmpPts.end(), tmpNearbySegPoints.begin(), tmpNearbySegPoints.end());
			frontPoints.push_back(tmpPts);
			outIntersectionPoints.insert(outIntersectionPoints.end(), tmpNearbySegPoints.begin(), tmpNearbySegPoints.end());
			nearbySegPoints.push_back(tmpNearbySegPoints);
			selfSegPoints.push_back(tmpGroupFrontPoints);
		}

		//next
		if (pGroup->next.empty())
		{
			currentOtherGroups.push_back(pGroup);
			currentGroups.push_back(pGroup);
			backPoints.push_back(tmpGroupBackPoints);
			nearbySegPoints.push_back(tmpGroupBackPoints);
			selfSegPoints.push_back(tmpGroupBackPoints);
		}

		//previous
		if (pGroup->previous.empty())
		{
			currentOtherGroups.push_back(pGroup);
			currentGroups.push_back(pGroup);
			frontPoints.push_back(tmpGroupFrontPoints);
			nearbySegPoints.push_back(tmpGroupFrontPoints);
			selfSegPoints.push_back(tmpGroupFrontPoints);
		}

		for (size_t i = 0; i < currentOtherGroups.size(); ++i)
		{
			linestring_2t debugStopLine, debugNearbyStopLine;
			HadLaneGroup* pOtherGroup = currentOtherGroups[i];
			HadLaneGroup* pGroup = currentGroups[i];
			std::vector<MapPoint3D64> tmpNearbySegPoints = nearbySegPoints[i];

			// nearby stop line
			segment_t nearbySeg;
			std::vector<MapPoint3D64> tmpNearbySegModifyPoints;
			if (tmpNearbySegPoints.size() > 1)
			{
				auto copyNearybySegPoints = tmpNearbySegPoints;
				coordinatesTransform.convert(copyNearybySegPoints.data(), copyNearybySegPoints.size());
				debugStopLine = LINESTRING_2T(copyNearybySegPoints);
				for (size_t j = 0; j < debugStopLine.size(); ++j)
				{
					for (auto p : intersectionLaneLinePoints)
					{
						bool isNearyByLine = false;
						if (debugStopLine.size() == 2)
							isNearyByLine = bg::distance(debugStopLine, p) < 1000;
						if (bg::distance(debugStopLine.at(j), p) < 5000 || isNearyByLine)
						{
							tmpNearbySegModifyPoints.push_back(tmpNearbySegPoints[j]);
						}
					}

				}
				tmpNearbySegPoints.assign(tmpNearbySegModifyPoints.begin(), tmpNearbySegModifyPoints.end());
				copyNearybySegPoints = tmpNearbySegPoints;
				coordinatesTransform.convert(copyNearybySegPoints.data(), copyNearybySegPoints.size());
				linestring_t copyNearbyBySegPointsGeo = LINESTRING_T(copyNearybySegPoints);
				bg::unique(copyNearbyBySegPointsGeo);
				if (copyNearbyBySegPointsGeo.size() < 2)
					tmpNearbySegModifyPoints.clear();
				nearbySeg = segment_t(POINT_T(copyNearybySegPoints.front()), POINT_T(copyNearybySegPoints.back()));
			}
			else
			{
				continue;
			}

			//加入没找到，使用自身边界
			if (tmpNearbySegModifyPoints.size() < 2)
			{
				tmpNearbySegModifyPoints.clear();
				tmpNearbySegPoints = selfSegPoints[i];
				if (tmpNearbySegPoints.size() > 1)
				{
					auto copyNearybySegPoints = tmpNearbySegPoints;
					coordinatesTransform.convert(copyNearybySegPoints.data(), copyNearybySegPoints.size());
					debugStopLine = LINESTRING_2T(copyNearybySegPoints);
					for (size_t j = 0; j < debugStopLine.size(); ++j)
					{
						for (auto p : intersectionLaneLinePoints)
						{
							bool isNearyByLine = false;
							if (debugStopLine.size() == 2)
								isNearyByLine = bg::distance(debugStopLine, p) < 1000;
							if (bg::distance(debugStopLine.at(j), p) < 5000 || isNearyByLine)
							{
								tmpNearbySegModifyPoints.push_back(tmpNearbySegPoints[j]);
							}
						}

					}
					tmpNearbySegPoints.assign(tmpNearbySegModifyPoints.begin(), tmpNearbySegModifyPoints.end());
					copyNearybySegPoints = tmpNearbySegPoints;
					coordinatesTransform.convert(copyNearybySegPoints.data(), copyNearybySegPoints.size());
					nearbySeg = segment_t(POINT_T(copyNearybySegPoints.front()), POINT_T(copyNearybySegPoints.back()));
				}
				else
				{
					continue;
				}
			}

			// stop line
			std::vector<MapPoint3D64> tmpSegPoints;
			for (auto itb : pGroup->roadBoundaries)
			{
				auto pa = itb->location.vertexes.front();
				auto pb = itb->location.vertexes.back();
				coordinatesTransform.convert(&pa, 1);
				coordinatesTransform.convert(&pb, 1);
				segment_t paSeg;
				segment_t pbSeg;
				bg::closest_points(POINT_T(pa), nearbySeg, paSeg);
				bg::closest_points(POINT_T(pb), nearbySeg, pbSeg);
				double paSegLen = bg::length(paSeg);
				double pbSegLen = bg::length(pbSeg);
				if (paSegLen < 100.0)
					tmpSegPoints.push_back(itb->location.vertexes.front());
				if (pbSegLen < 100.0)
					tmpSegPoints.push_back(itb->location.vertexes.back());
			}
			if (tmpSegPoints.empty())
			{
				for (auto itb : pGroup->laneBoundaries)
				{
					auto pa = itb->location.vertexes.front();
					auto pb = itb->location.vertexes.back();
					coordinatesTransform.convert(&pa, 1);
					coordinatesTransform.convert(&pb, 1);
					segment_t paSeg;
					segment_t pbSeg;
					bg::closest_points(POINT_T(pa), nearbySeg, paSeg);
					bg::closest_points(POINT_T(pb), nearbySeg, pbSeg);
					double paSegLen = bg::length(paSeg);
					double pbSegLen = bg::length(pbSeg);
					if (paSegLen < 100.0)
						tmpSegPoints.push_back(itb->location.vertexes.front());
					if (pbSegLen < 100.0)
						tmpSegPoints.push_back(itb->location.vertexes.back());
				}
			}

			// push data
			if (tmpNearbySegPoints.size() > 1)
			{
				segment_t pNearbySeg = nearbySeg;
				stopLineNearbySegments.push_back(pNearbySeg);
				originStopLineNearbySegments.push_back(tmpNearbySegPoints);
				auto copyTmpNewNearbyBySegPoints = tmpNearbySegPoints;
				coordinatesTransform.convert(copyTmpNewNearbyBySegPoints.data(), copyTmpNewNearbyBySegPoints.size());
				debugNearbyStopLine = LINESTRING_2T(copyTmpNewNearbyBySegPoints);
				connectStopLineSegments.push_back(pNearbySeg);
			}
		}
	}

	void IntersectionCompiler::clearInfoData()
	{
		originPoints.clear();
		allNodes.clear();
		refLinkNodes.clear();
		originStopLineNearbySegments.clear();
		stopLineNearbySegments.clear();
		originStopLineSegments.clear();
		stopLineSegments.clear();
		roadBoundaryPts.clear();
		intersectionLaneLines.clear();
		intersectionOnlyLaneLines.clear();
		intersectionLaneLinePoints.clear();
		newIntersectionBoundaries.clear();
		connectStopLineSegments.clear();
		frontPoints.clear();
		backPoints.clear();
		outIntersectionPoints.clear();
	}

	inline void IntersectionCompiler::getNearestLaneInfo(std::vector<LaneInfo>& laneInfos, LaneInfo& laneInfo)
	{
		for (auto type = LaneInfoType::ROAD_BOUNDARY; type <= LaneInfoType::LANE_BOUNDARY; type = (LaneInfoType)(int(type) + 1))
		{
			std::vector<LaneInfo> infos;
			std::for_each(laneInfos.begin(), laneInfos.end(), [&](LaneInfo& info) {
				if (type == info._type)
					infos.push_back(info);
			});
			if (!infos.empty())
			{
				std::sort(infos.begin(), infos.end(), [](LaneInfo& l, LaneInfo& r) {
					return l._length < r._length;
				});
				laneInfo = infos.front();
				return;
			}
		}
		laneInfo = laneInfos.front();
	}

	bool IntersectionCompiler::getExpandPolyByOffset(
		const ring_2t& ring,
		const int64& offsetSize,
		ring_2t& resultRing)
	{
		multi_polygon_2t bufferResults;
		bg::strategy::buffer::distance_symmetric<int64> distance_strategy(offsetSize);
		bg::buffer(ring, bufferResults, distance_strategy, side_strategy, join_strategy, end_strategy, circle_strategy);
		if (bufferResults.empty())
			return false;
		resultRing = bufferResults[0].outer();
		return true;
	}

}


