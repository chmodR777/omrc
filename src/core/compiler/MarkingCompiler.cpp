#include "stdafx.h"
#include "MarkingCompiler.h"
#include "algorithm/grap_point_algorithm.h"
#include "../framework/SpatialSeacher.h"
#include "math3d/vector_math.h"
#include "CompileSetting.h"

#ifdef PRINT_ARROW_LOCATION
#include <fstream>
#include <iostream>
#include <sstream>
#endif



namespace OMDB
{
	void MarkingCompiler::compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile)
    {
		//路口屏蔽
		std::set<HadObject*> inInTurnwaitingLaneObjects;
        for (auto obj : pGrid->query(ElementType::HAD_LANE))
        {
            HadLane* pLane = (HadLane*)obj;
            if ((pLane->leftBoundary && !pLane->leftBoundary->turnwaitingPAs.empty())
                || (pLane->rightBoundary && !pLane->rightBoundary->turnwaitingPAs.empty()))
            {
                for (HadObject* obj : pLane->objects)
                {
                    if (obj->objectType == ElementType::HAD_OBJECT_ARROW)
                    {
						inInTurnwaitingLaneObjects.insert(obj);

                    }
                }

            }

        }

		std::vector<HadGrid*> nearbyGrids = { pGrid };
		for_each(nearby.begin(), nearby.end(), [&](HadGrid* g)->void {nearbyGrids.push_back(g); });

		std::vector<std::vector<MapPoint3D64>> allLines;
		for (auto& rdsLineInfo : compilerData.m_rdsLines) 
		{
			allLines.push_back(rdsLineInfo._originPoints);
		}
		//跨网格
		for (auto& tmpArrowObj : pGrid->query(ElementType::HAD_OBJECT_ARROW))
		{
			if (inInTurnwaitingLaneObjects.count((HadObject*)tmpArrowObj))
				continue;

			HadArrow* hadArrow = (HadArrow*)tmpArrowObj;
			processCrossGrid(pGrid, hadArrow, allLines);
		}

		if (allLines.empty())
			return;
		for (auto& tmpItem : allLines)
		{
			coordinatesTransform.convert(tmpItem.data(), tmpItem.size());
		}

		std::vector<segment_t> m_segments;
		std::vector<std::vector<size_t>> m_sizes;
		for (size_t i = 0; i < allLines.size(); ++i)
		{
			for (size_t j = 0; j < allLines[i].size() - 1; ++j)
			{
				std::vector<size_t> tmpSize;
				segment_t tmpSeg(POINT_T(allLines[i][j]), POINT_T(allLines[i][j + 1]));
				m_segments.push_back(tmpSeg);
				m_sizes.push_back(std::vector<size_t>{i, j});
			}
		}
		parameters param;
		index_getter_segment originInd(m_segments);
		rtree_type_segment	rtree(boost::irange<std::size_t>(0lu, m_segments.size()), param, originInd);
		for (auto& tmpArrowObj : pGrid->query(ElementType::HAD_OBJECT_ARROW))
		{
			HadArrow* hadArrow = (HadArrow*)tmpArrowObj;

			//判断是否为普通路
			if (CompileSetting::instance()->isNotCompileUrbanData)
			{
				if (!isProDataLevel(hadArrow->laneGroups))
					continue;
			}

			if (!CompileSetting::instance()->isCompileTurnWaiting)
			{
				if (!isCreateRdsForIntersection(pGrid, hadArrow->laneGroups))
					continue;
			}

			if (hadArrow->originId == 182733551689638797)
				printInfo("");

			auto polyPoints = hadArrow->polygon.vertexes;
			if (polyPoints.size() < 4)
				continue;
			coordinatesTransform.convert(polyPoints.data(), polyPoints.size());
			point_t p_1 = getArrowCenterPoint(LINESTRING_T(polyPoints));

			bool isIntersection = isIntersectionArrow(polyPoints);

			int withinRefLanes = getObjectPos(p_1,hadArrow);

			point_t pos;
			Vector3 dir;
			double laneWidth;
			getObjectWidthAndDir(allLines, rtree, p_1, withinRefLanes, isIntersection, polyPoints, m_segments, m_sizes, pos, dir, laneWidth);
			std::vector<MarkingObj> markingObjs{ MarkingObj{0.0, (RDS::RdsMarking::MarkingType)hadArrow->arrowClass} };
			for (size_t i = 0; i < markingObjs.size(); i++)
			{
				float w, h;
				MarkingObj obj = markingObjs[i];
				getMarkingRate(obj.type, w, h);
				std::vector<MapPoint3D64> outPts;
				Vector3 upDir = vec3(0.0f, 0.0f, 1.0f);
				MapPoint3D64 tmpPos = MapPoint3D64_make(pos.get<0>(), pos.get<1>(), pos.get<2>());
				generateContourByPosAndDir(tmpPos, dir, upDir, w * laneWidth, h * laneWidth, outPts);
				coordinatesTransform.invert(outPts.data(), outPts.size());
				hadArrow->polygon.vertexes = outPts;
				coordinatesTransform.invert(&tmpPos, 1);
				tmpPos.z /= 10;
				hadArrow->postion = tmpPos;
				if (withinRefLanes <= 1)
				{
					if (!outOfRoadFace(hadArrow, nearbyGrids, outPts))
					{
						addRdsMarking(pTile, hadArrow, (RDS::RdsMarking::MarkingType)hadArrow->arrowClass);
					}
				}
				else {
					addRdsMarking(pTile, hadArrow, (RDS::RdsMarking::MarkingType)hadArrow->arrowClass);
				}
			}
		}
	}
	void MarkingCompiler::processCrossGrid(HadGrid* const pGrid, HadArrow* hadArrow, std::vector<std::vector<MapPoint3D64>> & allLines)
	{
		if (hadArrow->originId == 84206638776606923
			|| hadArrow->originId == 84206629070987369
			|| hadArrow->originId == 84206584904966144
			|| hadArrow->originId == 84206533176615042)
		{
			int count = 0;
		}
		if (hadArrow == nullptr)
		{
			return;
		}

		bool isCrossGird = false;
		for each (HadLane * hadlane  in hadArrow->refLanes)
		{
			if (hadlane->linkGroup == nullptr)
				continue;

			if (pGrid != hadlane->owner)
			{
				OMDB::LineString3d left_HadLine;
				LineString3d leftSide = hadlane->leftBoundary->location;
				if (directionEqual(hadlane->leftBoundary, hadlane->linkGroup, 3))
				{
					std::reverse(leftSide.vertexes.begin(), leftSide.vertexes.end());
				}
				LineString3d rightSide = hadlane->rightBoundary->location;
				if (directionEqual(hadlane->rightBoundary, hadlane->linkGroup, 3))
				{
					std::reverse(rightSide.vertexes.begin(), rightSide.vertexes.end());
				}
				left_HadLine.vertexes.insert(left_HadLine.vertexes.end(), leftSide.vertexes.begin(), leftSide.vertexes.end());
				if (left_HadLine.vertexes.size() < 2)
					continue;
				allLines.push_back(left_HadLine.vertexes);

				OMDB::LineString3d right_HadLine;
				right_HadLine.vertexes.insert(right_HadLine.vertexes.end(), rightSide.vertexes.begin(), rightSide.vertexes.end());
				if (right_HadLine.vertexes.size() < 2)
					continue;
				allLines.push_back(right_HadLine.vertexes);
				//printInfo("cross arrow  (%lld,%lld)", hadArrow->postion.pos.lon / 1000, hadArrow->postion.pos.lat / 1000);
			}

		}
	}

	bool clipperLibPath_compare(ClipperLib::Path firstPath, ClipperLib::Path secondPath)
	{
		ClipperLib::Clipper clipper;
		ClipperLib::PolyTree polyTree;
		clipper.AddPath(firstPath, ClipperLib::ptSubject, true);
		clipper.AddPath(secondPath, ClipperLib::ptClip, true);
		clipper.Execute(ClipperLib::ctIntersection, polyTree, ClipperLib::pftEvenOdd);
		if (polyTree.Total())
		{
			double polyArea = 0;
			for (auto child : polyTree.Childs) {
				polyArea += std::abs(ClipperLib::Area(child->Contour));
			}

			double firstPathArea = std::abs(ClipperLib::Area(firstPath));
			double secondPathArea = std::abs(ClipperLib::Area(secondPath));
			double firstAreaRate = polyArea / firstPathArea;
			double secondAreaRate = polyArea / secondPathArea;
			if (firstAreaRate >= 0.8 && secondAreaRate >= 0.8) {
				return 1;
			}
		}
		return 0;
	}

	bool MarkingCompiler::outOfRoadFace(const HadArrow* hadArrow, const std::vector<HadGrid*>& nearby, std::vector<MapPoint3D64> outPts)
	{
		// 目前只处理道路边界包含的情况
		bool containsBoundaries = false;
		for (HadLaneGroup* pLinkGroup : hadArrow->laneGroups) {
			// 忽略路口的箭头
			if (pLinkGroup->inIntersection) {
				return false;
			}

			if (pLinkGroup->roadBoundaries.size() == 2 || pLinkGroup->laneBoundaries.size() >= 2) {
				containsBoundaries = true;
				break;
			}
		}
		if (!containsBoundaries)
			return false;

		// 箭头多边形
		MapPoint3D64& startPt = outPts.front();
		MapPoint3D64& endPt = outPts.back();
		if (!floatEqual(startPt.pos.distanceSquare(endPt.pos), 0)) {
			outPts.push_back(startPt);
		}

		// 解决数据所属车道组错误的问题
		BoundingBox2d bbox = makeBoundingBox2d(outPts);
		std::vector<HadLaneGroup*> laneGroups = hadArrow->laneGroups;
		std::vector<HadLaneGroup*> pNearbyGroups = SpatialSeacher::seachNearby(nearby, bbox);
		for (auto pNearbyGroup : pNearbyGroups) {
			if (std::find(laneGroups.begin(), laneGroups.end(), pNearbyGroup) == laneGroups.end()) {
				laneGroups.push_back(pNearbyGroup);
			}
		}

		ClipperLib::Path outPtsPath;
		for (auto& vertex : outPts) {
			outPtsPath << ClipperLib::IntPoint(vertex.pos.lon, vertex.pos.lat, vertex.z);
		}

		auto intPointDistanceSquare = [](ClipperLib::IntPoint pt0, ClipperLib::IntPoint pt1) {
			int64 dx = pt0.X - pt1.X;
			int64 dy = pt0.Y - pt1.Y;

			return dx * dx + dy * dy;
		};

		bool flag = true;
		int connectNum = 2;
		std::vector<ClipperLib::Path> allPolyPaths;
		double outPtsArea = std::abs(ClipperLib::Area(outPtsPath));
		for (HadLaneGroup* pLinkGroup : laneGroups) {
			std::vector<std::vector<HadLaneGroup*>> allConnectedLaneGroups;
			getConnectedGroups(pLinkGroup, allConnectedLaneGroups, connectNum);
			for (auto& connectedLaneGroups : allConnectedLaneGroups) {
				std::vector<MapPoint3D64> firstBoundaryVertexes;
				std::vector<MapPoint3D64> secondBoundaryVertexes;
				createConnectedBoundary(pLinkGroup, connectedLaneGroups, firstBoundaryVertexes, secondBoundaryVertexes);

				Polygon3d roadFacePolygon;
				if (!firstBoundaryVertexes.empty() && !secondBoundaryVertexes.empty())
					makeLaneGroupPolygon(firstBoundaryVertexes, secondBoundaryVertexes, roadFacePolygon);

				// 路面
				ClipperLib::Path roadFacePath;
				for (auto& vertex : roadFacePolygon.vertexes) {
					roadFacePath << ClipperLib::IntPoint(vertex.pos.lon, vertex.pos.lat, vertex.z);
				}

				if (!roadFacePath.empty()) {
					ClipperLib::Clipper clipper;
					ClipperLib::PolyTree polyTree;
					clipper.AddPath(outPtsPath, ClipperLib::ptSubject, true);
					clipper.AddPath(roadFacePath, ClipperLib::ptClip, true);
					clipper.Execute(ClipperLib::ctIntersection, polyTree, ClipperLib::pftEvenOdd);
					if (polyTree.Total())
					{
						double polyArea = 0;
						for (auto child : polyTree.Childs) {
							polyArea += std::abs(ClipperLib::Area(child->Contour));
							auto& startPt = child->Contour.front();
							auto& endPt = child->Contour.back();
							if (!floatEqual(intPointDistanceSquare(startPt, endPt), 0)) {
								child->Contour.push_back(startPt);
							}
							allPolyPaths.push_back(child->Contour);
						}
						double polyAreaRate = polyArea / outPtsArea;
						if (polyAreaRate >= 0.8) {
							flag = false;
							return flag;
						}
					}
				}
			}
		}

		auto totalPolyArea = 0.0;
		auto ip = std::unique(allPolyPaths.begin(), allPolyPaths.end(), clipperLibPath_compare);
		allPolyPaths.resize(std::distance(allPolyPaths.begin(), ip));
		for (auto& polyPath : allPolyPaths) {
			totalPolyArea += std::abs(ClipperLib::Area(polyPath));
		}
		double polyAreaRate = totalPolyArea / outPtsArea;
		if (polyAreaRate >= 0.8) {
			flag = false;
		}

		return flag;
	}

	void MarkingCompiler::createConnectedBoundary(HadLaneGroup* pCurrentGroup, std::vector<HadLaneGroup*> connectedLaneGroups,
		std::vector<MapPoint3D64>& firstBoundaryVertexes, std::vector<MapPoint3D64>& secondBoundaryVertexes)
	{
		int connectDistance = 20000; // ≈20m
		auto extendBoundaryVertexes = [connectDistance](
			std::vector<MapPoint3D64>& previousVertexes, 
			std::vector<MapPoint3D64>& boundaryVertexes, 
			std::vector<MapPoint3D64>& nextVertexes) {
				// 向前延长20米
				auto prevDistance = 0.0;
				auto prevPt = boundaryVertexes.front();
				for (auto iter = previousVertexes.rbegin(); iter != previousVertexes.rend(); iter++) {
					double distance = prevPt.pos.distance(iter->pos);
					if (floatEqual(distance, 0)) {
						continue;
					}

					prevDistance += distance;
					boundaryVertexes.insert(boundaryVertexes.begin(), *iter);
					prevPt = boundaryVertexes.front();
					if (prevDistance > connectDistance) {
						break;
					}
				};

				// 向后延长20米
				auto nextDistance = 0.0;
				auto nextPt = boundaryVertexes.back();
				for (auto iter = nextVertexes.begin(); iter != nextVertexes.end(); iter++) {
					double distance = nextPt.pos.distance(iter->pos);
					if (floatEqual(distance, 0)) {
						continue;
					}

					nextDistance += distance;
					boundaryVertexes.push_back(*iter);
					nextPt = boundaryVertexes.back();
					if (nextDistance > connectDistance) {
						break;
					}
				};
		};

		auto begin = connectedLaneGroups.begin();
		auto end = connectedLaneGroups.end();
		auto groupIdx = std::find(begin, end, pCurrentGroup) - begin;

		std::vector<MapPoint3D64> firstPreviousVertexes;
		std::vector<MapPoint3D64> firstNextVertexes;

		std::vector<MapPoint3D64> secondPreviousVertexes;
		std::vector<MapPoint3D64> secondNextVertexes;
		for (int idx = 0; idx < connectedLaneGroups.size(); idx++) {
			LineString3d left, right;
			auto pGroup = connectedLaneGroups[idx];
			getLaneGroupBoundary(pGroup, left, right);
			if (idx < groupIdx) {
				firstPreviousVertexes.insert(firstPreviousVertexes.end(), left.vertexes.begin(), left.vertexes.end());
				secondPreviousVertexes.insert(secondPreviousVertexes.end(), right.vertexes.begin(), right.vertexes.end());
			} else if (idx > groupIdx) {
				firstNextVertexes.insert(firstNextVertexes.end(), left.vertexes.begin(), left.vertexes.end());
				secondNextVertexes.insert(secondNextVertexes.end(), right.vertexes.begin(), right.vertexes.end());
			} else {
				firstBoundaryVertexes = left.vertexes;
				secondBoundaryVertexes = right.vertexes;
			}
		}

		extendBoundaryVertexes(firstPreviousVertexes, firstBoundaryVertexes, firstNextVertexes);
		extendBoundaryVertexes(secondPreviousVertexes, secondBoundaryVertexes, secondNextVertexes);
	}

	int MarkingCompiler::getObjectPos(point_t& pos, const HadArrow* hadArrow) {
		int withinRefLanes = 0;
		if (hadArrow == nullptr)
		{
			return withinRefLanes;
		}
		if (hadArrow->refLanes.size() <=0)
		{
			return withinRefLanes;
		}
		
		double minDistance = DBL_MAX;
		MapPoint3D64 tmpPos = MapPoint3D64_make(pos.get<0>(), pos.get<1>(), pos.get<2>());
		coordinatesTransform.invert(&tmpPos, 1);
		tmpPos.z /= 10;
		point_t tmpPosPoint = POINT_T(tmpPos);
		point_t nearyPosition = {};
		for (HadLane* hadlane : hadArrow->refLanes)
		{
			if (hadlane->linkGroup == nullptr)
				continue;
			
			std::vector<MapPoint3D64> points;
			LineString3d leftSide = hadlane->leftBoundary->location;
			if (directionEqual(hadlane->leftBoundary, hadlane->linkGroup, 3))
			{
				std::reverse(leftSide.vertexes.begin(), leftSide.vertexes.end());
			}
			LineString3d rightSide = hadlane->rightBoundary->location;
			if (directionEqual(hadlane->rightBoundary, hadlane->linkGroup, 3))
			{
				std::reverse(rightSide.vertexes.begin(), rightSide.vertexes.end());
			}
			points.insert(points.end(), leftSide.vertexes.begin(), leftSide.vertexes.end());
			points.insert(points.end(), rightSide.vertexes.rbegin(), rightSide.vertexes.rend());
			points.push_back(leftSide.vertexes.at(0));
			polygon_t tmpLine = POLYGON_T(points);
			if (bg::within(tmpPosPoint, tmpLine))
			{
				OMDB::LineString3d left_HadLine, right_HadLine;
				left_HadLine.vertexes.insert(left_HadLine.vertexes.end(), leftSide.vertexes.begin(), leftSide.vertexes.end());
				if (left_HadLine.vertexes.size() < 2)
					continue;

				right_HadLine.vertexes.insert(right_HadLine.vertexes.end(), rightSide.vertexes.begin(), rightSide.vertexes.end());
				if (right_HadLine.vertexes.size() < 2)
					continue;

				withinRefLanes++;
				size_t si = 0, ei = 0;
				MapPoint3D64 grappedPt{ 0 };
				MapPoint3D64 grappedPt1{ 0 };
				coordinatesTransform.convert(left_HadLine.vertexes.data(), left_HadLine.vertexes.size());
				coordinatesTransform.convert(right_HadLine.vertexes.data(), right_HadLine.vertexes.size());
				MapPoint3D64 centerPoint = MapPoint3D64_make(pos.get<0>(), pos.get<1>(), pos.get<2>());
				bool bGrap = GrapPointAlgorithm::grapOrMatchNearestPoint(centerPoint, left_HadLine.vertexes, grappedPt, si, ei, 10.0);
				bool bGrap1 = GrapPointAlgorithm::grapOrMatchNearestPoint(centerPoint, right_HadLine.vertexes, grappedPt1, si, ei, 10.0);

				point_t sp_m;
				point_t sp_1 = POINT_T(grappedPt);
				point_t sp_2 = POINT_T(grappedPt1);
				bg::divide_value(sp_1, 2.0);
				bg::divide_value(sp_2, 2.0);
				bg::convert(sp_1, sp_m);
				bg::add_point(sp_m, sp_2);
				double distance = bg::distance(pos, sp_m);
				if (distance < minDistance) {
					nearyPosition = sp_m;//取左右两边中心点
					minDistance = distance;
				}
			}
			
		}
		if (withinRefLanes == 1)
			pos = nearyPosition;
		
		return withinRefLanes;
	}

	void  MarkingCompiler::getObjectWidthAndDir(
		const std::vector<std::vector<MapPoint3D64>>& allLines,
		const rtree_type_segment& rtree,
		const point_t& p_1,
		const int withinRefLanes,
		const bool isIntersection,
		const std::vector<MapPoint3D64>& polyPoints,
		const std::vector<segment_t>& segments,
		const std::vector<std::vector<size_t>>& sizes,
		point_t& pos,
		Vector3& dir,
		double& laneWidth)
	{
		MarkingLaneData markingLaneData;
		MarkingLane firstMarkingLane;
		bool isInit = true;
		rtree.query(bgi::nearest(p_1, 6),
			boost::make_function_output_iterator([&](size_t const& id) {
				if (isInit)
				{
					firstMarkingLane = buildMarkingLane(id, p_1, allLines, segments, sizes);
					if (std::abs(p_1.get<2>() - firstMarkingLane.verSeg.second.get<2>()) > 6000)
						return;
					if (firstMarkingLane.segLength < 100) 
						return;
					markingLaneData.firstDatas.push_back(firstMarkingLane);
					isInit = false;
					return;
				}
				if (!isInit)
				{
					auto tmpMarkingLane = buildMarkingLane(id, p_1, allLines, segments, sizes);
					if (std::abs(p_1.get<2>() - tmpMarkingLane.verSeg.second.get<2>()) > 6000)
						return;
					if (tmpMarkingLane.segLength < 100)
						return;
					if (bg::dot_product(firstMarkingLane.verSegUnitDir, tmpMarkingLane.verSegUnitDir) > 0)
						markingLaneData.firstDatas.push_back(tmpMarkingLane);
					else
						markingLaneData.secondDatas.push_back(tmpMarkingLane);
				}
				}));

		std::sort(markingLaneData.firstDatas.begin(), markingLaneData.firstDatas.end(), [&](const MarkingLane& a, const MarkingLane& b)->bool {
			if (a.verSegLength == b.verSegLength) 
				return b.segLength < a.segLength ? true : false;
			return a.verSegLength < b.verSegLength ? true : false;
			});

		std::sort(markingLaneData.secondDatas.begin(), markingLaneData.secondDatas.end(), [&](const MarkingLane& a, const MarkingLane& b)->bool {
			if (a.verSegLength == b.verSegLength)
				return b.segLength < a.segLength ? true : false;
			return a.verSegLength < b.verSegLength ? true : false;
			});

		// 箭头只在一个车道里面时
		if (withinRefLanes ==1 && !markingLaneData.isSingleBoundary() && !isIntersection)
		{
			point_t sp_1 = markingLaneData.firstDatas[0].verSeg.second;
			point_t sp_2 = markingLaneData.secondDatas[0].verSeg.second;
			point_t sp_m;
			laneWidth = bg::distance(sp_1, sp_2);
			if (laneWidth > 4000)
				laneWidth = 4000;
			bg::divide_value(sp_1, 2.0);
			bg::divide_value(sp_2, 2.0);
			bg::convert(sp_1, sp_m);
			bg::add_point(sp_m, sp_2);
			pos = sp_m;


			point_t temp = { polyPoints.at(2).pos.lon - polyPoints.at(1).pos.lon ,polyPoints.at(2).pos.lat - polyPoints.at(1).pos.lat, polyPoints.at(2).z - polyPoints.at(1).z };
			if (bg::dot_product(markingLaneData.firstDatas[0].segDir, markingLaneData.secondDatas[0].segDir) > 0)
			{
				// 优先选择与原矩形框角度相近的方向
				double firstSegDirDegree = minimalDegree(temp, markingLaneData.firstDatas[0].segDir);
				double secondSegDirDegree = minimalDegree(temp, markingLaneData.secondDatas[0].segDir);
				if (firstSegDirDegree <= secondSegDirDegree)
				{
					dir = vec3(markingLaneData.firstDatas[0].segDir.get<0>(), markingLaneData.firstDatas[0].segDir.get<1>(), markingLaneData.firstDatas[0].segDir.get<2>());
				} else {
					dir = vec3(markingLaneData.secondDatas[0].segDir.get<0>(), markingLaneData.secondDatas[0].segDir.get<1>(), markingLaneData.secondDatas[0].segDir.get<2>());
				}
			}else {
				//处理车道边界反过来的情况
				if (bg::dot_product(markingLaneData.secondDatas[0].segDir, temp) > 0)
				{
					dir = vec3(markingLaneData.secondDatas[0].segDir.get<0>(), markingLaneData.secondDatas[0].segDir.get<1>(), markingLaneData.secondDatas[0].segDir.get<2>());
				}else {
					dir = vec3(markingLaneData.firstDatas[0].segDir.get<0>(), markingLaneData.firstDatas[0].segDir.get<1>(), markingLaneData.firstDatas[0].segDir.get<2>());
				}
			}
			
			dir.normalize();
		}
		else
		{
			laneWidth = 2500;
			pos = p_1;
			dir = vec3(polyPoints[3].pos.lon - polyPoints[0].pos.lon, polyPoints[3].pos.lat - polyPoints[0].pos.lat, polyPoints[3].z - polyPoints[0].z);
			dir.normalize();
		}
	}

	MarkingCompiler::MarkingLane MarkingCompiler::buildMarkingLane(
		const size_t& id,
		const point_t& searchPoint,
		const std::vector<std::vector<MapPoint3D64>>& allLines,
		const std::vector<segment_t>& segments,
		const std::vector<std::vector<size_t>>& sizes)
	{
		MarkingLane markingLane;

		// seg
		markingLane.seg = segments[id];
		bg::convert(markingLane.seg.second, markingLane.segDir);
		bg::subtract_point(markingLane.segDir, markingLane.seg.first);
		markingLane.segLength = bg::length(markingLane.seg);
		if (markingLane.segLength != 0)
		{
			bg::convert(markingLane.segDir, markingLane.segUnitDir);
			bg::divide_value(markingLane.segUnitDir, markingLane.segLength);
			markingLane.segUnitDir.set<2>(0); // z值需归0,不然计算点积会错误
		}

		// vertical seg
		bg::closest_points(searchPoint, segments[id], markingLane.verSeg);
		bg::convert(markingLane.verSeg.second, markingLane.verSegDir);
		bg::subtract_point(markingLane.verSegDir, markingLane.verSeg.first);
		markingLane.verSegLength = bg::length(markingLane.verSeg);
		if (markingLane.verSegLength != 0)
		{
			bg::convert(markingLane.verSegDir, markingLane.verSegUnitDir);
			bg::divide_value(markingLane.verSegUnitDir, markingLane.verSegLength);
			markingLane.verSegUnitDir.set<2>(0); // z值需归0,不然计算点积会错误
		}

		// id and point
		markingLane.beginPoint = allLines[sizes[id][0]][sizes[id][1]];
		markingLane.endPoint = allLines[sizes[id][0]][sizes[id][1] + 1];
		markingLane.lineId = sizes[id][0];
		markingLane.segId = sizes[id][1];
		markingLane.id = id;

		return markingLane;
	}

	void MarkingCompiler::generateContourByPosAndDir(
		MapPoint3D64 pos, 
		Vector3 dir1, 
		Vector3 upDir, 
		float width, 
		float height, 
		std::vector<MapPoint3D64>& outPoints)
	{
		Vector3 normal = cross(dir1, upDir);
		normal.normalize();
		MapPoint3D64 up = pos + MapPoint3D64_make((dir1 * height * 0.5f).x, (dir1 * height * 0.5f).y, (dir1 * height * 0.5f).z);
		MapPoint3D64 down = pos - MapPoint3D64_make((dir1 * height * 0.5f).x, (dir1 * height * 0.5f).y, (dir1 * height * 0.5f).z);
		MapPoint3D64 left1 = down - MapPoint3D64_make((normal * width * 0.5f).x, (normal * width * 0.5f).y, (normal * width * 0.5f).z);
		MapPoint3D64 right1 = down + MapPoint3D64_make((normal * width * 0.5f).x, (normal * width * 0.5f).y, (normal * width * 0.5f).z);
		MapPoint3D64 left2 = up - MapPoint3D64_make((normal * width * 0.5f).x, (normal * width * 0.5f).y, (normal * width * 0.5f).z);
		MapPoint3D64 right2 = up + MapPoint3D64_make((normal * width * 0.5f).x, (normal * width * 0.5f).y, (normal * width * 0.5f).z);
		left1.z /= 10;
		right1.z /= 10;
		right2.z /= 10;
		left2.z /= 10;
		outPoints.emplace_back(left1);
		outPoints.emplace_back(right1);
		outPoints.emplace_back(right2);
		outPoints.emplace_back(left2);
	}

	uint16 MarkingCompiler::minimalDegree(point_t dir1, point_t dir2)
	{
		double angle1 = std::atan2(dir1.get<1>(), dir1.get<0>());
		angle1 = int(angle1 * 180 / MATH_PI);
		double angle2 = std::atan2(dir2.get<1>(), dir2.get<0>());
		angle2 = int(angle2 * 180 / MATH_PI);
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
		angle = (int32)angle % 180;
		if (angle > 90) {
			angle = 180 - angle;
		}
		return angle;
	}

	void MarkingCompiler::addRdsMarking(
		RdsTile* pTile,
		HadArrow* hadArrow,
		RDS::RdsMarking::MarkingType type)
	{
		if (pTile == nullptr || hadArrow == nullptr)
		{
			return;
		}
		for (auto laneGroup : hadArrow->laneGroups) {
			RdsGroup* pRdsGroup = queryGroup(laneGroup->originId, pTile);
			if (pRdsGroup)
			{
				for (auto object : pRdsGroup->objects)
				{
					if (object->getEntityType() == EntityType::RDS_MARKING)
					{
						MapPoint3D64 markingPt;
						RdsMarking* marking = (RdsMarking*)object;
						convert(marking->position, markingPt);
						if (markingPt.pos.distance(hadArrow->postion.pos) < 2500)
						{
							return;
						}
					}
				}
			}
		}
		RdsMarking* pMarking = (RdsMarking*)createObject(pTile, EntityType::RDS_MARKING);
		pMarking->markingType = type;
		pMarking->color = (RdsMarking::MarkingColor)hadArrow->color;
		Polygon3d polygon = hadArrow->polygon;
		if (polygon.vertexes.size() != 4)
			return;
		convert(polygon, pMarking->contour);
		convert(hadArrow->postion, pMarking->position);
		for (auto laneGroup : hadArrow->laneGroups) {
			RdsGroup* pRdsGroup = queryGroup(laneGroup->originId, pTile);
			if (pRdsGroup)
				pRdsGroup->objects.push_back(pMarking);
		}
	}

	void MarkingCompiler::getMarkingRate(
		RDS::RdsMarking::MarkingType markingType,
		float& width,
		float& height)
	{
		switch (markingType)
		{
		case RDS::RdsMarking::MarkingType::eDiamond:
		{
			width = 1.0f;	// w
			height = 1.0f;	// h
		}
		break;
		case RDS::RdsMarking::MarkingType::eStraightArrow:
		{
			width = 1.0f;	// w
			height = 1.0f;	// h
		}
		break;
		case RDS::RdsMarking::MarkingType::eRightTurnArrow:
		{
			width = 1.0f;	// w
			height = 1.0f;	// h
		}
		break;
		case RDS::RdsMarking::MarkingType::eStraightOrRightTurnArrow:
		{
			width = 1.0f;	//w
			height = 1.0f;	//h
		}
		break;
		case RDS::RdsMarking::MarkingType::eLeftTurnArrow:
		{
			width = 1.0f;	// w
			height = 1.0f;	// h
		}
		break;
		case RDS::RdsMarking::MarkingType::eStraightOrLeftTurnArrow:
		{
			width = 1.0f;	// w
			height = 1.0f;	// h
		}
		break;
		case RDS::RdsMarking::MarkingType::eLeftOrRightTurnArrow:
		{
			width = 1.0f;	// w
			height = 1.0f;	// h
		}
		break;
		case RDS::RdsMarking::MarkingType::eLeftStraightOrRightTurnArrow:
		{
			width = 1.0f;	// w
			height = 1.0f;	// h
		}
		break;
		case RDS::RdsMarking::MarkingType::eUTurnArrow:
		{
			width = 1.0f;	// w
			height = 1.0f;	// h
		}
		break;
		case RDS::RdsMarking::MarkingType::eStraightOrUTurnArrow:
		{
			width = 1.0f;	// w
			height = 1.0f;	// h
		}
		break;
		case RDS::RdsMarking::MarkingType::eUTurnOrRightTurnArrow:
		{
			width = 1.0f;	// w
			height = 1.0f;	// h
		}
		break;
		case RDS::RdsMarking::MarkingType::eUTurnStraightOrRightTurnArrow:
		{
			width = 1.0f;	// w
			height = 1.0f;	// h
		}
		break;
		case RDS::RdsMarking::MarkingType::eLeftOrUTurnArrow:
		{
			width = 1.0f;	// w
			height = 1.0f;	// h
		}
		break;
		case RDS::RdsMarking::MarkingType::eLeftUTurnOrStraightArrow:
		{
			width = 1.0f;	// w
			height = 1.0f;	// h
		}
		break;
		case RDS::RdsMarking::MarkingType::eLeftRightOrUTurnArrow:
		{
			width = 1.0f;	// w
			height = 1.0f;	// h
		}
		break;
		case RDS::RdsMarking::MarkingType::eLeftCurve:
		{
			width = 1.0f;	// w
			height = 1.0f;	// h
		}
		break;
		case RDS::RdsMarking::MarkingType::eRightCurve:
		{
			width = 1.0f;	// w
			height = 1.0f;	// h
		}
		break;
		case RDS::RdsMarking::MarkingType::eURightTurnArrow:
		{
			width = 1.0f;	// w
			height = 1.0f;	// h
		}
		break;
		case RDS::RdsMarking::MarkingType::eStraightOrURightTurnArrow:
		{
			width = 1.0f;	// w
			height = 1.0f;	// h
		}
		break;
		case RDS::RdsMarking::MarkingType::eLeftZipper:
		{
			width = 1.0f;	// w
			height = 1.0f;	// h
		}
		break;
		case RDS::RdsMarking::MarkingType::eRightZipper:
		{
			width = 1.0f;	// w
			height = 1.0f;	// h
		}
		break;
		default:
		{
			width = 1.0f;	// w
			height = 1.0f;	// h
		}
		break;
		}
	}

	bool MarkingCompiler::isIntersectionArrow(
		const std::vector<MapPoint3D64>& pArrowPoly)
	{
		bool isIntersection = false;
		auto& gridIntersectionDatas = compilerData.m_rdsIntersections;
		getIntersectionBox2TRTree()->query(bgi::intersects(BOX_2T(pArrowPoly)),
			boost::make_function_output_iterator([&](size_t const& id) {
				if (!isIntersection)
				{
					auto tmpPoints = gridIntersectionDatas[id]._intersectionPoints;
					auto tmpRing = gridIntersectionDatas[id]._intersectionPoly2T;
					bg::correct(tmpRing);
					auto tmpArrowRing = RING_2T(pArrowPoly);
					bg::correct(tmpArrowRing);
					if (bg::intersects(tmpArrowRing, tmpRing))
					{
						point_t tmpCenterPoint, tmpArrowCenterPoint;
						bg::centroid(BOX_T(pArrowPoly), tmpCenterPoint);
						bg::centroid(BOX_T(tmpPoints), tmpArrowCenterPoint);
						if (std::abs(tmpCenterPoint.get<2>() - tmpArrowCenterPoint.get<2>()) < 3000)
							isIntersection = true;
					}
				}
				}));
		return isIntersection;
	}

}

