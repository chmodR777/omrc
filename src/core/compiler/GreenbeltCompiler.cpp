#include "stdafx.h"
#include <algorithm>
#include "GreenbeltCompiler.h"
#include "../framework/SpatialSeacher.h"
#include "clipper.hpp"
#include "math3d/vector_math.h"
#include "algorithm/grap_point_algorithm.h"
namespace OMDB
{
	double GreenbeltCompiler::ROAD_BOUNDARY_ZVALUE_TOLERANCE = 200;
	double GreenbeltCompiler::ROAD_BOUNDARY_DISCARD_TOLERANCE = 50000;
	double GreenbeltCompiler::ROAD_BOUNDARY_DISTANCE_TOLERANCE = 20000;
	double GreenbeltCompiler::MAX_ROAD_BOUNDARY_DISTANCE_TOLERANCE = 30000;
	double GreenbeltCompiler::ROAD_BOUNDARY_EXTRA_DISTANCE_TOLERANCE = 6000;


	void GreenbeltCompiler::compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile)
	{
		std::map<int64, std::set<int64>> forwardGroupTable;
		std::map<int64, std::set<int64>> backwardGroupTable;

		std::vector<HadGrid*> nearbyGrids = { pGrid };
		for_each(nearby.begin(), nearby.end(), [&](HadGrid* g)->void {nearbyGrids.push_back(g); });
		matchNearbyRoadBoundary(pGrid, nearbyGrids, forwardGroupTable, backwardGroupTable);


//		point_2t  p(12139597115, 3117734432);

//		+[0]{ pos = {lon = 12140032736 lat = 3116616408 } z = 1484 }	MapPoint3D64
//			+ [1]{ pos = {lon = 12139969425 lat = 3116719902 } z = 1484 }	MapPoint3D64
//+ [2]{ pos = {lon = 12139964663 lat = 3116727355 } z = 1489 }	MapPoint3D64//

			point_2t  p1(12140032,3116616);
			point_2t  p2(12139969,3116719);
			point_2t  p3(12139964,3116727);

		std::set<int64> visitedIds;
		for (auto obj : pGrid->query(ElementType::HAD_ROAD_BOUNDARY))
		{
			HadRoadBoundary* pRoadBoundary = (HadRoadBoundary*)obj;
			if (visitedIds.find(pRoadBoundary->originId) != visitedIds.end()) {
				continue;
			}

			//判断是否为普通路
			std::vector<HadLaneGroup*> tmpGroups;
			for (auto itemGroup : pRoadBoundary->linkGroups)
				tmpGroups.push_back(itemGroup.second);
			if (CompileSetting::instance()->isNotCompileUrbanData)
			{
				if (!isProDataLevel(tmpGroups))
					continue;
			}

			std::vector<MapPoint3D64> boundaryVertexes;
			std::vector<MapPoint3D64> nearbyBoundaryVertexes;
			createTopologyRoadBoundary(pRoadBoundary, boundaryVertexes,
				nearbyBoundaryVertexes, pGrid, forwardGroupTable, backwardGroupTable, visitedIds);
			if (boundaryVertexes.empty() || nearbyBoundaryVertexes.empty()) {
				continue;
			}

			std::vector<MapPoint3D64> linkGroupRoadBoundaryPoints;
			createNearbyRoadBoundary(boundaryVertexes, nearbyBoundaryVertexes, linkGroupRoadBoundaryPoints);
			double roadBoundaryDistance = calcBoundaryDistance(linkGroupRoadBoundaryPoints);

			std::vector<MapPoint3D64> nearbyLinkGroupRoadBoundaryPoints;
			createNearbyRoadBoundary(nearbyBoundaryVertexes, boundaryVertexes, nearbyLinkGroupRoadBoundaryPoints);
			double nearbyRoadBoundaryDistance = calcBoundaryDistance(nearbyLinkGroupRoadBoundaryPoints);
			if (roadBoundaryDistance < ROAD_BOUNDARY_DISCARD_TOLERANCE || nearbyRoadBoundaryDistance < ROAD_BOUNDARY_DISCARD_TOLERANCE) {
				continue;
			}

			RdsGreenbelt* pGreenbelt = (RdsGreenbelt*)createObject(pTile, EntityType::RDS_GREENBELT);
			OMDB::LineString3d location = { linkGroupRoadBoundaryPoints };
			RDS::LineString3d line;
			convert(location, line);
			pGreenbelt->contour.lines.push_back(line);

			OMDB::LineString3d nearbyLocation = { nearbyLinkGroupRoadBoundaryPoints };
			RDS::LineString3d nearbyLine;
			convert(nearbyLocation, nearbyLine);
			pGreenbelt->contour.lines.push_back(nearbyLine);

		}
	}

	void GreenbeltCompiler::matchNearbyRoadBoundary(HadGrid* const grid, const std::vector<HadGrid*>& nearby,
		std::map<int64, std::set<int64>>& forwardGroupTable, std::map<int64, std::set<int64>>& backwardGroupTable)
	{
		std::vector<HadLaneGroup*> linkGroups;
		for (auto obj : grid->query(ElementType::HAD_LANE_GROUP)) {
			HadLaneGroup* pGroup = (HadLaneGroup*)obj;

			//普通路不编译高速绿化带
			if (!isProDataLevel(pGroup))
				continue;

			linkGroups.push_back(pGroup);
			if (pGroup->crossGrid) {
				for (auto pTmpLinkGroup : pGroup->previous) {
					HadLaneGroup* previousLinkGroup = (HadLaneGroup*)pTmpLinkGroup;
					if (previousLinkGroup->crossGrid) {
						linkGroups.push_back(previousLinkGroup);
					}
				}
				for (auto pTmpLinkGroup : pGroup->next) {
					HadLaneGroup* nextLinkGroup = (HadLaneGroup*)pTmpLinkGroup;
					if (nextLinkGroup->crossGrid) {
						linkGroups.push_back(nextLinkGroup);
					}
				}
			}
		}

		for (auto obj : linkGroups) {
			HadLaneGroup* pLinkGroup = (HadLaneGroup*)obj;
			if (isMultiDigitized(pLinkGroup) || isInTollArea(pLinkGroup)) {
				continue;
			}

			std::vector<HadLaneGroup*> pNearbyLinkGroups = SpatialSeacher::seachNearby2d(grid, pLinkGroup, 1e4);
			expandNearbyLinkGroups(pLinkGroup, nearby, pNearbyLinkGroups);
			for (HadLaneGroup* pNearbyLinkGroup : pNearbyLinkGroups) {
				if (isMultiDigitized(pNearbyLinkGroup) || isInTollArea(pNearbyLinkGroup)) {
					continue;
				}
				if (pLinkGroup->roadBoundaries.size() != 2 || pNearbyLinkGroup->roadBoundaries.size() != 2) {
					continue;
				}
				if ((std::find(pLinkGroup->next.begin(), pLinkGroup->next.end(), pNearbyLinkGroup) != pLinkGroup->next.end())
					|| (std::find(pLinkGroup->previous.begin(), pLinkGroup->previous.end(), pNearbyLinkGroup) != pLinkGroup->previous.end())) {
					continue;
				}

				HadRoadBoundary* pLinkGroupRoadBoundary = nullptr;
				HadRoadBoundary* pNearbyLinkGroupRoadBoundary = nullptr;
				findNearbyRoadBoundary(nearby,
					pLinkGroup, pNearbyLinkGroup, pLinkGroupRoadBoundary, 
					pNearbyLinkGroupRoadBoundary, forwardGroupTable, backwardGroupTable);
				if (pLinkGroupRoadBoundary == nullptr || pNearbyLinkGroupRoadBoundary == nullptr) {
					continue;
				}

				int64 litOriginId = pLinkGroupRoadBoundary->originId;
				int64 bigOriginId = pNearbyLinkGroupRoadBoundary->originId;
				if (litOriginId > bigOriginId) {
					std::swap(litOriginId, bigOriginId);
				}
				auto forwardGroupTableIter = forwardGroupTable.find(litOriginId);
				if (forwardGroupTableIter != forwardGroupTable.end() && forwardGroupTableIter->second.find(bigOriginId) != forwardGroupTableIter->second.end()) {
					continue; // 同组数据已处理过
				}

				auto backwardGroupTableIter = backwardGroupTable.find(bigOriginId);
				if (forwardGroupTableIter == forwardGroupTable.end()) {
					std::set<int64> forwardSet;
					forwardSet.insert(bigOriginId);
					forwardGroupTable.emplace(litOriginId, forwardSet);
				}
				else {
					forwardGroupTableIter->second.insert(bigOriginId);
				}
				if (backwardGroupTableIter == backwardGroupTable.end()) {
					std::set<int64> backwardSet;
					backwardSet.insert(litOriginId);
					backwardGroupTable.emplace(bigOriginId, backwardSet);
				}
				else {
					backwardGroupTableIter->second.insert(litOriginId);
				}
			}
		}
	}

	void GreenbeltCompiler::findNearbyRoadBoundary(const std::vector<HadGrid*>& nearby, 
		HadLaneGroup* pLinkGroup, HadLaneGroup* pNearbyLinkGroup,
		HadRoadBoundary* &pLinkGroupRoadBoundary, HadRoadBoundary* &pNearbyLinkGroupRoadBoundary, 
		std::map<int64, std::set<int64>>& forwardGroupTable, std::map<int64, std::set<int64>>& backwardGroupTable)
	{
		HadRoadBoundary* boundary = pLinkGroup->roadBoundaries[0];
		HadRoadBoundary* rightBoundary = pLinkGroup->roadBoundaries[1];

		HadRoadBoundary* nearbyBoundary = pNearbyLinkGroup->roadBoundaries[0];
		HadRoadBoundary* nearbyRightBoundary = pNearbyLinkGroup->roadBoundaries[1];

		// 该网格被隧道抬高了,导致生成的绿化带和最后路面高度不一致,并且该网格没有生成绿化带
		// TODO 并且当前该网格没有普通路数据,后期如果普通路也要生成绿化带时,此处就不能注释了
		if (boundary->owner->getId() == 20169111 || nearbyBoundary->owner->getId() == 20169111) {
			return;
		}

		// 上下分离道路接着一个共享车道边界的道路时可能会出现这种情况
		if (filterRoadBoundary(boundary, nearbyBoundary)) {
			return;
		}

		// 过滤已匹配的车道边界
		std::set<int64> nearbyGroups = findNearbyGroups(boundary->originId, forwardGroupTable, backwardGroupTable);
		if (nearbyGroups.find(nearbyBoundary->originId) != nearbyGroups.end()) {
			return;
		}

		auto checkNearbyLinkGroups = [&](HadLaneGroup* pLinkGroup, HadRoadBoundary* boundary, HadRoadBoundary* rightBoundary,
			HadLaneGroup* pNearbyLinkGroup, HadRoadBoundary* nearbyBoundary, HadRoadBoundary* nearbyRightBoundary,
			int& grappedPoints, double& avgDistance, bool& nearbyLinkGroupsFlag) {
				auto getNearbyDistance = [](MapPoint3D64& point, std::vector<MapPoint3D64>& points)->double {
					size_t si, ei;
					MapPoint3D64 minGrappedPt = {};
					GrapPointAlgorithm::grapOrMatchNearestEndPoint(point, points, minGrappedPt, si, ei);
					if (minGrappedPt.pos.lon != 0 && minGrappedPt.pos.lat != 0) {
						Vector2 v;
						v.x = (float)(point.pos.lon - minGrappedPt.pos.lon);
						v.y = (float)(point.pos.lat - minGrappedPt.pos.lat);
						double distance = v.length();
						return distance;
					}
					return DBL_MAX;
				};

				for (auto& point : boundary->location.vertexes) {
					size_t si, ei;
					MapPoint3D64 minGrappedPt = {};
					GrapPointAlgorithm::grapPoint(point, nearbyBoundary->location.vertexes, minGrappedPt, si, ei);
					if (minGrappedPt.pos.lon != 0 && minGrappedPt.pos.lat != 0 && fabs(point.z - minGrappedPt.z) < ROAD_BOUNDARY_ZVALUE_TOLERANCE) {
						Vector2 v;
						v.x = (float)(point.pos.lon - minGrappedPt.pos.lon);
						v.y = (float)(point.pos.lat - minGrappedPt.pos.lat);
						double distance = v.length();
						if (distance > MAX_ROAD_BOUNDARY_DISTANCE_TOLERANCE ||
							distance > getNearbyDistance(minGrappedPt, rightBoundary->location.vertexes) ||
							distance > getNearbyDistance(point, nearbyRightBoundary->location.vertexes) ||
							!isNearbyLinkGroups(nearby, pLinkGroup, boundary, point, pNearbyLinkGroup, nearbyBoundary, minGrappedPt)) {
							nearbyLinkGroupsFlag = false;
							return;
						}

						avgDistance += distance;
						grappedPoints += 1;
					}
				}
		};

		auto calcRoadDistance = [](LineString3d& edge)->double {
			double distance = 0;
			for (int idx = 0; idx < edge.vertexes.size() - 1; idx++) {
				auto& currPt = edge.vertexes[idx];
				auto& nextPt = edge.vertexes[idx + 1];
				distance += currPt.pos.distance(nextPt.pos);
			}
			return distance;
		};

		int grappedPoints = 0;
		double avgDistance = 0;
		bool nearbyLinkGroupsFlag = true;
		double boundaryDistance = calcRoadDistance(boundary->location);
		double nearbyBoundaryDistance = calcRoadDistance(nearbyBoundary->location);
		if (boundaryDistance < nearbyBoundaryDistance) {
			checkNearbyLinkGroups(pLinkGroup, boundary, rightBoundary,
				pNearbyLinkGroup, nearbyBoundary, nearbyRightBoundary,
				grappedPoints, avgDistance, nearbyLinkGroupsFlag);
		}
		else
		{
			checkNearbyLinkGroups(pNearbyLinkGroup, nearbyBoundary, nearbyRightBoundary, 
				pLinkGroup, boundary, rightBoundary,
				grappedPoints, avgDistance, nearbyLinkGroupsFlag);
		}
		
		if (grappedPoints == 0 || !nearbyLinkGroupsFlag) {
			return;
		}
		if (grappedPoints < 2) {
			std::vector<MapPoint3D64> linkGroupRoadBoundaryPoints;
			createNearbyRoadBoundary(boundary->location.vertexes, nearbyBoundary->location.vertexes, linkGroupRoadBoundaryPoints);
			bool roadBoundaryOverlaped = linkGroupRoadBoundaryPoints.size() >= 2 ? true : false;
			if (!roadBoundaryOverlaped) {
				std::vector<MapPoint3D64> nearbyLinkGroupRoadBoundaryPoints;
				createNearbyRoadBoundary(nearbyBoundary->location.vertexes, boundary->location.vertexes, nearbyLinkGroupRoadBoundaryPoints);
				roadBoundaryOverlaped = nearbyLinkGroupRoadBoundaryPoints.size() >= 2 ? true : false;
			}
			if (!roadBoundaryOverlaped) {
				return;
			}
		}

		avgDistance /= grappedPoints;
		if (avgDistance < ROAD_BOUNDARY_DISTANCE_TOLERANCE) {
			pLinkGroupRoadBoundary = boundary;
			pNearbyLinkGroupRoadBoundary = nearbyBoundary;
		}
	}

	bool GreenbeltCompiler::isNearbyLinkGroups(const std::vector<HadGrid*>& nearby, 
		HadLaneGroup* pLinkGroup, HadRoadBoundary* boundary, MapPoint3D64& point,
		HadLaneGroup* pNearbyLinkGroup, HadRoadBoundary* nearbyBoundary, MapPoint3D64& nearbyPoint) {
		std::vector<MapPoint3D64> polygon;
		BoundingBox2d bbox = makeBoundingBox2d(point, nearbyPoint);
		polygon.emplace_back(MapPoint3D64_make(bbox.min.lon, bbox.min.lat, 0));
		polygon.emplace_back(MapPoint3D64_make(bbox.max.lon, bbox.min.lat, 0));
		polygon.emplace_back(MapPoint3D64_make(bbox.max.lon, bbox.max.lat, 0));
		polygon.emplace_back(MapPoint3D64_make(bbox.min.lon, bbox.max.lat, 0));
		polygon.emplace_back(MapPoint3D64_make(bbox.min.lon, bbox.min.lat, 0));

		// 过滤自身车道组和前后车道组
		auto filterLinkGroup = [](HadLaneGroup* pGroup, HadLaneGroup* pNearbyGroup)-> bool {
			if (pGroup == pNearbyGroup) {
				return true;
			}

			if ((std::find(pGroup->next.begin(), pGroup->next.end(), pNearbyGroup) != pGroup->next.end())
				|| (std::find(pGroup->previous.begin(), pGroup->previous.end(), pNearbyGroup) != pGroup->previous.end())) {
				return true;
			}
			return false;
		};

		std::vector<HadLaneGroup*> pNearbyLinkGroups = SpatialSeacher::seachNearby(nearby, bbox);
		for (HadLaneGroup* tmpNearbyLinkGroup : pNearbyLinkGroups) {
			if (filterLinkGroup(pLinkGroup, tmpNearbyLinkGroup) || filterLinkGroup(pNearbyLinkGroup, tmpNearbyLinkGroup)) {
				continue;
			}
			if (tmpNearbyLinkGroup->roadBoundaries.size() != 2) {
				continue;
			}

			bool filterBoundary = false;
			for (auto roadBoundary : tmpNearbyLinkGroup->roadBoundaries) {
				// 判断中间是否存在共享车道边界的情况
				if (boundary == roadBoundary || nearbyBoundary == roadBoundary) {
					return false;
				}

				// 判断是否为过滤车道组
				if (filterRoadBoundary(boundary, roadBoundary) || filterRoadBoundary(nearbyBoundary, roadBoundary)) {
					filterBoundary = true;
				}
			}
			if (filterBoundary) {
				continue;
			}

			HadRoadBoundary* roadBoundary = tmpNearbyLinkGroup->roadBoundaries[0];
			if (hasIntersect(polygon, roadBoundary->location.vertexes)) {
				if (existMiddleLaneGroup(point, pLinkGroup, boundary, pNearbyLinkGroup, nearbyBoundary, tmpNearbyLinkGroup)) {
					return false;
				}
			}
		}
		return true;
	}

	bool GreenbeltCompiler::existMiddleLaneGroup(MapPoint3D64& pt, HadLaneGroup* pLinkGroup, HadRoadBoundary* boundary,
		HadLaneGroup* pNearbyLinkGroup, HadRoadBoundary* nearbyBoundary, HadLaneGroup* middleLangGroup)
	{
		auto grapPoint = [](MapPoint3D64& point, std::vector<MapPoint3D64>& nearbyPoints, MapPoint3D64& nearbyGrappedPt, size_t& si, size_t& ei) -> bool {
			if (GrapPointAlgorithm::grapPoint(point, nearbyPoints, nearbyGrappedPt, si, ei) &&
				point.pos.distance(nearbyGrappedPt.pos) < ROAD_BOUNDARY_DISTANCE_TOLERANCE &&
				fabs(point.z - nearbyGrappedPt.z) < ROAD_BOUNDARY_ZVALUE_TOLERANCE) {
				return true;
			}
			return false;
		};

		size_t si = 0, ei = 0;
		MapPoint3D64 grappedPt = {};
		for (auto roadBoundary : middleLangGroup->roadBoundaries) {
			// 判断高程差
			GrapPointAlgorithm::grapOrMatchNearestPoint(pt, roadBoundary->location.vertexes, grappedPt, si, ei);
			if (grappedPt.pos.lon != 0 && grappedPt.pos.lat != 0 && fabs(pt.z - grappedPt.z) < ROAD_BOUNDARY_ZVALUE_TOLERANCE) {
				return true;
			}
		}

		// (12017071,3027156)
		// 判断是否存在从高速路中间下辅路的情况
		std::vector<MapPoint3D64> boundaryVertexes;
		createConnectedRoadBoundary(pLinkGroup, boundary, boundaryVertexes);

		std::vector<MapPoint3D64> nearbyBoundaryVertexes;
		createConnectedRoadBoundary(pNearbyLinkGroup, nearbyBoundary, nearbyBoundaryVertexes);

		Polygon3d tmpPolygon;
		makePolygon(boundaryVertexes, nearbyBoundaryVertexes, tmpPolygon);
		for (auto roadBoundary : middleLangGroup->roadBoundaries) {
			std::vector<MapPoint3D64> middleVertexes;
			createConnectedRoadBoundary(middleLangGroup, roadBoundary, middleVertexes);
			for (auto& pt : middleVertexes) {
				if (GrapPointAlgorithm::isPointInPolygon(pt, tmpPolygon.vertexes)) {
					// 判断高程差
					if (grapPoint(pt, boundaryVertexes, grappedPt, si, ei)) {
						return true;
					}
				}
			}
		}
		return false;
	}

	void GreenbeltCompiler::makePolygon(std::vector<MapPoint3D64>& boundaryVertexes, std::vector<MapPoint3D64>& nearbyBoundaryVertexes, Polygon3d& polygon)
	{
		polygon.vertexes.clear();
		MapPoint3D64 prevVertex = {};
		for (auto& vertex : boundaryVertexes) {
			if (!floatEqual(vertex.pos.distanceSquare(prevVertex.pos), 0)) {
				polygon.vertexes.push_back(vertex);
				prevVertex = vertex;
			}
		}
		for (auto& vertex : nearbyBoundaryVertexes) {
			if (!floatEqual(vertex.pos.distanceSquare(prevVertex.pos), 0)) {
				polygon.vertexes.push_back(vertex);
				prevVertex = vertex;
			}
		}

		// 闭环
		MapPoint3D64& startPt = polygon.vertexes.front();
		MapPoint3D64& endPt = polygon.vertexes.back();
		if (!floatEqual(startPt.pos.distanceSquare(endPt.pos), 0)) {
			polygon.vertexes.push_back(startPt);
		}
	}

	void GreenbeltCompiler::createConnectedRoadBoundary(HadLaneGroup* pLinkGroup, HadRoadBoundary* boundary, std::vector<MapPoint3D64>& boundaryVertexes)
	{ 
		int connectNum = 5;
		std::deque<HadRoadBoundary*> connectedBoundarys;
		std::deque<HadLaneGroup*> connectedLaneGroups;
		connectedBoundarys.push_front(boundary);
		connectedLaneGroups.push_front(pLinkGroup);
		connectForwardRoadBoundary(pLinkGroup, boundary, connectedBoundarys, connectedLaneGroups, connectNum);
		connectBackwardRoadBoundary(pLinkGroup, boundary, connectedBoundarys, connectedLaneGroups, connectNum);
		for (auto idx = 0; idx < connectedBoundarys.size(); idx++)
		{
			HadRoadBoundary* roadBoundary = connectedBoundarys[idx];
			HadLaneGroup* laneGroup = connectedLaneGroups[idx];
			LineString3d leftSide = roadBoundary->location;
			if (directionEqual(roadBoundary, laneGroup, 3))
			{
				std::reverse(leftSide.vertexes.begin(), leftSide.vertexes.end());
			}
			boundaryVertexes.insert(boundaryVertexes.end(), leftSide.vertexes.begin(), leftSide.vertexes.end());
		}
	}

	forceinline bool GreenbeltCompiler::filterRoadBoundary(HadRoadBoundary* pBoundary, HadRoadBoundary* pRoadBoundary)
	{
		auto inIntersection = [](HadRoadBoundary* prb)->bool {
			if (prb->linkGroups.size() == 1) {
				if (prb->linkGroups.begin()->second->inIntersection)
					return true;
			}
			return false;
		};

		// 过滤路口道路边界
		if (inIntersection(pBoundary) && inIntersection(pRoadBoundary)) {
			return true;
		}

		// 过滤自身或待匹配道路边界的前后道路边界
		if ((std::find(pBoundary->next.begin(), pBoundary->next.end(), pRoadBoundary) != pBoundary->next.end())
			|| (std::find(pBoundary->previous.begin(), pBoundary->previous.end(), pRoadBoundary) != pBoundary->previous.end())) {
			return true;
		}

		return pBoundary == pRoadBoundary || pBoundary->originId == pRoadBoundary->originId;
	}

	bool GreenbeltCompiler::hasIntersect(std::vector<MapPoint3D64>& points1, std::vector<MapPoint3D64>& points2)
	{
		MapPoint3D64 basePoint = points1[0];
		ClipperLib::Path path_v1;
		for (int i = 0; i < points1.size(); i++)
			path_v1 << ClipperLib::IntPoint(points1[i].pos.lon - basePoint.pos.lon, points1[i].pos.lat - basePoint.pos.lat, 0.0);
		ClipperLib::Path path_v2;
		for (int i = 0; i < points2.size(); i++)
			path_v2 << ClipperLib::IntPoint(points2[i].pos.lon - basePoint.pos.lon, points2[i].pos.lat - basePoint.pos.lat, 0.0);
		ClipperLib::Clipper clipper;
		clipper.AddPath(path_v2, ClipperLib::ptSubject, false);
		clipper.AddPath(path_v1, ClipperLib::ptClip, true);
		ClipperLib::PolyTree polyTree;
		clipper.Execute(ClipperLib::ctIntersection, polyTree, ClipperLib::pftNonZero);
		if (polyTree.Total() > 0)
			return true;
		return false;
	}

	BoundingBox2d GreenbeltCompiler::makeBoundingBox2d(MapPoint3D64& point, MapPoint3D64& nearbyPoint)
	{
		std::vector<int64> vx;
		std::vector<int64> vy;
		vx.push_back(point.pos.lon);
		vy.push_back(point.pos.lat);
		if (nearbyPoint.pos.lon != point.pos.lon) {
			vx.push_back(nearbyPoint.pos.lon);
		}
		else {
			vx.push_back(nearbyPoint.pos.lon + 10);
		}
		if (nearbyPoint.pos.lat != point.pos.lat) {
			vy.push_back(nearbyPoint.pos.lat);
		}
		else {
			vy.push_back(nearbyPoint.pos.lat + 10);
		}
		std::sort(vx.begin(), vx.end());
		std::sort(vy.begin(), vy.end());

		BoundingBox2d bbox;
		bbox.min.lon = vx[0];
		bbox.min.lat = vy[0];
		bbox.max.lon = *vx.rbegin();
		bbox.max.lat = *vy.rbegin();
		return bbox;
	}

	void GreenbeltCompiler::createNearbyRoadBoundary(std::vector<MapPoint3D64>& points, 
		std::vector<MapPoint3D64>& nearbyPoints, std::vector<MapPoint3D64>& roadBoundaryPoints)
	{
		auto grapPoint = [](MapPoint3D64& point, std::vector<MapPoint3D64>& nearbyPoints, MapPoint3D64& nearbyGrappedPt, size_t& si, size_t& ei) -> bool {
			if (GrapPointAlgorithm::grapPoint(point, nearbyPoints, nearbyGrappedPt, si, ei, 500) && 
				point.pos.distance(nearbyGrappedPt.pos) < ROAD_BOUNDARY_DISTANCE_TOLERANCE &&
				fabs(point.z - nearbyGrappedPt.z) < ROAD_BOUNDARY_ZVALUE_TOLERANCE) {
				return true;
			}
			return false;
		};
		
		int startIdx = -1, endIdx = -1;
		for (int idx = 0; idx < points.size(); idx++) {
			size_t si, ei;
			MapPoint3D64& point = points[idx];
			MapPoint3D64 nearbyGrappedPt = {};
			if (grapPoint(point, nearbyPoints, nearbyGrappedPt, si, ei)) {
				startIdx = idx;
				auto& firstPt = points[0];
				if (point.pos.distance(firstPt.pos) < ROAD_BOUNDARY_EXTRA_DISTANCE_TOLERANCE) {
					startIdx = 0;
				}
				break;
			}
		};

		for (int idx = points.size() - 1; idx >= 0; idx--) {
			size_t si, ei;
			MapPoint3D64& point = points[idx];
			MapPoint3D64 nearbyGrappedPt = {};
			if (grapPoint(point, nearbyPoints, nearbyGrappedPt, si, ei)) {
				endIdx = idx;
				auto& endPt = points[points.size() - 1];
				if (point.pos.distance(endPt.pos) < ROAD_BOUNDARY_EXTRA_DISTANCE_TOLERANCE) {
					endIdx = points.size() - 1;
				}
				break;
			}
		};

		if (startIdx != -1 && endIdx != -1) {
			for (int idx = startIdx; idx <= endIdx; idx++) {
				MapPoint3D64& point = points[idx];
				roadBoundaryPoints.push_back(point);
			}
		} else if(startIdx != -1){
			roadBoundaryPoints.push_back(points[startIdx]);
		} else if (endIdx != -1) {
			roadBoundaryPoints.push_back(points[endIdx]);
		}

		// 补充尾部抓点
		for (int idx = 0; idx < nearbyPoints.size(); idx++) {
			size_t si, ei;
			MapPoint3D64 firstGrappedPt = {};
			MapPoint3D64 firstNearbyPoint = nearbyPoints[idx];
			if (grapPoint(firstNearbyPoint, points, firstGrappedPt, si, ei)) {
				if (si >= endIdx) {
					roadBoundaryPoints.push_back(firstGrappedPt);
					// 尾点可能距离对侧线在本方线上抓的点非常近,此时尾点也需补充进去
					if (ei == points.size() - 1 && firstGrappedPt.pos.distance(points[ei].pos) < ROAD_BOUNDARY_EXTRA_DISTANCE_TOLERANCE) {
						roadBoundaryPoints.push_back(points[ei]);
					}
				}
				break;
			}
		}

		// 补充首部抓点
		for (int idx = nearbyPoints.size() - 1; idx >= 0; idx--) {
			size_t si, ei;
			MapPoint3D64 lastGrappedPt = {};
			MapPoint3D64 lastNearbyPoint = nearbyPoints[idx];
			if (grapPoint(lastNearbyPoint, points, lastGrappedPt, si, ei)) {
				if (ei <= startIdx) {
					roadBoundaryPoints.emplace(roadBoundaryPoints.begin(), lastGrappedPt);
					// 首点可能距离对侧线在本方线上抓的点非常近,此时首点也需补充进去
					if (si == 0 && lastGrappedPt.pos.distance(points[si].pos) < ROAD_BOUNDARY_EXTRA_DISTANCE_TOLERANCE) {
						roadBoundaryPoints.emplace(roadBoundaryPoints.begin(), points[si]);
					}
				}
				break;
			}
		}
		auto ip = std::unique(roadBoundaryPoints.begin(), roadBoundaryPoints.end(), mapPoint3D64_compare);
		roadBoundaryPoints.resize(std::distance(roadBoundaryPoints.begin(), ip));
	}

	void GreenbeltCompiler::createTopologyRoadBoundary(HadRoadBoundary* boundary, std::vector<MapPoint3D64>& boundaryVertexes, std::vector<MapPoint3D64>& nearbyBoundaryVertexes,
		HadGrid* const grid, std::map<int64, std::set<int64>>& forwardGroupTable, std::map<int64, std::set<int64>>& backwardGroupTable, std::set<int64>& visitedIds) {
		auto forwardGroupTableIter = forwardGroupTable.find(boundary->originId);
		auto backwardGroupTableIter = backwardGroupTable.find(boundary->originId);
		if (forwardGroupTableIter == forwardGroupTable.end() && backwardGroupTableIter == backwardGroupTable.end()) {
			return;
		}

		std::set<int64> nearbyGroups = findNearbyGroups(boundary->originId, forwardGroupTable, backwardGroupTable);
		
		// previous
		std::set<int64> previousCycles;
		std::vector<HadRoadBoundary* > linkedBoundarys;
		HadRoadBoundary* previousBoundary = boundary;
		while (previousBoundary->previous.size() == 1) {
			previousBoundary = (HadRoadBoundary*)previousBoundary->previous.front();
			if (previousCycles.find(previousBoundary->originId) != previousCycles.end()) {
				break;
			}

			std::set<int64> previousGroups = findNearbyGroups(previousBoundary->originId, forwardGroupTable, backwardGroupTable);
			if (!previousGroups.empty()) {
				previousCycles.insert(previousBoundary->originId);
				linkedBoundarys.emplace(linkedBoundarys.begin(), previousBoundary);
				for_each(previousGroups.begin(), previousGroups.end(),
					[&](int64 previousId)->void {
						nearbyGroups.insert(previousId);
					}
				);
			} else {
				break;
			}
		}

		// current
		linkedBoundarys.push_back(boundary);

		// next
		std::set<int64> nextCycles;
		HadRoadBoundary* nextBoundary = boundary;
		while (nextBoundary->next.size() == 1) {
			nextBoundary = (HadRoadBoundary*)nextBoundary->next.front();
			if (nextCycles.find(nextBoundary->originId) != nextCycles.end()) {
				break;
			}

			std::set<int64> nextGroups = findNearbyGroups(nextBoundary->originId, forwardGroupTable, backwardGroupTable);
			if (!nextGroups.empty()) {
				nextCycles.insert(nextBoundary->originId);
				linkedBoundarys.push_back(nextBoundary);
				for_each(nextGroups.begin(), nextGroups.end(),
					[&](int64 nextId)->void {
						nearbyGroups.insert(nextId);
					}
				);
			} else {
				break;
			}
		}

		// nearby previous
		std::set<int64> nearbyPreviousCycles;
		std::vector<HadRoadBoundary* > nearbyLinkedBoundarys;
		HadRoadBoundary* nearbyBoundary = nullptr;
		for (auto nearbyId : nearbyGroups) {
			nearbyBoundary = (HadRoadBoundary*)grid->query(nearbyId, ElementType::HAD_ROAD_BOUNDARY);
			if (nearbyBoundary != nullptr) {
				break;
			}
		}
		if (nearbyBoundary == nullptr) {
			return;
		}

		HadRoadBoundary* nearbyPreviousBoundary = nearbyBoundary;
		while (nearbyPreviousBoundary->previous.size() == 1) {
			nearbyPreviousBoundary = (HadRoadBoundary*)nearbyPreviousBoundary->previous.front();
			if (nearbyPreviousCycles.find(nearbyPreviousBoundary->originId) != nearbyPreviousCycles.end()) {
				break;
			}

			if (nearbyGroups.find(nearbyPreviousBoundary->originId) != nearbyGroups.end()) {
				nearbyLinkedBoundarys.emplace(nearbyLinkedBoundarys.begin(), nearbyPreviousBoundary);
			} else {
				break;
			}
			nearbyPreviousCycles.insert(nearbyPreviousBoundary->originId);
		}

		// nearby current
		nearbyLinkedBoundarys.push_back(nearbyBoundary);

		// nearby next
		std::set<int64> nearbyNextCycles;
		HadRoadBoundary* nearbyNextBoundary = nearbyBoundary;
		while (nearbyNextBoundary->next.size() == 1) {
			nearbyNextBoundary = (HadRoadBoundary*)nearbyNextBoundary->next.front();
			if (nearbyNextCycles.find(nearbyNextBoundary->originId) != nearbyNextCycles.end()) {
				break;
			}

			if (nearbyGroups.find(nearbyNextBoundary->originId) != nearbyGroups.end()) {
				nearbyLinkedBoundarys.push_back(nearbyNextBoundary);
			} else {
				break;
			}
			nearbyNextCycles.insert(nearbyNextBoundary->originId);
		}

		// linkedBoundarys
		for (int i = 0; i < linkedBoundarys.size(); i++) {
			HadRoadBoundary* linkedBoundary = linkedBoundarys[i];
			visitedIds.insert(linkedBoundary->originId);
			boundaryVertexes.insert(boundaryVertexes.end(), linkedBoundary->location.vertexes.begin(), linkedBoundary->location.vertexes.end());
		}

		// nearbyLinkedBoundarys
		for (int i = 0; i < nearbyLinkedBoundarys.size(); i++) {
			HadRoadBoundary* nearbyLinkedBoundary = nearbyLinkedBoundarys[i];
			visitedIds.insert(nearbyLinkedBoundary->originId);
			nearbyBoundaryVertexes.insert(nearbyBoundaryVertexes.end(), nearbyLinkedBoundary->location.vertexes.begin(), nearbyLinkedBoundary->location.vertexes.end());
		}
	}

	std::set<int64> GreenbeltCompiler::findNearbyGroups(int64 originId, 
		std::map<int64, std::set<int64>>& forwardGroupTable, std::map<int64, std::set<int64>>& backwardGroupTable)
	{
		std::set<int64> nearbyGroups;
		auto forwardGroupTableIter = forwardGroupTable.find(originId);
		auto backwardGroupTableIter = backwardGroupTable.find(originId);
		if (forwardGroupTableIter == forwardGroupTable.end() && backwardGroupTableIter == backwardGroupTable.end()) {
			return nearbyGroups;
		}

		if (forwardGroupTableIter != forwardGroupTable.end()) {
			for_each(forwardGroupTableIter->second.begin(), forwardGroupTableIter->second.end(),
				[&](int64 forwardId)->void {
					nearbyGroups.insert(forwardId);
				}
			);
		}

		if (backwardGroupTableIter != backwardGroupTable.end()) {
			for_each(backwardGroupTableIter->second.begin(), backwardGroupTableIter->second.end(),
				[&](int64 backwardId)->void {
					nearbyGroups.insert(backwardId);
				}
			);
		}

		return nearbyGroups;
	}

	void GreenbeltCompiler::expandNearbyLinkGroups(HadLaneGroup* pLinkGroup, const std::vector<HadGrid*>& nearby, std::vector<HadLaneGroup*>& pNearbyLinkGroups)
	{
		std::vector<HadLaneGroup*> crossGridLGs;
		auto expandCrossGridLinkGroups = [&](HadLaneGroup* pGroup) {
			if (pGroup->crossGrid) {
				for (auto pTmpLinkGroup : pGroup->previous) {
					HadLaneGroup* previousLinkGroup = (HadLaneGroup*)pTmpLinkGroup;
					if (previousLinkGroup->crossGrid && previousLinkGroup->owner) {
						std::vector<HadLaneGroup*> previousNearbyLinkGroups = SpatialSeacher::seachNearby2d(previousLinkGroup->owner, previousLinkGroup, 1e4);
						for (HadLaneGroup* previousNearbyLinkGroup : previousNearbyLinkGroups) {
							crossGridLGs.push_back(previousNearbyLinkGroup);
						}
						crossGridLGs.push_back(previousLinkGroup);
					}
				}

				for (auto pTmpLinkGroup : pGroup->next) {
					HadLaneGroup* nextLinkGroup = (HadLaneGroup*)pTmpLinkGroup;
					if (nextLinkGroup->crossGrid && nextLinkGroup->owner) {
						std::vector<HadLaneGroup*> nextNearbyLinkGroups = SpatialSeacher::seachNearby2d(nextLinkGroup->owner, nextLinkGroup, 1e4);
						for (HadLaneGroup* nextNearbyLinkGroup : nextNearbyLinkGroups) {
							crossGridLGs.push_back(nextNearbyLinkGroup);
						}
						crossGridLGs.push_back(nextLinkGroup);
					}
				}
			}
		};

		BoundingBox2d bbox = BoundingBox2d::expand(pLinkGroup->extent.boundingbox2d(), 1e4);
		std::vector<HadLaneGroup*> bboxNearbyLinkGroups = SpatialSeacher::seachNearby(nearby, bbox);
		for (auto bboxNearbyLinkGroup : bboxNearbyLinkGroups) {
			crossGridLGs.push_back(bboxNearbyLinkGroup);
		}

		expandCrossGridLinkGroups(pLinkGroup);
		for (HadLaneGroup* pNearbyLinkGroup : pNearbyLinkGroups) {
			expandCrossGridLinkGroups(pNearbyLinkGroup);
		}

		for (HadLaneGroup* pCrossGridLg : crossGridLGs) {
			if (pLinkGroup == pCrossGridLg) {
				continue;
			}
			if (std::find(pNearbyLinkGroups.begin(), pNearbyLinkGroups.end(), pCrossGridLg) == pNearbyLinkGroups.end()) {
				pNearbyLinkGroups.push_back(pCrossGridLg);
			}
		}
	}

	double GreenbeltCompiler::calcBoundaryDistance(std::vector<MapPoint3D64>& points)
	{
		double distance = 0;
		for (int idx = 0; idx < points.size() - 1; idx++) {
			auto& currPt = points[idx];
			auto& nextPt = points[idx + 1];
			distance += currPt.pos.distance(nextPt.pos);
		}
		return distance;
	}

}