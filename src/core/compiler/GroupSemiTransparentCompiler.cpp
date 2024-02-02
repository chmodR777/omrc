#include "stdafx.h"

#include "GroupSemiTransparentCompiler.h"
#include "algorithm/grap_point_algorithm.h"
#include "polyline_intersector.h"
#include "math3d/vector_math.h"
#include "math3d/vector2.h"
#include "clipper.hpp"
#include <algorithm>
namespace OMDB
{
	double GroupSemiTransparentCompiler::SEMI_TRANSPARENT_CLIP_DISTANCE_TOLERANCE = 100000;
	double GroupSemiTransparentCompiler::SEMI_TRANSPARENT_DISTANCE_TOLERANCE = 60000;
	double GroupSemiTransparentCompiler::BOUNDARY_DISTANCE_TOLERANCE = 20000;
	double GroupSemiTransparentCompiler::BOUNDARY_ZVALUE_TOLERANCE = 500;

	bool GroupSemiTransparentCompiler::isCrossGrid(HadLaneGroup* pGroup)
	{
		if (pGroup->crossGrid)
			return true;

		for (auto association : pGroup->associations) {
			for (auto pLA : association->relLaneGroupAssociations) {
				if (pLA.first && pLA.first->crossGrid)
					return true;
				if (pLA.second && pLA.second->crossGrid)
					return true;
				for (auto overedFillArea : pLA.overedFillAreas) {
					for (auto overedLA : overedFillArea->overedLgAssociations) {
						if (overedLA->first && overedLA->first->crossGrid)
							return true;
						if (overedLA->second && overedLA->second->crossGrid)
							return true;
					}
				}
			}
		}

		return false;
	}

	// 车道组是否包含当前网格号
	bool GroupSemiTransparentCompiler::containsCurrentGrid(std::vector<HadSkeleton*>& groups, HadGrid* const pGrid)
	{
		for (auto group : groups) {
			HadLaneGroup* pGroup = (HadLaneGroup*)group;
			if (pGroup->owner == pGrid)
				return true;
			for (auto association : pGroup->associations) {
				for (auto pLA : association->relLaneGroupAssociations) {
					if (pLA.first && pLA.first->owner == pGrid)
						return true;
					if (pLA.second && pLA.second->owner == pGrid)
						return true;
					for (auto overedFillArea : pLA.overedFillAreas) {
						for (auto overedLA : overedFillArea->overedLgAssociations) {
							if (overedLA->first && overedLA->first->owner == pGrid)
								return true;
							if (overedLA->second && overedLA->second->owner == pGrid)
								return true;
						}
					}
				}
			}
		}
		return false;
	}

	void GroupSemiTransparentCompiler::compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile)
	{
		// 处理当前网格的车道组
		for (auto obj : pGrid->query(ElementType::HAD_LANE_GROUP))
		{
			HadLaneGroup* pLaneGroup = (HadLaneGroup*)obj;

			//判断是否为普通路
			if (CompileSetting::instance()->isNotCompileUrbanData)
			{
				if (!isProDataLevel(pLaneGroup))
					continue;
			}

			if (isNotSemiTransparentGroup(pLaneGroup)) {
				continue;
			}

			m_mainRoadSemiTransparent = false;
			if (!isMultiDigitized(pLaneGroup)) {
				m_mainRoadSemiTransparent = true;
			}

			MultiPoint3d semiTransparentPoints;
			std::vector<std::vector<HadLaneGroup*>> allConnectedGroups;
			getConnectedGroups(pLaneGroup, allConnectedGroups, semiTransparentPoints);
			for (auto connectedGroups : allConnectedGroups) {
				clipGroups(pTile, pLaneGroup, connectedGroups, semiTransparentPoints);
			}
		}

		// 处理跨网格的车道组
		for (auto nearbyGrid : nearby) {
			for (auto obj : nearbyGrid->query(ElementType::HAD_LANE_GROUP))
			{
				HadLaneGroup* pLaneGroup = (HadLaneGroup*)obj;

				//判断是否为普通路
				if (CompileSetting::instance()->isNotCompileUrbanData)
				{
					if (!isProDataLevel(pLaneGroup))
						continue;
				}

				if (isNotSemiTransparentGroup(pLaneGroup)) {
					continue;
				}

				m_mainRoadSemiTransparent = false;
				if (!isMultiDigitized(pLaneGroup)) {
					m_mainRoadSemiTransparent = true;
				}

				MultiPoint3d semiTransparentPoints;
				bool isCrossGridConnectedLaneGroup = false;
				std::vector<std::vector<HadLaneGroup*>> allConnectedGroups;
				getConnectedGroups(pLaneGroup, allConnectedGroups, semiTransparentPoints);
				for (auto connectedGroups : allConnectedGroups) {
					for (auto connectedGroup : connectedGroups) {
						if (isCrossGrid(connectedGroup)) {
							auto& previous = connectedGroup->previous;
							auto& next = connectedGroup->next;
							if (containsCurrentGrid(previous, pGrid) || containsCurrentGrid(next, pGrid)) {
								isCrossGridConnectedLaneGroup = true;
							}
						}
						if (isCrossGridConnectedLaneGroup)
							break;
					}
					if (isCrossGridConnectedLaneGroup)
						break;
				}

				if (isCrossGridConnectedLaneGroup) {
					for (auto connectedGroups : allConnectedGroups) {
						clipGroups(pTile, pLaneGroup, connectedGroups, semiTransparentPoints);
					}
				}

			}
		}
	}

	void GroupSemiTransparentCompiler::getConnectedGroups(HadLaneGroup* pLaneGroup, std::vector<std::vector<HadLaneGroup*>>& allConnectedGroups, MultiPoint3d& semiTransparentPoints)
	{
		if (pLaneGroup->roadBoundaries.size() != 2 && pLaneGroup->laneBoundaries.size() < 2) {
			return;
		}

		// 使用道路边界和车道边界
		LineString3d leftSide, rightSide;
		getLaneGroupBoundary(pLaneGroup, leftSide, rightSide);

		// 一个断头路可能连着两条道路,这里保存相应的列表
		semiTransparentPoints = constructSemiTransparentPoints(pLaneGroup);
		double leftBoundaryDistance = calcBoundaryDistance(leftSide.vertexes);
		double rightBoundaryDistance = calcBoundaryDistance(rightSide.vertexes);
		if (min(leftBoundaryDistance, rightBoundaryDistance) < SEMI_TRANSPARENT_DISTANCE_TOLERANCE) {
			bool forward = getLaneGroupSkeletons(pLaneGroup->previous).empty();
			std::vector<HadLaneGroup*> topoGroups = getTopoGroups(pLaneGroup, forward);
			for (auto topoGroup : topoGroups) {
				std::vector<HadLaneGroup*> connectedGroups;
				connectedGroups.push_back(pLaneGroup);

				double leftDistance = leftBoundaryDistance;
				double rightDistance = rightBoundaryDistance;
				connectGroup(pLaneGroup, topoGroup, connectedGroups, leftDistance, rightDistance, forward);
				allConnectedGroups.push_back(connectedGroups);
			}
		}

		// 没找到拓扑关系
		if (allConnectedGroups.empty()) {
			std::vector<HadLaneGroup*> connectedGroups;
			connectedGroups.push_back(pLaneGroup);
			allConnectedGroups.push_back(connectedGroups);
		}
	}

	void GroupSemiTransparentCompiler::connectGroup(HadLaneGroup* pLaneGroup, HadLaneGroup* pTopoGroup,
		std::vector<HadLaneGroup*>& connectedGroups, double& leftBoundaryDistance, double& rightBoundaryDistance, bool forward)
	{
		HadLaneGroup* pCurrentGroup = pLaneGroup;
		HadLaneGroup* pNextGroup = pTopoGroup;
		if (pNextGroup == nullptr) {
			return;
		}

		LineString3d leftSide, rightSide;
		getLaneGroupBoundary(pNextGroup, leftSide, rightSide);
		leftBoundaryDistance += calcBoundaryDistance(leftSide.vertexes);
		rightBoundaryDistance += calcBoundaryDistance(rightSide.vertexes);
		if (forward) {
			pLaneGroup = pNextGroup;
			connectedGroups.push_back(pLaneGroup);
		} else {
			std::swap(pCurrentGroup, pNextGroup);
			pLaneGroup = pCurrentGroup;
			connectedGroups.emplace(connectedGroups.begin(), pLaneGroup);
		}

		if (min(leftBoundaryDistance, rightBoundaryDistance) < SEMI_TRANSPARENT_DISTANCE_TOLERANCE) {
			std::vector<HadLaneGroup*> topoGroups = getTopoGroups(pLaneGroup, forward);
			if (topoGroups.size() == 1) {
				connectGroup(pLaneGroup, topoGroups[0], connectedGroups, leftBoundaryDistance, rightBoundaryDistance, forward);
			}
		}
	}

	void GroupSemiTransparentCompiler::clipGroups(RdsTile* pTile, HadLaneGroup* pLaneGroup,
		std::vector<HadLaneGroup*>& connectedGroups, MultiPoint3d& semiTransparentPoints)
	{
		LineString3d leftSide, rightSide;
		mergeGroupBoundary(connectedGroups, leftSide, rightSide);
		double leftBoundaryDistance = calcBoundaryDistance(leftSide.vertexes);
		double rightBoundaryDistance = calcBoundaryDistance(rightSide.vertexes);
		if (min(leftBoundaryDistance, rightBoundaryDistance) < SEMI_TRANSPARENT_CLIP_DISTANCE_TOLERANCE) {
			// 不需要裁减,设置半透明点即可返回
			for_each(connectedGroups.begin(), connectedGroups.end(), [&](HadLaneGroup* connectedGroup) {
				RdsGroup* pGroup = (RdsGroup*)queryGroup(connectedGroup->originId, pTile);
				setupSemiTransparentPoints(pGroup, semiTransparentPoints, min(leftBoundaryDistance, rightBoundaryDistance));
				deleteSemiTransparentObjects(pTile, connectedGroup, pGroup, semiTransparentPoints);
			});
			return;
		}

		// 对环形数据进行裁减
		bool leftSelfIntersect = isSelfIntersect(leftSide);
		bool rightSelfIntersect = isSelfIntersect(rightSide);
		if (leftSelfIntersect && rightSelfIntersect) {
			double leftDistance = 0, rightDistance = 0;
			LineString3d clippedLeftSide, clippedRightSide;
			if (pLaneGroup->previous.empty()) {
				for_each(connectedGroups.begin(), connectedGroups.end(), [&](HadLaneGroup* connectedGroup) {
					bool cliped = clipGroup(pTile, pLaneGroup, connectedGroup, semiTransparentPoints,
						leftDistance, rightDistance, clippedLeftSide, clippedRightSide);
					if (cliped) {
						return;
					}
				});
			} else {
				for_each(connectedGroups.rbegin(), connectedGroups.rend(), [&](HadLaneGroup* connectedGroup) {
					bool cliped = clipGroup(pTile, pLaneGroup, connectedGroup, semiTransparentPoints,
						leftDistance, rightDistance, clippedLeftSide, clippedRightSide);
					if (cliped) {
						return;
					}
				});
			}
		}
		else
		{
			// 不需要裁减,设置半透明点即可返回
			for_each(connectedGroups.begin(), connectedGroups.end(), [&](HadLaneGroup* connectedGroup) {
				RdsGroup* pGroup = (RdsGroup*)queryGroup(connectedGroup->originId, pTile);
				setupSemiTransparentPoints(pGroup, semiTransparentPoints, min(leftBoundaryDistance, rightBoundaryDistance));
				deleteSemiTransparentObjects(pTile, connectedGroup, pGroup, semiTransparentPoints);
			});
		}
	}

	bool GroupSemiTransparentCompiler::clipGroup(RdsTile* pTile, 
		HadLaneGroup* pLaneGroup, HadLaneGroup* connectedGroup, MultiPoint3d& semiTransparentPoints,
		double& leftBoundaryDistance, double& rightBoundaryDistance, LineString3d& leftSide, LineString3d& rightSide)
	{
		auto clipBoundary = [](LineString3d& src, LineString3d& dst, double& boundaryDistance)->bool {
			MapPoint3D64 prevVertex = {};
			if (!dst.vertexes.empty())
				prevVertex = dst.vertexes.back();
			for (int idx = 0; idx < src.vertexes.size(); idx++) {
				auto& vertex = src.vertexes[idx];
				if (dst.vertexes.empty()) {
					dst.vertexes.push_back(vertex);
					prevVertex = vertex;
					continue;
				}

				double distance = vertex.pos.distance(prevVertex.pos);
				if (floatEqual(distance, 0)) {
					continue;
				}

				dst.vertexes.push_back(vertex);
				boundaryDistance += distance;
				prevVertex = vertex;
				if (boundaryDistance >= SEMI_TRANSPARENT_DISTANCE_TOLERANCE) {
					return idx != src.vertexes.size() - 1;
				}
			};
			return false;
		};

		LineString3d left, right;
		if (connectedGroup->roadBoundaries.size() == 2)
		{
			left = connectedGroup->roadBoundaries[0]->location;
			right = connectedGroup->roadBoundaries[1]->location;
		}
		else
		{
			left = connectedGroup->laneBoundaries[0]->location;
			auto rightIdx = connectedGroup->laneBoundaries.size() - 1;
			right = connectedGroup->laneBoundaries[rightIdx]->location;
		}
		if (pLaneGroup->previous.empty()) {
			if (connectedGroup->roadBoundaries.size() == 2)
			{
				if (directionEqual(connectedGroup->roadBoundaries[0], connectedGroup, 3)) {
					std::reverse(left.vertexes.begin(), left.vertexes.end());
				}
				if (directionEqual(connectedGroup->roadBoundaries[1], connectedGroup, 3)) {
					std::reverse(right.vertexes.begin(), right.vertexes.end());
				}
			}
			else
			{
				if (directionEqual(connectedGroup->laneBoundaries[0], connectedGroup, 3)) {
					std::reverse(left.vertexes.begin(), left.vertexes.end());
				}
				auto rightIdx = connectedGroup->laneBoundaries.size() - 1;
				if (directionEqual(connectedGroup->laneBoundaries[rightIdx], connectedGroup, 3)) {
					std::reverse(right.vertexes.begin(), right.vertexes.end());
				}
			}
		}
		else {
			if (connectedGroup->roadBoundaries.size() == 2)
			{
				if (directionEqual(connectedGroup->roadBoundaries[0], connectedGroup, 2)) {
					std::reverse(left.vertexes.begin(), left.vertexes.end());
				}
				if (directionEqual(connectedGroup->roadBoundaries[1], connectedGroup, 2)) {
					std::reverse(right.vertexes.begin(), right.vertexes.end());
				}
			}
			else
			{
				if (directionEqual(connectedGroup->laneBoundaries[0], connectedGroup, 2)) {
					std::reverse(left.vertexes.begin(), left.vertexes.end());
				}
				auto rightIdx = connectedGroup->laneBoundaries.size() - 1;
				if (directionEqual(connectedGroup->laneBoundaries[rightIdx], connectedGroup, 2)) {
					std::reverse(right.vertexes.begin(), right.vertexes.end());
				}
			}
		}

		RdsGroup* pGroup = (RdsGroup*)queryGroup(connectedGroup->originId, pTile);
		bool leftBoundaryClipped = clipBoundary(left, leftSide, leftBoundaryDistance);
		bool rightBoundaryClipped = clipBoundary(right, rightSide, rightBoundaryDistance);
		if (leftBoundaryClipped || rightBoundaryClipped) {
			RdsGroup* newGroup = (RdsGroup*)createObject(pTile, EntityType::RDS_GROUP);
			clipGroup(pTile, pGroup, newGroup, leftSide, rightSide);
			setupSemiTransparentPoints(newGroup, semiTransparentPoints, min(leftBoundaryDistance, rightBoundaryDistance));
			deleteSemiTransparentObjects(pTile, connectedGroup, pGroup, semiTransparentPoints);
			return true;
		}

		setupSemiTransparentPoints(pGroup, semiTransparentPoints, min(leftBoundaryDistance, rightBoundaryDistance));
		deleteSemiTransparentObjects(pTile, connectedGroup, pGroup, semiTransparentPoints);
		return false;
	}

	void GroupSemiTransparentCompiler::setupSemiTransparentPoints(RdsGroup* pGroup, 
		MultiPoint3d& semiTransparentPoints, int32 semiTransparentLength) {
		// 可能跨网格
		if (pGroup != nullptr) {
			RDS::MultiPoint3d newSemiTransparentPoints;
			convert(semiTransparentPoints, newSemiTransparentPoints);
			pGroup->semiTransparentLength = semiTransparentLength;
			for (auto& point : newSemiTransparentPoints.postions) {
				pGroup->semiTransparentPoints.postions.push_back(point);
			}
		}
	}

	void GroupSemiTransparentCompiler::deleteSemiTransparentObjects(RdsTile* pTile, HadLaneGroup* pLaneGroup, RdsGroup* pGroup, MultiPoint3d& semiTransparentPoints)
	{

		auto isPointInTolerance = [](MapPoint3D64& refPt, MultiPoint3d& semiTransparentPoints, double tolerance)->bool {
			for (int idx = 0; idx < semiTransparentPoints.postions.size(); idx += 2) {
				auto& firstPt = semiTransparentPoints.postions[idx];
				auto& secondPt = semiTransparentPoints.postions[idx + 1];
				auto d1 = refPt.pos.distance(firstPt.pos);
				auto d2 = refPt.pos.distance(secondPt.pos);
				if (d1 < tolerance || d2 < tolerance) {
					return true;
				}
			}
			return false;
		};

		auto isLineInTolerance = [&](std::vector<MapPoint3D64>& refPts, MultiPoint3d& semiTransparentPoints, double tolerance)->bool {
			bool bFlag = false;
			for (auto& refPt : refPts) {
				if (isPointInTolerance(refPt, semiTransparentPoints, tolerance)) {
					bFlag = true;
					break;
				}
			}
			if (!bFlag) {
				for (auto& stPoint : semiTransparentPoints.postions) {
					size_t si, ei;
					MapPoint3D64 grappedPt = {};
					GrapPointAlgorithm::grapOrMatchNearestPoint(stPoint, refPts, grappedPt, si, ei);
					if (stPoint.pos.distance(grappedPt.pos) < tolerance) {
						bFlag = true;
						break;
					}
				}
			}
			return bFlag;
		};

		// 可能跨网格
		if (pGroup != nullptr) {
			// 半透明处几乎被导流带压盖的LA补面取消半透明,但不删除LA补面数据
			for (auto iter = pGroup->objects.begin(); iter != pGroup->objects.end();) {
				auto object = *iter;
				if (object->getEntityType() == EntityType::RDS_ROAD) {
					RdsRoad* pRoad = (RdsRoad*)object;
					if (pRoad->roadType == RdsRoad::RoadType::LA) {
						MultiLineString3d refLine;
						convert(pRoad->contour, refLine);
						Polygon3d laPolygon;
						buildRoadFacePolygon(refLine, laPolygon);
						if (isRoadFaceCoveredByDiversion(pGroup, laPolygon)) {
							// deleteObject(object, pTile);
							iter = pGroup->objects.erase(iter);
							continue;
						}
					}
				}

				iter++;
			}

			for (auto iter = pGroup->objects.begin(); iter != pGroup->objects.end();) {
				auto object = *iter;
				// 删除半透明点50米内的桥墩
				if (object->getEntityType() == EntityType::RDS_PIER) {
					RdsPier* pPier = (RdsPier*)object;
					MapPoint3D64 refPt;
					convert(pPier->position, refPt);
					if (isPointInTolerance(refPt, semiTransparentPoints, SEMI_TRANSPARENT_DISTANCE_TOLERANCE)) {
						deleteObject(object, pTile);
						iter = pGroup->objects.erase(iter);
						continue;
					}
				}

				// 删除半透明点5米内的LA补面
				if (object->getEntityType() == EntityType::RDS_ROAD) {
					RdsRoad* pRoad = (RdsRoad*)object;
					if (pRoad->roadType == RdsRoad::RoadType::LA) {
						bool deleteLaRoadFace = false;
						std::set<HadLaneGroup*> groups;
						std::set<HadLaneGroup*> semiTransparentGroups;
						for_each(pRoad->groups.begin(), pRoad->groups.end(),
							[&](auto g)->void {groups.emplace((HadLaneGroup*)g); });
						for (auto group : pRoad->groups) {
							HadLaneGroup* g = (HadLaneGroup*)group;
							if (isSemiTransparentGroup(g)) {
								semiTransparentGroups.emplace(g);
							}
							if (containsMultiDigitized(g, groups, pLaneGroup)) {
								deleteLaRoadFace = true;
							}
						}
						if (semiTransparentGroups.size() > 1) {
							deleteLaRoadFace = false;
							MultiLineString3d refLine;
							convert(pRoad->contour, refLine);
							for (auto& line : refLine.lines) {
								if (isLineInTolerance(line.vertexes, semiTransparentPoints, 5000)) {
									deleteLaRoadFace = true;
									break;
								}

								if (deleteLaRoadFace)
									break;
							}
							if (deleteLaRoadFace && getLaRoadFaceDistance(refLine, semiTransparentPoints) > 10000) {
								Polygon3d laPolygon;
								buildRoadFacePolygon(refLine, laPolygon);
								bool coveredByDiversion = false;
								for (auto group : groups) {
									RdsGroup* rdsGroup = (RdsGroup*)queryGroup(group->originId, pTile);
									coveredByDiversion = isRoadFaceCoveredByDiversion(rdsGroup, laPolygon);
									if (coveredByDiversion) {
										break;
									}
								}
								if (!coveredByDiversion) {
									deleteObject(object, pTile);
									iter = pGroup->objects.erase(iter);
									continue;
								}
							}
						}
						// 连着主路的LA补面取消半透明,但不删除LA补面数据
						else if (semiTransparentGroups.size() == 1) {
							if (deleteLaRoadFace) {
								deleteLaRoadFace = false;
								MultiLineString3d refLine;
								convert(pRoad->contour, refLine);
								for (auto& line : refLine.lines) {
									if (isLineInTolerance(line.vertexes, semiTransparentPoints, 5000)) {
										deleteLaRoadFace = true;
										break;
									}

									if (deleteLaRoadFace)
										break;
								}
							}
							if (deleteLaRoadFace && semiTransparentGroups.size() <= 1) {
								// deleteObject(object, pTile);
								iter = pGroup->objects.erase(iter);
								continue;
							}
						}
					}
				}

				// 连着主路的导流带取消半透明,但不删除导流带数据
				if (object->getEntityType() == EntityType::RDS_DIVERSION) {
					bool deleteDiversion = false;
					RdsDiversion* pDiversion = (RdsDiversion*)object;
					std::set<HadLaneGroup*> semiTransparentGroups;
					std::set<HadLaneGroup*> groups;
					for_each(pDiversion->groups.begin(), pDiversion->groups.end(), 
						[&](auto g)->void {groups.emplace((HadLaneGroup*)g); });
					for (auto group : pDiversion->groups) {
						HadLaneGroup* g = (HadLaneGroup*)group;
						if (isSemiTransparentGroup(g)) {
							semiTransparentGroups.emplace(g);
						}
						if (containsMultiDigitized(g, groups, pLaneGroup)) {
							if (!m_mainRoadSemiTransparent)
								deleteDiversion = true;
						}
					}
					if (deleteDiversion) {
						Polygon3d polygon;
						convert(pDiversion->contour, polygon);
						deleteDiversion = isLineInTolerance(polygon.vertexes, semiTransparentPoints, 5000);
					}
					if (deleteDiversion) { // && semiTransparentGroups.size() <= 1
						// deleteObject(object, pTile);
						iter = pGroup->objects.erase(iter);
						continue;
					}
				}

				iter++;
			}

		}
	}

	bool GroupSemiTransparentCompiler::containsMultiDigitized(HadLaneGroup* g, std::set<HadLaneGroup*>& groups, HadLaneGroup* semiTransparentGroup)
	{
		auto isConnectedGroup = [](HadLaneGroup* g, HadLaneGroup* pLaneGroup)->bool {
			std::vector<HadSkeleton*> previous = g->previous;
			std::vector<HadSkeleton*> next = g->next;
			auto pIter = std::find(previous.begin(), previous.end(), pLaneGroup);
			auto nIter = std::find(next.begin(), next.end(), pLaneGroup);
			if (pIter != previous.end() || nIter != next.end()) {
				if (pIter != previous.end())
					previous.erase(pIter);
				if (nIter != next.end())
					next.erase(nIter);
				if (!previous.empty() && !next.empty()) {
					return true;
				}
			}
			return false;
		};

		auto isTopologyConnected = [&](HadLaneGroup* group)->bool {
			// 检查前驱节点是否分岔路
			for (auto prev : group->previous) {
				HadLaneGroup* prevG = (HadLaneGroup*)prev;
				if (isConnectedGroup(prevG, group))
					return true;
				if (groups.count(prevG)) {
					for (auto pprev : prevG->previous) {
						HadLaneGroup* pprevG = (HadLaneGroup*)pprev;
						if (isConnectedGroup(pprevG, prevG))
							return true;
					}
				}
			}
			// 检查后继节点是否分岔路
			for (auto next : group->next) {
				HadLaneGroup* nextG = (HadLaneGroup*)next;
				if (isConnectedGroup(nextG, group))
					return true;
				if (groups.count(nextG)) {
					for (auto nnext : nextG->next) {
						HadLaneGroup* nnextG = (HadLaneGroup*)nnext;
						if (isConnectedGroup(nnextG, nextG))
							return true;
					}
				}
			}
			return false;
		};

		// 上下线分离
		if (!isMultiDigitized(g))
			return true;
		// 剔除当前车道组之后是分岔路
		if (isConnectedGroup(g, semiTransparentGroup))
			return true;
		if (isTopologyConnected(semiTransparentGroup))
			return true;
		if (isTopologyConnected(g))
			return true;

		return false;
	}

	double GroupSemiTransparentCompiler::getLaRoadFaceDistance(MultiLineString3d& refLine, MultiPoint3d& semiTransparentPoints)
	{
		if (semiTransparentPoints.postions.size() != 2 || refLine.lines.size() != 2)
			return DBL_MAX;

		auto getMinDistanceSemiTransparentPoint = [&](LineString3d& line, MapPoint3D64& point)->double {
			double minDistance = DBL_MAX;
			for (auto& postion : semiTransparentPoints.postions) {
				size_t si, ei;
				MapPoint3D64 nearestEndPoint = {};
				GrapPointAlgorithm::grapOrMatchNearestEndPoint(postion, line.vertexes, nearestEndPoint, si, ei);
				double distance = nearestEndPoint.pos.distance(postion.pos);
				if (minDistance > distance) {
					point = nearestEndPoint;
					minDistance = distance;
				}
			}
			return minDistance;
		};

		double minDistance = DBL_MAX;
		MapPoint3D64 nearestpoint;
		LineString3d* nearestLine = nullptr;
		for (auto& line : refLine.lines) {
			MapPoint3D64 point;
			auto distance = getMinDistanceSemiTransparentPoint(line, point);
			if (minDistance > distance) {
				nearestLine = &line;
				nearestpoint = point;
				minDistance = distance;
			}
		}

		size_t si, ei;
		MapPoint3D64 otherEndPoint = {};
		auto firstLine = &refLine.lines[0];
		auto secondLine = &refLine.lines[1];
		LineString3d* otherLine = firstLine == nearestLine ? secondLine : firstLine;
		GrapPointAlgorithm::grapOrMatchNearestEndPoint(nearestpoint, (*otherLine).vertexes, otherEndPoint, si, ei);
		return nearestpoint.pos.distance(otherEndPoint.pos);
	}

	bool GroupSemiTransparentCompiler::isRoadFaceCoveredByDiversion(RdsGroup* pGroup, Polygon3d& laPolygon)
	{
		if (pGroup == nullptr)
			return false;

		// LA关系面
		double laHight = 0.0;
		ClipperLib::Path laPath;
		for (auto& vertex : laPolygon.vertexes) {
			laHight += vertex.z;
			laPath << ClipperLib::IntPoint(vertex.pos.lon, vertex.pos.lat, vertex.z);
		}
		laHight /= laPolygon.vertexes.size();
		double laArea = std::abs(ClipperLib::Area(laPath));

		// 被同网格导流带全覆盖的LA补面,可以删除
		for (auto iter = pGroup->objects.begin(); iter != pGroup->objects.end(); iter++) {
			auto object = *iter;
			if (object->getEntityType() == EntityType::RDS_DIVERSION) {
				Polygon3d polygon;
				RdsDiversion* pDiversion = (RdsDiversion*)object;

				// 导流带多边形
				double fillAreaHight = 0.0;
				ClipperLib::Path fillAreaPath;
				convert(pDiversion->contour, polygon);
				for (auto& vertex : polygon.vertexes) {
					fillAreaHight += vertex.z;
					fillAreaPath << ClipperLib::IntPoint(vertex.pos.lon, vertex.pos.lat, vertex.z);
				}
				fillAreaHight /= polygon.vertexes.size();
				if (fabs(fillAreaHight - laHight) > 500) { // 判断高程差
					continue;
				}

				ClipperLib::Clipper clipper;
				ClipperLib::PolyTree polyTree;
				clipper.AddPath(fillAreaPath, ClipperLib::ptSubject, true);
				clipper.AddPath(laPath, ClipperLib::ptClip, true);
				clipper.Execute(ClipperLib::ctIntersection, polyTree, ClipperLib::pftEvenOdd);
				if (polyTree.Total()) {
					for (auto polyNode : polyTree.Childs) {
						double polyArea = std::abs(ClipperLib::Area(polyNode->Contour));
						double polyLaAreaRate = polyArea / laArea;
						if (polyLaAreaRate > 0.8) {
							return true;
						}
					}
				}
			}
		}
		return false;
	}

	void GroupSemiTransparentCompiler::buildRoadFacePolygon(MultiLineString3d refLine, Polygon3d& polygon)
	{
		polygon.vertexes.clear();
		MapPoint3D64 prevVertex = {};
		LineString3d& leftSide = refLine.lines[0];
		LineString3d& rightSide = refLine.lines[1];
		for_each(leftSide.vertexes.begin(), leftSide.vertexes.end(), [&](MapPoint3D64& vertex) {
			if (!floatEqual(vertex.pos.distanceSquare(prevVertex.pos), 0)) {
				polygon.vertexes.push_back(vertex);
				prevVertex = vertex;
			}
		});
		for_each(rightSide.vertexes.rbegin(), rightSide.vertexes.rend(), [&](MapPoint3D64& vertex) {
			if (!floatEqual(vertex.pos.distanceSquare(prevVertex.pos), 0)) {
				polygon.vertexes.push_back(vertex);
				prevVertex = vertex;
			}
		});

		// 闭环
		MapPoint3D64& startPt = polygon.vertexes.front();
		MapPoint3D64& endPt = polygon.vertexes.back();
		if (!floatEqual(startPt.pos.distanceSquare(endPt.pos), 0)) {
			polygon.vertexes.push_back(startPt);
		}
	}

	void GroupSemiTransparentCompiler::clipGroup(RdsTile* pTile, 
		RdsGroup* pGroup, RdsGroup* newGroup, LineString3d& leftSide, LineString3d& rightSide)
	{
		Polygon3d polygon;
		makeLaneGroupPolygon(leftSide.vertexes, rightSide.vertexes, polygon);
		std::vector<RdsObject*>	objects = pGroup->objects;
		for (auto object : objects) {
			auto type = object->getEntityType();
			switch (type)
			{
			case EntityType::UNKNOWN:
				break;
			// point
			case EntityType::RDS_PIER:
				clipPier(pTile, pGroup, newGroup, (RdsPier*)object, polygon);
				break;
			case EntityType::RDS_SPEEDLIMITBOARD:
				clipSpeedLimitBoard(pTile, pGroup, newGroup, (RdsSpeedLimitBoard*)object, polygon);
				break;

			// multipoint
			case EntityType::RDS_TOLL:
				clipToll(pTile, pGroup, newGroup, (RdsToll*)object, polygon);
				break;

			// line
			case EntityType::RDS_LINE:
				clipLine(pTile, pGroup, newGroup, (RdsLine*)object, leftSide, rightSide);
				break;
			case EntityType::RDS_GUARDRAIL:
				clipGuardrail(pTile, pGroup, newGroup, (RdsGuardrail*)object, leftSide, rightSide);
				break;

			// multiline
			case EntityType::RDS_ROAD:
				clipRoad(pTile, pGroup, newGroup, (RdsRoad*)object, leftSide, rightSide);
				break;
			case EntityType::RDS_GREENBELT:
				clipGreenbelt(pTile, pGroup, newGroup, (RdsGreenbelt*)object, leftSide, rightSide);
				break;
			case EntityType::RDS_TUNNEL:
				clipTunnel(pTile, pGroup, newGroup, (RdsTunnel*)object, leftSide, rightSide);
				break;

			// polygon
			case EntityType::RDS_TEXT:
				clipText(pTile, pGroup, newGroup, (RdsText*)object, polygon);
				break;
			case EntityType::RDS_MARKING:
				clipMarking(pTile, pGroup, newGroup, (RdsMarking*)object, polygon);
				break;
			case EntityType::RDS_DIVERSION:
				clipDiversion(pTile, pGroup, newGroup, (RdsDiversion*)object, polygon);
				break;
			case EntityType::RDS_INTERSECTION:
				clipIntersection(pTile, pGroup, newGroup, (RdsIntersection*)object, polygon);
				break;
			default:
				break;
			}
		}
	}	

	void GroupSemiTransparentCompiler::clipPier(RdsTile* pTile, RdsGroup* pGroup, RdsGroup* newGroup, RdsPier* pPier, Polygon3d& polygon)
	{
		UNREFERENCED_PARAMETER(pTile);

		MapPoint3D64 refPt;
		convert(pPier->position, refPt);
		if (GrapPointAlgorithm::isPointInPolygon(refPt, polygon.vertexes)) {
			newGroup->objects.push_back(pPier);
			pGroup->objects.erase(std::find(pGroup->objects.begin(), pGroup->objects.end(), pPier));
		}
	}

	void GroupSemiTransparentCompiler::clipSpeedLimitBoard(RdsTile* pTile, RdsGroup* pGroup, RdsGroup* newGroup, RdsSpeedLimitBoard* pSpeedLimitBoard, Polygon3d& polygon)
	{
		UNREFERENCED_PARAMETER(pTile);

		MapPoint3D64 refPt;
		convert(pSpeedLimitBoard->position, refPt);
		if (GrapPointAlgorithm::isPointInPolygon(refPt, polygon.vertexes)) {
			newGroup->objects.push_back(pSpeedLimitBoard);
			pGroup->objects.erase(std::find(pGroup->objects.begin(), pGroup->objects.end(), pSpeedLimitBoard));
		}
	}

	void GroupSemiTransparentCompiler::clipToll(RdsTile* pTile, RdsGroup* pGroup, RdsGroup* newGroup, RdsToll* pToll, Polygon3d& polygon)
	{
		UNREFERENCED_PARAMETER(pTile);

		for (auto position : pToll->positions.postions) {
			MapPoint3D64 refPt;
			convert(position, refPt);
			if (GrapPointAlgorithm::isPointInPolygon(refPt, polygon.vertexes)) {
				newGroup->objects.push_back(pToll);
				pGroup->objects.erase(std::find(pGroup->objects.begin(), pGroup->objects.end(), pToll));
				break;
			}
		}
	}

	void GroupSemiTransparentCompiler::clipLine(RdsTile* pTile, RdsGroup* pGroup,
		RdsGroup* newGroup, RdsLine* pLine, LineString3d& leftSide, LineString3d& rightSide)
	{
		UNREFERENCED_PARAMETER(pGroup);
		UNREFERENCED_PARAMETER(rightSide);

		size_t si, ei;
		MapPoint3D64 grappedPt = {};
		LineString3d srcLine, dstLine;
		auto& line = pLine->location;
		
		convert(line, srcLine);
		if (clipLineString(leftSide, srcLine, dstLine)) {
			// 新建line
			RdsLine* newLine = (RdsLine*)createObject(pTile, EntityType::RDS_LINE);
			convert(dstLine, newLine->location);
			newLine->width = pLine->width;
			newLine->lineType = pLine->lineType;
			newLine->color = pLine->color;
			newLine->side = pLine->side;
			newGroup->objects.push_back(newLine);

			// 更新旧line坐标
			convert(srcLine, line);
		}
	}

	void GroupSemiTransparentCompiler::clipGuardrail(RdsTile* pTile, RdsGroup* pGroup, RdsGroup* newGroup, RdsGuardrail* pGuardrail, LineString3d& leftSide, LineString3d& rightSide)
	{
		UNREFERENCED_PARAMETER(pGroup);
		UNREFERENCED_PARAMETER(rightSide);

		size_t si, ei;
		MapPoint3D64 grappedPt = {};
		LineString3d srcLine, dstLine;
		auto& line = pGuardrail->location;

		convert(line, srcLine);
		if (clipLineString(leftSide, srcLine, dstLine)) {
			// 新建guardrail
			RdsGuardrail* newGuardrail = (RdsGuardrail*)createObject(pTile, EntityType::RDS_GUARDRAIL);
			convert(dstLine, newGuardrail->location);
			newGuardrail->railType = pGuardrail->railType;
			newGroup->objects.push_back(newGuardrail);

			// 更新旧guardrail坐标
			convert(srcLine, line);
		}
	}

	void GroupSemiTransparentCompiler::clipRoad(RdsTile* pTile, RdsGroup* pGroup,
		RdsGroup* newGroup, RdsRoad* pRoad, LineString3d& leftSide, LineString3d& rightSide)
	{
		UNREFERENCED_PARAMETER(pGroup);
		auto& leftEndPt = leftSide.vertexes.back();
		if (pRoad->contour.lines.size() != 2) {
			return;
		}

		size_t si, ei;
		MapPoint3D64 grappedPt = {};
		LineString3d srcLine0, dstLine0;
		LineString3d srcLine1, dstLine1;
		auto& line0 = pRoad->contour.lines[0];
		auto& line1 = pRoad->contour.lines[1];

		convert(line0, srcLine0);
		GrapPointAlgorithm::grapOrMatchNearestPoint(leftEndPt, srcLine0.vertexes, grappedPt, si, ei);
		double distance0 = leftEndPt.pos.distance(grappedPt.pos);

		convert(line1, srcLine1);
		GrapPointAlgorithm::grapOrMatchNearestPoint(leftEndPt, srcLine1.vertexes, grappedPt, si, ei);
		double distance1 = leftEndPt.pos.distance(grappedPt.pos);
		bool leftSideClipped = false, rightSideClipped = false;
		if (distance0 < distance1)
		{
			leftSideClipped = clipLineString(leftSide, srcLine0, dstLine0);
			rightSideClipped = clipLineString(rightSide, srcLine1, dstLine1);
		}
		else
		{
			leftSideClipped = clipLineString(leftSide, srcLine1, dstLine1);
			rightSideClipped = clipLineString(rightSide, srcLine0, dstLine0);
		}

		// 新建road
		if (leftSideClipped && rightSideClipped) {
			RdsRoad* newRoad = (RdsRoad*)createObject(pTile, EntityType::RDS_ROAD);
			RDS::LineString3d lineLeftSide;
			convert(dstLine0, lineLeftSide);
			RDS::LineString3d lineRightSide;
			convert(dstLine1, lineRightSide);
			newRoad->contour.lines.resize(2);
			newRoad->contour.lines[0] = lineLeftSide;
			newRoad->contour.lines[1] = lineRightSide;
			newGroup->objects.push_back(newRoad);

			// 更新旧road坐标
			convert(srcLine0, line0);
			convert(srcLine1, line1);
		}
	}

	void GroupSemiTransparentCompiler::clipGreenbelt(RdsTile* pTile, RdsGroup* pGroup, RdsGroup* newGroup, RdsGreenbelt* pGreenbelt, LineString3d& leftSide, LineString3d& rightSide)
	{
		UNREFERENCED_PARAMETER(pGroup);
		auto& leftEndPt = leftSide.vertexes.back();
		if (pGreenbelt->contour.lines.size() != 2) {
			return;
		}

		size_t si, ei;
		MapPoint3D64 grappedPt = {};
		LineString3d srcLine0, dstLine0;
		LineString3d srcLine1, dstLine1;
		auto& line0 = pGreenbelt->contour.lines[0];
		auto& line1 = pGreenbelt->contour.lines[1];

		convert(line0, srcLine0);
		GrapPointAlgorithm::grapOrMatchNearestPoint(leftEndPt, srcLine0.vertexes, grappedPt, si, ei);
		double distance0 = leftEndPt.pos.distance(grappedPt.pos);

		convert(line1, srcLine1);
		GrapPointAlgorithm::grapOrMatchNearestPoint(leftEndPt, srcLine1.vertexes, grappedPt, si, ei);
		double distance1 = leftEndPt.pos.distance(grappedPt.pos);
		bool leftSideClipped = false, rightSideClipped = false;
		if (distance0 < distance1)
		{
			leftSideClipped = clipLineString(leftSide, srcLine0, dstLine0);
			rightSideClipped = clipLineString(rightSide, srcLine1, dstLine1);
		}
		else
		{
			leftSideClipped = clipLineString(leftSide, srcLine1, dstLine1);
			rightSideClipped = clipLineString(rightSide, srcLine0, dstLine0);
		}

		// 新建greenbelt
		if (leftSideClipped && rightSideClipped) {
			RdsGreenbelt* newGreenbelt = (RdsGreenbelt*)createObject(pTile, EntityType::RDS_GREENBELT);
			RDS::LineString3d lineLeftSide;
			convert(dstLine0, lineLeftSide);
			RDS::LineString3d lineRightSide;
			convert(dstLine1, lineRightSide);
			newGreenbelt->contour.lines.resize(2);
			newGreenbelt->contour.lines[0] = lineLeftSide;
			newGreenbelt->contour.lines[1] = lineRightSide;
			newGroup->objects.push_back(newGreenbelt);

			// 更新旧greenbelt坐标
			convert(srcLine0, line0);
			convert(srcLine1, line1);
		}
	}

	void GroupSemiTransparentCompiler::clipTunnel(RdsTile* pTile, RdsGroup* pGroup, RdsGroup* newGroup, RdsTunnel* pTunnel, LineString3d& leftSide, LineString3d& rightSide)
	{
		UNREFERENCED_PARAMETER(pGroup);
		auto& leftEndPt = leftSide.vertexes.back();
		if (pTunnel->contour.lines.size() != 2) {
			return;
		}

		size_t si, ei;
		MapPoint3D64 grappedPt = {};
		LineString3d srcLine0, dstLine0;
		LineString3d srcLine1, dstLine1;
		auto& line0 = pTunnel->contour.lines[0];
		auto& line1 = pTunnel->contour.lines[1];

		convert(line0, srcLine0);
		GrapPointAlgorithm::grapOrMatchNearestPoint(leftEndPt, srcLine0.vertexes, grappedPt, si, ei);
		double distance0 = leftEndPt.pos.distance(grappedPt.pos);

		convert(line1, srcLine1);
		GrapPointAlgorithm::grapOrMatchNearestPoint(leftEndPt, srcLine1.vertexes, grappedPt, si, ei);
		double distance1 = leftEndPt.pos.distance(grappedPt.pos);
		bool leftSideClipped = false, rightSideClipped = false;
		if (distance0 < distance1)
		{
			leftSideClipped = clipLineString(leftSide, srcLine0, dstLine0);
			rightSideClipped = clipLineString(rightSide, srcLine1, dstLine1);
		}
		else
		{
			leftSideClipped = clipLineString(leftSide, srcLine1, dstLine1);
			rightSideClipped = clipLineString(rightSide, srcLine0, dstLine0);
		}

		// 新建tunnel
		if (leftSideClipped && rightSideClipped) {
			RdsTunnel* newTunnel = (RdsTunnel*)createObject(pTile, EntityType::RDS_TUNNEL);
			RDS::LineString3d lineLeftSide;
			convert(dstLine0, lineLeftSide);
			RDS::LineString3d lineRightSide;
			convert(dstLine1, lineRightSide);
			newTunnel->contour.lines.resize(2);
			newTunnel->contour.lines[0] = lineLeftSide;
			newTunnel->contour.lines[1] = lineRightSide;
			newTunnel->height = pTunnel->height;
			newTunnel->thickness = pTunnel->thickness;
			newTunnel->tunnelType = pTunnel->tunnelType;
			newGroup->objects.push_back(newTunnel);

			// 更新旧tunnel坐标
			convert(srcLine0, line0);
			convert(srcLine1, line1);
		}
	}

	void GroupSemiTransparentCompiler::clipText(RdsTile* pTile, RdsGroup* pGroup, RdsGroup* newGroup, RdsText* pText, Polygon3d& polygon)
	{
		UNREFERENCED_PARAMETER(pTile);

		Polygon3d refPolygon;
		convert(pText->contour, refPolygon);
		if (hasIntersect(refPolygon.vertexes, polygon.vertexes)) {
			newGroup->objects.push_back(pText);
			pGroup->objects.erase(std::find(pGroup->objects.begin(), pGroup->objects.end(), pText));
		}
	}

	void GroupSemiTransparentCompiler::clipMarking(RdsTile* pTile, RdsGroup* pGroup, RdsGroup* newGroup, RdsMarking* pMarking, Polygon3d& polygon)
	{
		UNREFERENCED_PARAMETER(pTile);

		Polygon3d refPolygon;
		convert(pMarking->contour, refPolygon);
		if (hasIntersect(refPolygon.vertexes, polygon.vertexes)) {
			newGroup->objects.push_back(pMarking);
			pGroup->objects.erase(std::find(pGroup->objects.begin(), pGroup->objects.end(), pMarking));
		}
	}

	void GroupSemiTransparentCompiler::clipDiversion(RdsTile* pTile, RdsGroup* pGroup, RdsGroup* newGroup, RdsDiversion* pDiversion, Polygon3d& polygon)
	{
		UNREFERENCED_PARAMETER(pTile);

		Polygon3d refPolygon;
		convert(pDiversion->contour, refPolygon);
		if (hasIntersect(refPolygon.vertexes, polygon.vertexes)) {
			newGroup->objects.push_back(pDiversion);
			pGroup->objects.erase(std::find(pGroup->objects.begin(), pGroup->objects.end(), pDiversion));
		}
	}

	void GroupSemiTransparentCompiler::clipIntersection(RdsTile* pTile, RdsGroup* pGroup, RdsGroup* newGroup, RdsIntersection* pIntersection, Polygon3d& polygon)
	{
		UNREFERENCED_PARAMETER(pTile);

		Polygon3d refPolygon;
		convert(pIntersection->contour, refPolygon);
		if (hasIntersect(refPolygon.vertexes, polygon.vertexes)) {
			newGroup->objects.push_back(pIntersection);
			pGroup->objects.erase(std::find(pGroup->objects.begin(), pGroup->objects.end(), pIntersection));
		}
	}

	bool GroupSemiTransparentCompiler::clipLineString(LineString3d& refLine, LineString3d& src, LineString3d& dst)
	{
		auto& points = src.vertexes;
		auto& nearbyPoints = refLine.vertexes;
		auto grapPoint = [](MapPoint3D64& point, std::vector<MapPoint3D64>& nearbyPoints, MapPoint3D64& nearbyGrappedPt, size_t& si, size_t& ei) -> bool {
			if (GrapPointAlgorithm::grapPoint(point, nearbyPoints, nearbyGrappedPt, si, ei, 800) &&
				point.pos.distance(nearbyGrappedPt.pos) < BOUNDARY_DISTANCE_TOLERANCE &&
				fabs(point.z - nearbyGrappedPt.z) < BOUNDARY_ZVALUE_TOLERANCE) {
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
				break;
			}
		};

		for (int idx = points.size() - 1; idx >= 0; idx--) {
			size_t si, ei;
			MapPoint3D64& point = points[idx];
			MapPoint3D64 nearbyGrappedPt = {};
			if (grapPoint(point, nearbyPoints, nearbyGrappedPt, si, ei)) {
				endIdx = idx;
				break;
			}
		};

		// 抓不到点,不用切割
		if (startIdx == -1 && endIdx == -1) {
			return false;
		}

		// 补充切割点坐标
		size_t si, ei;
		MapPoint3D64 clippedPt = {};
		auto& refEndPt = refLine.vertexes.back();
		if (endIdx == points.size() - 1) { // forward
			for (int idx = startIdx; idx <= endIdx; idx++) {
				MapPoint3D64& point = points[idx];
				dst.vertexes.push_back(point);
			}

			int dstSize = dst.vertexes.size();
			bool clipped = grapPoint(refEndPt, points, clippedPt, si, ei);
			src.vertexes.resize(src.vertexes.size() - dstSize);
			if (clipped) {
				src.vertexes.push_back(clippedPt);
				dst.vertexes.emplace(dst.vertexes.begin(), clippedPt);
			}
		}
		else if (startIdx == 0) {
			auto iter = points.begin();
			for (; iter <= points.begin() + endIdx; iter++) {
				MapPoint3D64& point = *iter;
				dst.vertexes.push_back(point);
			}

			bool clipped = grapPoint(refEndPt, points, clippedPt, si, ei);
			src.vertexes.assign(iter, points.end());
			if (clipped) {
				dst.vertexes.push_back(clippedPt);
				src.vertexes.emplace(src.vertexes.begin(), clippedPt);
			}
		} else {
			//printInfo("clipLineString::%lld,%lld", refEndPt.pos.lon, refEndPt.pos.lat);
		}
		return true;
	}

	bool GroupSemiTransparentCompiler::hasIntersect(std::vector<MapPoint3D64>& points1, std::vector<MapPoint3D64>& points2)
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

	void GroupSemiTransparentCompiler::mergeGroupBoundary(std::vector<HadLaneGroup*>& connectedGroups, LineString3d& leftSide, LineString3d& rightSide)
	{
		auto mergeBoundary = [](LineString3d& src, LineString3d& dst) {
			MapPoint3D64 prevVertex = {};
			if (!dst.vertexes.empty())
				prevVertex = dst.vertexes.back();
			// 人工画的线会出现前一条边和后一条边重合的情况,需剔除这种情况
			for_each(src.vertexes.begin(), src.vertexes.end(), [&](MapPoint3D64& vertex) {
				const float CONNECT_POINT_IN_EPSILON = 2000.f;  // ≈2m
				double distance = vertex.pos.distance(prevVertex.pos);
				if (!floatEqualWithEpsilon(distance, 0, CONNECT_POINT_IN_EPSILON)) {
					dst.vertexes.push_back(vertex);
					prevVertex = vertex;
				}
			});
		};

		if (!connectedGroups.empty()) {
			auto connectedGroup = connectedGroups[0];
			getLaneGroupBoundary(connectedGroup, leftSide, rightSide);
		}
		for (auto idx = 1; idx < connectedGroups.size(); idx++) {
			LineString3d left, right;
			auto connectedGroup = connectedGroups[idx];
			getLaneGroupBoundary(connectedGroup, left, right);

			mergeBoundary(left, leftSide);
			mergeBoundary(right, rightSide);
		}
	}

	MultiPoint3d GroupSemiTransparentCompiler::constructSemiTransparentPoints(HadLaneGroup* pLaneGroup)
	{
		LineString3d leftSide, rightSide;
		getLaneGroupBoundary(pLaneGroup, leftSide, rightSide);

		MultiPoint3d semiTransparentPoints;
		auto previous = getLaneGroupSkeletons(pLaneGroup->previous);
		auto next = getLaneGroupSkeletons(pLaneGroup->next);
		if (previous.empty() && next.empty()) {
			semiTransparentPoints.postions.push_back(*leftSide.vertexes.begin());
			semiTransparentPoints.postions.push_back(*rightSide.vertexes.begin());
			semiTransparentPoints.postions.push_back(*leftSide.vertexes.rbegin());
			semiTransparentPoints.postions.push_back(*rightSide.vertexes.rbegin());
		} else if (previous.empty()) {
			semiTransparentPoints.postions.push_back(*leftSide.vertexes.begin());
			semiTransparentPoints.postions.push_back(*rightSide.vertexes.begin());
		} else {
			semiTransparentPoints.postions.push_back(*leftSide.vertexes.rbegin());
			semiTransparentPoints.postions.push_back(*rightSide.vertexes.rbegin());
		}
		return semiTransparentPoints;
	}

	std::vector<HadLaneGroup*> GroupSemiTransparentCompiler::getTopoGroups(HadLaneGroup* pLaneGroup, bool forward)
	{
		auto connectedLaneGroup = [&](HadLaneGroup* pCurrent, HadLaneGroup* pNext)->HadLaneGroup* {
			HadLaneGroup* pGroup = (pCurrent != pLaneGroup) ? pCurrent : pNext;
			if (pGroup->roadBoundaries.size() != 2 && pGroup->laneBoundaries.size() < 2) {
				return nullptr;
			}
			if (!m_mainRoadSemiTransparent) {
				if (!isMultiDigitized(pGroup)) {
					return nullptr;
				}
			}

			// 检测道路边界是否相连
			// 这里与TopoBuilder不一样的地方是这里需要两条边界都是相连的
			// 考虑到有些是共享道路边界,此时是iItem->startNode与jItem->endNode相连
			// 10893644,3431005车道距离小于2m,因此改为1.5米
			const float CONNECT_POINT_EPSILON = 1500.f;  // ≈1.5m
			for (auto& iItem : pCurrent->roadBoundaries)
			{
				bool isConnectedLaneGroup = false;
				auto currentStartPt = iItem->startNode->position;
				auto currentEndPt = iItem->endNode->position;
				if (directionEqual(iItem, pCurrent, 3)) {
					currentStartPt = iItem->endNode->position;
					currentEndPt = iItem->startNode->position;
				}
				for (auto& jItem : pNext->roadBoundaries)
				{
					auto nextStartPt = jItem->startNode->position;
					auto nextEndPt = jItem->endNode->position;
					if (directionEqual(jItem, pNext, 3)) {
						nextStartPt = jItem->endNode->position;
						nextEndPt = jItem->startNode->position;
					}
					if (iItem->endNode->originId == jItem->startNode->originId ||
						iItem->startNode->originId == jItem->endNode->originId ||
						currentEndPt.pos.distance(nextStartPt.pos) < CONNECT_POINT_EPSILON ||
						currentStartPt.pos.distance(nextEndPt.pos) < CONNECT_POINT_EPSILON) {
						isConnectedLaneGroup = true;
						break;
					}
				}
				if (!isConnectedLaneGroup)
					return nullptr;
			}
			if (!pCurrent->roadBoundaries.empty() && !pNext->roadBoundaries.empty()) {
				return pGroup;
			}

			// 检测车道边界是否相连
			for (int i = 0; i < pCurrent->laneBoundaries.size(); i++)
			{
				bool isConnectedLaneGroup = false;
				if (i != 0 && i != pCurrent->laneBoundaries.size() - 1)
					continue;
				auto& iItem = pCurrent->laneBoundaries[i];
				auto currentStartPt = iItem->startNode->position;
				auto currentEndPt = iItem->endNode->position;
				if (directionEqual(iItem, pCurrent, 3)) {
					currentStartPt = iItem->endNode->position;
					currentEndPt = iItem->startNode->position;
				}
				for (int j = 0; j < pNext->laneBoundaries.size(); j++)
				{
					if (j != 0 && j != pNext->laneBoundaries.size() - 1)
						continue;
					auto& jItem = pNext->laneBoundaries[j];
					auto nextStartPt = jItem->startNode->position;
					auto nextEndPt = jItem->endNode->position;
					if (directionEqual(jItem, pNext, 3)) {
						nextStartPt = jItem->endNode->position;
						nextEndPt = jItem->startNode->position;
					}
					if (iItem->endNode->originId == jItem->startNode->originId ||
						iItem->startNode->originId == jItem->endNode->originId ||
						currentEndPt.pos.distance(nextStartPt.pos) < CONNECT_POINT_EPSILON ||
						currentStartPt.pos.distance(nextEndPt.pos) < CONNECT_POINT_EPSILON) {
						isConnectedLaneGroup = true;
						break;
					}
				}
				if (!isConnectedLaneGroup)
					return nullptr;
			}

			return pGroup;
		};

		std::vector<HadLaneGroup*> retGroups;
		// pLaneGroup->previous.empty()
		if (forward) {
			for (auto pGroup : getLaneGroupSkeletons(pLaneGroup->next)) {
				auto retGroup = connectedLaneGroup(pLaneGroup, (HadLaneGroup*)pGroup);
				if (retGroup != nullptr && !retGroup->inIntersection) {
					retGroups.push_back(retGroup);
				}
			}
		}
		// backward
		else {
			for (auto pGroup : getLaneGroupSkeletons(pLaneGroup->previous)) {
				auto retGroup = connectedLaneGroup((HadLaneGroup*)pGroup, pLaneGroup);
				if (retGroup != nullptr && !retGroup->inIntersection) {
					retGroups.push_back(retGroup);
				}
			}
		}
		return retGroups;
	}

	double GroupSemiTransparentCompiler::calcBoundaryDistance(std::vector<MapPoint3D64>& points)
	{
		double distance = 0;
		for (int idx = 0; idx < points.size() - 1; idx++) {
			auto& currPt = points[idx];
			auto& nextPt = points[idx + 1];
			distance += currPt.pos.distance(nextPt.pos);
		}
		return distance;
	}

	bool GroupSemiTransparentCompiler::isSelfIntersect(LineString3d& lineString)
	{
		static const double tolerance = 10;
		std::vector<MapPoint3D64>& lineStringVertexes = lineString.vertexes;
		for (auto iter = lineStringVertexes.begin(); iter != lineStringVertexes.end(); iter++) {
			if (iter + 1 == lineStringVertexes.end() || iter + 2 == lineStringVertexes.end()) {
				break;
			}

			std::vector<MapPoint3D64> points;
			MapPoint3D64& pointStart = *iter;
			MapPoint3D64& pointEnd = *(iter + 1);
			points.assign(iter + 2, lineStringVertexes.end());
			
			size_t si, ei;
			MapPoint3D64 intersectPt = {};
			if (PolylineIntersector::intersect(pointStart, pointEnd, points, tolerance, intersectPt, si, ei)) {
				return true;
			}
		}
		return false;
	}

}
