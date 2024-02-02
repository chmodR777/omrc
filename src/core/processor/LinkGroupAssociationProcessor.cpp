#include "stdafx.h"
#include <algorithm>
#include "math3d/vector_math.h"
#include "../framework/SpatialSeacher.h"
#include "LinkGroupAssociationProcessor.h"
#include "algorithm/grap_point_algorithm.h"
#define DB_HAD_APPLY_PA_REFERENCE	 -99
#define HAD_GRID_NDS_LEVEL	13

namespace OMDB
{
	void LinkGroupAssociationProcessor::process(DbMesh* const pMesh, HadGrid* pGrid)
	{
		UNREFERENCED_PARAMETER(pMesh);
		UNREFERENCED_PARAMETER(pGrid);
	}

	void LinkGroupAssociationProcessor::processRelation(DbMesh* const pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby)
	{
		for (auto lga : pMesh->query(RecordType::DB_HAD_LG_ASSOCIATION))
		{
			DbLgAssociation* plga = (DbLgAssociation*)lga;
			HadLaneGroupAssociation* pAssociation = (HadLaneGroupAssociation*)pGrid->alloc(ElementType::HAD_LG_ASSOCIATIOIN);
			pAssociation->originId = plga->uuid;
			for (auto& relLga : plga->relLgAssociations) {
				HadLaneGroup* first = (HadLaneGroup*)pGrid->query(relLga.firstLgLinkId, ElementType::HAD_LANE_GROUP);
				if (first == nullptr) {
					first = (HadLaneGroup*)queryNearby(pGrid, nearby, relLga.firstLgLinkId, ElementType::HAD_LANE_GROUP);
				}
				HadLaneGroup* second = (HadLaneGroup*)pGrid->query(relLga.secondLgLinkId, ElementType::HAD_LANE_GROUP);
				if (second == nullptr) {
					second = (HadLaneGroup*)queryNearby(pGrid, nearby, relLga.secondLgLinkId, ElementType::HAD_LANE_GROUP);
				}

				HadRelLaneGroupAssociation relLgAssociation;
				constructLaneGroupAssociation(relLgAssociation, pAssociation->originId, relLga.directType, first, second);
				pAssociation->relLaneGroupAssociations.push_back(relLgAssociation);
				if (first != nullptr) {
					first->associations.push_back(pAssociation);
				}
				if (second != nullptr) {
					second->associations.push_back(pAssociation);
				}
			}

			pGrid->insert(pAssociation->originId, pAssociation);
			m_maxGroupId = max(m_maxGroupId, pAssociation->originId);
		}

		// 通过导流区压盖生成LA关系
		generateLaneGroupAssociations(pGrid, *nearby);
	}

	void LinkGroupAssociationProcessor::generateLaneGroupAssociations(HadGrid* const pGrid,const std::vector<HadGrid*>& nearby)
	{
		// LA关系
		std::vector<HadRelLaneGroupAssociation*> allLaneGroupAssociations;
		std::map<int64, std::vector<HadRelLaneGroupAssociation*>> groupedLaneGroupAssociations;
		groupLaneGroupAssociations(pGrid, nearby, allLaneGroupAssociations, groupedLaneGroupAssociations);
		auto inIntersection = [](HadLaneGroup* pGroup)->bool {
			if (pGroup && pGroup->inIntersection)
				return true;
			return false;
		};

		auto checkLaneTopology = [&](HadRelLaneGroupAssociation* lgAssociation, HadRelLaneGroupAssociation* generatedLgAssociation)->bool {
			std::vector<HadSkeleton*>& firstPrevious = lgAssociation->firstLaneBoundary->previous;
			std::vector<HadSkeleton*>& firstNext = lgAssociation->firstLaneBoundary->next;
			std::vector<HadSkeleton*>& secondPrevious = lgAssociation->secondLaneBoundary->previous;
			std::vector<HadSkeleton*>& secondNext = lgAssociation->secondLaneBoundary->next;
			if (std::find(firstPrevious.begin(), firstPrevious.end(), generatedLgAssociation->firstLaneBoundary) == firstPrevious.end() && 
				std::find(firstNext.begin(), firstNext.end(), generatedLgAssociation->firstLaneBoundary) == firstNext.end() &&
				std::find(secondPrevious.begin(), secondPrevious.end(), generatedLgAssociation->firstLaneBoundary) == secondPrevious.end() &&
				std::find(secondNext.begin(), secondNext.end(), generatedLgAssociation->firstLaneBoundary) == secondNext.end()) {
				return false;
			}

			if (std::find(firstPrevious.begin(), firstPrevious.end(), generatedLgAssociation->secondLaneBoundary) == firstPrevious.end() &&
				std::find(firstNext.begin(), firstNext.end(), generatedLgAssociation->secondLaneBoundary) == firstNext.end() &&
				std::find(secondPrevious.begin(), secondPrevious.end(), generatedLgAssociation->secondLaneBoundary) == secondPrevious.end() &&
				std::find(secondNext.begin(), secondNext.end(), generatedLgAssociation->secondLaneBoundary) == secondNext.end()) {
				return false;
			}
			// 过滤路口生成的车道组
			if (inIntersection(generatedLgAssociation->first) || inIntersection(generatedLgAssociation->second)) {
				return false;
			}

			return true;
		};

		// 按拓扑连接LA关系
		auto connectLaneGroupAssociations = [&](HadFillArea* pFillArea, 
				HadRelLaneGroupAssociation* lgAssociation, 
				std::vector<HadRelLaneGroupAssociation*>& laneGroupAssociations) {
			// previous
			HadRelLaneGroupAssociation* previousLgAssociation = lgAssociation;
			while (previousLgAssociation->first->previous.size() >= 1) {
				bool containsPreviousLgAssociation = false;
				for (auto& pFirstLg : previousLgAssociation->first->previous) {
					HadLaneGroup* firstLaneGroup = (HadLaneGroup*)pFirstLg;
					int directType = previousLgAssociation->directType;
					std::vector<HadSkeleton*>& previous = directType == 2
						? previousLgAssociation->second->previous : previousLgAssociation->second->next;
					for (auto& pSecondLg : previous) {
						HadLaneGroup* secondLaneGroup = (HadLaneGroup*)pSecondLg;
						if (firstLaneGroup == secondLaneGroup) {
							break;
						}

						// 过滤循环LA关系
						auto laIter = findLgAssociationIter(previousLgAssociation, firstLaneGroup, secondLaneGroup,
							pGrid, nearby, allLaneGroupAssociations, groupedLaneGroupAssociations);
						if (laIter != allLaneGroupAssociations.end() && (*laIter)->avgDistance == 0) {
							break;
						}

						if (lgAssociation == *laIter) {
							lgAssociation->applyDiversion = true;
							lgAssociation->overedFillAreas.clear();
							// pFillArea->applyDiversion = false;
							pFillArea->isGore = false;
							containsPreviousLgAssociation = false;
							break;
						}

						if (laIter != allLaneGroupAssociations.end() && !isXUTurnLA(previousLgAssociation, *laIter)) {
							previousLgAssociation = (*laIter);
							if (previousLgAssociation->first == secondLaneGroup && previousLgAssociation->second == firstLaneGroup) {
								// 保持往前寻找,不然会陷入死循环
								std::swap(previousLgAssociation->first, previousLgAssociation->second);
								std::swap(previousLgAssociation->firstLaneBoundary, previousLgAssociation->secondLaneBoundary);
							}
							containsPreviousLgAssociation = true;
							laneGroupAssociations.emplace(laneGroupAssociations.begin(), previousLgAssociation);
						} else if (laIter == allLaneGroupAssociations.end()) {
							// 断头路不生成LA关系
							HadRelLaneGroupAssociation relLgAssociation;
							HadLaneGroupAssociation* pAssociation = nullptr;
							if (isNotSemiTransparentGroup(previousLgAssociation->first) && isNotSemiTransparentGroup(previousLgAssociation->second)) {
								constructLaneGroupAssociation(relLgAssociation, 0, directType, firstLaneGroup, secondLaneGroup);
								pAssociation = generateLaneGroupAssociation(pGrid, pFillArea, relLgAssociation);
							}
							if (pAssociation) {
								if (checkLaneTopology(previousLgAssociation, &pAssociation->relLaneGroupAssociations[0])) {
									previousLgAssociation = &pAssociation->relLaneGroupAssociations[0];
									if (previousLgAssociation->first == secondLaneGroup && previousLgAssociation->second == firstLaneGroup) {
										// 保持往前寻找,不然会陷入死循环
										std::swap(previousLgAssociation->first, previousLgAssociation->second);
										std::swap(previousLgAssociation->firstLaneBoundary, previousLgAssociation->secondLaneBoundary);
									}
									allLaneGroupAssociations.push_back(previousLgAssociation);
									firstLaneGroup->associations.push_back(pAssociation);
									secondLaneGroup->associations.push_back(pAssociation);
									laneGroupAssociations.emplace(laneGroupAssociations.begin(), previousLgAssociation);
									containsPreviousLgAssociation = true;
								}
							}
						}
					}
					if (containsPreviousLgAssociation) {
						break;
					}
				}
				if (!containsPreviousLgAssociation) {
					break;
				}
			}

			// current
			laneGroupAssociations.push_back(lgAssociation);

			// next
			HadRelLaneGroupAssociation* nextLgAssociation = lgAssociation;
			while (nextLgAssociation->first->next.size() >= 1) {
				bool containsNextLgAssociation = false;
				for (auto& pFirstLg : nextLgAssociation->first->next) {
					HadLaneGroup* firstLaneGroup = (HadLaneGroup*)pFirstLg;
					int directType = nextLgAssociation->directType;
					std::vector<HadSkeleton*>& next = directType == 2
						? nextLgAssociation->second->next : nextLgAssociation->second->previous;
					for (auto& pSecondLg : next) {
						HadLaneGroup* secondLaneGroup = (HadLaneGroup*)pSecondLg;
						if (firstLaneGroup == secondLaneGroup) {
							break;
						}

						// 过滤循环LA关系
						auto laIter = findLgAssociationIter(nextLgAssociation, firstLaneGroup, secondLaneGroup,
							pGrid, nearby, allLaneGroupAssociations, groupedLaneGroupAssociations);
						if (laIter != allLaneGroupAssociations.end() && (*laIter)->avgDistance == 0) {
							break;
						}

						if (lgAssociation == *laIter) {
							lgAssociation->applyDiversion = true;
							lgAssociation->overedFillAreas.clear();
							// pFillArea->applyDiversion = false;
							pFillArea->isGore = false;
							containsNextLgAssociation = false;
							break;
						}

						if (laIter != allLaneGroupAssociations.end() && !isXUTurnLA(nextLgAssociation, *laIter)) {
							nextLgAssociation = (*laIter);
							if (nextLgAssociation->first == secondLaneGroup && nextLgAssociation->second == firstLaneGroup) {
								// 保持往后寻找,不然会陷入死循环
								std::swap(nextLgAssociation->first, nextLgAssociation->second);
								std::swap(nextLgAssociation->firstLaneBoundary, nextLgAssociation->secondLaneBoundary);
							}
							laneGroupAssociations.push_back(nextLgAssociation);
							containsNextLgAssociation = true;
						} else if (laIter == allLaneGroupAssociations.end()) {
							// 断头路不生成LA关系
							HadRelLaneGroupAssociation relLgAssociation;
							HadLaneGroupAssociation* pAssociation = nullptr;
							if (isNotSemiTransparentGroup(nextLgAssociation->first) && isNotSemiTransparentGroup(nextLgAssociation->second)) {
								constructLaneGroupAssociation(relLgAssociation, 0, directType, firstLaneGroup, secondLaneGroup);
								pAssociation = generateLaneGroupAssociation(pGrid, pFillArea, relLgAssociation);
							}
							if (pAssociation) {
								if (checkLaneTopology(nextLgAssociation, &pAssociation->relLaneGroupAssociations[0])) {
									nextLgAssociation = &pAssociation->relLaneGroupAssociations[0];
									if (nextLgAssociation->first == secondLaneGroup && nextLgAssociation->second == firstLaneGroup) {
										// 保持往后寻找,不然会陷入死循环
										std::swap(nextLgAssociation->first, nextLgAssociation->second);
										std::swap(nextLgAssociation->firstLaneBoundary, nextLgAssociation->secondLaneBoundary);
									}
									allLaneGroupAssociations.push_back(nextLgAssociation);
									firstLaneGroup->associations.push_back(pAssociation);
									secondLaneGroup->associations.push_back(pAssociation);
									laneGroupAssociations.push_back(nextLgAssociation);
									containsNextLgAssociation = true;
								}
							}
						}
					}
					if (containsNextLgAssociation) {
						break;
					}
				}
				if (!containsNextLgAssociation) {
					break;
				}
			};
		};

		// 本网格导流带
		std::vector<HadFillArea*> fillAreas;
		for (auto obj : pGrid->query(ElementType::HAD_OBJECT_FILL_AREA))
		{
			HadFillArea* pFillArea = (HadFillArea*)obj;
			fillAreas.push_back(pFillArea);
		}

		// 跨网格导流带
		for (auto obj : pGrid->query(ElementType::HAD_LANE_GROUP))
		{
			HadLaneGroup* pLaneGroup = (HadLaneGroup*)obj;
			if (pLaneGroup->crossGrid) {
				getNearbyFillAreas(pLaneGroup->extent.boundingbox2d(), nearby, fillAreas);
			}
		}

		// 本网格的LA关系可能压着跨网格的导流带
		// 因此要把跨网格的导流带也拿过来计算LA关系是否压面
		std::vector<HadGrid*> nearbyGrids = { pGrid };
		for_each(nearby.begin(), nearby.end(), [&](HadGrid* g)->void {nearbyGrids.push_back(g); });
		for (auto obj : fillAreas)
		{
			double fillAreaHight = 0.0;
			ClipperLib::Path fillAreaPath;
			HadFillArea* pFillArea = (HadFillArea*)obj;
			constructFillAreaPath(pFillArea, fillAreaPath, fillAreaHight);
			if (!ClipperLib::Orientation(fillAreaPath)) {
				std::reverse(pFillArea->polygon.vertexes.begin(), pFillArea->polygon.vertexes.end());
			}

			// 周边车道组
			std::vector<HadLaneGroup*> pNearbyLaneGroups;
			pFillArea->extent = makeBoundingBox2d(pFillArea);
			findNearbyLaneGroups(pGrid, pFillArea, nearbyGrids, pNearbyLaneGroups);

			// 周边LA关系
			std::vector<HadRelLaneGroupAssociation*> laneGroupAssociations;
			for (HadLaneGroup* laneGroup : pNearbyLaneGroups) {
				auto laIter = groupedLaneGroupAssociations.find(laneGroup->originId);
				if (laIter != groupedLaneGroupAssociations.end()) {
					for (auto pLA : laIter->second) {
						if (std::find(laneGroupAssociations.begin(),
							laneGroupAssociations.end(), pLA) == laneGroupAssociations.end()) {
							laneGroupAssociations.push_back(pLA);
						}
					}
				}
			}

			if (!laneGroupAssociations.empty()) {
				for (auto laneGroupAssociation : laneGroupAssociations) {
					if (laneGroupAssociation->avgDistance == 0) {
						continue;
					}

					bool laGeometryOverFillArea = false;
					std::vector<HadRelLaneGroupAssociation*> connectedLgAssociations;
					connectLaneGroupAssociations(pFillArea, laneGroupAssociation, connectedLgAssociations);
					for (auto connectedLgAssociation : connectedLgAssociations) {
						if (laGeometrysOverFillArea(pFillArea, *connectedLgAssociation)) {
							laGeometryOverFillArea = true;
							break;
						}
					}

					if (laGeometryOverFillArea) {
						pFillArea->isGore = true;
						// pFillArea->applyDiversion = true;
						auto begin = connectedLgAssociations.begin();
						auto end = connectedLgAssociations.end();
						pFillArea->overedLgAssociations.assign(begin, end);
						for (auto connectedLgAssociation : connectedLgAssociations) {
							connectedLgAssociation->applyDiversion = false;
							std::vector<HadFillArea*>& overedFillAreas = connectedLgAssociation->overedFillAreas;
							if (std::find(overedFillAreas.begin(), overedFillAreas.end(), pFillArea) == overedFillAreas.end()) {
								overedFillAreas.push_back(pFillArea);
							}
						}
					}
				}
			}
		}
	}

	HadLaneGroupAssociation* LinkGroupAssociationProcessor::generateLaneGroupAssociation(
		HadGrid* const pGrid, HadFillArea* pFillArea, HadRelLaneGroupAssociation& lgAssociation) {
		// 没找到对应的车道边界,不构成LA
		HadLaneGroupAssociation* pAssociation = nullptr;
		if (lgAssociation.firstLaneBoundary == nullptr || lgAssociation.secondLaneBoundary == nullptr) {
			return pAssociation;
		}

		// 导流带多边形
		double fillAreaHight = 0.0;
		ClipperLib::Path fillAreaPath;
		constructFillAreaPath(pFillArea, fillAreaPath, fillAreaHight);

		// LA关系面
		double laHight = 0.0;
		ClipperLib::Path laPath;
		constructLgAssociationPath(lgAssociation, laPath, laHight);
		if (fabs(fillAreaHight - laHight) > 500) { // 判断高程差
			return false;
		}

		ClipperLib::Clipper clipper;
		ClipperLib::PolyTree polyTree;
		clipper.AddPath(fillAreaPath, ClipperLib::ptSubject, true);
		clipper.AddPath(laPath, ClipperLib::ptClip, true);
		clipper.Execute(ClipperLib::ctIntersection, polyTree, ClipperLib::pftEvenOdd);
		if (polyTree.Total())
		{
			double polyArea = 0;
			double laArea = std::abs(ClipperLib::Area(laPath));
			double fillArea = std::abs(ClipperLib::Area(fillAreaPath));
			for (auto child : polyTree.Childs) {
				polyArea += std::abs(ClipperLib::Area(child->Contour));
			}
			double polyLaAreaRate = polyArea / laArea;
			double polyFillAreaRate = polyArea / fillArea;
			if (polyFillAreaRate > 0.9 || polyLaAreaRate > 0.9) {
				pAssociation = (HadLaneGroupAssociation*)pGrid->alloc(ElementType::HAD_LG_ASSOCIATIOIN);
				pAssociation->originId = m_maxGroupId + 1;
				pAssociation->relLaneGroupAssociations.push_back(lgAssociation);
				pGrid->insert(pAssociation->originId, pAssociation);
				m_maxGroupId = max(m_maxGroupId, pAssociation->originId);
			}
		}

		return pAssociation;
	}

	forceinline double getIntersectionPoint(
		std::vector<MapPoint3D64>& firstEdgeVertexes,
		std::vector<MapPoint3D64>& secondEdgeVertexes,
		MapPoint3D64& intersectionPt)
	{
		double minDistance = DBL_MAX;
		for (auto& firstVertex : firstEdgeVertexes) {
			int secondIdx = GrapPointAlgorithm::findNearestPoint(secondEdgeVertexes, firstVertex);
			auto& secondVertex = secondEdgeVertexes[secondIdx];
			if (firstVertex == secondVertex) {
				intersectionPt = firstVertex;
				return 0;
			}

			double distance = firstVertex.pos.distance(secondVertex.pos);
			if (distance < minDistance) {
				intersectionPt = firstVertex;
				minDistance = distance;
			}
		}
		return minDistance;
	}

	bool LinkGroupAssociationProcessor::isXUTurnLA(HadRelLaneGroupAssociation* lgAssociation, HadRelLaneGroupAssociation* pLA)
	{
		// 通过车道边界拓扑关系判断是否为车道边界相交LA关系
		auto isUTurnLA = [](HadRelLaneGroupAssociation* la)->bool {
			std::vector<HadSkeleton*>& firstPrevious = la->firstLaneBoundary->previous;
			std::vector<HadSkeleton*>& firstNext = la->firstLaneBoundary->next;
			std::vector<HadSkeleton*>& secondPrevious = la->secondLaneBoundary->previous;
			std::vector<HadSkeleton*>& secondNext = la->secondLaneBoundary->next;
			if (std::find(firstPrevious.begin(), firstPrevious.end(), la->secondLaneBoundary) != firstPrevious.end() ||
				std::find(firstNext.begin(), firstNext.end(), la->secondLaneBoundary) != firstNext.end() ||
				std::find(secondPrevious.begin(), secondPrevious.end(), la->firstLaneBoundary) != secondPrevious.end() ||
				std::find(secondNext.begin(), secondNext.end(), la->firstLaneBoundary) != secondNext.end()) {
				return true;
			}

			return false;
		};

		// 检查是否为X型的
		if (isUTurnLA(lgAssociation) && isUTurnLA(pLA)) {
			MapPoint3D64 intersectionPt1 = {};
			auto& firstLaneEdge = lgAssociation->firstLaneBoundary->location;
			auto& secondLaneEdge = lgAssociation->secondLaneBoundary->location;
			getIntersectionPoint(firstLaneEdge.vertexes, secondLaneEdge.vertexes, intersectionPt1);

			MapPoint3D64 intersectionPt2 = {};
			auto& pLAFirstLaneEdge = pLA->firstLaneBoundary->location;
			auto& pLASecondLaneEdge = pLA->secondLaneBoundary->location;
			getIntersectionPoint(pLAFirstLaneEdge.vertexes, pLASecondLaneEdge.vertexes, intersectionPt2);
			if (floatEqual(intersectionPt1.pos.distanceSquare(intersectionPt2.pos), 0)) {
				return true;
			}
		}

		return false;
	}

	std::vector<HadLaneGroupAssociation::HadRelLaneGroupAssociation*>::iterator 
		LinkGroupAssociationProcessor::findLgAssociationIter(
			HadRelLaneGroupAssociation* lgAssociation, HadLaneGroup* firstLaneGroup, HadLaneGroup* secondLaneGroup,
			HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, std::vector<HadRelLaneGroupAssociation*>& allLaneGroupAssociations,
			std::map<int64, std::vector<HadRelLaneGroupAssociation*>>& groupedLaneGroupAssociations)
	{
		auto begin = allLaneGroupAssociations.begin();
		auto end = allLaneGroupAssociations.end();
		std::vector<HadRelLaneGroupAssociation*>::iterator iter = std::find_if(begin, end, [&](HadRelLaneGroupAssociation* pLA)->bool {
			return (pLA->first == firstLaneGroup && pLA->second == secondLaneGroup) ||
				(pLA->first == secondLaneGroup && pLA->second == firstLaneGroup);
			});
		if (iter != allLaneGroupAssociations.end()) {
			return iter;
		}

		bool findNearbyLa = false;
		for (HadGrid* nearbyGrid : nearby) {
			if (pGrid == nearbyGrid) {
				continue;
			}

			for (auto la : nearbyGrid->query(ElementType::HAD_LG_ASSOCIATIOIN)) {
				HadLaneGroupAssociation* pLaneGroupAssociation = (HadLaneGroupAssociation*)la;
				for (auto& relLgAssociation : pLaneGroupAssociation->relLaneGroupAssociations) {
					HadRelLaneGroupAssociation* pLA = &relLgAssociation;
					if (pLA->firstLaneBoundary == nullptr || pLA->secondLaneBoundary == nullptr) {
						continue;
					}

					if ((pLA->first == firstLaneGroup && pLA->second == secondLaneGroup) ||
						(pLA->first == secondLaneGroup && pLA->second == firstLaneGroup)) {
						if (!isXUTurnLA(lgAssociation, pLA)) {
							appendGroupLaneGroupAssociation(pLA, allLaneGroupAssociations, groupedLaneGroupAssociations);
						}
						findNearbyLa = true;
						break;
					}
				}

				if (findNearbyLa)
					break;
			}
			if (findNearbyLa)
				break;
		}

		if (findNearbyLa) {
			auto newBegin = allLaneGroupAssociations.begin();
			auto newEnd = allLaneGroupAssociations.end();
			return std::find_if(newBegin, newEnd, [&](HadRelLaneGroupAssociation* pLA)->bool {
				return (pLA->first == firstLaneGroup && pLA->second == secondLaneGroup) ||
					(pLA->first == secondLaneGroup && pLA->second == firstLaneGroup);
				});
		}

		return allLaneGroupAssociations.end();
	}

	bool LinkGroupAssociationProcessor::laGeometrysOverFillArea(HadFillArea* pFillArea, HadRelLaneGroupAssociation& lgAssociation)
	{
		// 导流带多边形
		double fillAreaHight = 0.0;
		ClipperLib::Path fillAreaPath;
		constructFillAreaPath(pFillArea, fillAreaPath, fillAreaHight);

		// LA关系面
		double laHight = 0.0;
		ClipperLib::Path laPath;
		constructLgAssociationPath(lgAssociation, laPath, laHight);
		if (fabs(fillAreaHight - laHight) > 500) { // 判断高程差
			return false;
		}

		ClipperLib::Clipper clipper;
		ClipperLib::PolyTree polyTree;
		clipper.AddPath(fillAreaPath, ClipperLib::ptSubject, true);
		clipper.AddPath(laPath, ClipperLib::ptClip, true);
		clipper.Execute(ClipperLib::ctIntersection, polyTree, ClipperLib::pftEvenOdd);
		if (polyTree.Total())
		{
			for (auto polyNode : polyTree.Childs) {
				double laArea = std::abs(ClipperLib::Area(laPath));
				double fillArea = std::abs(ClipperLib::Area(fillAreaPath));
				double polyArea = std::abs(ClipperLib::Area(polyNode->Contour));
				double polyLaAreaRate = polyArea / laArea;
				double polyFillAreaRate = polyArea / fillArea;
				double fillAreaOverLaAreaRate = fillArea / laArea;
				if (polyFillAreaRate > 0.1 || polyLaAreaRate > 0.1 || fillAreaOverLaAreaRate < 0.5) {
					return true;
				}
			}
		}

		return false;
	}

	void LinkGroupAssociationProcessor::constructFillAreaPath(HadFillArea* pFillArea, ClipperLib::Path& fillAreaPath, double& fillAreaHight)
	{
		// 导流带多边形
		for (auto& vertex : pFillArea->polygon.vertexes) {
			fillAreaHight += vertex.z;
			fillAreaPath << ClipperLib::IntPoint(vertex.pos.lon, vertex.pos.lat, vertex.z);
		}
		fillAreaHight /= pFillArea->polygon.vertexes.size();
	}

	void LinkGroupAssociationProcessor::constructLgAssociationPath(HadRelLaneGroupAssociation& lgAssociation, ClipperLib::Path& laPath, double& laHight)
	{
		// LA关系面
		LineString3d firstLaneBoundaryLocation = lgAssociation.firstLaneBoundary->location;
		LineString3d secondLaneBoundaryLocation = lgAssociation.secondLaneBoundary->location;
		MapPoint3D64 firstLaneVec = getBoundaryVector(firstLaneBoundaryLocation.vertexes);
		MapPoint3D64 secondLaneVec = getBoundaryVector(secondLaneBoundaryLocation.vertexes);
		for_each(firstLaneBoundaryLocation.vertexes.begin(), firstLaneBoundaryLocation.vertexes.end(),
			[&](MapPoint3D64& vertex)->void {
				laPath << ClipperLib::IntPoint(vertex.pos.lon, vertex.pos.lat, vertex.z);
				laHight += vertex.z;
			}
		);
		if (dot(firstLaneVec, secondLaneVec) > 0) {
			for_each(secondLaneBoundaryLocation.vertexes.rbegin(), secondLaneBoundaryLocation.vertexes.rend(),
				[&](MapPoint3D64& vertex)->void {
					laPath << ClipperLib::IntPoint(vertex.pos.lon, vertex.pos.lat, vertex.z);
					laHight += vertex.z;
				}
			);
		} else {
			for_each(secondLaneBoundaryLocation.vertexes.begin(), secondLaneBoundaryLocation.vertexes.end(),
				[&](MapPoint3D64& vertex)->void {
					laPath << ClipperLib::IntPoint(vertex.pos.lon, vertex.pos.lat, vertex.z);
					laHight += vertex.z;
				}
			);
		}
		MapPoint3D64& firstVertex = *firstLaneBoundaryLocation.vertexes.begin();
		laPath << ClipperLib::IntPoint(firstVertex.pos.lon, firstVertex.pos.lat, firstVertex.z);
		laHight += firstVertex.z;
		laHight /= laPath.size();
	}

	void LinkGroupAssociationProcessor::constructLaneGroupAssociation(HadRelLaneGroupAssociation& lgAssociation, 
		int64 groupId, int directType, HadLaneGroup* first, HadLaneGroup* second)
	{
		lgAssociation.first = first;
		lgAssociation.second = second;
		lgAssociation.groupId = groupId;
		lgAssociation.directType = directType;
		lgAssociation.firstLaneBoundary = nullptr;
		lgAssociation.secondLaneBoundary = nullptr;

		// 有LA关系,但找不到对应的车道组,比如20596286:133973747226317825
		if (lgAssociation.first != nullptr && lgAssociation.second != nullptr) {
			HadLaneBoundary* pLaneGroupLaneBoundary = nullptr;
			HadLaneBoundary* pNearbyLaneGroupLaneBoundary = nullptr;
			double minDistance = findNearbyLaneBoundary(lgAssociation.first, lgAssociation.second, pLaneGroupLaneBoundary, pNearbyLaneGroupLaneBoundary);
			if (pLaneGroupLaneBoundary != nullptr && pNearbyLaneGroupLaneBoundary != nullptr && pLaneGroupLaneBoundary->originId != pNearbyLaneGroupLaneBoundary->originId) {
				// LA面不应存在共享车道边界的情况,如果出现大概率是哪里出错了
				if (pLaneGroupLaneBoundary->linkGroups.size() == 1 && pNearbyLaneGroupLaneBoundary->linkGroups.size() == 1) {
					lgAssociation.firstLaneBoundary = pLaneGroupLaneBoundary;
					lgAssociation.secondLaneBoundary = pNearbyLaneGroupLaneBoundary;
					lgAssociation.avgDistance = minDistance;
				}
			}
		}
	}

	void LinkGroupAssociationProcessor::getNearbyFillAreas(BoundingBox2d& extent, const std::vector<HadGrid*>& grids, std::vector<HadFillArea*>& nearbyFillAreas)
	{
		BoundingBox2d currBbox = BoundingBox2d::expand(extent, 1e4);
		NdsRect ndsRect = BoundingBox2d::toNdsRect(currBbox);

		NdsGridIdIterator gridIdIterator;
		NdsGridIdIterator_constructWithNdsRect(&gridIdIterator, ndsRect, HAD_GRID_NDS_LEVEL);
		NdsGridId gridId;
		while ((gridId = NdsGridIdIterator_next(&gridIdIterator)) != invalidNdsGridId)
		{
			int32 meshId = NdsGridId_toMeshId(gridId);
			auto iter = std::find_if(grids.begin(), grids.end(), [&](HadGrid* pGrid)->bool {
				if (pGrid->getId() == meshId)
				{
					return true;
				}
				else
				{
					return false;
				}
				});
			if (iter == grids.end())
				continue;

			HadGrid* grid = *iter;
			if (grid->getBoundingbox2d().overlap(currBbox, 0) ||
				grid->getBoundingbox2d().contain(currBbox, 0))
			{
				std::vector<HadElement*>& objects = grid->query(ElementType::HAD_OBJECT_FILL_AREA);
				for (auto& o : objects)
				{
					// 不相交且不包含的过滤掉
					HadFillArea* nearbyFillArea = (HadFillArea*)o;
					BoundingBox2d nearbyBbox = nearbyFillArea->extent;
					if (!(currBbox.overlap(nearbyBbox, 0) ||
						currBbox.contain(nearbyBbox, 0) ||
						nearbyBbox.contain(currBbox, 0))) {
						continue;
					}

					if (std::find(nearbyFillAreas.begin(), nearbyFillAreas.end(), nearbyFillArea) == nearbyFillAreas.end()) {
						nearbyFillAreas.push_back(nearbyFillArea);
					}
				}
			}
		}
	}

	MapPoint3D64 LinkGroupAssociationProcessor::getBoundaryVector(std::vector<MapPoint3D64>& vertexes)
	{
		MapPoint3D64 vec = {};
		for (int idx = 0; idx < vertexes.size() - 1; idx++) {
			auto& currPt = vertexes[idx];
			auto& nextPt = vertexes[idx + 1];
			MapPoint3D64 tmpVec = nextPt - currPt;
			vec = vec + tmpVec;
		}
		return vec;
	}

	bool LinkGroupAssociationProcessor::isNotSemiTransparentGroup(HadLaneGroup* pLaneGroup)
	{
		auto isNotSemiTransparent = [](HadSkeleton* skeleton)->bool {
			return !skeleton->previous.empty() && !skeleton->next.empty();
		};

		// 过滤双向通行的道路边界,断头路前后都存在拓扑关系
		for (auto pBoundary : pLaneGroup->roadBoundaries) {
			if (pBoundary->linkGroups.size() > 1) {
				continue;
			}

			if (isNotSemiTransparent(pBoundary)) {
				return true;
			}
		}
		for (auto pBoundary : pLaneGroup->laneBoundaries) {
			if (pBoundary->linkGroups.size() > 1) {
				continue;
			}

			if (isNotSemiTransparent(pBoundary)) {
				return true;
			}
		}

		return isNotSemiTransparent(pLaneGroup);
	}

	double LinkGroupAssociationProcessor::findNearbyLaneBoundary(HadLaneGroup* pLaneGroup, HadLaneGroup* pNearbyLaneGroup,
		HadLaneBoundary*& pLaneGroupLaneBoundary, HadLaneBoundary*& pNearbyLaneGroupLaneBoundary)
	{
		double minDistance = 30000;
		for (HadLaneBoundary* boundary : pLaneGroup->laneBoundaries) {
			std::vector<double> avgDistances(pNearbyLaneGroup->laneBoundaries.size());
			std::vector<int> grappedPoints(pNearbyLaneGroup->laneBoundaries.size());
			for_each(boundary->location.vertexes.begin(), boundary->location.vertexes.end(),
				[&](const MapPoint3D64& point)->void {
					for (int i = 0; i < pNearbyLaneGroup->laneBoundaries.size(); i++) {
						HadLaneBoundary* nearbyBoundary = pNearbyLaneGroup->laneBoundaries[i];
						std::vector<MapPoint3D64> nearbyBoundaryVertexes = nearbyBoundary->location.vertexes;

						size_t si, ei;
						MapPoint3D64 minGrappedPt = {};
						GrapPointAlgorithm::grapOrMatchNearestPoint(point, nearbyBoundaryVertexes, minGrappedPt, si, ei);
						if (minGrappedPt.pos.lon != 0 && minGrappedPt.pos.lat != 0) {
							Vector2 v;
							v.x = (float)(point.pos.lon - minGrappedPt.pos.lon);
							v.y = (float)(point.pos.lat - minGrappedPt.pos.lat);
							double distance = v.length();
							avgDistances[i] += distance;
							grappedPoints[i] += 1;
						}
					}

				}
			);
			for (int i = 0; i < pNearbyLaneGroup->laneBoundaries.size(); i++) {
				if (grappedPoints[i] == 0) {
					continue;
				}

				avgDistances[i] /= grappedPoints[i];
				if (avgDistances[i] < minDistance) {
					minDistance = avgDistances[i];
					pLaneGroupLaneBoundary = boundary;
					pNearbyLaneGroupLaneBoundary = pNearbyLaneGroup->laneBoundaries[i];
				}
			}

		}
		return minDistance;
	}

	void LinkGroupAssociationProcessor::groupLaneGroupAssociations(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby,
		std::vector<HadRelLaneGroupAssociation*>& allLaneGroupAssociations, 
		std::map<int64, std::vector<HadRelLaneGroupAssociation*>>& groupedLaneGroupAssociations)
	{
		BoundingBox2d gridBbox = BoundingBox2d::expand(pGrid->getBoundingbox2d(), 1e4);
		auto containsGridLaneGroup = [&](HadLaneGroup* const laneGroup) {
			if (laneGroup != nullptr && laneGroup->owner == pGrid) {
				return true;
			}

			// 不相交且不包含的过滤掉
			if (laneGroup != nullptr) {
				BoundingBox2d laneGroupBbox = laneGroup->extent.boundingbox2d();
				if (gridBbox.overlap(laneGroupBbox, 0) ||
					gridBbox.contain(laneGroupBbox, 0) ||
					laneGroupBbox.contain(gridBbox, 0)) {
					return true;
				}
			}

			return false;
		};

		for (auto la : pGrid->query(ElementType::HAD_LG_ASSOCIATIOIN)) {
			HadLaneGroupAssociation* pLaneGroupAssociation = (HadLaneGroupAssociation*)la;
			for (auto& relLgAssociation : pLaneGroupAssociation->relLaneGroupAssociations) {
				HadRelLaneGroupAssociation* pLA = &relLgAssociation;
				appendGroupLaneGroupAssociation(pLA, allLaneGroupAssociations, groupedLaneGroupAssociations);
			}
		}

		// process nearby
		for (auto nearbyGrid : nearby) {
			for (auto la : nearbyGrid->query(ElementType::HAD_LG_ASSOCIATIOIN)) {
				HadLaneGroupAssociation* pLaneGroupAssociation = (HadLaneGroupAssociation*)la;
				for (auto& relLgAssociation : pLaneGroupAssociation->relLaneGroupAssociations) {
					HadRelLaneGroupAssociation* pLA = &relLgAssociation;
					if (containsGridLaneGroup(pLA->first) || containsGridLaneGroup(pLA->second)) {
						appendGroupLaneGroupAssociation(pLA, allLaneGroupAssociations, groupedLaneGroupAssociations);
					}
				}
			}
		}
	}

	void LinkGroupAssociationProcessor::appendGroupLaneGroupAssociation(HadRelLaneGroupAssociation* const pLA, 
		std::vector<HadRelLaneGroupAssociation*>& allLaneGroupAssociations, 
		std::map<int64, std::vector<HadRelLaneGroupAssociation*>>& groupedLaneGroupAssociations)
	{
		if (pLA->firstLaneBoundary == nullptr || pLA->secondLaneBoundary == nullptr) {
			return;
		}

		// LG_ID
		auto firstIter = groupedLaneGroupAssociations.find(pLA->first->originId);
		if (firstIter == groupedLaneGroupAssociations.end()) {
			std::vector<HadRelLaneGroupAssociation*> firstGroups;
			firstGroups.push_back(pLA);
			groupedLaneGroupAssociations.emplace(pLA->first->originId, firstGroups);
		}
		else {
			firstIter->second.push_back(pLA);
		}
		// LG_ID_PAIR
		auto secondIter = groupedLaneGroupAssociations.find(pLA->second->originId);
		if (secondIter == groupedLaneGroupAssociations.end()) {
			std::vector<HadRelLaneGroupAssociation*> secondGroups;
			secondGroups.push_back(pLA);
			groupedLaneGroupAssociations.emplace(pLA->second->originId, secondGroups);
		}
		else {
			secondIter->second.push_back(pLA);
		}
		allLaneGroupAssociations.push_back(pLA);
	}

	void LinkGroupAssociationProcessor::findNearbyLaneGroups(HadGrid* const pGrid, HadFillArea* const pFillArea, const std::vector<HadGrid*>& nearby,std::vector<HadLaneGroup*>& pNearbyLaneGroups)
	{
		BoundingBox2d bbox = pFillArea->extent;
		std::vector<HadLaneGroup*> tmpLaneGroups = SpatialSeacher::seachNearby(nearby,bbox);
		for (HadLaneGroup* tmpLaneGroup : tmpLaneGroups) {
			if (std::find(pNearbyLaneGroups.begin(), pNearbyLaneGroups.end(), tmpLaneGroup) == pNearbyLaneGroups.end()) {
				pNearbyLaneGroups.push_back(tmpLaneGroup);
			}
		}

		for (HadLaneGroup* laneGroup : pFillArea->laneGroups) {
			if (std::find(pNearbyLaneGroups.begin(), pNearbyLaneGroups.end(), laneGroup) == pNearbyLaneGroups.end()) {
				pNearbyLaneGroups.push_back(laneGroup);
			}
			std::vector<HadLaneGroup*> pTmpLaneGroups = SpatialSeacher::seachNearby2d(pGrid, laneGroup, 5 * 1e3);
			for (HadLaneGroup* pTmpLaneGroup : pTmpLaneGroups) {
				if (std::find(pNearbyLaneGroups.begin(), pNearbyLaneGroups.end(), pTmpLaneGroup) == pNearbyLaneGroups.end()) {
					pNearbyLaneGroups.push_back(pTmpLaneGroup);
				}
			}
		}
	}

	BoundingBox2d LinkGroupAssociationProcessor::makeBoundingBox2d(HadFillArea* const pFillArea)
	{
		std::vector<int64> vx;
		std::vector<int64> vy;
		for (auto& point : pFillArea->polygon.vertexes) {
			vx.push_back(point.pos.lon);
			vy.push_back(point.pos.lat);
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

}
