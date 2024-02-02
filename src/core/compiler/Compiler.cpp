#include "stdafx.h"
#include <array>
#include "Compiler.h"
#include <algorithm>
#include "algorithm/grap_point_algorithm.h"
#include "cq_math.h"
#include "math3d/vector_math.h"
#include "map_point3d64_converter.h"
#include <mutex>

using namespace RDS;
namespace OMDB
{
	void Compiler::Compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile)
	{
		bool isConvert = false;
		for (auto obj : pGrid->query(ElementType::HAD_LANE_GROUP))
		{
			HadLaneGroup* pGroup = (HadLaneGroup*)obj;
			if (!pGroup->roadBoundaries.empty())
			{
				if (!pGroup->roadBoundaries.front()->location.vertexes.empty())
				{
					coordinatesTransform.setBasePoint(pGroup->roadBoundaries.front()->location.vertexes.front());
					isConvert = true;
				}
			}
			if (!isConvert)
			{
				if (!pGroup->laneBoundaries.empty())
				{
					if (!pGroup->laneBoundaries.front()->location.vertexes.empty())
					{
						coordinatesTransform.setBasePoint(pGroup->laneBoundaries.front()->location.vertexes.front());
						isConvert = true;
					}
				}
			}
			if (isConvert)
				break;
		}
		if (!isConvert)
			return;
		for (auto obj : pGrid->query(ElementType::HAD_INTERSECTION))
		{
			HadIntersection* hadInts = (HadIntersection*)obj;
			if (hadInts == nullptr || hadInts->refLaneGroups.size() == 0)
				continue;

			if (hadInts->doStraightIntersection && isStraightIntersection(hadInts))
			{
				for (auto refLaneGroup : hadInts->refLaneGroups)
				{
					refLaneGroup->inIntersection = 0;
				}
				hadInts->refLaneGroups.clear();
			}
			hadInts->doStraightIntersection = false;
		}
		return compile(pGrid, nearby, pTile);
	}

	void Compiler::convert(const MapPoint3D64& src, Point3d& dst)
	{
		const MapPoint3D64* pSrc = &src;
		RDS::Point3d* pDst = &dst;
		memcpy(pDst, pSrc, sizeof(MapPoint3D64));
	}

	void Compiler::convert(const OMDB::LineString3d& src, RDS::LineString3d& dst)
	{
		dst.vertexes.resize(src.vertexes.size());
		memcpy(dst.vertexes.data(), src.vertexes.data(), sizeof(MapPoint3D64) * src.vertexes.size());
	}

	void Compiler::convert(const OMDB::MultiPoint3d& src, RDS::MultiPoint3d& dst)
	{
		dst.postions.resize(src.postions.size());
		memcpy(dst.postions.data(), src.postions.data(), sizeof(MapPoint3D64) * src.postions.size());
	}

	void Compiler::convert(const OMDB::MultiLineString3d& src, RDS::MultiLineString3d& dst)
	{
		int size = src.lines.size();
		dst.lines.resize(size);
		for (int i = 0; i < size; i++)
		{
			convert(src.lines[i], dst.lines[i]);
		}
	}

	void Compiler::convert(const OMDB::Polygon3d& src, RDS::Polygon3d& dst)
	{
		dst.vertexes.resize(src.vertexes.size());
		memcpy(dst.vertexes.data(), src.vertexes.data(), sizeof(MapPoint3D64) * src.vertexes.size());
	}

	void Compiler::convert(const OMDB::MultiPolygon3d& src, RDS::MultiPolygon3d& dst)
	{
		int size = src.polygons.size();
		dst.polygons.resize(size);
		for (int i = 0; i < size; i++)
		{
			convert(src.polygons[i], dst.polygons[i]);
		}
	}

	void Compiler::convert(const RDS::Point3d& src, MapPoint3D64& dst)
	{
		const RDS::Point3d* pSrc = &src;
		MapPoint3D64* pDst = &dst;
		memcpy(pDst, pSrc, sizeof(Point3d));
	}

	void Compiler::convert(const RDS::Polygon3d& src, OMDB::Polygon3d& dst)
	{
		dst.vertexes.resize(src.vertexes.size());
		memcpy(dst.vertexes.data(), src.vertexes.data(), sizeof(Point3d) * src.vertexes.size());
	}

	void Compiler::convert(const RDS::LineString3d& src, OMDB::LineString3d& dst)
	{
		dst.vertexes.resize(src.vertexes.size());
		memcpy(dst.vertexes.data(), src.vertexes.data(), sizeof(Point3d) * src.vertexes.size());
	}

	void Compiler::convert(const RDS::MultiLineString3d& src, OMDB::MultiLineString3d& dst)
	{
		int size = src.lines.size();
		dst.lines.resize(size);
		for (int i = 0; i < size; i++)
		{
			convert(src.lines[i], dst.lines[i]);
		}
	}

	bool Compiler::isStraightIntersection(HadIntersection* const hadIntersct)
	{
		auto isStraightSkeleton = [](HadSkeleton* firstSkeleton, HadSkeleton* secondSkeleton)->bool {
			std::vector<HadSkeleton*>& firstPrevious = firstSkeleton->previous;
			std::vector<HadSkeleton*>& firstNext = firstSkeleton->next;
			if (firstPrevious.size() != 1 || firstNext.size() != 1) {
				return false;
			}
			
			std::vector<HadSkeleton*>& secondPrevious = secondSkeleton->previous;
			std::vector<HadSkeleton*>& secondNext = secondSkeleton->next;
			if (secondPrevious.size() != 1 || secondNext.size() != 1) {
				return false;
			}

			if (std::find(firstPrevious.begin(), firstPrevious.end(), secondSkeleton) == firstPrevious.end() &&
				std::find(firstNext.begin(), firstNext.end(), secondSkeleton) == firstNext.end()) {
				return false;
			}

			if (std::find(secondPrevious.begin(), secondPrevious.end(), firstSkeleton) == secondPrevious.end() &&
				std::find(secondNext.begin(), secondNext.end(), firstSkeleton) == secondNext.end()) {
				return false;
			}

			return true;
		};

		auto isStraightGroup = [&](HadLaneGroup* currGroup, HadLaneGroup* nextGroup)->bool {
			auto roadBoundarySize = currGroup->roadBoundaries.size();
			auto nextRoadBoundarySize = nextGroup->roadBoundaries.size();
			if (roadBoundarySize != nextRoadBoundarySize)
				return false;

			auto laneBoundarySize = currGroup->laneBoundaries.size();
			auto nextLaneBoundarySize = nextGroup->laneBoundaries.size();
			if (laneBoundarySize != nextLaneBoundarySize)
				return false;

			// check LaneGroup
			if (!isStraightSkeleton(currGroup, nextGroup))
				return false;

			// check RoadBoundary
			for (int idx = 0; idx < roadBoundarySize; idx++) {
				if (!isStraightSkeleton(currGroup->roadBoundaries[idx], nextGroup->roadBoundaries[idx]))
					return false;
			}

			// check LaneBoundary
			for (int idx = 0; idx < laneBoundarySize; idx++) {
				if (!isStraightSkeleton(currGroup->laneBoundaries[idx], nextGroup->laneBoundaries[idx]))
					return false;
			}
			return roadBoundarySize != 0 || laneBoundarySize != 0;
		};

		for (auto idx = 0; idx < hadIntersct->refLaneGroups.size() - 1; idx++)
		{
			auto currLaneGroup = hadIntersct->refLaneGroups[idx];
			auto nextLaneGroup = hadIntersct->refLaneGroups[idx + 1];
			if (!isStraightGroup(currLaneGroup, nextLaneGroup))
				return false;
		}

		return true;
	}

	bool Compiler::isTunnelArea(HadLaneGroup* pLinkGroup)
	{
		bool inTunnelArea = false;
		for (auto& relLink : pLinkGroup->relLinks) {
			for (auto& wayType : relLink.first->wayTypes) {
				if (wayType == LINK_IS_IN_TUNNEL) {
					inTunnelArea = true;
					break;
				}

				if (wayType == DB_HAD_APPLY_PA_REFERENCE) {
					std::vector<std::pair<double, double>> intervals;
					for (auto paValue : relLink.first->attributes) {
						if (paValue->name == LINK_WAY_TYPE_PA_NAME && paValue->value == LINK_IS_IN_TUNNEL) {

							intervals.emplace_back(std::make_pair(paValue->start, paValue->end));
						}
					}
					if (!intervals.empty())
					{
						// 排序区间
						std::sort(intervals.begin(), intervals.end(), [](const std::pair<double, double>& s1, const std::pair<double, double>& s2) {
							if (s1.first != s2.first)
								return s1.first < s2.first;

							else
								return s1.second < s2.second;

							});
						// 合并重叠的区间
						std::vector<std::pair<double, double>> merged;
						std::pair<double, double> curRange = intervals.front();
						for (int i = 1; i < intervals.size(); i++)
						{
							if (curRange.second + 0.01 >= intervals[i].first)
							{
								curRange.second = max(curRange.second + 0.01, intervals[i].second);

							}
							else
							{
								merged.push_back(curRange);
								curRange = intervals[i];
							}
						}
						merged.push_back(curRange);

						for (auto& range : merged)
						{
							if (range.first <= relLink.second.start + 0.01 && relLink.second.end <= range.second + 0.01) {
								inTunnelArea = true;
								break;
							}
						}

					}

				}


			}
		}
		return inTunnelArea;
	}

	bool Compiler::isInTollArea(HadLaneGroup* pLinkGroup)
	{
		bool inTollArea = false;
		for (auto& relLink : pLinkGroup->relLinks) {
			for (auto& wayType : relLink.first->wayTypes) {
				if (wayType == LINK_IS_IN_TOLL_AREA) {
					inTollArea = true;
					break;
				}
				if (wayType == DB_HAD_APPLY_PA_REFERENCE) {
					for (auto paValue : relLink.first->attributes) {
						if (paValue->name == LINK_WAY_TYPE_PA_NAME && paValue->value == LINK_IS_IN_TOLL_AREA) {
							if (paValue->start <= relLink.second.start && relLink.second.end <= paValue->end) {
								inTollArea = true;
								break;
							}
						}
					}
				}
			}
		}
		return inTollArea;
	}

	bool Compiler::isMultiDigitized(HadLaneGroup* pLinkGroup)
	{
		bool multiDigitized = true;
		for (auto& relLink : pLinkGroup->relLinks) {
			if (relLink.first->multi_digitized == DB_HAD_APPLY_PA_REFERENCE) {
				for (auto paValue : relLink.first->attributes) {
					if (paValue->name == LINK_MULTI_DIGITIZED_PA_NAME) {
						if (paValue->start <= relLink.second.start && relLink.second.end <= paValue->end) {
							multiDigitized = false;
							break;
						}
					}
				}
			}
			else if (relLink.first->multi_digitized != 0) {
				multiDigitized = false;
			}
		}
		// 潮汐车道
		for (auto& lane : pLinkGroup->lanes) {
			if (lane->conditionType == 1) {
				multiDigitized = false;
				break;
			}
		}
		return multiDigitized;
	}

	bool Compiler::isSemiTransparentGroup(HadLaneGroup* pLaneGroup)
	{
		return !isNotSemiTransparentGroup(pLaneGroup);
	}

	bool Compiler::isNotSemiTransparentGroup(HadLaneGroup* pLaneGroup)
	{
		auto getAllLinks = [](std::map<int64, HadLaneGroup*>& langGroups)->std::vector<HadLink*> {
			std::vector<HadLink*> links;
			for (auto& pair : langGroups) {
				for (auto& l : pair.second->relLinks) {
					links.push_back(l.first);
				}
			}
			return links;
		};

		auto isConnectedLink = [](std::vector<HadLink*>& pLinks, std::vector<HadLink*>& nLinks)->bool {
			for (auto pLink : pLinks) {
				for (auto nLink : nLinks) {
					if (pLink == nLink)
						return true;
					if (std::find(pLink->next.begin(), pLink->next.end(), nLink) != pLink->next.end() ||
						std::find(nLink->previous.begin(), nLink->previous.end(), pLink) != nLink->previous.end())
						return true;
				}
			}
			return false;
		};

		// 过滤双向通行的道路边界,断头路前后都存在拓扑关系
		for (auto pBoundary : pLaneGroup->roadBoundaries) {
			if (pBoundary->linkGroups.size() > 1) {
				continue;
			}

			if (isNotSemiTransparentRoadBoundary(pBoundary)) {
				if (!pBoundary->previous.empty() && !pBoundary->next.empty()) {
					bool previousLinkConnected = false;
					bool nextLinkConnected = false;
					auto pCurrLinks = getAllLinks(pBoundary->linkGroups);
					for (auto previousSkeleton : pBoundary->previous) {
						HadRoadBoundary* previousBoundary = (HadRoadBoundary*)previousSkeleton;
						auto previousLinks = getAllLinks(previousBoundary->linkGroups);
						if (isConnectedLink(previousLinks, pCurrLinks)) {
							previousLinkConnected = true;
							break;
						}
					}
					for (auto nextSkeleton : pBoundary->next) {
						HadRoadBoundary* nextBoundary = (HadRoadBoundary*)nextSkeleton;
						auto nextLinks = getAllLinks(nextBoundary->linkGroups);
						if (isConnectedLink(pCurrLinks, nextLinks)) {
							nextLinkConnected = true;
							break;
						}
					}
					if (!previousLinkConnected || !nextLinkConnected) {
						continue;
					}
				}
				return true;
			}
		}
		for (auto pBoundary : pLaneGroup->laneBoundaries) {
			if (pBoundary->linkGroups.size() > 1) {
				continue;
			}

			if (isNotSemiTransparentLaneBoundary(pBoundary)) {
				if (!pBoundary->previous.empty() && !pBoundary->next.empty()) {
					bool previousLinkConnected = false;
					bool nextLinkConnected = false;
					auto pCurrLinks = getAllLinks(pBoundary->linkGroups);
					for (auto previousSkeleton : pBoundary->previous) {
						HadLaneBoundary* previousBoundary = (HadLaneBoundary*)previousSkeleton;
						auto previousLinks = getAllLinks(previousBoundary->linkGroups);
						if (isConnectedLink(previousLinks, pCurrLinks)) {
							previousLinkConnected = true;
							break;
						}
					}
					for (auto nextSkeleton : pBoundary->next) {
						HadLaneBoundary* nextBoundary = (HadLaneBoundary*)nextSkeleton;
						auto nextLinks = getAllLinks(nextBoundary->linkGroups);
						if (isConnectedLink(pCurrLinks, nextLinks)) {
							nextLinkConnected = true;
							break;
						}
					}
					if (!previousLinkConnected || !nextLinkConnected) {
						continue;
					}
				}
				return true;
			}
		}

		return isNotSemiTransparentLaneGroup(pLaneGroup);
	}

	std::vector<HadSkeleton*> Compiler::getLaneGroupSkeletons(std::vector<HadSkeleton*>& skeletons)
	{
		//判断是否为普通路
		auto copy = skeletons;
		if (CompileSetting::instance()->isNotCompileUrbanData) {
			for (auto iter = copy.begin(); iter != copy.end();) {
				auto g = (HadLaneGroup*)*iter;
				if (!isProDataLevel(g)) {
					iter = copy.erase(iter);
					continue;
				}
				iter++;
			}
		}
		return copy;
	}

	bool Compiler::isNotSemiTransparentLaneGroup(HadLaneGroup* skeleton)
	{
		auto previous = getLaneGroupSkeletons(skeleton->previous);
		auto next = getLaneGroupSkeletons(skeleton->next);
		return !previous.empty() && !next.empty();
	}

	bool Compiler::isNotSemiTransparentRoadBoundary(HadRoadBoundary* skeleton)
	{
		//判断是否为普通路
		auto getSkeletons = [](std::vector<HadSkeleton*>& skeletons)->std::vector<HadSkeleton*> {
			auto copy = skeletons;
			if (CompileSetting::instance()->isNotCompileUrbanData) {
				for (auto iter = copy.begin(); iter != copy.end();) {
					auto pBoundary = (HadRoadBoundary*)*iter;
					bool isNotProDataLevel = false;
					for (auto pair : pBoundary->linkGroups) {
						if (!isProDataLevel(pair.second)) {
							isNotProDataLevel = true;
							break;
						}
					}
					if (isNotProDataLevel) {
						iter = copy.erase(iter);
						continue;
					}
					iter++;
				}
			}
			return copy;
		};

		auto previous = getSkeletons(skeleton->previous);
		auto next = getSkeletons(skeleton->next);
		return !previous.empty() && !next.empty();
	}

	bool Compiler::isNotSemiTransparentLaneBoundary(HadLaneBoundary* skeleton)
	{
		//判断是否为普通路
		auto getSkeletons = [](std::vector<HadSkeleton*>& skeletons)->std::vector<HadSkeleton*> {
			auto copy = skeletons;
			if (CompileSetting::instance()->isNotCompileUrbanData) {
				for (auto iter = copy.begin(); iter != copy.end();) {
					auto pBoundary = (HadLaneBoundary*)*iter;
					bool isNotProDataLevel = false;
					for (auto pair : pBoundary->linkGroups) {
						if (!isProDataLevel(pair.second)) {
							isNotProDataLevel = true;
							break;
						}
					}
					if (isNotProDataLevel) {
						iter = copy.erase(iter);
						continue;
					}
					iter++;
				}
			}
			return copy;
		};

		auto previous = getSkeletons(skeleton->previous);
		auto next = getSkeletons(skeleton->next);
		return !previous.empty() && !next.empty();
	}

	bool Compiler::isExistFillArea(HadLaneGroup* pLaneGroup)
	{
		for (size_t i = 0; i < pLaneGroup->objects.size(); i++)
		{
			HadObject* pObject = pLaneGroup->objects[i];
			if (pObject->objectType == ElementType::HAD_OBJECT_FILL_AREA)
			{
				return true;
			}
		}
		return false;
	}

	bool Compiler::isExistAssociation(HadLaneGroup* pLaneGroup)
	{
		return !pLaneGroup->associations.empty();
	}

	bool Compiler::isProDataLevel(HadLaneGroup* pLaneGroup)
	{
		bool tmp = false;
		for (auto& relLink : pLaneGroup->relLinks) 
		{
			for (auto& tmpDataLevel : relLink.first->dataLevels)
			{
				if (tmpDataLevel.dataLevel == 1)
					tmp = true;
			}
		}
		return tmp;
	}

	bool Compiler::isProDataLevel(const std::vector<HadLaneGroup*>& pLaneGroups)
	{
		bool tmp = false;
		if (!pLaneGroups.empty())
		{
			for (auto& pLaneGroup : pLaneGroups)
			{
				for (auto& relLink : pLaneGroup->relLinks)
				{
					for (auto& tmpDataLevel : relLink.first->dataLevels)
					{
						if (tmpDataLevel.dataLevel == 1)
							tmp = true;
					}
				}
			}
		}
		else
		{
			tmp = true;
		}
		return tmp;
	}


	bool Compiler::isUTurn(HadLaneGroup* pLinkGroup)
	{
	  for (auto& relLink : pLinkGroup->relLinks) {
            for (auto& wayType : relLink.first->wayTypes) {
                if (wayType == LINK_IS_IN_UTURN) {
					return true;
                }

                if (wayType == DB_HAD_APPLY_PA_REFERENCE) {
                    std::vector<RangeF> intervals;
                    for (auto paValue : relLink.first->attributes) {
                        if (paValue->name == LINK_WAY_TYPE_PA_NAME && paValue->value == LINK_IS_IN_UTURN) {

                            intervals.emplace_back(RangeF_make(paValue->start, paValue->end));
                        }
                    }
                    if (!intervals.empty())
                    {
                        // 排序区间
                        std::sort(intervals.begin(), intervals.end(), [](RangeF& s1, RangeF& s2) {
                            if (s1.lower != s2.lower)
                                return s1.lower < s2.lower;

                            else
                                return s1.upper < s2.upper;

                            });
                        // 合并重叠的区间
                        std::vector<RangeF> merged;
						RangeF curRange = merged.front();
                        for (int i = 1; i < intervals.size(); i++)
                        {
                            if (curRange.upper + 0.01 >= intervals[i].lower)
                            {
                                curRange.upper = max(curRange.upper + 0.01, intervals[i].upper);

                            }
                            else
                            {
                                merged.push_back(curRange);
                                curRange = intervals[i];
                            }
                        }
                        merged.push_back(curRange);

                        for (auto& range : merged)
                        {
                            if (range.lower <= relLink.second.start + 0.01 && relLink.second.end <= range.upper + 0.01) {
								return true;
                            }
                        }

                    }

                }


            }
        }
        return false;
	}

	bool Compiler::isRoundabout(HadLaneGroup* pLinkGroup)
	{
		for (auto& relLink : pLinkGroup->relLinks) {
			for (auto& wayType : relLink.first->wayTypes) {
				if (wayType == LINK_IS_IN_ROUNDABOUT) {
					return true;
				}

				if (wayType == DB_HAD_APPLY_PA_REFERENCE) {
					std::vector<RangeF> intervals;
					for (auto paValue : relLink.first->attributes) {
						if (paValue->name == LINK_WAY_TYPE_PA_NAME && paValue->value == LINK_IS_IN_ROUNDABOUT) {

							intervals.emplace_back(RangeF_make(paValue->start, paValue->end));
						}
					}
					if (!intervals.empty())
					{
						// 排序区间
						std::sort(intervals.begin(), intervals.end(), [](RangeF& s1, RangeF& s2) {
							if (s1.lower != s2.lower)
								return s1.lower < s2.lower;

							else
								return s1.upper < s2.upper;

							});
						// 合并重叠的区间
						std::vector<RangeF> merged;
						RangeF curRange = merged.front();
						for (int i = 1; i < intervals.size(); i++)
						{
							if (curRange.upper + 0.01 >= intervals[i].lower)
							{
								curRange.upper = max(curRange.upper + 0.01, intervals[i].upper);

							}
							else
							{
								merged.push_back(curRange);
								curRange = intervals[i];
							}
						}
						merged.push_back(curRange);

						for (auto& range : merged)
						{
							if (range.lower <= relLink.second.start + 0.01 && relLink.second.end <= range.upper + 0.01) {
								return true;
							}
						}

					}

				}


			}
		}
		return false;
	}

	bool Compiler::isCreateRdsForIntersection(HadGrid* const pGrid, const std::vector<HadLaneGroup*> pGroups)
	{
		bool isCreateLineForIntersection = true;
		for (auto& tmpLaneGroup : pGroups)
		{
			if (tmpLaneGroup->inIntersection)
			{
				isCreateLineForIntersection = false;
				HadIntersection* pInt = (HadIntersection*)pGrid->query(tmpLaneGroup->inIntersection, ElementType::HAD_INTERSECTION);
				if (pInt != nullptr)
				{
					if (pInt->isUTurn || pInt->isRoundabout)
						isCreateLineForIntersection = true;
				}
				break;
			}
		}
		return isCreateLineForIntersection;
	}

	// 	RdsTile* Compiler::getOrCreateTile(int32 id, RdsDatabase* pDatabase)

    bool Compiler::isTouchWithHadLaneGroup(HadLaneGroup* laneGroup1, HadLaneGroup* laneGroup2)
    {
        for (auto lg : laneGroup1->next)
        {
            if (lg == laneGroup2)
            {
                return true;
            }
        }
        for (auto lg : laneGroup1->previous)
        {
            if (lg == laneGroup2)
            {
                return true;
            }
        }
        return false;
    }

    // 	RdsTile* Compiler::getOrCreateTile(int32 id, RdsDatabase* pDatabase)

	//     {
	//         RdsTile* pTile = pDatabase->query(id);
	//         if (pTile == nullptr)
	//         {
	// 			pTile = new RdsTile();
	// 			pTile->meshId = id;
	//             pDatabase->insert(id, pTile);
	//         }
	// 
	// 		Rect rect;
	// 		NdsGridId_getRect(MeshId_toNdsGridId(id), &rect);
	// 
	// 		Point topLeft02;
	// 		Point topLeft84 = { rect.left, rect.top };
	// 		Math_wgsToMars(&topLeft84, &topLeft02);
	// 
	// 		pTile->latitude = (int64)topLeft02.y * 1000;
	// 		pTile->longitude = (int64)topLeft02.y * 1000;
	//         return pTile;
	// 
	//     }

	RDS::RdsObject* Compiler::createObject(RdsTile* pTile, EntityType type)
	{
		auto obj = pTile->createObject(type);
		return obj;
	}

	RDS::RdsGroup* Compiler::queryGroup(int64 originId, RdsTile* pTile)
	{
		std::vector<RdsObject*>& pObjects = pTile->query(EntityType::RDS_GROUP);

		RdsGroup* pGroup = nullptr;
		for_each(pObjects.begin(), pObjects.end(), [&](RdsObject* pObject)->void {

			RdsGroup* gp = (RdsGroup*)pObject;
			if (gp != nullptr)
			{
				if (gp->originId == originId)
					pGroup = gp;
			}
			});

		return pGroup;

	}

	void Compiler::deleteGroup(RdsGroup* pGroup, RdsTile* pTile)
	{
		std::vector<RdsObject*>& pObjects = pTile->query(EntityType::RDS_GROUP);

		auto iter = std::find_if(pObjects.begin(), pObjects.end(), [&](RdsObject* pObject)->bool {
			RdsGroup* gp = (RdsGroup*)pObject;
			return gp != nullptr && gp->originId == pGroup->originId;
			});

		if (iter != pObjects.end())
		{
			pObjects.erase(iter);
		}
	}

	void Compiler::deleteObject(RdsObject* pObject, RdsTile* pTile)
	{
		std::vector<RdsObject*>& pObjects = pTile->query(pObject->getEntityType());

		auto iter = std::find_if(pObjects.begin(), pObjects.end(), [&](RdsObject* o)->bool {
			return o != nullptr && o == pObject;
			});

		if (iter != pObjects.end())
		{
			pObjects.erase(iter);
		}
	}

	void Compiler::connectForwardRoadBoundary(HadLaneGroup* pGroup, HadRoadBoundary* pBoundary,
		std::deque<HadRoadBoundary*>& connectedBoundarys, std::deque<HadLaneGroup*>& connectedLaneGroups, int connectNum)
	{
		//1连1的情况。即：当前道路边的入度为1，前序道路边的出度为1。
		if (connectNum)
		{
			if (pBoundary->previous.size() == 1 && pBoundary->previous[0]->next.size() == 1)
			{
				for (auto grp : pGroup->previous)
				{
					HadLaneGroup* pCurGroup = (HadLaneGroup*)grp;
					HadRoadBoundary* pCurBoundary = (HadRoadBoundary*)pBoundary->previous[0];
					std::vector<HadRoadBoundary*> roadBoundaries = pCurGroup->roadBoundaries;
					if (std::find(roadBoundaries.begin(), roadBoundaries.end(), pCurBoundary) == roadBoundaries.end())
						continue;

					if (std::find(connectedBoundarys.begin(), connectedBoundarys.end(), pCurBoundary) != connectedBoundarys.end())
						return;

					if (std::find(connectedLaneGroups.begin(), connectedLaneGroups.end(), pCurGroup) != connectedLaneGroups.end())
						return;

					connectedBoundarys.push_front(pCurBoundary);
					connectedLaneGroups.push_front(pCurGroup);
					connectForwardRoadBoundary(pCurGroup, pCurBoundary, connectedBoundarys, connectedLaneGroups, --connectNum);
				}
			}
			else if (pGroup->previous.size() == 1 && pGroup->previous[0]->next.size() == 1)
			{
				for (auto prb : pBoundary->previous)
				{
					HadRoadBoundary* pCurBoundary = (HadRoadBoundary*)prb;
					HadLaneGroup* pCurGroup = (HadLaneGroup*)pGroup->previous[0];
					std::vector<HadRoadBoundary*> roadBoundaries = pCurGroup->roadBoundaries;
					if (std::find(roadBoundaries.begin(), roadBoundaries.end(), pCurBoundary) == roadBoundaries.end())
						continue;

					if (std::find(connectedBoundarys.begin(), connectedBoundarys.end(), pCurBoundary) != connectedBoundarys.end())
						return;

					if (std::find(connectedLaneGroups.begin(), connectedLaneGroups.end(), pCurGroup) != connectedLaneGroups.end())
						return;

					connectedBoundarys.push_front(pCurBoundary);
					connectedLaneGroups.push_front(pCurGroup);
					connectForwardRoadBoundary(pCurGroup, pCurBoundary, connectedBoundarys, connectedLaneGroups, --connectNum);
				}
			}
		}
	}

	void Compiler::connectBackwardRoadBoundary(HadLaneGroup* pGroup, HadRoadBoundary* pBoundary,
		std::deque<HadRoadBoundary*>& connectedBoundarys, std::deque<HadLaneGroup*>& connectedLaneGroups, int connectNum)
	{
		//1连1的情况。即：当前道路边的入度为1，前序道路边的出度为1。
		if (connectNum)
		{
			if (pBoundary->next.size() == 1 && pBoundary->next[0]->previous.size() == 1)
			{
				for (auto grp : pGroup->next)
				{
					HadLaneGroup* pCurGroup = (HadLaneGroup*)grp;
					HadRoadBoundary* pCurBoundary = (HadRoadBoundary*)pBoundary->next[0];
					std::vector<HadRoadBoundary*> roadBoundaries = pCurGroup->roadBoundaries;
					if (std::find(roadBoundaries.begin(), roadBoundaries.end(), pCurBoundary) == roadBoundaries.end())
						continue;

					if (std::find(connectedBoundarys.begin(), connectedBoundarys.end(), pCurBoundary) != connectedBoundarys.end())
						return;

					if (std::find(connectedLaneGroups.begin(), connectedLaneGroups.end(), pCurGroup) != connectedLaneGroups.end())
						return;

					connectedBoundarys.push_back(pCurBoundary);
					connectedLaneGroups.push_back(pCurGroup);
					connectBackwardRoadBoundary(pCurGroup, pCurBoundary, connectedBoundarys, connectedLaneGroups, --connectNum);
				}
			}
			else if (pGroup->next.size() == 1 && pGroup->next[0]->previous.size() == 1)
			{
				for (auto prb : pBoundary->next)
				{
					HadRoadBoundary* pCurBoundary = (HadRoadBoundary*)prb;
					HadLaneGroup* pCurGroup = (HadLaneGroup*)pGroup->next[0];
					std::vector<HadRoadBoundary*> roadBoundaries = pCurGroup->roadBoundaries;
					if (std::find(roadBoundaries.begin(), roadBoundaries.end(), pCurBoundary) == roadBoundaries.end())
						continue;

					if (std::find(connectedBoundarys.begin(), connectedBoundarys.end(), pCurBoundary) != connectedBoundarys.end())
						return;

					if (std::find(connectedLaneGroups.begin(), connectedLaneGroups.end(), pCurGroup) != connectedLaneGroups.end())
						return;

					connectedBoundarys.push_back(pCurBoundary);
					connectedLaneGroups.push_back(pCurGroup);
					connectBackwardRoadBoundary(pCurGroup, pCurBoundary, connectedBoundarys, connectedLaneGroups, --connectNum);
				}
			}
		}
	}

	void Compiler::connectForwardLaneBoundary(HadLaneGroup* pGroup, HadLaneBoundary* pBoundary, 
		std::deque<HadLaneBoundary*>& connectedBoundarys, std::deque<HadLaneGroup*>& connectedLaneGroups, int connectNum)
	{
		//1连1的情况。即：当前道路边的入度为1，前序道路边的出度为1。
		if (connectNum)
		{
			if (pBoundary->previous.size() == 1 && pBoundary->previous[0]->next.size() == 1)
			{
				for (auto grp : pGroup->previous)
				{
					HadLaneGroup* pCurGroup = (HadLaneGroup*)grp;
					HadLaneBoundary* pCurBoundary = (HadLaneBoundary*)pBoundary->previous[0];
					std::vector<HadLaneBoundary*> laneBoundaries = pCurGroup->laneBoundaries;
					if (std::find(laneBoundaries.begin(), laneBoundaries.end(), pCurBoundary) == laneBoundaries.end())
						continue;

					if (std::find(connectedBoundarys.begin(), connectedBoundarys.end(), pCurBoundary) != connectedBoundarys.end())
						return;

					if (std::find(connectedLaneGroups.begin(), connectedLaneGroups.end(), pCurGroup) != connectedLaneGroups.end())
						return;

					connectedBoundarys.push_front(pCurBoundary);
					connectedLaneGroups.push_front(pCurGroup);
					connectForwardLaneBoundary(pCurGroup, pCurBoundary, connectedBoundarys, connectedLaneGroups, --connectNum);
				}
			}
			else if (pGroup->previous.size() == 1 && pGroup->previous[0]->next.size() == 1)
			{
				for (auto prb : pBoundary->previous)
				{
					HadLaneBoundary* pCurBoundary = (HadLaneBoundary*)prb;
					HadLaneGroup* pCurGroup = (HadLaneGroup*)pGroup->previous[0];
					std::vector<HadLaneBoundary*> laneBoundaries = pCurGroup->laneBoundaries;
					if (std::find(laneBoundaries.begin(), laneBoundaries.end(), pCurBoundary) == laneBoundaries.end())
						continue;

					if (std::find(connectedBoundarys.begin(), connectedBoundarys.end(), pCurBoundary) != connectedBoundarys.end())
						return;

					if (std::find(connectedLaneGroups.begin(), connectedLaneGroups.end(), pCurGroup) != connectedLaneGroups.end())
						return;

					connectedBoundarys.push_front(pCurBoundary);
					connectedLaneGroups.push_front(pCurGroup);
					connectForwardLaneBoundary(pCurGroup, pCurBoundary, connectedBoundarys, connectedLaneGroups, --connectNum);
				}
			}
		}
	}

	void Compiler::connectBackwardLaneBoundary(HadLaneGroup* pGroup, HadLaneBoundary* pBoundary, 
		std::deque<HadLaneBoundary*>& connectedBoundarys, std::deque<HadLaneGroup*>& connectedLaneGroups, int connectNum)
	{
		//1连1的情况。即：当前道路边的入度为1，前序道路边的出度为1。
		if (connectNum)
		{
			if (pBoundary->next.size() == 1 && pBoundary->next[0]->previous.size() == 1)
			{
				for (auto grp : pGroup->next)
				{
					HadLaneGroup* pCurGroup = (HadLaneGroup*)grp;
					HadLaneBoundary* pCurBoundary = (HadLaneBoundary*)pBoundary->next[0];
					std::vector<HadLaneBoundary*> laneBoundaries = pCurGroup->laneBoundaries;
					if (std::find(laneBoundaries.begin(), laneBoundaries.end(), pCurBoundary) == laneBoundaries.end())
						continue;

					if (std::find(connectedBoundarys.begin(), connectedBoundarys.end(), pCurBoundary) != connectedBoundarys.end())
						return;

					if (std::find(connectedLaneGroups.begin(), connectedLaneGroups.end(), pCurGroup) != connectedLaneGroups.end())
						return;

					connectedBoundarys.push_back(pCurBoundary);
					connectedLaneGroups.push_back(pCurGroup);
					connectBackwardLaneBoundary(pCurGroup, pCurBoundary, connectedBoundarys, connectedLaneGroups, --connectNum);
				}
			}
			else if (pGroup->next.size() == 1 && pGroup->next[0]->previous.size() == 1)
			{
				for (auto prb : pBoundary->next)
				{
					HadLaneBoundary* pCurBoundary = (HadLaneBoundary*)prb;
					HadLaneGroup* pCurGroup = (HadLaneGroup*)pGroup->next[0];
					std::vector<HadLaneBoundary*> laneBoundaries = pCurGroup->laneBoundaries;
					if (std::find(laneBoundaries.begin(), laneBoundaries.end(), pCurBoundary) == laneBoundaries.end())
						continue;

					if (std::find(connectedBoundarys.begin(), connectedBoundarys.end(), pCurBoundary) != connectedBoundarys.end())
						return;

					if (std::find(connectedLaneGroups.begin(), connectedLaneGroups.end(), pCurGroup) != connectedLaneGroups.end())
						return;

					connectedBoundarys.push_back(pCurBoundary);
					connectedLaneGroups.push_back(pCurGroup);
					connectBackwardLaneBoundary(pCurGroup, pCurBoundary, connectedBoundarys, connectedLaneGroups, --connectNum);
				}
			}
		}
	}

	void Compiler::getConnectedGroups(HadLaneGroup* pLaneGroup, std::vector<std::vector<HadLaneGroup*>>& allConnectedGroups, int connectNum)
	{
		// previous
		std::vector<std::vector<HadLaneGroup*>> tmpConnectedGroups;
		std::vector<HadLaneGroup*> previousGroups;
		for (auto p : pLaneGroup->previous)
			previousGroups.push_back((HadLaneGroup*)p);
		for (auto topoGroup : previousGroups) {
			std::vector<HadLaneGroup*> connectedGroups;
			getConnectGroup(pLaneGroup, topoGroup, connectedGroups, false, connectNum);
			tmpConnectedGroups.push_back(connectedGroups);
		}

		// current
		if (!tmpConnectedGroups.empty()) {
			for (auto& tmpConnectedGroup : tmpConnectedGroups)
				tmpConnectedGroup.push_back(pLaneGroup);
		}
		else
		{
			std::vector<HadLaneGroup*> connectedGroups;
			connectedGroups.push_back(pLaneGroup);
			tmpConnectedGroups.push_back(connectedGroups);
		}

		// next
		std::vector<HadLaneGroup*> nextGroups;
		for (auto n : pLaneGroup->next)
			nextGroups.push_back((HadLaneGroup*)n);
		for (auto topoGroup : nextGroups) {
			std::vector<HadLaneGroup*> connectedGroups;
			getConnectGroup(pLaneGroup, topoGroup, connectedGroups, true, connectNum);
			for (auto& tmpConnectedGroup : tmpConnectedGroups) {
				std::vector<HadLaneGroup*> newConnectedGroup{ tmpConnectedGroup.begin(), tmpConnectedGroup.end() };
				newConnectedGroup.insert(newConnectedGroup.end(), connectedGroups.begin(), connectedGroups.end());
				allConnectedGroups.push_back(newConnectedGroup);
			}
		}
		if (nextGroups.empty()) {
			allConnectedGroups = tmpConnectedGroups;
		}
	}

	void Compiler::getConnectGroup(HadLaneGroup* pLaneGroup, HadLaneGroup* pTopoGroup, 
		std::vector<HadLaneGroup*>& connectedGroups, bool forward, int connectNum)
	{
		HadLaneGroup* pCurrentGroup = pLaneGroup;
		HadLaneGroup* pNextGroup = pTopoGroup;
		if (pNextGroup == nullptr) {
			return;
		}

		if (forward) {
			pLaneGroup = pNextGroup;
			connectedGroups.push_back(pLaneGroup);
		}
		else {
			std::swap(pCurrentGroup, pNextGroup);
			pLaneGroup = pCurrentGroup;
			connectedGroups.emplace(connectedGroups.begin(), pLaneGroup);
		}

		if (connectedGroups.size() < connectNum) {
			std::vector<HadLaneGroup*> topoGroups;
			if (forward) {
				for (auto n : pLaneGroup->next)
					topoGroups.push_back((HadLaneGroup*)n);
			} else {
				for (auto p : pLaneGroup->previous)
					topoGroups.push_back((HadLaneGroup*)p);
			}

			if (topoGroups.size() == 1) {
				getConnectGroup(pLaneGroup, topoGroups[0], connectedGroups, forward, connectNum);
			}
		}
	}

    bool Compiler::checkOverlayOnLaneGroup(HadLaneGroup* pLaneGroup)
    {
        auto getLaneGroupPolygon = [&](HadLaneGroup* laneGroup)->std::vector<MapPoint3D64> {
			
				MapPoint3D64* pPolygon = NULL;
               size_t nPolygonPtNum = 0;

			   LineString3d leftSide, rightSide;
			   getLaneGroupBoundary(laneGroup, leftSide, rightSide);

			   Polygon3d polyGon;
			   makeLaneGroupPolygon(leftSide.vertexes, rightSide.vertexes, polyGon);
               return polyGon.vertexes;
        };

		std::vector<MapPoint3D64> boxPts;
		boxPts.emplace_back(pLaneGroup->extent.min);
		boxPts.emplace_back(pLaneGroup->extent.max);

        auto laneGroupPolyGon = getLaneGroupPolygon(pLaneGroup);

		bool bOverlay = false;
        getLaneGroupBox2DRTree()->query(bgi::intersects(BOX_2T(boxPts)),

            boost::make_function_output_iterator([&](size_t const& id) {
                HadLaneGroup* nearByLaneGroup = compilerData.gridLaneGroups[id];
                if (pLaneGroup->originId == nearByLaneGroup->originId || bOverlay)
                    return;


                auto nearLaneGroupPolyGon = getLaneGroupPolygon(nearByLaneGroup);

                std::vector<ring_2t> results;
                bg::intersection(RING_2T(laneGroupPolyGon), RING_2T(nearLaneGroupPolyGon), results);

                if (!results.empty())
                {
                    point_2t tmpCenterPoint2D;
                    bg::centroid(results.front(), tmpCenterPoint2D);
					point_t tmpCenterPoint = point_t(
						tmpCenterPoint2D.get<0>(),
						tmpCenterPoint2D.get<1>(),
						(pLaneGroup->extent.max.z + pLaneGroup->extent.min.z) * 5);
                    point_t currentNearbyPoint = getNearestPoint(LINESTRING_T(laneGroupPolyGon), tmpCenterPoint);
                    point_t otherNearbyPoint = getNearestPoint(LINESTRING_T(nearLaneGroupPolyGon), tmpCenterPoint);
                    int tol = otherNearbyPoint.get<2>() - currentNearbyPoint.get<2>();
					if (tol > 1000)
					{
						bOverlay = true;
					}
                }
                }));

		return bOverlay;
        
    }

    bool Compiler::checkOverlayOnLink(HadLink* pLink)
    {
		for (auto pLaneGroup : pLink->groups)
		{
			if (checkOverlayOnLaneGroup(pLaneGroup))
			{
				return true;
			}
		}
		return false;
    }

    bool Compiler::sideEqual(HadRoadBoundary* const pBoundary, HadLaneGroup* const pGroup, int side)
	{
		if (pBoundary == nullptr || pGroup == nullptr)
			return false;
		if (pBoundary->relLgs.count(pGroup->originId) && pBoundary->relLgs[pGroup->originId].side == side)
			return true;
		return false;
	}

	bool Compiler::directionEqual(HadLaneBoundary* const pBoundary, HadLaneGroup* const pGroup, int direction)
	{
		if (pBoundary == nullptr || pGroup == nullptr)
			return false;
		if (pBoundary->relLgs.count(pGroup->originId) && pBoundary->relLgs[pGroup->originId].lgMarkDirect == direction)
			return true;
		return false;
	}

	bool Compiler::directionEqual(HadRoadBoundary* const pBoundary, HadLaneGroup* const pGroup, int direction)
	{
		if (pBoundary == nullptr || pGroup == nullptr)
			return false;
		if (pBoundary->relLgs.count(pGroup->originId) && pBoundary->relLgs[pGroup->originId].direction == direction)
			return true;
		return false;
	}

	LineString3d Compiler::getBoundaryLocation(HadLaneBoundary* const pBoundary, HadLaneGroup* const pGroup)
	{
		LineString3d location = pBoundary->location;
		if (directionEqual(pBoundary, pGroup, 3)) {
			std::reverse(location.vertexes.begin(), location.vertexes.end());
		}
		return location;
	}

	LineString3d Compiler::getBoundaryLocation(HadRoadBoundary* const pBoundary, HadLaneGroup* const pGroup)
	{
		LineString3d location = pBoundary->location;
		if (directionEqual(pBoundary, pGroup, 3)) {
			std::reverse(location.vertexes.begin(), location.vertexes.end());
		}
		return location;
	}

	void Compiler::projectLineOnRoadSurface(const LineString3d& leftRdBoundary, const LineString3d& rightRdBoundary, const std::vector<MapPoint3D64>& line, std::vector<MapPoint3D64>& lineOnRoadSurface, double tolerance/* = 0*/)
	{
		auto getPointIter = [](std::vector<MapPoint3D64>& points, MapPoint3D64& pt) {
			return std::find_if(points.begin(), points.end(), [&](MapPoint3D64& p) {
				return &p == &pt;
				});
		};

		// 根据道路边界线调整车道线高度
		for_each(line.begin(), line.end(), [&](const MapPoint3D64& point) -> void
			{
				MapPoint3D64 pointOnRoadSurface;
				projectPointOnRoadSurface(leftRdBoundary, rightRdBoundary, point, pointOnRoadSurface, tolerance);
				lineOnRoadSurface.push_back(pointOnRoadSurface);
			});

		// 增加坐标点密度,不然可能会由于密度太低坐标直连渲染在路面下方
		const float INTERPOLATION_POINT_IN_LINE_EPSILON = 10000.f;  // ≈10m
		for_each(leftRdBoundary.vertexes.begin(), leftRdBoundary.vertexes.end(), [&](const MapPoint3D64& leftRdPoint) -> void
			{
				size_t start, end;
				MapPoint3D64 grappedPt, nearestPt;
				if (GrapPointAlgorithm::grapOrMatchNearestPoint(leftRdPoint, lineOnRoadSurface, grappedPt, nearestPt, start, end)) {
					if (grappedPt.pos.distance(nearestPt.pos) > INTERPOLATION_POINT_IN_LINE_EPSILON) {
						MapPoint3D64 pointOnRoadSurface;
						projectPointOnRoadSurface(leftRdBoundary, rightRdBoundary, grappedPt, pointOnRoadSurface, tolerance);
						lineOnRoadSurface.insert(getPointIter(lineOnRoadSurface, lineOnRoadSurface[end]), pointOnRoadSurface);
					}
				}
			});

		for_each(rightRdBoundary.vertexes.begin(), rightRdBoundary.vertexes.end(), [&](const MapPoint3D64& rightRdPoint) -> void
			{
				size_t start, end;
				MapPoint3D64 grappedPt, nearestPt;
				if (GrapPointAlgorithm::grapOrMatchNearestPoint(rightRdPoint, lineOnRoadSurface, grappedPt, nearestPt, start, end)) {
					if (grappedPt.pos.distance(nearestPt.pos) > INTERPOLATION_POINT_IN_LINE_EPSILON) {
						MapPoint3D64 pointOnRoadSurface;
						projectPointOnRoadSurface(leftRdBoundary, rightRdBoundary, grappedPt, pointOnRoadSurface, tolerance);
						lineOnRoadSurface.insert(getPointIter(lineOnRoadSurface, lineOnRoadSurface[end]), pointOnRoadSurface);
					}
				}
			});
	}

	void Compiler::projectPointOnRoadSurface(const LineString3d& leftRdBoundary, const LineString3d& rightRdBoundary, const MapPoint3D64& point, MapPoint3D64& pointOnRoadSurface, double tolerance/* = 0*/)
	{
		pointOnRoadSurface = point;
		MapPoint3D64 firstGrapPoint;
		MapPoint3D64 secondGrapPoint;

		size_t start, end;
		bool firstGrapPointOk = false;
		bool secondGrapPointOk = false;
		firstGrapPointOk = GrapPointAlgorithm::grapPoint(point, leftRdBoundary.vertexes, firstGrapPoint, start, end);
		secondGrapPointOk = GrapPointAlgorithm::grapPoint(point, rightRdBoundary.vertexes, secondGrapPoint, start, end);
		if (!firstGrapPointOk && !secondGrapPointOk) {
			return;
		}
		else if (firstGrapPointOk && !secondGrapPointOk) {
			int secondGrapPointIdx = GrapPointAlgorithm::findNearestPoint(rightRdBoundary.vertexes, point);
			secondGrapPoint = rightRdBoundary.vertexes[secondGrapPointIdx];
		}
		else if (!firstGrapPointOk && secondGrapPointOk) {
			int firstGrapPointIdx = GrapPointAlgorithm::findNearestPoint(leftRdBoundary.vertexes, point);
			firstGrapPoint = leftRdBoundary.vertexes[firstGrapPointIdx];
		}

		// 大圆弧时存在本侧抓不到点,对侧抓到点,此时距离很大
		double d1 = firstGrapPoint.pos.distance(point.pos);
		MapPoint3D64 leftRdNearestPoint = GrapPointAlgorithm::nearestEndPoint(point, leftRdBoundary.vertexes);
		if (d1 > leftRdNearestPoint.pos.distance(point.pos)) {
			return;
		}

		double d2 = secondGrapPoint.pos.distance(point.pos);
		MapPoint3D64 rightRdNearestPoint = GrapPointAlgorithm::nearestEndPoint(point, rightRdBoundary.vertexes);
		if (d2 > rightRdNearestPoint.pos.distance(point.pos)) {
			return;
		}

		double sum = d1 + d2;
		// 大圆弧时两个点都在对侧抓到,此时距离很大
		double grapPointSum = firstGrapPoint.pos.distance(secondGrapPoint.pos);
		if (sum / grapPointSum > 1.1) {
			return;
		}

		double z = firstGrapPoint.z * d2 / sum + secondGrapPoint.z * d1 / sum;
		if (tolerance)
		{
			if (fabs(point.z - z) < tolerance) {
				pointOnRoadSurface.z = (int32)z;
			}
		}
		else
		{
			pointOnRoadSurface.z = (int32)z;
		}
	}

	void Compiler::densityRoadSurface(const std::vector<MapPoint3D64>& line, std::vector<MapPoint3D64>& lineOnRoadSurface, double maxtolerance/* = 100000*/)
	{
		lineOnRoadSurface = line;
		if (lineOnRoadSurface.empty())
			return;
		auto iter = lineOnRoadSurface.begin();
		while (iter != lineOnRoadSurface.end() - 1) {
			auto& currPt = *iter;
			auto& nextPt = *(iter + 1);
			double sum = currPt.pos.distance(nextPt.pos);
			if (sum < maxtolerance) {
				iter++;
				continue;
			}

			MapPoint3D64 pt = currPt;
			pt.pos.lon += nextPt.pos.lon;
			pt.pos.lat += nextPt.pos.lat;
			pt.z += nextPt.z;

			pt.pos.lon /= 2;
			pt.pos.lat /= 2;
			pt.z /= (int32)2;
			iter = lineOnRoadSurface.insert(iter + 1, pt);
			iter--;
		}
	}

	/// @brief 计算线段方向矢量
	Vector3d Compiler::calDirection(MapPoint3D64 start, MapPoint3D64 end)
	{
		MapPoint3D64Converter c;
		c.setBasePoint(start);
		c.convert(&start, 1);
		c.convert(&end, 1);

		Vector3d v;
		v.x = end.pos.lon - start.pos.lon;
		v.y = end.pos.lat - start.pos.lat;
		v.z = end.z - start.z;

		double length = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
		double lenInv = 1.0 / length;
		v.x *= lenInv;
		v.y *= lenInv;
		v.z *= lenInv;

		return v;
	}

	MapPoint3D64 Compiler::_nearestLineStringEnd(const MapPoint3D64& pt, const LineString3d& lineString)
	{
		int64 frontDis = _squaredHDistance(pt, lineString.vertexes.front());
		int64 backDis = _squaredHDistance(pt, lineString.vertexes.back());

		return frontDis < backDis ? lineString.vertexes.front() : lineString.vertexes.back();
	}

	float Compiler::calcLength(const std::vector<MapPoint3D64> polyline)
	{
		float len = 0.0f;
		MapPoint64 minPt = MapPoint64::make(INT64_MAX, INT64_MAX);
		for (auto iter = polyline.begin(); iter != polyline.end(); ++iter)
		{
			if ((*iter).pos.lon < minPt.lon)
			{
				minPt.lon = (*iter).pos.lon;
			}

			if ((*iter).pos.lat < minPt.lat)
			{
				minPt.lat = (*iter).pos.lat;
			}
		}

		for (size_t i = 0; i < polyline.size() - 1; i++)
		{
			MapPoint64 temp = polyline[i].pos - minPt;
			double dis = (polyline[i].pos - minPt).distance(polyline[i + 1].pos - minPt);
			len += dis;
		}

		return len;
	}

	float Compiler::calcLength(const std::vector<MapPoint3D64> polyline, const MapPoint3D64& pt)
	{
		float len = 0.0f;

		MapPoint64 minPt = MapPoint64::make(INT64_MAX, INT64_MAX);
		for (auto iter = polyline.begin(); iter != polyline.end(); ++iter)
		{
			if ((*iter).pos.lon < minPt.lon)
			{
				minPt.lon = (*iter).pos.lon;
			}

			if ((*iter).pos.lat < minPt.lat)
			{
				minPt.lat = (*iter).pos.lat;
			}
		}

		size_t si;
		size_t ei;
		MapPoint3D64 grapPt;
		if (!GrapPointAlgorithm::grapPoint(pt, polyline, grapPt, si, ei))
		{
			len = (pt.pos - minPt).distance(polyline[0].pos - minPt);
		}
		else
		{
			for (size_t i = 0; i < si; i++)
			{
				MapPoint64 temp = polyline[i].pos - minPt;
				double dis = (polyline[i].pos - minPt).distance(polyline[i + 1].pos - minPt);
				len += dis;
			}
			double dis = (polyline[si].pos - minPt).distance(grapPt.pos - minPt);
			len += dis;
		}

		return len;
	}

	float Compiler::calcLength(MapPoint3D64& pt1, MapPoint3D64& pt2)
	{
		double dis = (pt1.pos).distance(pt2.pos);
		return dis;
	}

	bool Compiler::_intersect(MapPoint3D64* points1, size_t pointCount1, MapPoint3D64* points2, size_t pointCount2, ClipperLib::PolyTree& polyTree)
	{
		ClipperLib::Path path_v1;
		for (int i = 0; i < pointCount1; i++)
			path_v1 << ClipperLib::IntPoint(points1[i].pos.lon, points1[i].pos.lat, 0);

		ClipperLib::Path path_v2;
		for (int i = 0; i < pointCount2; i++)
			path_v2 << ClipperLib::IntPoint(points2[i].pos.lon, points2[i].pos.lat, 0);

		ClipperLib::Clipper clipper;
		clipper.AddPath(path_v1, ClipperLib::ptSubject, true);
		clipper.AddPath(path_v2, ClipperLib::ptClip, true);
		clipper.Execute(ClipperLib::ctIntersection, polyTree, ClipperLib::pftNonZero);

		if (polyTree.Total() > 0)
			return true;
		else
			return false;
	}

	int64 Compiler::_squaredHDistance(const MapPoint3D64& pt1, const MapPoint3D64& pt2)
	{
		return (pt1.pos.lon - pt2.pos.lon) * (pt1.pos.lon - pt2.pos.lon) + (pt1.pos.lat - pt2.pos.lat) * (pt1.pos.lat - pt2.pos.lat);
	}

	void Compiler::getLaneGroupBoundary(HadLaneGroup* pGroup, LineString3d& leftSide, LineString3d& rightSide, bool isUseHullGroup)
	{
		// 使用道路边界和车道边界
		if (pGroup->roadBoundaries.size() == 2)
		{
			leftSide = pGroup->roadBoundaries[0]->location;
			if (directionEqual(pGroup->roadBoundaries[0], pGroup, 3))
			{
				std::reverse(leftSide.vertexes.begin(), leftSide.vertexes.end());
			}
			rightSide = pGroup->roadBoundaries[1]->location;
			if (directionEqual(pGroup->roadBoundaries[1], pGroup, 3))
			{
				// 反向，进行点的反转
				std::reverse(rightSide.vertexes.begin(), rightSide.vertexes.end());
			}
		}
		else if (pGroup->laneBoundaries.size() >= 2)
		{
			leftSide = pGroup->laneBoundaries[0]->location;
			if (directionEqual(pGroup->laneBoundaries[0], pGroup, 3))
			{
				std::reverse(leftSide.vertexes.begin(), leftSide.vertexes.end());
			}
			auto rightIdx = pGroup->laneBoundaries.size() - 1;
			rightSide = pGroup->laneBoundaries[rightIdx]->location;
			if (directionEqual(pGroup->laneBoundaries[rightIdx], pGroup, 3))
			{
				// 反向，进行点的反转
				std::reverse(rightSide.vertexes.begin(), rightSide.vertexes.end());
			}
		}
		auto leftSideIp = std::unique(leftSide.vertexes.begin(), leftSide.vertexes.end(), mapPoint3D64_compare);
		leftSide.vertexes.resize(std::distance(leftSide.vertexes.begin(), leftSideIp));
		auto rightSideIp = std::unique(rightSide.vertexes.begin(), rightSide.vertexes.end(), mapPoint3D64_compare);
		rightSide.vertexes.resize(std::distance(rightSide.vertexes.begin(), rightSideIp));

		if (isUseHullGroup && !isProDataLevel(pGroup))
		{
			auto resultIt = std::find(hullGroups.begin(), hullGroups.end(), pGroup);
			if (resultIt != hullGroups.end())
			{
				size_t index = std::distance(hullGroups.begin(), resultIt);  
				bool isLeftHull = isLeftHulls.at(index);
				if (pGroup->laneBoundaries.size() >= 3)
				{
					if (isLeftHull)
					{
						leftSide = pGroup->laneBoundaries[1]->location;
						if (directionEqual(pGroup->laneBoundaries[1], pGroup, 3))
							std::reverse(leftSide.vertexes.begin(), leftSide.vertexes.end());
						auto leftSideIp = std::unique(leftSide.vertexes.begin(), leftSide.vertexes.end(), mapPoint3D64_compare);
						leftSide.vertexes.resize(std::distance(leftSide.vertexes.begin(), leftSideIp));
					}
					else
					{
						auto rightIdx = pGroup->laneBoundaries.size() - 2;
						rightSide = pGroup->laneBoundaries[rightIdx]->location;
						if (directionEqual(pGroup->laneBoundaries[rightIdx], pGroup, 3))
							std::reverse(rightSide.vertexes.begin(), rightSide.vertexes.end());
						auto rightSideIp = std::unique(rightSide.vertexes.begin(), rightSide.vertexes.end(), mapPoint3D64_compare);
						rightSide.vertexes.resize(std::distance(rightSide.vertexes.begin(), rightSideIp));
					}
				}
			}
		}

	}

	void Compiler::makeLaneGroupPolygon(const std::vector<MapPoint3D64>& leftSide, const std::vector<MapPoint3D64>& rightSide, Polygon3d& polygon)
	{
		polygon.vertexes.clear();
		MapPoint3D64 prevVertex = {};
		for_each(leftSide.begin(), leftSide.end(), [&](auto& vertex) {
			if (!floatEqual(vertex.pos.distanceSquare(prevVertex.pos), 0)) {
				polygon.vertexes.push_back(vertex);
				prevVertex = vertex;
			}
		});
		for_each(rightSide.rbegin(), rightSide.rend(), [&](auto& vertex) {
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

	size_t Compiler::makeLaneGroupPolygon(HadLaneGroup* pGroup, MapPoint3D64*& polygon)
	{
		if (pGroup->laneBoundaries.size() < 2)
			return 0;

		size_t laneBoundaryNum = pGroup->laneBoundaries.size();
		size_t nPolyline1PtNum = pGroup->laneBoundaries[0]->location.vertexes.size();
		size_t nPolyline2PtNum = pGroup->laneBoundaries[laneBoundaryNum - 1]->location.vertexes.size();
		size_t nPtNum = nPolyline1PtNum + nPolyline2PtNum + 1 + (laneBoundaryNum - 2) * 2;

		//添加路面左边线几何点。
		polygon = new MapPoint3D64[nPtNum];
		MapPoint3D64* pCurPt = polygon;
		memcpy(pCurPt, pGroup->laneBoundaries[0]->location.vertexes.data(), nPolyline1PtNum * sizeof(MapPoint3D64));
		pCurPt += nPolyline1PtNum;

		//添加除道路边线外的各车道边界线末点。
		for (int i = 1; i <= laneBoundaryNum - 2; i++)
		{
			*pCurPt = pGroup->laneBoundaries[i]->location.vertexes.back();
			pCurPt++;
		}

		//添加路面右边线几何点。
		std::vector<MapPoint3D64> Line2Reverse(nPolyline2PtNum);
		std::reverse_copy(pGroup->laneBoundaries[laneBoundaryNum - 1]->location.vertexes.begin(), pGroup->laneBoundaries[laneBoundaryNum - 1]->location.vertexes.end(), Line2Reverse.begin());
		memcpy(pCurPt, Line2Reverse.data(), nPolyline2PtNum * sizeof(MapPoint3D64));
		pCurPt += nPolyline2PtNum;

		//添加除道路边线外的各车道边界线首点。
		for (size_t i = laneBoundaryNum - 2; i >= 1; i--)
		{
			*pCurPt = pGroup->laneBoundaries[i]->location.vertexes.front();
			pCurPt++;
		}

		polygon[nPtNum - 1] = pGroup->laneBoundaries[0]->location.vertexes[0];

		return nPtNum;
	}

	BoundingBox2d Compiler::makeBoundingBox2d(const std::vector<MapPoint3D64>& vertexes)
	{
		std::vector<int64> vx;
		std::vector<int64> vy;
		for (auto& vertex : vertexes) {
			vx.push_back(vertex.pos.lon);
			vy.push_back(vertex.pos.lat);
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

	Box2TRTree::Ptr Compiler::getIntersectionBox2TRTree()
	{
		auto& tmp = compilerData.m_rtreeIntersectionBox;
		if (tmp)
			return tmp;

		Box2TRTree::guardParam param;
		auto& gridIntersectionBoxes = compilerData.m_rdsIntersectionBoxes;
		Box2TRTree::indexGetterBox originIndBox(gridIntersectionBoxes);
		auto rtreeBox = std::make_shared<Box2TRTree::RTree>(boost::irange<std::size_t>(0lu, gridIntersectionBoxes.size()), param, originIndBox);
		compilerData.m_rtreeIntersectionBox = rtreeBox;
		return rtreeBox;
	}

	Box2TRTree::Ptr Compiler::getRoadBox2TRTree()
	{
		auto& tmp = compilerData.m_rtreeRoadBox;
		if (tmp)
			return tmp;

		Box2TRTree::guardParam param;
		auto& gridRoadBoxes = compilerData.m_rdsRoadBoxes;
		Box2TRTree::indexGetterBox originIndBox(gridRoadBoxes);
		auto rtreeBox = std::make_shared<Box2TRTree::RTree>(boost::irange<std::size_t>(0lu, gridRoadBoxes.size()), param, originIndBox);
		compilerData.m_rtreeRoadBox = rtreeBox;
		return rtreeBox;
	}

	SegmentRTree::Ptr Compiler::getIntersectionStopLineSegmentRTree()
	{
		auto& tmp = compilerData.rtreeGridIntersectionStopLines;
		if (tmp)
			return tmp;
		SegmentRTree::guardParam param;
		SegmentRTree::indexGetterSegment originIndSeg(compilerData.gridIntectionStopLines);
		auto tmpRTree = std::make_shared<SegmentRTree::RTree>(boost::irange<std::size_t>(0lu, compilerData.gridIntectionStopLines.size()), param, originIndSeg);
		compilerData.rtreeGridIntersectionStopLines = tmpRTree;
		return tmpRTree;
	}

	Box2TRTree::Ptr Compiler::getLaneGroupBox2DRTree()
	{
        auto& tmp = compilerData.m_rtreeLaneGroup;
        if (tmp)
            return tmp;
		Box2TRTree::guardParam param;
		Box2TRTree::indexGetterBox originIndBox(compilerData.m_laneGroupBoxes);
        auto rtreeBox = std::make_shared<Box2TRTree::RTree>(boost::irange<std::size_t>(0lu, compilerData.m_laneGroupBoxes.size()), param, originIndBox);
		compilerData.m_rtreeLaneGroup = rtreeBox;
        return rtreeBox;
	}

	Box2TRTree::Ptr Compiler::getStopLineBox2TRTree()
	{
		auto& tmp = compilerData.m_rtreeStopLineBox;
		if (tmp)
			return tmp;

		Box2TRTree::guardParam param;
		auto& gridBoxes = compilerData.m_rdsStopLineBoxes;
		Box2TRTree::indexGetterBox originIndBox(gridBoxes);
		auto rtreeBox = std::make_shared<Box2TRTree::RTree>(boost::irange<std::size_t>(0lu, gridBoxes.size()), param, originIndBox);
		compilerData.m_rtreeStopLineBox = rtreeBox;
		return rtreeBox;
	}

	point_t Compiler::getNearestPoint(
		const std::vector<point_t>& points,
		const point_t& originPoint)
	{
		parameters param;
		index_getter originInd(points);
		rtree_type rtree(boost::irange<std::size_t>(0lu, points.size()), param, originInd);
		std::vector<point_t> resultPoints;
		rtree.query(bgi::nearest(originPoint, 1),
			boost::make_function_output_iterator([&](size_t const& id) {
				resultPoints.push_back(points[id]);
				}));
		if (!resultPoints.empty())
			return resultPoints.front();
		return point_t(0, 0);
	}


	void Compiler::getClosestSeg(
		const point_t& point,
		std::vector<point_t>& line,
		std::vector<point_t>::iterator& it_min1,
		std::vector<point_t>::iterator& it_min2)
	{
		std::vector<point_t>::iterator prev_min_dist;
		double dist_min = 1.0e10;
		auto first = line.begin();
		auto last = line.end();
		auto it = first;
		auto prev = it++;
		if (it == last)
		{
			it_min1 = it_min2 = first;
			dist_min = bg::distance(point, *first);
			return;
		}
		segment_t tmpSeg;
		bg::closest_points(point, segment_t(*prev, *it), tmpSeg);
		dist_min = bg::length(tmpSeg);
		prev_min_dist = prev;
		for (++prev, ++it; it != last; ++prev, ++it)
		{
			bg::closest_points(point, segment_t(*prev, *it), tmpSeg);
			double const dist = bg::length(tmpSeg);
			if (dist == 0.0)
			{
				dist_min = 0.0;
				it_min1 = prev;
				it_min2 = it;
				return;
			}
			else if (dist < dist_min)
			{
				dist_min = dist;
				prev_min_dist = prev;
			}
		}
		it_min1 = it_min2 = prev_min_dist;
		++it_min2;
	}

	void Compiler::getClosestSeg(
		const MapPoint3D64& point,
		std::vector<MapPoint3D64>& line,
		std::vector<MapPoint3D64>::iterator& it_min1,
		std::vector<MapPoint3D64>::iterator& it_min2)
	{
		std::vector<MapPoint3D64>::iterator prev_min_dist;
		double dist_min = 1.0e10;
		auto first = line.begin();
		auto last = line.end();
		auto it = first;
		auto prev = it++;

		//
		auto tmpPoint = point;
		coordinatesTransform.convert(&tmpPoint, 1);
		//

		if (it == last)
		{
			it_min1 = it_min2 = first;

			//
			auto tmpFirst = *first;
			coordinatesTransform.convert(&tmpFirst, 1);
			//

			dist_min = bg::distance(POINT_T(tmpPoint), POINT_T(tmpFirst));
			return;
		}

		//
		auto tmpPrev = *prev;
		auto tmpIt = *it;
		coordinatesTransform.convert(&tmpPrev, 1);
		coordinatesTransform.convert(&tmpIt, 1);
		//

		segment_t tmpSeg;
		bg::closest_points(POINT_T(tmpPoint), segment_t(POINT_T(tmpPrev), POINT_T(tmpIt)), tmpSeg);
		dist_min = bg::length(tmpSeg);

		prev_min_dist = prev;
		for (++prev, ++it; it != last; ++prev, ++it)
		{
			//
			tmpPrev = *prev;
			tmpIt = *it;
			coordinatesTransform.convert(&tmpPrev, 1);
			coordinatesTransform.convert(&tmpIt, 1);
			//

			bg::closest_points(POINT_T(tmpPoint), segment_t(POINT_T(tmpPrev), POINT_T(tmpIt)), tmpSeg);
			double const dist = bg::length(tmpSeg);

			if (dist == 0.0)
			{
				dist_min = 0.0;
				it_min1 = prev;
				it_min2 = it;
				return;
			}
			else if (dist < dist_min)
			{
				dist_min = dist;
				prev_min_dist = prev;
			}
		}
		it_min1 = it_min2 = prev_min_dist;
		++it_min2;
	}

	void Compiler::getClosestSeg(
		const point_2t& point,
		std::vector<point_2t>& line,
		std::vector<point_2t>::iterator& it_min1,
		std::vector<point_2t>::iterator& it_min2)
	{
		std::vector<point_2t>::iterator prev_min_dist;
		double dist_min = 1.0e10;
		auto first = line.begin();
		auto last = line.end();
		auto it = first;
		auto prev = it++;
		if (it == last)
		{
			it_min1 = it_min2 = first;
			dist_min = bg::distance(point, *first);
			return;
		}
		segment_2t tmpSeg;
		bg::closest_points(point, segment_2t(*prev, *it), tmpSeg);
		dist_min = bg::length(tmpSeg);
		prev_min_dist = prev;
		for (++prev, ++it; it != last; ++prev, ++it)
		{
			bg::closest_points(point, segment_2t(*prev, *it), tmpSeg);
			double const dist = bg::length(tmpSeg);
			if (dist == 0.0)
			{
				dist_min = 0.0;
				it_min1 = prev;
				it_min2 = it;
				return;
			}
			else if (dist < dist_min)
			{
				dist_min = dist;
				prev_min_dist = prev;
			}
		}
		it_min1 = it_min2 = prev_min_dist;
		++it_min2;
	}
}

