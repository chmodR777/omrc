#include "stdafx.h"
#include "GroupCompiler.h"
#include "math3d/vector2.h"
#include "../framework/SpatialSeacher.h"

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include "CompileSetting.h"
#include "algorithm/polyline_simplifier.h"


#include "grap_point_algorithm.h"

namespace OMDB
{
	// 待调整图幅
	const static std::set<int32> meshId{ 
		20169125, 20169136, 20169137, 20169139, 20169142, 20169148, 20169149,	//(12139059,3121932)  北横
		20169110, 20169111, 20169154,	// (12146343,3114418) 上中
		20169199, 20169210, 20169208 };	// (12156728, 3126991) 军工路
	// 隧道最小层高
	#define FLOOR_MIX_HEIGHT (7000)	
	// 统一调整高度
	#define ADJUST_UNIQUE_HEIGHT (5000)
	// 坡度斜率
	#define SLOPE_GRADIENT  (-0.02)	

	//#define CONVERT_TAG

	void GroupCompiler::compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile)
	{
		m_pGrid = pGrid;
		CompileSetting* pSetting = CompileSetting::instance();

		//只抽稀道路边界、车道边界、车道中心线，不影响CLM生成
		simplifierLine(pGrid, nearby);

		//抽稀1公里以内比较直的路和处理车道组凹包问题
		setCurrentLaneGroupData(pGrid);
		interpolateConvexHullPoints(pGrid);

		//绝对高度转相对高度
		if (pSetting->isModifyHeightFromAbsToRel)
			modifyGridHeight(pGrid, nearby);

        std::vector<HadGrid*> grids = nearby;
        grids.push_back(pGrid);
        for (auto grid : grids)
        {
			constructLaneGroupRtree(grid);
        }

#ifdef CONVERT_TAG
		//coordinatesTransform.setBasePoint(MapPoint3D64{ MapPoint64{0,0},0 });
#endif

		if (!pSetting->isDaimlerShangHai)
		{
			if (!pSetting->isGenerateHdData)
			{
				if (meshId.count(pGrid->getId()))
				{
					adjustRoad(pGrid, nearby);
				}
			}
		}

		for (auto obj : pGrid->query(ElementType::HAD_LANE_GROUP))
		{
			HadLaneGroup* pLinkGroup = (HadLaneGroup*)obj;
			RdsGroup* pGroup = (RdsGroup*)createObject(pTile, EntityType::RDS_GROUP);
			pGroup->originId = pLinkGroup->originId;

		}
		
	}
	void GroupCompiler::adjustRoad(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby)
	{

		std::vector<HadGrid*> grids = nearby;
		grids.push_back(pGrid);
		for (auto grid : grids)
		{
			setLaneGroupData(grid);
		}
		updateInterpolatedLine();
		updateRefLine();

		grabLowTunnel(pGrid, nearby);
		divideToGroup();
		mergeGroup();
		adjust();
	}

	void GroupCompiler::grabLowTunnel(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby)
	{

		std::vector<HadGrid*> grids = nearby;
		grids.emplace_back(pGrid);
		for (auto grid : grids)
		{
			for (auto obj : grid->query(ElementType::HAD_LANE_GROUP))
			{
				bool isIntersect = false;
				HadLaneGroup* pGroup = (HadLaneGroup*)obj;
				if (!isTunnelArea(pGroup) ||
					!allGroupMaps_.count(pGroup->originId))
				{
					continue;
				}
			
				auto currentLaneGroupInfo = allGroupMaps_[pGroup->originId];
				
				getLaneGroupBox2DRTree()->query(bgi::intersects(currentLaneGroupInfo->_groupBox2d),

					boost::make_function_output_iterator([&](size_t const& id) {
						PtrLaneGroupInfo nearByLaneGroup = allGroupDatas_[id];
						if (nearByLaneGroup->_laneGroup->originId == currentLaneGroupInfo->_laneGroup->originId)
							return;

						if (isTouchWithHadLaneGroup(nearByLaneGroup->_laneGroup, currentLaneGroupInfo->_laneGroup))
						{
							return;
						}
						std::vector<ring_2t> results;
						bg::intersection(nearByLaneGroup->_groupPoly, currentLaneGroupInfo->_groupPoly, results);
					
						if (!results.empty())
						{
							point_2t tmpCenterPoint2D;
							bg::centroid(results.front(), tmpCenterPoint2D);
							point_t tmpCenterPoint = point_t(
								tmpCenterPoint2D.get<0>(),
								tmpCenterPoint2D.get<1>(),
								currentLaneGroupInfo->_centerPoint.get<2>());
							point_t currentNearbyPoint = getNearestPoint(currentLaneGroupInfo->_groupPoints, tmpCenterPoint);
							point_t otherNearbyPoint = getNearestPoint(nearByLaneGroup->_groupPoints, tmpCenterPoint);

							int tol =  otherNearbyPoint.get<2>()- currentNearbyPoint.get<2>();
							if (tol > 0)
							{
							
								nearByLaneGroup->floor_height = min(nearByLaneGroup->floor_height,
									tol);
								if (nearByLaneGroup->floor_height < FLOOR_MIX_HEIGHT)
								{
									if (!_lowfloorTunnels2Maps.count(nearByLaneGroup->id()))
									{
										_lowfloorTunnels.emplace_back(nearByLaneGroup);
										_lowfloorTunnels2Maps.emplace(nearByLaneGroup->id(), nearByLaneGroup);
									}
								}
							
							}
							
						}
						}));

			}
		}

	}

	void GroupCompiler::divideToGroup()
	{
		_groupedTunnels.clear();
		_groupId2TunnelsMaps.clear();
		_tunnel2GroupIdMaps.clear();
		std::set<int64> lowTunnelIds;

		auto bfs_link = [&](PtrLaneGroupInfo startGroup, std::set<int64>& groupsOut)
		{
			groupsOut.clear();

			std::queue<PtrLaneGroupInfo> queueLink;

			queueLink.push(startGroup);

			while (!queueLink.empty())
			{
				PtrLaneGroupInfo group = queueLink.front();
				queueLink.pop();
				groupsOut.emplace(group->id());

				// 插入队列尾部
				for (auto lane : group->_laneGroup->next)
				{
					
					if (lowTunnelIds.find(lane->originId) != lowTunnelIds.end()
						&& groupsOut.find(lane->originId) == groupsOut.end())
					{
						queueLink.push(allGroupMaps_[lane->originId]);
					}
                    /*else {
                        break;
                    }*/
					
				}
				for (auto lane : group->_laneGroup->previous)
				{
		
					if (lowTunnelIds.find(lane->originId) != lowTunnelIds.end()
						&& groupsOut.find(lane->originId) == groupsOut.end())
					{
						queueLink.push(allGroupMaps_[lane->originId]);
					}
                    /*	else {
                            break;
                        }*/

				}
			}
		};
	
		std::set<int64> groupedTunnels;

		for (PtrLaneGroupInfo tunnel : _lowfloorTunnels)
		{
			lowTunnelIds.insert(tunnel->id());
		}
		int32 groudId = 1;
		

		for (PtrLaneGroupInfo tunnel : _lowfloorTunnels)
		{
			if (groupedTunnels.find(tunnel->id()) == groupedTunnels.end())
			{
				std::set<int64> bfs_linksOut;
				// 广度遍历拓扑
				bfs_link(tunnel, bfs_linksOut);

				if (!bfs_linksOut.empty())
				{
					std::vector<PtrLaneGroupInfo> group;
					for (auto id : bfs_linksOut)
					{
						group.emplace_back(allGroupMaps_[id]);
						_tunnel2GroupIdMaps.insert(id, groudId);
						groupedTunnels.insert(id);
					}
					_groupedTunnels.emplace_back(group);
					_groupId2TunnelsMaps.insert(groudId, group);
					
					groudId++;
				
				}
			}
		}

	
		
	}

	void GroupCompiler::mergeGroup()
	{
		//是否有共同的平滑区,如果有合并
		std::unordered_map<PtrLaneGroupInfo, std::vector<PtrLaneGroupInfo> > adjustGroupMap;
		// 计算坡度区域车道组lanegroup
		for (auto groups : _groupedTunnels)
		{
			for (PtrLaneGroupInfo lane : groups)
			{
			
				std::vector<PtrLaneGroupInfo> adjustGroup;
				std::vector<PtrSlopeAdjustTask> adjustTasks;
				calcAdjustLaeGroup(groups, lane, adjustGroup, adjustTasks);

				if (!adjustGroup.empty())
				{
					adjustGroupMap.emplace(lane, adjustGroup);
				}
			}
		}
	
	/*	for (auto ptLane1 : adjustGroupMap)
		{
			if (!_tunnel2GroupIdMaps.exist(ptLane1.first->id()))
			{
				continue;
			}
			for (auto ptLane2 : adjustGroupMap)
			{
				if (!_tunnel2GroupIdMaps.exist(ptLane2.first->id()))
				{
					continue;
				}
				if (ptLane1.first->id() == ptLane2.first->id())
				{
					continue;
				}
				if (_tunnel2GroupIdMaps[ptLane1.first->id()] == _tunnel2GroupIdMaps[ptLane2.first->id()])
				{
					continue;
				}

				for (auto ptLane1Adj : ptLane1.second)
				{
					for (auto ptLane2Adj : ptLane2.second)
					{
						if (ptLane1Adj->id() == ptLane2Adj->id())
						{
							if (_lowfloorTunnels2Maps.find(ptLane1Adj->id()) == _lowfloorTunnels2Maps.end())
							{
								_lowfloorTunnels.emplace_back(ptLane1Adj);
								_lowfloorTunnels2Maps.emplace(ptLane1Adj->id(), ptLane1Adj);
							}
						}
					}
				}
			}
		}*/

		for (auto ptLane1 : adjustGroupMap)
		{
			std::set<int64> idPtLane1;
			for (PtrLaneGroupInfo ptLane1Adj : ptLane1.second)
			{
				idPtLane1.emplace(ptLane1Adj->_laneGroup->originId);
			}
			for (auto ptLane2 : adjustGroupMap)
			{
				if (ptLane1.first->id() == 190540230064291329 && ptLane2.first->id() == 125763006379791361)
				{
					int a = 0;
				}
				if (ptLane1.first->id() == ptLane2.first->id())
				{
					continue;
				}
				if (_tunnel2GroupIdMaps[ptLane1.first->id()] == _tunnel2GroupIdMaps[ptLane2.first->id()])
				{
					continue;
				}
				std::set<int64> idPtLane2;
				std::set<int64> ids(idPtLane1);
				for (PtrLaneGroupInfo ptLane2Adj : ptLane2.second)
				{
					idPtLane2.emplace(ptLane2Adj->_laneGroup->originId);
					ids.emplace(ptLane2Adj->_laneGroup->originId);
				}


				if (ids.size() < (idPtLane1.size() + idPtLane2.size()))
				{
					for (int64 id : ids)
					{
						if (idPtLane1.count(id) && idPtLane2.count(id))
						{
							if (_lowfloorTunnels2Maps.find(id) == _lowfloorTunnels2Maps.end())
							{
								_lowfloorTunnels.emplace_back(allGroupMaps_[id]);
								_lowfloorTunnels2Maps.emplace(id, allGroupMaps_[id]);
							}
						}

					}


				}
			}
		}
		divideToGroup();

	}

	void GroupCompiler::calcAdjustLaeGroup(const std::vector<PtrLaneGroupInfo>& topy_group, PtrLaneGroupInfo startGroup,
		std::vector<PtrLaneGroupInfo>& adjustGroup,std::vector<PtrSlopeAdjustTask>& adjustTasks)
	{
		if (topy_group.empty())
		{
			return;
		}

		adjustGroup.clear();
		adjustTasks.clear();
		int32 low_value = 0;
		bool bCroosid = false;
		std::unordered_map<int64, PtrSlopeAdjustTask> adjustTaskMaps;
		int32 meshId = topy_group.front()->_laneGroup->owner->getId();
		
		for (PtrLaneGroupInfo lane : topy_group)
		{
			if (lane->floor_height !=0.0 || isTunnelArea(lane->_laneGroup))
			{
				low_value = max(low_value, FLOOR_MIX_HEIGHT - lane->floor_height);
			}
			if (!lane->_laneGroup->next.size() || !lane->_laneGroup->previous.size())
			{
				bCroosid = true;
			}
			
		}
#ifdef ADJUST_UNIQUE_HEIGHT
		low_value = ADJUST_UNIQUE_HEIGHT;
#endif

		for (PtrLaneGroupInfo lane : topy_group)
		{
			PtrSlopeAdjustTask ajustTask = std::make_shared<SlopeAdjustTask>();
			ajustTask->init(lane, abs(low_value), 0, true, nullptr);
			adjustTaskMaps.emplace(lane->_laneGroup->originId, ajustTask);
			adjustTasks.emplace_back(ajustTask);
		}

	
		//计算下降坡度

		std::queue<PtrLaneGroupInfo> queueLink;

		if (startGroup)
		{
			queueLink.push(startGroup);
		}
		else
		{
			for (PtrLaneGroupInfo lane : topy_group)
			{
				queueLink.push(lane);
			}

		}
		

		while (!queueLink.empty())
		{
			PtrLaneGroupInfo laneInfo = queueLink.front();
			queueLink.pop();

			PtrSlopeAdjustTask task = nullptr;
			if (adjustTaskMaps.find(laneInfo->_laneGroup->originId) == adjustTaskMaps.end())
			{
				continue;
			}
			task = adjustTaskMaps[laneInfo->_laneGroup->originId];

			double lastHeight = 0.0;
			bool bFinished = calcAdjustLineWillBeFinished(laneInfo, task->startHeight, task->slope, task->bFromStartAdjust, lastHeight);
			
			//LA
			for (HadLaneGroupAssociation* laAssoc : laneInfo->_laneGroup->associations)
			{
				for (HadLaneGroupAssociation::HadRelLaneGroupAssociation la : laAssoc->relLaneGroupAssociations)
				{
					HadLaneGroup* laLane = nullptr;
					if (la.first && la.first->originId != laneInfo->_laneGroup->originId)
					{
						laLane = la.first;
					}
					if (la.second && la.second->originId != laneInfo->_laneGroup->originId)
					{
						laLane = la.second;
					}
					if (!laLane)
					{
						continue;
					}
					if (adjustTaskMaps.find(laLane->originId) != adjustTaskMaps.end()
						|| allGroupMaps_.find(laLane->originId) == allGroupMaps_.end())
					{
						continue;
					}
					queueLink.push(allGroupMaps_[laLane->originId]);
					PtrSlopeAdjustTask ajustTask = std::make_shared<SlopeAdjustTask>();
					ajustTask->init(allGroupMaps_[laLane->originId], task->startHeight, task->slope, task->bFromStartAdjust, laneInfo);
					adjustTaskMaps.emplace(laLane->originId, ajustTask);
					adjustGroup.emplace_back(allGroupMaps_[laLane->originId]);
					adjustTasks.emplace_back(ajustTask);
				}
				

			}
			if (bFinished)
			{
				continue;
			}

			
			// 插入队列尾部
			for (auto lane_next : laneInfo->_laneGroup->next)
			{
				if (adjustTaskMaps.find(lane_next->originId) != adjustTaskMaps.end()
					|| allGroupMaps_.find(lane_next->originId) == allGroupMaps_.end())
				{
					continue;
				}
				queueLink.push(allGroupMaps_[lane_next->originId]);

				PtrSlopeAdjustTask ajustTask = std::make_shared<SlopeAdjustTask>();
				ajustTask->init(allGroupMaps_[lane_next->originId], task->bFromStartAdjust ? lastHeight : task->startHeight, 
					SLOPE_GRADIENT, true, laneInfo);
				adjustTaskMaps.emplace(lane_next->originId, ajustTask);
				adjustGroup.emplace_back(allGroupMaps_[lane_next->originId]);
				adjustTasks.emplace_back(ajustTask);
			}

			for (auto lane_prev : laneInfo->_laneGroup->previous)
			{

				if (adjustTaskMaps.find(lane_prev->originId) != adjustTaskMaps.end() ||
					allGroupMaps_.find(lane_prev->originId) == allGroupMaps_.end())
				{
					continue;
				}
				queueLink.push(allGroupMaps_[lane_prev->originId]);

				PtrSlopeAdjustTask ajustTask = std::make_shared<SlopeAdjustTask>();
				ajustTask->init(allGroupMaps_[lane_prev->originId], task->bFromStartAdjust ? task->startHeight : lastHeight,
					SLOPE_GRADIENT, false, laneInfo);
				adjustTaskMaps.emplace(lane_prev->originId, ajustTask);
				adjustGroup.emplace_back(allGroupMaps_[lane_prev->originId]);
				adjustTasks.emplace_back(ajustTask);
			}
		}


	}

	void GroupCompiler::adjust()
	{
		std::vector<PtrSlopeAdjustTask> adjustTasks;
		std::unordered_map<int64, PtrSlopeAdjustTask> adjustTaskMaps_;

	
		std::vector<PtrLaneGroupInfo> adjustGroup;
	
		for (auto groups : _groupedTunnels)
		{
			std::vector<PtrLaneGroupInfo> topy_groups;
			std::vector<PtrSlopeAdjustTask> adjustTasksTemp;
			for (PtrLaneGroupInfo lane : groups)
			{
				topy_groups.emplace_back(lane);
			}
			calcAdjustLaeGroup(topy_groups, nullptr, adjustGroup, adjustTasksTemp);
			adjustTasks.insert(adjustTasks.end(), adjustTasksTemp.begin(), adjustTasksTemp.end());
		}
	
		

		for (PtrSlopeAdjustTask task : adjustTasks)
		{
	/*		printInfo("[meshid:%d],[id: %I64d],[fromId:%I64d],[bFromStart:%d],[floor:%f],[adjustHeight:%f],[slopeRatio:%f],[groupId:%d]", 
				task->laneInfo->_laneGroup->owner->getId(), task->laneInfo->id(), task->laneInfo_From?task->laneInfo_From->id():0,
				task->bFromStartAdjust,
				task->laneInfo->floor_height,task->startHeight, task->slope,
				_tunnel2GroupIdMaps.exist(task->laneInfo->id())?_tunnel2GroupIdMaps[task->laneInfo->id()]:0);*/

			if (task->laneInfo->id() == 90810861739839757 )
			{
				int a = 0;
			}
			//if (task->laneInfo->_laneGroup->owner->getId() == m_pGrid->getId())
			{
				adjustLaneGroup(task->laneInfo, task->startHeight, task->slope, task->bFromStartAdjust);
				_adjustLaneGroupMaps.insert(task->laneInfo->id(), task->laneInfo);
				
			}
		}

		interpolateObject();
		//对齐高度差
		for (PtrSlopeAdjustTask task : adjustTasks)
		{
		
			if (task->laneInfo->_laneGroup->owner->getId() == m_pGrid->getId())
			{
				HadLaneGroup* laneGroup = task->laneInfo->_laneGroup;

				for (HadRoadBoundary* rb : laneGroup->roadBoundaries)
				{
					for (auto nextRb : rb->next)
					{
						HadRoadBoundary* nextRoadBoundary = (HadRoadBoundary*)nextRb;
						if (nextRoadBoundary->owner->getId() == m_pGrid->getId())
						{
							if (rb->location.vertexes.back().pos == nextRoadBoundary->location.vertexes.front().pos)
							{
								rb->location.vertexes.back().z = nextRoadBoundary->location.vertexes.front().z;
							}
							
						}
					}
				}

                for (HadLaneBoundary* lb : laneGroup->laneBoundaries)
                {
                    for (auto nextLB : lb->next)
                    {
						HadLaneBoundary* nextLaneBoundary = (HadLaneBoundary*)nextLB;
                        if (nextLaneBoundary->owner->getId() == m_pGrid->getId())
                        {
                            if (lb->location.vertexes.back().pos == nextLaneBoundary->location.vertexes.front().pos)
                            {
								lb->location.vertexes.back().z = nextLaneBoundary->location.vertexes.front().z;
                            }

                        }
                    }
                }

			}
		}
	}
	void GroupCompiler::setLaneGroupData(HadGrid* const pGrid)
	{
		for (auto obj : pGrid->query(ElementType::HAD_LANE_GROUP))
		{
			HadLaneGroup* pGroup = (HadLaneGroup*)obj;
			
			if (!pGroup->roadBoundaries.empty())
			{
				PtrLaneGroupInfo tmp = std::make_shared<laneGroupInfo>();
				std::vector<MapPoint3D64> groupPoints;
				std::vector<MapPoint3D64> leftBoundary(pGroup->roadBoundaries[0]->location.vertexes.begin(), pGroup->roadBoundaries[0]->location.vertexes.end());
				if (directionEqual(pGroup->roadBoundaries[0], pGroup, 3))
					std::reverse(leftBoundary.begin(), leftBoundary.end());
				for (size_t i = 0; i < pGroup->roadBoundaries.size(); ++i)
				{
					std::vector<MapPoint3D64> tmpRoadVertexes(pGroup->roadBoundaries[i]->location.vertexes.begin(), pGroup->roadBoundaries[i]->location.vertexes.end());
					std::vector<MapPoint3D64> tmpBoundaryVertexes(pGroup->roadBoundaries[i]->location.vertexes.begin(), pGroup->roadBoundaries[i]->location.vertexes.end());
					if (directionEqual(pGroup->roadBoundaries[i], pGroup, 3))
					{
						std::reverse(tmpRoadVertexes.begin(), tmpRoadVertexes.end());
					}
					if (i % 2 == 1)
					{
						std::reverse(tmpRoadVertexes.begin(), tmpRoadVertexes.end());
					}
#ifdef CONVERT_TAG
	coordinatesTransform.convert(tmpRoadVertexes.data(), tmpRoadVertexes.size());
#endif
				
					groupPoints.insert(groupPoints.end(), tmpRoadVertexes.begin(), tmpRoadVertexes.end());

					if (i == 0)
					{
						auto tmpLeftVertexes = tmpBoundaryVertexes;
						if (directionEqual(pGroup->roadBoundaries[i], pGroup, 3))
						{
							std::reverse(tmpLeftVertexes.begin(), tmpLeftVertexes.end());
							tmp->isReversedL = true;
						}
#ifdef CONVERT_TAG
						coordinatesTransform.convert(tmpLeftVertexes.data(), tmpLeftVertexes.size());
						
#endif
						tmp->_originLineL = LINESTRING_T(tmpLeftVertexes);
						tmp->_raisedLineL = tmp->_originLineL;
					}
					else if (i == 1)
					{
						auto tmpRightVertexes = tmpBoundaryVertexes;
						if (directionEqual(pGroup->roadBoundaries[i], pGroup, 3))
						{
							std::reverse(tmpRightVertexes.begin(), tmpRightVertexes.end());
							tmp->isReversedR = true;
						}
#ifdef CONVERT_TAG
	coordinatesTransform.convert(tmpRightVertexes.data(), tmpRightVertexes.size());
#endif
						
						tmp->_originLineR = LINESTRING_T(tmpRightVertexes);
						tmp->_raisedLineR = tmp->_originLineR;
					}
				}
#ifdef CONVERT_TAG
	coordinatesTransform.convert(groupPoints.data(), groupPoints.size());
#endif
				
				tmp->_laneGroup = pGroup;
				tmp->_groupBox = BOX_T(groupPoints);
				tmp->_groupBox2d = tmp->_groupBox;
				tmp->_groupBox2d.max_corner().set<2>(0);
				tmp->_groupBox2d.min_corner().set<2>(0);

				tmp->_groupPoly = RING_2T(groupPoints);
				tmp->_groupPoints = LINESTRING_T(groupPoints);
				bg::correct(tmp->_groupPoly);
				bg::centroid(tmp->_groupBox, tmp->_centerPoint);
				allGroupDatas_.emplace_back(tmp);
				allGroupBoxes.emplace_back(tmp->_groupBox);

				allGroupBoxes2D.emplace_back(tmp->_groupBox2d);
				allGroupMaps_.emplace(pGroup->originId, tmp);
			}
		}
	
	}

	void GroupCompiler::updateInterpolatedLine()
	{
		for (auto tmp : allGroupDatas_)
		{
			if (tmp->_originLineL.size() > 1 && tmp->_originLineR.size() > 1)
			{
				linestring_t simplifiedLineL, simplifiedLineR;
				simplifiedLineL = tmp->_originLineL;
				simplifiedLineR = tmp->_originLineR;
				auto tmpOriginLineL = simplifiedLineL;
				for (size_t i = 1; i < simplifiedLineR.size() - 1; ++i)
				{
					point_t point = simplifiedLineR.at(i);
					std::vector<point_t>::iterator ita, itb;
					getClosestSeg(point, simplifiedLineL, ita, itb);
					segment_t tmpSeg;
					bg::closest_points(point, segment_t(*ita, *itb), tmpSeg);
					simplifiedLineL.insert(itb, tmpSeg.second);
				}
				for (size_t i = 1; i < tmpOriginLineL.size() - 1; ++i)
				{
					point_t point = tmpOriginLineL[i];
					std::vector<point_t>::iterator ita, itb;
					getClosestSeg(point, simplifiedLineR, ita, itb);
					segment_t tmpSeg;
					bg::closest_points(point, segment_t(*ita, *itb), tmpSeg);
					simplifiedLineR.insert(itb, tmpSeg.second);
				}
				tmp->_raisedLineL = simplifiedLineL;
				tmp->_raisedLineR = simplifiedLineR;
			}
		}
	}

	void GroupCompiler::updateRefLine()
	{
		for (auto tmp : allGroupDatas_)
		{
			if (tmp->_originLineL.size() > 1 && tmp->_originLineR.size() > 1)
			{
				point_t tmpBeginPoint = point_t(
					(tmp->_originLineL.front().get<0>() + tmp->_originLineR.front().get<0>()) / 2,
					(tmp->_originLineL.front().get<1>() + tmp->_originLineR.front().get<1>()) / 2,
					(tmp->_originLineL.front().get<2>() + tmp->_originLineR.front().get<2>()) / 2);
				tmp->refLine.push_back(tmpBeginPoint);

				for (size_t j = 1; j < tmp->_originLineL.size() - 1; j++)
				{
					point_t tmpPoint = tmp->_originLineL.at(j);
					segment_t tmpSeg;
					bg::closest_points(tmpPoint, tmp->_originLineR, tmpSeg);
					point_t tmpMiddlePoint = point_t(
						(tmpPoint.get<0>() + tmpSeg.second.get<0>()) / 2,
						(tmpPoint.get<1>() + tmpSeg.second.get<1>()) / 2,
						(tmpPoint.get<2>() + tmpSeg.second.get<2>()) / 2);
					tmp->refLine.push_back(tmpMiddlePoint);
				}

				point_t tmpEndPoint = point_t(
					(tmp->_originLineL.back().get<0>() + tmp->_originLineR.back().get<0>()) / 2,
					(tmp->_originLineL.back().get<1>() + tmp->_originLineR.back().get<1>()) / 2,
					(tmp->_originLineL.back().get<2>() + tmp->_originLineR.back().get<2>()) / 2);
				tmp->refLine.push_back(tmpEndPoint);
			}
			tmp->raisedRefLine = tmp->refLine;
			tmp->raisedRefLine_relative = tmp->refLine;
			for (auto& pt : tmp->raisedRefLine_relative)
			{
				pt.set<2>(0);
			}
			
		}
	}

	bool GroupCompiler::adjustLine(
		const linestring_t& line,
		const double& height,
		const double slopeRatio,
		linestring_t& outLine,
		double& outHeight)
	{
		if (line.empty())
			return true;
		
		auto tmpPoint = line.front();
		tmpPoint.set<2>(tmpPoint.get<2>() + height);
		outLine.push_back(tmpPoint);
		bool isEnd = false;
		outHeight = height;
		double lastAbsHeight = abs(height);
		for (size_t i = 1; i < line.size(); i++)
		{
			
			double tmpDis = std::abs(bg::distance(line[i - 1], line[i]));
			tmpPoint = line[i];
			double detH = tmpDis * slopeRatio;
			if (outHeight + detH >=0)
			{
				tmpPoint.set<2>(tmpPoint.get<2>() + outHeight + detH);
				outLine.push_back(tmpPoint);
				outHeight += detH;
				lastAbsHeight -= abs(detH);
			}
			else
			{
				outLine.push_back(tmpPoint);
				lastAbsHeight = 0;
			}
		}
		return lastAbsHeight <= 0;
	}

	void GroupCompiler::adjustLaneGroup(PtrLaneGroupInfo& groupInfo, double adjustHeight,
		const double slopeRatio, bool bFromStart)
	{
		
		linestring_t tmp;
		double tmpHeight = 0.0;
		auto tmpLine = groupInfo->refLine;
		if (!bFromStart)
		{
			bg::reverse(tmpLine);
		}
		
		bool isFinished = adjustLine(tmpLine, adjustHeight, slopeRatio, tmp, tmpHeight);
		if (!bFromStart)
		{
			bg::reverse(tmp);
		}
		groupInfo->raisedRefLine = tmp;
		groupInfo->_startRaiseHeight = tmpHeight;
		groupInfo->_isConvergence = isFinished;

		linestring_t relative_line;
		for (int i = 0; i < groupInfo->raisedRefLine.size() ;i ++ )
		{
			relative_line.emplace_back(point_t(groupInfo->raisedRefLine[i].get<0>(), 
				groupInfo->raisedRefLine[i].get<1>(),
				groupInfo->raisedRefLine[i].get<2>() - groupInfo->refLine[i].get<2>()));
		}
		groupInfo->raisedRefLine_relative = relative_line;

		interpolateBoundary(groupInfo);

		interpolateLaneBoundary(groupInfo);

		interpolateLane(groupInfo);
	
	}

	void GroupCompiler::interpolateBoundary(
		PtrLaneGroupInfo& groupInfo)
	{
		//raise left and right boundary
		if (groupInfo->_raisedLineL.size() > 1 &&
			groupInfo->_raisedLineR.size() > 1 &&
			groupInfo->raisedRefLine.size() > 1)
		{
			//left
			groupInfo->_raisedLineL.front().set<2>(groupInfo->raisedRefLine.front().get<2>());
			for (size_t i = 1; i < groupInfo->_raisedLineL.size() - 1; ++i)
			{
				segment_t tmpSeg;
				bg::closest_points(groupInfo->_raisedLineL.at(i), groupInfo->raisedRefLine, tmpSeg);
				groupInfo->_raisedLineL.at(i).set<2>(tmpSeg.second.get<2>());
			}
			groupInfo->_raisedLineL.back().set<2>(groupInfo->raisedRefLine.back().get<2>());

			//right
			groupInfo->_raisedLineR.front().set<2>(groupInfo->raisedRefLine.front().get<2>());
			for (size_t i = 1; i < groupInfo->_raisedLineR.size() - 1; ++i)
			{
				segment_t tmpSeg;
				bg::closest_points(groupInfo->_raisedLineR.at(i), groupInfo->raisedRefLine, tmpSeg);
				groupInfo->_raisedLineR.at(i).set<2>(tmpSeg.second.get<2>());
			}
			groupInfo->_raisedLineR.back().set<2>(groupInfo->raisedRefLine.back().get<2>());
		}

		if (groupInfo->_laneGroup->roadBoundaries.size() > 1)
		{
			//left
			auto& tmpLeftRoadBoundary = groupInfo->_laneGroup->roadBoundaries.at(0);
			LineString3d tmpLeftLocation;
			for (size_t i = 0; i < groupInfo->_raisedLineL.size(); ++i)
			{
				auto tmpPoint = groupInfo->_raisedLineL[i];
				tmpLeftLocation.vertexes.push_back(MapPoint3D64_make(tmpPoint.get<0>(), tmpPoint.get<1>(), tmpPoint.get<2>() / 10));
			}
#ifdef CONVERT_TAG
			coordinatesTransform.invert(tmpLeftLocation.vertexes.data(), tmpLeftLocation.vertexes.size());
#endif
			if (groupInfo->isReversedL)
			{
				std::reverse(tmpLeftLocation.vertexes.begin(), tmpLeftLocation.vertexes.end());
			}
			tmpLeftRoadBoundary->location = tmpLeftLocation;

			//right
			auto& tmpRightRoadBoundary = groupInfo->_laneGroup->roadBoundaries.at(1);
			LineString3d tmpRightLocation;
			for (size_t i = 0; i < groupInfo->_raisedLineR.size(); ++i)
			{
				auto tmpPoint = groupInfo->_raisedLineR[i];
				tmpRightLocation.vertexes.push_back(MapPoint3D64_make(tmpPoint.get<0>(), tmpPoint.get<1>(), tmpPoint.get<2>() / 10));
			}
#ifdef CONVERT_TAG
			coordinatesTransform.invert(tmpRightLocation.vertexes.data(), tmpRightLocation.vertexes.size());
#endif
			if (groupInfo->isReversedR)
			{
				std::reverse(tmpRightLocation.vertexes.begin(), tmpRightLocation.vertexes.end());
			}
			tmpRightRoadBoundary->location = tmpRightLocation;
		}

	}

	void GroupCompiler::interpolateLaneBoundary(
		PtrLaneGroupInfo& groupInfo)
	{
		static std::set<int64> ids;
		static std::set<int64> ids_pa;
		multi_linestring_t linestring;
		linestring.emplace_back(groupInfo->raisedRefLine_relative);
		for (HadLaneBoundary* laneBoundary : groupInfo->_laneGroup->laneBoundaries)
		{
			if (ids.count(laneBoundary->originId))
			{
				//continue;
			}
			std::vector<MapPoint3D64> tmpVertexes(laneBoundary->location.vertexes.begin(), laneBoundary->location.vertexes.end());
			interpolatePoints(tmpVertexes, groupInfo->raisedRefLine);
			laneBoundary->location.vertexes = tmpVertexes;
			ids.insert(laneBoundary->originId);

			for (HadPartAttribute* pa : laneBoundary->attributes)
			{
				if (ids_pa.count(pa->originId))
				{
					//continue;

				}
				HadLaneBoundaryPAType paType = (HadLaneBoundaryPAType)pa->name;
				if (paType >= HadLaneBoundaryPAType::BOUNDARY_TYPE && paType <= HadLaneBoundaryPAType::LATERAL_OFFSET)

				{
					std::vector<MapPoint3D64> tmpVertexes = pa->points.postions;
					interpolatePoints(tmpVertexes, groupInfo->raisedRefLine);
					pa->points.postions = tmpVertexes;
				}
			

				
				ids_pa.insert(pa->originId);
			}

		}
		
	}

	void GroupCompiler::interpolateLane(
		PtrLaneGroupInfo& groupInfo)
	{
		static std::set<int64> ids;
		for (HadLane* lane : groupInfo->_laneGroup->lanes)
		{
			if (ids.count(lane->originId))
			{
			//	continue;
			}
			std::vector<MapPoint3D64> tmpVertexes(lane->location.vertexes.begin(), lane->location.vertexes.end());
			interpolatePoints(tmpVertexes, groupInfo->raisedRefLine);
			lane->location = LineString3d{ tmpVertexes };
			ids.insert(lane->originId);
		}

	}


	void GroupCompiler::interpolateObject()
	{

		auto hasObject = [&](HadLaneGroup* laneGroup, HadObject* object)->bool {

			for (auto obj : laneGroup->objects)
			{
				if (obj == object)
				{
					return true;
				}
			}
			return false;
		};
		
		auto collect_laneGroup = [&](const std::vector<MapPoint3D64>& points,HadObject* object)->multi_linestring_t {

			linestring_t ls_point = LINESTRING_T(points);
			multi_linestring_t refLine;
			bool bNeedAdjust = false;
			
			point_t center_pt;
			bg::centroid(ls_point, center_pt);

			
			for (auto laneGroup : object->laneGroups)
			{

				if (allGroupMaps_.count(laneGroup->originId))
				{
					PtrLaneGroupInfo ptLaneGroup = allGroupMaps_[laneGroup->originId];

					segment_t seg_temp;
					bg::closest_points(center_pt, ptLaneGroup->_groupPoints, seg_temp);

					if (abs(seg_temp.first.get<2>() - seg_temp.second.get<2>()) > 500)
					{
						continue;
					}

					if (_adjustLaneGroupMaps.exist(laneGroup->originId) )
					{
						bNeedAdjust = true;
					}
					refLine.emplace_back(ptLaneGroup->raisedRefLine_relative);
				}

			}

			if (!bNeedAdjust)
			{
				refLine.clear();
			}

			return refLine;
		};
		
		std::vector<ElementType> elementTypes;
		elementTypes.emplace_back(ElementType::HAD_OBJECT_FILL_AREA);
		elementTypes.emplace_back(ElementType::HAD_OBJECT_ARROW);
		elementTypes.emplace_back(ElementType::HAD_OBJECT_CROSS_WALK);
		elementTypes.emplace_back(ElementType::HAD_OBJECT_TRAFFIC_SIGN);
		elementTypes.emplace_back(ElementType::HAD_OBJECT_STOPLOCATION);
		elementTypes.emplace_back(ElementType::HAD_OBJECT_TEXT);
		elementTypes.emplace_back(ElementType::HAD_OBJECT_BARRIER);
		elementTypes.emplace_back(ElementType::HAD_OBJECT_WALL);


		for (ElementType eType : elementTypes)
		{
			for (HadElement* elment : m_pGrid->query(eType))
			{
				HadObject* object = (HadObject*)elment;
				
				switch (eType)
				{

				case OMDB::ElementType::HAD_OBJECT_CROSS_WALK:
				{
					HadCrossWalk* f = (HadCrossWalk*)object;
					std::vector<MapPoint3D64> tmpVertexes(f->polygon.vertexes);
					auto refLine = collect_laneGroup(tmpVertexes, object);
					interpolatePoints(tmpVertexes, refLine);
					f->polygon.vertexes = tmpVertexes;
				}
				break;

				case OMDB::ElementType::HAD_OBJECT_TRAFFIC_SIGN:
				{
					HadTrafficSign* f = (HadTrafficSign*)object;
					std::vector<MapPoint3D64> tmpVertexes(f->polygon.vertexes);
					auto refLine = collect_laneGroup(tmpVertexes, object);
					interpolatePoints(tmpVertexes, refLine);
					f->polygon.vertexes = tmpVertexes;
				}
				break;
				case OMDB::ElementType::HAD_OBJECT_FILL_AREA:
				{
					HadFillArea* f = (HadFillArea*)object;
					std::vector<MapPoint3D64> tmpVertexes(f->polygon.vertexes);
					auto refLine = collect_laneGroup(tmpVertexes, object);
					interpolatePoints(tmpVertexes, refLine);
					f->polygon.vertexes = tmpVertexes;
				}
				break;
				case OMDB::ElementType::HAD_OBJECT_STOPLOCATION:
				{
					HadStopLocation* f = (HadStopLocation*)object;
					std::vector<MapPoint3D64> tmpVertexes(f->location.vertexes);
					auto refLine = collect_laneGroup(tmpVertexes, object);
					interpolatePoints(tmpVertexes, refLine);
					f->location.vertexes = tmpVertexes;
				}
				break;
				case OMDB::ElementType::HAD_OBJECT_SYMBOL:
				case OMDB::ElementType::HAD_OBJECT_TEXT:
				case OMDB::ElementType::HAD_OBJECT_ARROW:
				{
					HadSymbol* f = (HadSymbol*)object;
					std::vector<MapPoint3D64> tmpVertexes(f->polygon.vertexes);
					auto refLine = collect_laneGroup(tmpVertexes, object);
					interpolatePoints(tmpVertexes, refLine);
					f->polygon.vertexes = tmpVertexes;
				}
				break;

				case OMDB::ElementType::HAD_OBJECT_WALL:
				{
					HadWall* f = (HadWall*)object;
					for (LineString3d& line : f->location.lines)
					{
						std::vector<MapPoint3D64> tmpVertexes(line.vertexes);
						auto refLine = collect_laneGroup(tmpVertexes, object);
						interpolatePoints(tmpVertexes, refLine);
						line.vertexes = tmpVertexes;
					}


				}
				break;

				case OMDB::ElementType::HAD_OBJECT_BARRIER:
				{
					HadBarrier* f = (HadBarrier*)object;
					for (LineString3d& line : f->location.lines)
					{
						std::vector<MapPoint3D64> tmpVertexes(line.vertexes);
						auto refLine = collect_laneGroup(tmpVertexes, object);
						interpolatePoints(tmpVertexes, refLine);
						line.vertexes = tmpVertexes;
					}
				}
				break;

				default:
					break;
				}

			}
		}
	
	}


	void GroupCompiler::interpolatePoints(std::vector<MapPoint3D64>& points, linestring_t& refLine)
	{
#ifdef CONVERT_TAG
		coordinatesTransform.convert(points.data(), points.size());
#endif
		

        if (refLine.empty())
        {
            return;
        }

		std::set<size_t> grapIded;
		for (size_t i = 0; i < points.size(); ++i)
		{

			segment_t tmpSeg;
			bg::closest_points(POINT_T(points.at(i)), refLine, tmpSeg);
			
			points[i].z = tmpSeg.second.get<2>() / 10;

		}
#ifdef CONVERT_TAG
		coordinatesTransform.invert(points.data(), points.size());
#endif
		
	}

	void GroupCompiler::interpolatePoints(std::vector<MapPoint3D64>& points, multi_linestring_t& relative_line)
	{
#ifdef CONVERT_TAG
		coordinatesTransform.convert(points.data(), points.size());
#endif
		if (relative_line.empty())
		{
			return;
		}
		std::set<size_t> grapIded;

		for (size_t i = 0; i < points.size(); ++i)
		{
		
			segment_t tmpSeg;

			bg::closest_points(POINT_T(points.at(i)), relative_line, tmpSeg);

			points[i].z += tmpSeg.second.get<2>() / 10;

		}
		
#ifdef CONVERT_TAG
		coordinatesTransform.invert(points.data(), points.size());
#endif
	}

	bool GroupCompiler::calcAdjustLineWillBeFinished(PtrLaneGroupInfo& laneInfo,double adjustHeight,
		const double slopeRatio,bool bFromStart,double& lastHeight )
	{
		linestring_t tmp;
		lastHeight = 0.0;
		auto tmpLine = laneInfo->refLine;
		if (!bFromStart)
		{
			bg::reverse(tmpLine);
		}
		
		return adjustLine(tmpLine, adjustHeight, slopeRatio, tmp, lastHeight);
	}
	

	
	// 父类的makeLaneGroupPolygon使用的车道边界,因车道边界点虚和车道同行方向可能相反,无法鉴别
	// 子类只使用道路边界来组成道路面.
	size_t GroupCompiler::makeLaneGroupPolygon(HadLaneGroup* pGroup, MapPoint3D64*& polygon)
	{
		if (pGroup->roadBoundaries.size() < 2)
			return 0;

		size_t roadBoundaryNum = pGroup->roadBoundaries.size();
		size_t nPolyline1PtNum = pGroup->roadBoundaries[0]->location.vertexes.size();
		size_t nPolyline2PtNum = pGroup->roadBoundaries[roadBoundaryNum - 1]->location.vertexes.size();

		//添加路面左边线几何点。
		std::vector<MapPoint3D64> Line1Reverse(nPolyline1PtNum);
		if (directionEqual(pGroup->roadBoundaries[0], pGroup, 3))
			std::reverse_copy(pGroup->roadBoundaries[0]->location.vertexes.begin(), pGroup->roadBoundaries[0]->location.vertexes.end(), Line1Reverse.begin());
		else
		{
			std::copy(pGroup->roadBoundaries[0]->location.vertexes.begin(), pGroup->roadBoundaries[0]->location.vertexes.end(), Line1Reverse.begin());

		}
		//添加路面右边线几何点。
		std::vector<MapPoint3D64> Line2Reverse(nPolyline2PtNum);
		if (directionEqual(pGroup->roadBoundaries[roadBoundaryNum - 1], pGroup, 2))
			std::reverse_copy(pGroup->roadBoundaries[roadBoundaryNum - 1]->location.vertexes.begin(), pGroup->roadBoundaries[roadBoundaryNum - 1]->location.vertexes.end(), Line2Reverse.begin());
		else
		{
			std::copy(pGroup->roadBoundaries[roadBoundaryNum - 1]->location.vertexes.begin(), pGroup->roadBoundaries[roadBoundaryNum - 1]->location.vertexes.end(), Line2Reverse.begin());
		}

		size_t nPtNum = Line1Reverse.size() + Line2Reverse.size() + 1;
		polygon = new MapPoint3D64[nPtNum];

		memcpy(polygon, Line1Reverse.data(), Line1Reverse.size() * sizeof(MapPoint3D64));
		memcpy(polygon + Line1Reverse.size(), Line2Reverse.data(), Line2Reverse.size() * sizeof(MapPoint3D64));
		polygon[nPtNum - 1] = Line1Reverse.front();

		return nPtNum;

	}
	BoxRTree::Ptr GroupCompiler::getLaneGroupBoxRTree()
	{
		auto& tmp = rtreeLaneGroupBox;
		if (tmp)
			return tmp;
		BoxRTree::guardParam param;
		BoxRTree::indexGetterBox originIndBox(allGroupBoxes);
		auto rtreeBox = std::make_shared<BoxRTree::RTree>(boost::irange<std::size_t>(0lu, allGroupBoxes.size()), param, originIndBox);
		rtreeLaneGroupBox = rtreeBox;
		return rtreeBox;
	}
	BoxRTree::Ptr GroupCompiler::getLaneGroupBox2DRTree()
	{
		auto& tmp = rtreeLaneGroupBox;
		if (tmp)
			return tmp;
		BoxRTree::guardParam param;
		BoxRTree::indexGetterBox originIndBox(allGroupBoxes2D);
		auto rtreeBox = std::make_shared<BoxRTree::RTree>(boost::irange<std::size_t>(0lu, allGroupBoxes.size()), param, originIndBox);
		rtreeLaneGroupBox = rtreeBox;
		return rtreeBox;
	}
	

	bool GroupCompiler::isSimplyfyLine(const PtrLaneGroupInfo& currentGroupInfo)
	{
		int64 tolDis = 1e6;
		int64 realDis = bg::length(currentGroupInfo->_originLineL);
		PtrLaneGroupInfo tmpPreInfo = currentGroupInfo; 
		PtrLaneGroupInfo tmpNextInfo = currentGroupInfo; 
		linestring_t tmpLine;
		if (realDis > tolDis)
		{
			tmpLine = currentGroupInfo->_originLineL;
		}
		else
		{
			while (tmpNextInfo->_laneGroup->next.size() == 1 &&
				tmpPreInfo->_laneGroup->previous.size() == 1)
			{
				if (currentGridGroupMaps.find(tmpNextInfo->_laneGroup->next.front()->originId) != currentGridGroupMaps.end() &&
					currentGridGroupMaps.find(tmpPreInfo->_laneGroup->previous.front()->originId) != currentGridGroupMaps.end())
				{
					auto& nextLaneGroupInfo = currentGridGroupMaps[tmpNextInfo->_laneGroup->next.front()->originId];
					auto& previousLaneGroupInfo = currentGridGroupMaps[tmpPreInfo->_laneGroup->previous.front()->originId];
					realDis += bg::length(nextLaneGroupInfo->_originLineL);
					realDis += bg::length(previousLaneGroupInfo->_originLineL);
					if (realDis > tolDis)
					{
						linestring_t tmpSubLine = previousLaneGroupInfo->_originLineL;
						tmpSubLine.insert(tmpSubLine.end(), tmpLine.begin(), tmpLine.end());
						tmpSubLine.insert(tmpSubLine.end(), nextLaneGroupInfo->_originLineL.begin(), nextLaneGroupInfo->_originLineL.end());
						auto originLine = tmpSubLine;
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
							if (vAverage > 0.999)	// arccos(0.999) = 2.5 degree
							{
								return true;
							}
							else
							{
								return false;
							}
						}
						else
						{
							return false;
						}
					}
					else
					{
						tmpNextInfo = nextLaneGroupInfo;
						tmpPreInfo = previousLaneGroupInfo;
					}
				}
				else
				{
					return false;
				}
			}
		}

		return false;
	}

	void GroupCompiler::interpolateConvexHullPoints(HadGrid* const pGrid)
	{
		auto interpolatePoint = [&](
			const linestring_t& originLine,
			linestring_t& outLine)
		{
			if (originLine.size() > 3)
			{
				for (size_t i = 1; i < originLine.size() - 2; ++i)
				{
					point_t pa = originLine.at(i - 1);
					point_t pb = originLine.at(i);
					point_t pc = originLine.at(i + 1);
					point_t pd = originLine.at(i + 2);
					vector_t va = V3_N(S3_V3(pa, pb));
					vector_t vb = V3_N(S3_V3(pb, pc));
					vector_t vc = V3_N(S3_V3(pc, pd));
					vector_t v3 = bg::cross_product(va, vb);
					vector_t v4 = bg::cross_product(vb, vc);
					if (bg::dot_product(v3, v4) < 0)
					{
						point_t point = point_t(
							(pb.get<0>() + pc.get<0>()) / 2,
							(pb.get<1>() + pc.get<1>()) / 2,
							(pb.get<2>() + pc.get<2>()) / 2);
						std::vector<point_t>::iterator ita, itb;
						getClosestSeg(point, outLine, ita, itb);
						segment_t tmpSeg;
						bg::closest_points(point, segment_t(*ita, *itb), tmpSeg);
						outLine.insert(itb, tmpSeg.second);
					}
				}
			}
		};

		for (auto obj : pGrid->query(ElementType::HAD_LANE_GROUP))
		{
			HadLaneGroup* pGroup = (HadLaneGroup*)obj;
			if (currentGridGroupMaps.find(pGroup->originId) == currentGridGroupMaps.end())
				continue;
			auto& currentLaneGroupInfo = currentGridGroupMaps[pGroup->originId];

			if (!(isExistFillArea(pGroup) || isExistAssociation(pGroup) || pGroup->inIntersection))
			{
				if (isSimplyfyLine(currentLaneGroupInfo))
				{
					linestring_t resultLineL, resultLineR;
					bg::simplify(currentLaneGroupInfo->_originLineL, resultLineL, 1000.0);
					bg::simplify(currentLaneGroupInfo->_originLineR, resultLineR, 1000.0);
					currentLaneGroupInfo->_originLineL = resultLineL;
					currentLaneGroupInfo->_originLineR = resultLineR;
				}
			}

			if (currentLaneGroupInfo->_laneGroup->roadBoundaries.size() > 1)
			{
				auto tmpLineL = currentLaneGroupInfo->_originLineL;
				auto tmpLineR = currentLaneGroupInfo->_originLineR;
				interpolatePoint(currentLaneGroupInfo->_originLineL, tmpLineR);
				interpolatePoint(currentLaneGroupInfo->_originLineR, tmpLineL);

				//left
				auto& tmpLeftRoadBoundary = currentLaneGroupInfo->_laneGroup->roadBoundaries.at(0);
				LineString3d tmpLeftLocation;
				for (size_t i = 0; i < tmpLineL.size(); ++i)
				{
					auto& tmpPoint = tmpLineL[i];
					tmpLeftLocation.vertexes.push_back(MapPoint3D64_make(tmpPoint.get<0>(), tmpPoint.get<1>(), tmpPoint.get<2>() / 10));
				}
				coordinatesTransform.invert(tmpLeftLocation.vertexes.data(), tmpLeftLocation.vertexes.size());
				if (currentLaneGroupInfo->isReversedL)
				{
					std::reverse(tmpLeftLocation.vertexes.begin(), tmpLeftLocation.vertexes.end());
				}
				// 双向通行且逆向的道路边界不做插值
				if (!(tmpLeftRoadBoundary->linkGroups.size() > 1 && directionEqual(tmpLeftRoadBoundary, currentLaneGroupInfo->_laneGroup, 3)))
				{
					tmpLeftRoadBoundary->location = tmpLeftLocation;
					tmpLeftRoadBoundary->location.vertexes.front() = currentLaneGroupInfo->_originMapLineL.vertexes.front();
					tmpLeftRoadBoundary->location.vertexes.back() = currentLaneGroupInfo->_originMapLineL.vertexes.back();
				}

				//right
				auto& tmpRightRoadBoundary = currentLaneGroupInfo->_laneGroup->roadBoundaries.at(1);
				LineString3d tmpRightLocation;
				for (size_t i = 0; i < tmpLineR.size(); ++i)
				{
					auto& tmpPoint = tmpLineR[i];
					tmpRightLocation.vertexes.push_back(MapPoint3D64_make(tmpPoint.get<0>(), tmpPoint.get<1>(), tmpPoint.get<2>() / 10));
				}
				coordinatesTransform.invert(tmpRightLocation.vertexes.data(), tmpRightLocation.vertexes.size());
				if (currentLaneGroupInfo->isReversedR)
				{
					std::reverse(tmpRightLocation.vertexes.begin(), tmpRightLocation.vertexes.end());
				}
				// 双向通行且逆向的道路边界不做插值
				if (!(tmpRightRoadBoundary->linkGroups.size() > 1 && directionEqual(tmpRightRoadBoundary, currentLaneGroupInfo->_laneGroup, 3)))
				{
					tmpRightRoadBoundary->location = tmpRightLocation;
					tmpRightRoadBoundary->location.vertexes.front() = currentLaneGroupInfo->_originMapLineR.vertexes.front();
					tmpRightRoadBoundary->location.vertexes.back() = currentLaneGroupInfo->_originMapLineR.vertexes.back();
				}
			}
		}
	}

	void GroupCompiler::modifyGridHeight(
		HadGrid* const pGrid,
		const std::vector<HadGrid*>& nearby)
	{
		auto getBox = [&](Rect& tmpRect)-> linestring_t
		{
			std::vector<Point> pts;
			std::vector<Point> newPts;
			Point p0{ tmpRect.left, tmpRect.bottom };
			Point p1{ tmpRect.right, tmpRect.bottom };
			Point p2{ tmpRect.left, tmpRect.top };
			Point p3{ tmpRect.right, tmpRect.top };
			pts.push_back(p0);
			pts.push_back(p1);
			pts.push_back(p2);
			pts.push_back(p3);
			newPts.resize(pts.size());
			for (size_t i = 0; i < pts.size(); ++i)
			{
				auto& tmp = pts.at(i);
				auto& newTmp = newPts.at(i);
				Math_wgsToMars(&tmp, &newTmp);
			}
			std::vector<MapPoint3D64> tmpOriginPoints;
			for (auto& pp0 : newPts)
				tmpOriginPoints.push_back(MapPoint3D64_make((int64)pp0.x * 1000, (int64)pp0.y * 1000, 0));
			linestring_t tmpLine = LINESTRING_T(tmpOriginPoints);
			return tmpLine;
		};

		std::vector<uint32> nineGridIds;
		linestring_t tmpPoints;
		std::vector<linestring_t> debugLines;
		std::vector<HadGrid*> allGrids;
		allGrids.insert(allGrids.end(), nearby.begin(), nearby.end());
		allGrids.push_back(pGrid);
		//std::cout << allGrids.size() << std::endl;
		for (int row = -1; row <= 1; row++) 
		{
			for (int col = -1; col <= 1; col++) 
			{
				nineGridIds.push_back(NdsGridId_shift(MeshId_toNdsGridId(pGrid->getId()), row, col));
			}
		}

		CompileSetting* pSetting = CompileSetting::instance();
		int boundNum = pSetting->boundNumber;
		int dk = (boundNum + 1) * 2;
		if (dk < 2)
			dk = 2;
		int pn = dk + 1;
		int n = dk * 3;
		int k = dk - 1;

		auto getBoundaryPoints = [&](const point_t& p0, const point_t& p1)-> std::vector<point_t>
		{
			std::vector<point_t> tmpPoints;
			double dx = (p1.get<0>() - p0.get<0>()) / dk;
			double dy = (p1.get<1>() - p0.get<1>()) / dk;
			tmpPoints.push_back(p0);
			for (size_t i = 1; i < pn - 1; ++i)
			{
				int64 ndx = i * dx;
				int64 ndy = i * dy;
				point_t p = point_t(p0.get<0>() + ndx, p0.get<1>() + ndy);
				tmpPoints.push_back(p);
			}
			tmpPoints.push_back(p1);
			return tmpPoints;
		};

		std::vector<size_t> ids = std::vector<size_t>{2, 5, 8, 1, 4, 7, 0, 3, 6};
		std::vector<std::vector<std::vector<point_t>>> allGridPoints;
		for (size_t i = 0; i < ids.size(); ++i)
		{
			Rect tmpRect;
			NdsGridId_getRect(nineGridIds[ids.at(i)], &tmpRect);
			auto tmpBox = getBox(tmpRect);
			std::vector<point_t> topPoints = getBoundaryPoints(tmpBox.at(0), tmpBox.at(1));
			std::vector<point_t> bottomPoints = getBoundaryPoints(tmpBox.at(2), tmpBox.at(3));
			std::vector<point_t> leftPoints = getBoundaryPoints(tmpBox.at(0), tmpBox.at(2));
			std::vector<point_t> rightPoints = getBoundaryPoints(tmpBox.at(1), tmpBox.at(3));
			std::vector<std::vector<point_t>> oneGridPoints;
			oneGridPoints.push_back(topPoints);
			for (size_t j = 1; j < pn - 1; ++j)
			{
				std::vector<point_t> tmpRowPoints;
				tmpRowPoints.push_back(leftPoints.at(j));
				for (size_t k = 1; k < pn - 1; ++k)
				{
					point_t p = point_t(topPoints.at(k).get<0>(), leftPoints.at(j).get<1>());
					tmpRowPoints.push_back(p);
				}
				tmpRowPoints.push_back(rightPoints.at(j));
				oneGridPoints.push_back(tmpRowPoints);
			}
			oneGridPoints.push_back(bottomPoints);
			allGridPoints.push_back(oneGridPoints);
			debugLines.push_back(tmpBox);
		}
		std::vector<point_t> tmpNewPoints;
		std::vector<std::vector<point_t>> firstRoundPoints;
		for (size_t i = 0; i < 3; ++i)
		{
			for (size_t j = 0; j < dk; ++j)
			{
				std::vector<point_t> tmpRowPoints;
				for (size_t k = 0; k < 3; ++k)
				{
					auto& ps = allGridPoints.at(i * 3 + k).at(j);
					tmpRowPoints.insert(tmpRowPoints.end(), ps.begin(), ps.end() - 1);
					if (k == 2)
						tmpRowPoints.push_back(ps.back());
				}
				firstRoundPoints.push_back(tmpRowPoints);
				tmpNewPoints.insert(tmpNewPoints.end(), firstRoundPoints.back().begin(), firstRoundPoints.back().end());
			}
			if (i == 2)
			{
				std::vector<point_t> tmpRowPoints;
				for (size_t k = 0; k < 3; ++k)
				{
					auto& ps = allGridPoints.at(i * 3 + k).at(dk);
					tmpRowPoints.insert(tmpRowPoints.end(), ps.begin(), ps.end() - 1);
					if (k == 2)
						tmpRowPoints.push_back(ps.back());
				}
				firstRoundPoints.push_back(tmpRowPoints);
				tmpNewPoints.insert(tmpNewPoints.end(), firstRoundPoints.back().begin(), firstRoundPoints.back().end());
			}
		}

		//for (auto& d1 : tmpNewPoints)
		//{
		//	std::cout << d1.get<0>() << "," << d1.get<1>() << std::endl;
		//}
		//std::cout << "..................." << std::endl;

		//生成vertices和quad
		std::vector<gridVertex> tmpVertices;
		std::vector<gridQuad> tmpQuads;
		for (size_t i = 0; i < tmpNewPoints.size(); ++i)
		{
			gridVertex tmpVertex = gridVertex(tmpNewPoints.at(i), i);
			tmpVertices.push_back(tmpVertex);
		}
		for (size_t i = 0; i < n * n; ++i)
		{
			int x = i / n;
			int y = i % n;
			int a0 = x * (n + 1) + y;
			int a1 = x * (n + 1) + y + 1;
			int a2 = (x + 1) * (n + 1) + y + 1;
			int a3 = (x + 1) * (n + 1) + y;
			auto& v0 = tmpVertices.at(a0);
			auto& v1 = tmpVertices.at(a1);
			auto& v2 = tmpVertices.at(a2);
			auto& v3 = tmpVertices.at(a3);
			gridQuad tmpQuad = gridQuad(v0, v1, v2, v3, i);
			tmpQuads.push_back(tmpQuad);
		}
		std::vector<gridVertex*> tmpFirstVertices;
		for (size_t i = 0; i < tmpVertices.size(); ++i)
		{
			int x = i / (n + 1);
			int y = i % (n + 1);
			if (x == 0 || x == n || y == 0 || y == n)
				continue;
			auto& tmpVertex = tmpVertices.at(i);
			tmpFirstVertices.push_back(&tmpVertex);
			int M = i - x;
			tmpVertex._quads.at(0) = &tmpQuads.at(M - n -1);
			tmpVertex._quads.at(1) = &tmpQuads.at(M - n);
			tmpVertex._quads.at(2) = &tmpQuads.at(M);
			tmpVertex._quads.at(3) = &tmpQuads.at(M - 1);
		}
		for (size_t i = 0; i < tmpQuads.size(); ++i)
		{
			auto& tmp = tmpQuads.at(i);
			tmp._quadBox = box_2t(P3_P2(tmp._vertices.at(3)->_originPoint), P3_P2(tmp._vertices.at(1)->_originPoint));
		}

		//基于车道数据对Quad的高度进行调整
		std::vector<point_t> laneLinePoints;
		std::vector<point_2t> laneLinePoints2T;
		std::vector<MapPoint3D64> allPoints;
		for (auto& tmpGrid : allGrids)
		{
			//lane group
			for (auto tmpObj : tmpGrid->query(ElementType::HAD_LANE_GROUP))
			{
				HadLaneGroup* item = (HadLaneGroup*)tmpObj;
				if (isTunnelArea(item))
					continue;
				for (auto& obj : item->roadBoundaries)
					allPoints.insert(allPoints.end(), obj->location.vertexes.begin(), obj->location.vertexes.end());
				for (auto& obj : item->laneBoundaries)
					allPoints.insert(allPoints.end(), obj->location.vertexes.begin(), obj->location.vertexes.end());
				for (auto& obj : item->lanes)
					allPoints.insert(allPoints.end(), obj->location.vertexes.begin(), obj->location.vertexes.end());
			}
			linestring_t tmp = LINESTRING_T(allPoints);
			linestring_2t tmp2T = LINESTRING_2T(allPoints);
			laneLinePoints.insert(laneLinePoints.end(), tmp.begin(), tmp.end());
			laneLinePoints2T.insert(laneLinePoints2T.end(), tmp2T.begin(), tmp2T.end());
		}

		parameters param;
		index_2getter originInd(laneLinePoints2T);
		rtree_2type rtree(boost::irange<std::size_t>(0lu, laneLinePoints2T.size()), param, originInd);

		std::vector<point_2t> basePoints;
		std::vector<int> basePointIds;
		for (size_t i = 0; i < tmpQuads.size(); ++i)
		{
			auto& tmp = tmpQuads.at(i);
			std::vector<size_t> pointIdsInBox;
			std::vector<point_t> pointsInBox;
			rtree.query(bgi::intersects(tmp._quadBox),
				boost::make_function_output_iterator([&](size_t const& id) {
					pointIdsInBox.push_back(id);
					pointsInBox.push_back(laneLinePoints.at(id));
					})
			);

			if (pointsInBox.empty())
				continue;

			std::sort(pointsInBox.begin(), pointsInBox.end(), [&](const point_t& a, const point_t& b)->bool {
				return a.get<2>() < b.get<2>();
				});

			point_t resultPoint = pointsInBox.front();
			auto tmpMinPoint = point_t(
				resultPoint.get<0>(),
				resultPoint.get<1>(),
				resultPoint.get<2>() / 10);
			tmp.isBaseHeight = true;
			tmp._height = tmpMinPoint.get<2>();
			basePointIds.push_back(i);
			point_2t tmpCenter = point_2t(
				(tmp._quadBox.min_corner().get<0>() + tmp._quadBox.max_corner().get<0>()) / 2,
				(tmp._quadBox.min_corner().get<1>() + tmp._quadBox.max_corner().get<1>()) / 2);
			basePoints.push_back(tmpCenter);
		}

		//parameters basePointsParam;
		//index_2getter basePointOriginInd(basePoints);
		//rtree_2type basePointRtree(boost::irange<std::size_t>(0lu, basePoints.size()), basePointsParam, basePointOriginInd);
		//for (size_t i = 0; i < tmpQuads.size(); ++i)
		//{
		//	auto& tmp = tmpQuads.at(i);
		//	if (tmp.isBaseHeight)
		//		continue;
		//	point_2t tmpCenter = point_2t(
		//		(tmp._quadBox.min_corner().get<0>() + tmp._quadBox.max_corner().get<0>()) / 2,
		//		(tmp._quadBox.min_corner().get<1>() + tmp._quadBox.max_corner().get<1>()) / 2);
		//	basePointRtree.query(bgi::nearest(tmpCenter, 1),
		//		boost::make_function_output_iterator([&](size_t const& id) {
		//			tmp._height = tmpQuads.at(basePointIds.at(id))._height;
		//			tmp.isBaseHeight = true;
		//			})
		//	);
		//}

		auto isInBoundForQuad = [&](int& tmpIdex)-> bool
		{
			int x = tmpIdex / n;
			int y = tmpIdex % n;
			if (x > 0 && x < n - 1 && y > 0 && y < n - 1)
				return true;
			return false;
		};

		int tmpNN = n * n;
		auto getQuadHeightsForVertex = [&](const int& k, int& vi)-> std::vector<int64>
		{
			std::vector<int64> tmp;
			auto dk = 2 * k;
			int x = vi / (n + 1);
			int y = vi % (n + 1);
			int W = vi - x - k * (n + 1);
			if (x >= 0 && x < n && y >= 0 && y < n)
			{
				for (size_t i = 0; i < dk; ++i)
				{
					for (size_t j = 0; j < dk; ++j)
					{
						int tmpM = W + n * i + j;
						if (tmpM >= 0 && tmpM < tmpNN)
						{
							if (tmpQuads.at(tmpM).isBaseHeight)
							{
								tmp.push_back(tmpQuads.at(tmpM)._height);
							}
						}
					}
				}
				return tmp;
			}
		};

		for (int i = 0; i < tmpVertices.size(); ++i)
		{
			int x = i / (n + 1);
			int y = i % (n + 1);
			if (x == 0 || x == n || y == 0 || y == n)
				continue;
			auto& tmpVertex = tmpVertices.at(i);
			auto dP = tmpVertex._originPoint;
			std::vector<int64> quadMinHeights = getQuadHeightsForVertex(boundNum, i);
			if (!quadMinHeights.empty())
			{
				boost::accumulators::accumulator_set<int64, boost::accumulators::features<boost::accumulators::tag::mean, boost::accumulators::tag::variance>> acc;
				for (const auto& value : quadMinHeights)
				{
					//std::cout << value << " ";
					acc(value);
				}

				// 计算平均数,方差
				double mean = boost::accumulators::mean(acc);
				double var = boost::accumulators::variance(acc);
				//std::cout << "mean: " << mean << " var:" << var << std::endl;

				if (var < 100)
				{
					auto maxElement = std::max_element(quadMinHeights.begin(), quadMinHeights.end());
					tmpVertex._height = *maxElement;
				}
				else
				{
					tmpVertex._height = mean;
				}
			}

		}

		//for (size_t i = 0; i < tmpQuads.size(); ++i)
		//{
		//	auto& tmp = tmpQuads.at(i);
		//	for (auto& item : tmp._vertices)
		//		std::cout << item->_height << " ";
		//	std::cout << std::endl;
		//}

		//开始插值
		std::vector<box_2t> realBox2Ts;
		for (size_t i = 0; i < tmpQuads.size(); ++i)
			realBox2Ts.push_back(tmpQuads.at(i)._quadBox);
		parameters paramBox2T;
		index_getter_2box originIndBox2T(realBox2Ts);
		rtree_type_2box rtreeBox2T(boost::irange<std::size_t>(0lu, realBox2Ts.size()), paramBox2T, originIndBox2T);

		auto interpolateValueForQuad = [&](const point_2t& p, gridQuad* const pQuad, int32& resultValue)-> bool
		{
			double dx1 = p.get<0>() - pQuad->_vertices.at(0)->_originPoint.get<0>();
			double dx2 = pQuad->_vertices.at(1)->_originPoint.get<0>() - p.get<0>();
			double dy1 = pQuad->_vertices.at(0)->_originPoint.get<1>() - p.get<1>();
			double dy2 = p.get<1>() - pQuad->_vertices.at(3)->_originPoint.get<1>();
			if (dx1 >= 0.0 && dx2 >= 0.0 && dy1 >= 0.0 && dy2 >= 0.0)
			{
				auto S = (dx1 + dx2) * (dy1 + dy2);
				auto S0 = dx1 * dy1;
				auto S1 = dx2 * dy1;
				auto S2 = dx2 * dy2;
				auto S3 = dx1 * dy2;
				auto k0 = S0 / S;
				auto k1 = S1 / S;
				auto k2 = S2 / S;
				auto k3 = 1.0 - k0 - k1 - k2;
				int32 v0 = k0 * pQuad->_vertices.at(2)->_height;
				int32 v1 = k1 * pQuad->_vertices.at(3)->_height;
				int32 v2 = k2 * pQuad->_vertices.at(0)->_height;
				int32 v3 = k3 * pQuad->_vertices.at(1)->_height;
				resultValue = v0 + v1 + v2 + v3;
				return true;
			}
			return false;
		};

		auto interpolateMapPoints = [&](std::vector<MapPoint3D64>& pLine)-> void
		{
			if (pLine.empty())
				return;
			linestring_2t tmp2T = LINESTRING_2T(pLine);
			for (size_t j = 0; j < tmp2T.size(); ++j)
			{
				std::vector<size_t> resultIds;
				auto& _p = tmp2T.at(j);
				box_2t _b = box_2t(point_2t(_p.get<0>() - 3000, _p.get<1>() - 3000), point_2t(_p.get<0>() + 3000, _p.get<1>() + 3000));
				rtreeBox2T.query(bgi::nearest(_p, 4),
					boost::make_function_output_iterator([&](size_t const& id) {
						resultIds.push_back(id);
						}));
				std::vector<box_2t> debugBoxes;
				for (auto& id : resultIds)
					debugBoxes.push_back(realBox2Ts.at(id));
				bool isGetQuad = false;
				for (auto& id : resultIds)
				{
					auto& tmpQuad = tmpQuads.at(id);
					int32 resultValue{ 0 };
					if (interpolateValueForQuad(_p, &tmpQuad, resultValue))
					{
						pLine.at(j).z -= resultValue;
						isGetQuad = true;
						break;
					}
				}

				auto firstVertex = tmpQuads.at(resultIds.front())._vertices.front();
				if (!isGetQuad)
				{
					//std::cout << _p.get<0>() / 1000 << ", " << _p.get<1>() / 1000 << std::endl;
					//std::cout << pGrid->getId() << std::endl;
					for (auto& id : resultIds)
					{
						auto& tmpQuad = tmpQuads.at(id);
						for (auto& v : tmpQuad._vertices)
						{
							if (bg::distance(P3_P2(v->_originPoint), _p) < bg::distance(P3_P2(firstVertex->_originPoint), _p))
								firstVertex = v;
						}
					}
					pLine.at(j).z -= firstVertex->_originPoint.get<2>();
					isGetQuad = true;
				}
			}
		};

		auto interpolateMapPoint = [&](MapPoint3D64& pt)-> void
		{
			point_2t tmp2T = POINT_2T(pt);
			std::vector<size_t> resultIds;
			auto& _p = tmp2T;
			box_2t _b = box_2t(point_2t(_p.get<0>() - 3000, _p.get<1>() - 3000), point_2t(_p.get<0>() + 3000, _p.get<1>() + 3000));
			rtreeBox2T.query(bgi::nearest(_p, 4),
				boost::make_function_output_iterator([&](size_t const& id) {
					resultIds.push_back(id);
					}));
			std::vector<box_2t> debugBoxes;
			for (auto& id : resultIds)
				debugBoxes.push_back(realBox2Ts.at(id));
			bool isGetQuad = false;
			for (auto& id : resultIds)
			{
				auto& tmpQuad = tmpQuads.at(id);
				int32 resultValue{ 0 };
				if (interpolateValueForQuad(_p, &tmpQuad, resultValue))
				{
					pt.z -= resultValue;
					isGetQuad = true;
					break;
				}
			}

			auto firstVertex = tmpQuads.at(resultIds.front())._vertices.front();
			if (!isGetQuad)
			{
				//std::cout << _p.get<0>() / 1000 << ", " << _p.get<1>() / 1000 << std::endl;
				//std::cout << pGrid->getId() << std::endl;
				for (auto& id : resultIds)
				{
					auto& tmpQuad = tmpQuads.at(id);
					for (auto& v : tmpQuad._vertices)
					{
						if (bg::distance(P3_P2(v->_originPoint), _p) < bg::distance(P3_P2(firstVertex->_originPoint), _p))
							firstVertex = v;
					}
				}
				pt.z -= firstVertex->_originPoint.get<2>();
				isGetQuad = true;
			}
		};

		for (auto tmpObj : pGrid->query(ElementType::HAD_LINK))
		{
			HadLink* item = (HadLink*)tmpObj;
			interpolateMapPoints(item->location.vertexes);
		}
		for (auto tmpObj : pGrid->query(ElementType::HAD_NODE))
		{
			HadNode* item = (HadNode*)tmpObj;
			interpolateMapPoint(item->position);
		}
		for (auto tmpObj : pGrid->query(ElementType::HAD_ROAD_BOUNDARY_NODE))
		{
			HadRoadBoundaryNode* item = (HadRoadBoundaryNode*)tmpObj;
			interpolateMapPoint(item->position);
		}
		for (auto tmpObj : pGrid->query(ElementType::HAD_LANE_NODE))
		{
			HadLaneNode* item = (HadLaneNode*)tmpObj;
			interpolateMapPoint(item->position);
		}
		for (auto tmpObj : pGrid->query(ElementType::HAD_LANE_BOUNDARY_NODE))
		{
			HadLaneBoundaryNode* item = (HadLaneBoundaryNode*)tmpObj;
			interpolateMapPoint(item->position);
		}
		for (auto tmpObj : pGrid->query(ElementType::HAD_LANE_GROUP))
		{
			HadLaneGroup* item = (HadLaneGroup*)tmpObj;
			interpolateMapPoint(item->extent.min);
			interpolateMapPoint(item->extent.max);
		}
		for (auto tmpObj : pGrid->query(ElementType::HAD_LANE))
		{
			HadLane* item = (HadLane*)tmpObj;
			interpolateMapPoints(item->location.vertexes);
		}

		auto objs = pGrid->query(ElementType::HAD_LANE_BOUNDARY);
		std::sort(objs.begin(), objs.end(), [&](HadElement* const a, HadElement* const& b)->bool {
			return a->originId < b->originId;
			});
		for (auto it = objs.begin(); it != objs.end() - 1;)
		{
			if ((*it)->originId == (*(it + 1))->originId)
				std::cout << "origin id is repeated" << (*it)->originId << std::endl;
			else
				it++;
		}

		std::vector<int64> tmpOriginIds;
		std::vector<int64> tmpPaIds;
		for (auto tmpObj : objs)
		{
			HadLaneBoundary* item = (HadLaneBoundary*)tmpObj;
			interpolateMapPoints(item->location.vertexes);
			for (auto& tmpPa : item->attributes)
			{
				if (std::find(tmpPaIds.begin(), tmpPaIds.end(), tmpPa->originId) == tmpPaIds.end())
				{
					interpolateMapPoints(tmpPa->points.postions);
					tmpPaIds.push_back(tmpPa->originId);
				}
			}
			tmpOriginIds.push_back(item->originId);
		}
		for (auto tmpObj : pGrid->query(ElementType::HAD_ROAD_BOUNDARY))
		{
			HadRoadBoundary* item = (HadRoadBoundary*)tmpObj;
			interpolateMapPoints(item->location.vertexes);
			tmpOriginIds.push_back(item->originId);
		}

		for (auto obj : pGrid->query(ElementType::HAD_INTERSECTION))
		{
			HadIntersection* hadInts = (HadIntersection*)obj;
			for (auto iter = hadInts->refLaneGroups.begin(); iter != hadInts->refLaneGroups.end(); ++iter)
			{
				HadLaneGroup* pGroup = (HadLaneGroup*)(*iter);
				for (auto& item : pGroup->roadBoundaries)
				{
					HadRoadBoundary* itemObj = (HadRoadBoundary*)item;
					if (std::find(tmpOriginIds.begin(), tmpOriginIds.end(), itemObj->originId) == tmpOriginIds.end()) 
					{
						printInfo("intersection road boundary");
					}
				}
				for (auto& item : pGroup->laneBoundaries)
				{
					HadLaneBoundary* itemObj = (HadLaneBoundary*)item;
					if (std::find(tmpOriginIds.begin(), tmpOriginIds.end(), itemObj->originId) == tmpOriginIds.end())
					{
						printInfo("intersection lane boundary");
					}
				}
			}
		}

		//object
		for (auto tmpObj : pGrid->query(ElementType::HAD_OBJECT_CROSS_WALK))
		{
			HadCrossWalk* item = (HadCrossWalk*)tmpObj;
			interpolateMapPoints(item->polygon.vertexes);
		}
		for (auto tmpObj : pGrid->query(ElementType::HAD_OBJECT_STOPLOCATION))
		{
			HadStopLocation* item = (HadStopLocation*)tmpObj;
			interpolateMapPoints(item->location.vertexes);
		}
		for (auto tmpObj : pGrid->query(ElementType::HAD_OBJECT_TRAFFIC_LIGHTS))
		{
			HadTrafficLights* item = (HadTrafficLights*)tmpObj;
			interpolateMapPoints(item->location.vertexes);
		}
		for (auto tmpObj : pGrid->query(ElementType::HAD_OBJECT_FILL_AREA))
		{
			HadFillArea* item = (HadFillArea*)tmpObj;
			interpolateMapPoints(item->polygon.vertexes);
		}
		for (auto tmpObj : pGrid->query(ElementType::HAD_OBJECT_SYMBOL))
		{
			HadSymbol* item = (HadSymbol*)tmpObj;
			interpolateMapPoints(item->polygon.vertexes);
		}
		for (auto tmpObj : pGrid->query(ElementType::HAD_OBJECT_TEXT))
		{
			HadText* item = (HadText*)tmpObj;
			interpolateMapPoints(item->polygon.vertexes);
		}
		for (auto tmpObj : pGrid->query(ElementType::HAD_OBJECT_ARROW))
		{
			HadArrow* item = (HadArrow*)tmpObj;
			interpolateMapPoints(item->polygon.vertexes);
		}
		for (auto tmpObj : pGrid->query(ElementType::HAD_OBJECT_WALL))
		{
			HadWall* item = (HadWall*)tmpObj;
			for (auto& it : item->location.lines)
			{
				interpolateMapPoints(it.vertexes);
			}
		}
		for (auto tmpObj : pGrid->query(ElementType::HAD_OBJECT_TRAFFIC_SIGN))
		{
			HadTrafficSign* item = (HadTrafficSign*)tmpObj;
			interpolateMapPoints(item->polygon.vertexes);
		}
		for (auto tmpObj : pGrid->query(ElementType::HAD_OBJECT_BARRIER))
		{
			HadBarrier* item = (HadBarrier*)tmpObj;
			for (auto& it : item->location.lines)
			{
				interpolateMapPoints(it.vertexes);
			}
		}
		for (auto tmpObj : pGrid->query(ElementType::HAD_TOLLGATE))
		{
			HadTollGate* item = (HadTollGate*)tmpObj;
			interpolateMapPoint(item->position);
		}
		for (auto tmpObj : pGrid->query(ElementType::HAD_OBJECT_POLE))
		{
			HadPole* item = (HadPole*)tmpObj;
			interpolateMapPoints(item->location.vertexes);
		}

		return;
	}

	void GroupCompiler::interpolateHeightForTriangle(
		const Eigen::VectorXd& a,
		const Eigen::VectorXd& b,
		const Eigen::VectorXd& c,
		const Eigen::VectorXd& normal,
		const Eigen::VectorXd& h,
		Eigen::VectorXd& location)
	{
		using Eigen::Vector2d;
		using Eigen::Vector3d;
		const double d = normal.dot(a);
		const double t = d - location.dot(normal) / normal.dot(normal);
		Vector3d projected = normal;
		projected *= t;
		projected += location;
		int iMax;
		normal.cwiseAbs().maxCoeff(&iMax);
		int indices[2];
		if (iMax == 0) 
		{
			indices[0] = 1;
			indices[1] = 2;
		}
		else if (iMax == 1) 
		{
			indices[0] = 0;
			indices[1] = 2;
		}
		else 
		{
			printInfo("iMax == 2");
			indices[0] = 0;
			indices[1] = 1;
			return;
		}
		Vector2d a2D(a[indices[0]], a[indices[1]]);
		Vector2d b2D(b[indices[0]], b[indices[1]]);
		Vector2d c2D(c[indices[0]], c[indices[1]]);
		Vector2d projected2D(projected[indices[0]], projected[indices[1]]);
		Vector3d rhs(projected2D(0), projected2D(1), 1);
		Eigen::Matrix<double, 3, 3> A;
		A << a2D(0), b2D(0), c2D(0),
			a2D(1), b2D(1), c2D(1),
			1, 1, 1;
		Vector3d barycentricCoords = A.colPivHouseholderQr().solve(rhs);
		location(2) = h(0) * barycentricCoords(0) + h(1) * barycentricCoords(1) + h(2) * barycentricCoords(2);
		return;
	}

	void GroupCompiler::simplifierLine(
		HadGrid* const pGrid,
		const std::vector<HadGrid*>& nearby)
	{
		nearby;
		for (auto tmpObj : pGrid->query(ElementType::HAD_LANE_BOUNDARY))
		{
			HadLaneBoundary* item = (HadLaneBoundary*)tmpObj;
			PolylineSimplifier::simplify(item->location.vertexes);
		}
		for (auto tmpObj : pGrid->query(ElementType::HAD_ROAD_BOUNDARY))
		{
			HadRoadBoundary* item = (HadRoadBoundary*)tmpObj;
			PolylineSimplifier::simplify(item->location.vertexes);
		}
		for (auto tmpObj : pGrid->query(ElementType::HAD_LANE))
		{
			HadLane* item = (HadLane*)tmpObj;
			PolylineSimplifier::simplify(item->location.vertexes);
		}
	}

	void GroupCompiler::setCurrentLaneGroupData(HadGrid* const pGrid)
	{
		for (auto obj : pGrid->query(ElementType::HAD_LANE_GROUP))
		{
			HadLaneGroup* pGroup = (HadLaneGroup*)obj;
			if (!pGroup->roadBoundaries.empty())
			{
				PtrLaneGroupInfo tmp = std::make_shared<laneGroupInfo>();
				std::vector<MapPoint3D64> groupPoints;
				std::vector<MapPoint3D64> leftBoundary(pGroup->roadBoundaries[0]->location.vertexes.begin(), pGroup->roadBoundaries[0]->location.vertexes.end());
				if (directionEqual(pGroup->roadBoundaries[0], pGroup, 3))
					std::reverse(leftBoundary.begin(), leftBoundary.end());
				for (size_t i = 0; i < pGroup->roadBoundaries.size(); ++i)
				{
					std::vector<MapPoint3D64> tmpRoadVertexes(pGroup->roadBoundaries[i]->location.vertexes.begin(), pGroup->roadBoundaries[i]->location.vertexes.end());
					std::vector<MapPoint3D64> tmpBoundaryVertexes(pGroup->roadBoundaries[i]->location.vertexes.begin(), pGroup->roadBoundaries[i]->location.vertexes.end());
					if (directionEqual(pGroup->roadBoundaries[i], pGroup, 3))
					{
						std::reverse(tmpRoadVertexes.begin(), tmpRoadVertexes.end());
					}
					if (i % 2 == 1)
					{
						std::reverse(tmpRoadVertexes.begin(), tmpRoadVertexes.end());
					}
					groupPoints.insert(groupPoints.end(), tmpRoadVertexes.begin(), tmpRoadVertexes.end());

					if (i == 0)
					{
						auto tmpLeftVertexes = tmpBoundaryVertexes;
						tmp->_originMapLineL.vertexes = tmpLeftVertexes;
						if (directionEqual(pGroup->roadBoundaries[i], pGroup, 3))
						{
							std::reverse(tmpLeftVertexes.begin(), tmpLeftVertexes.end());
							tmp->isReversedL = true;
						}
						coordinatesTransform.convert(tmpLeftVertexes.data(), tmpLeftVertexes.size());
						tmp->_originLineL = LINESTRING_T(tmpLeftVertexes);
						tmp->_raisedLineL = tmp->_originLineL;
					}
					else if (i == 1)
					{
						auto tmpRightVertexes = tmpBoundaryVertexes;
						tmp->_originMapLineR.vertexes = tmpRightVertexes;
						if (directionEqual(pGroup->roadBoundaries[i], pGroup, 3))
						{
							std::reverse(tmpRightVertexes.begin(), tmpRightVertexes.end());
							tmp->isReversedR = true;
						}
						coordinatesTransform.convert(tmpRightVertexes.data(), tmpRightVertexes.size());
						tmp->_originLineR = LINESTRING_T(tmpRightVertexes);
						tmp->_raisedLineR = tmp->_originLineR;
					}
				}
				coordinatesTransform.convert(groupPoints.data(), groupPoints.size());
				tmp->_laneGroup = pGroup;
				currentGridGroupMaps.emplace(pGroup->originId, tmp);
			}
		}
	}

    void GroupCompiler::constructLaneGroupRtree(HadGrid* const  pGrid)
    {
        std::vector<box_2t> boxs3D;
        for (auto obj : pGrid->query(ElementType::HAD_LANE_GROUP))
        {
            HadLaneGroup* pGroup = (HadLaneGroup*)obj;
            std::vector<MapPoint3D64> v;
            v.emplace_back(pGroup->extent.max);
            v.emplace_back(pGroup->extent.min);

            compilerData.m_laneGroupBoxes.emplace_back(BOX_2T(v));
            compilerData.gridLaneGroups.emplace_back(pGroup);
        }
    }

}
