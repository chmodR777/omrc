#include "stdafx.h"
#include "TransparentCompiler.h"
#include "algorithm/grap_point_algorithm.h"
#include "cq_math_basic.h"
namespace OMDB
{
    extern MapPoint3D64 PT_MT(point_t pt);
    const float METER_PER_LAT_UNIT = 1.11f;
    const float LAT_UNIT_PER_METER = 1 / METER_PER_LAT_UNIT;

    
    void TransparentCompiler::compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile)
    {

        _pGrid = pGrid;
        _nearby = nearby;
        _pTile = pTile;

         

        //压盖的车道组
        overlayLaneGroup();
        //上方道路扩展150米
        expandUpperRoad(true, 150* LAT_UNIT_PER_METER* 1000);
        //上方道路扩展150米
        expandUpperRoad(false, 150 * LAT_UNIT_PER_METER * 1000);

        //下方道路向后扩展150米
        expandLowerRoad(false, 150 * LAT_UNIT_PER_METER * 1000);
        //下方道路向前扩展100米
        expandLowerRoad(true, 100 * LAT_UNIT_PER_METER * 1000);
      
        //写入数据
        writeRDS();
      

    }

  
    void TransparentCompiler::overlayLaneGroup()
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


        std::vector<HadGrid*> grids = _nearby;
        grids.emplace_back(_pGrid);

        for (HadGrid* grid : grids)
        {
            for (auto obj : grid->query(ElementType::HAD_LANE_GROUP))
            {
                HadLaneGroup* pLaneGroup = (HadLaneGroup*)obj;
                std::vector<MapPoint3D64> boxPts;
                boxPts.emplace_back(pLaneGroup->extent.min);
                boxPts.emplace_back(pLaneGroup->extent.max);

                auto laneGroupPolyGon = getLaneGroupPolygon(pLaneGroup);


                getLaneGroupBox2DRTree()->query(bgi::intersects(BOX_2T(boxPts)),

                    boost::make_function_output_iterator([&](size_t const& id) {
                        HadLaneGroup* nearByLaneGroup = compilerData.gridLaneGroups[id];
                        if (pLaneGroup->originId == nearByLaneGroup->originId)
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
                                PtrOverlayLaneGroup ptr = std::make_shared<OverlayLaneGroup>();
                                ptr->upperLG = nearByLaneGroup;
                                ptr->upperCrossPt = PT_MT(otherNearbyPoint);
                                ptr->lowerCrossPt = PT_MT(currentNearbyPoint);
                                auto iter = _overLapRel.try_emplace(pLaneGroup, std::vector<PtrOverlayLaneGroup>{}).first;
                                iter->second.emplace_back(ptr);

                                auto iter_ex = _overLapRelExpand.try_emplace(pLaneGroup, std::set<HadLaneGroup*>{}).first;
                                iter_ex->second.insert(nearByLaneGroup);
                            }
                        }
                        }));

            }
        }
    
      
        /*  for (const auto& pair : _overLapRel)
          {
              HadLaneGroup* lowLG = pair.first;
              const std::vector<PtrOverlayLaneGroup>& upLGs = pair.second;

              printInfo("lower lane group %I64d", lowLG->originId);
              for (const PtrOverlayLaneGroup ptrOverlay : upLGs)
              {
                  printInfo("   upper lane group %I64d", ptrOverlay->upperLG->originId);
              }
          }*/
    }


    void TransparentCompiler::expandUpperRoad(bool bforward, int length)
    {

        for (const auto& pair : _overLapRel)
        {
            HadLaneGroup* lowLG = pair.first;
            const std::vector<PtrOverlayLaneGroup>& upLGs = pair.second;

            auto iter_expandRel = _overLapRelExpand.try_emplace(lowLG, std::set<HadLaneGroup*>{}).first;
            
            for (const PtrOverlayLaneGroup ptrOverlay : upLGs)
            {
                // 取一条线
                LineString3d line;
                getCenterLine(ptrOverlay->upperLG, line);

                float lenFromStart = calcLength(line.vertexes, ptrOverlay->upperCrossPt);

                //广度遍历
                std::vector<HadLaneGroup*> lgExpands;
                if (bforward)
                {
                 
                    bfsLaneGroup(ptrOverlay->upperLG, bforward, length + lenFromStart, lgExpands);
                }
                else
                {
                    float lineLenght = calcLength(line.vertexes);
                    bfsLaneGroup(ptrOverlay->upperLG, bforward, length +(lineLenght- lenFromStart), lgExpands);
                }
            
                //加入集合
                for (HadLaneGroup* lgExpand : lgExpands)
                {
                    if (!isTouchWithHadLaneGroup(lgExpand,lowLG) &&  lgExpand != lowLG)
                    {
                        iter_expandRel->second.insert(lgExpand);
                    }
                    // LA关系也加入
                    for (HadLaneGroupAssociation* laAssoc : lgExpand->associations)
                    {
                        for (HadLaneGroupAssociation::HadRelLaneGroupAssociation la : laAssoc->relLaneGroupAssociations)
                        {
                            HadLaneGroup* lgLA = la.first == lgExpand ? la.second : la.first;
                            if (lgLA && !isTouchWithHadLaneGroup(lgLA, lowLG) && lgLA != lowLG)
                            {
                                iter_expandRel->second.insert(lgLA);
                            }


                        }
                    }
                    
                }
            }

        }


     /*   printInfo("expandUpperRoad begin");

        for (const auto& pair : _overLapRelExpand)
        {
            HadLaneGroup* lowLG = pair.first;
            const std::set<HadLaneGroup*>& upLGs = pair.second;

            printInfo("lower lane group %I64d", lowLG->originId);
            for (const HadLaneGroup* lg : upLGs)
            {
                printInfo("   upper lane group %I64d", lg->originId);
            }
        }

        printInfo("expandUpperRoad end");*/
    }
    

    void TransparentCompiler::expandLowerRoad(bool bforward, int length)
    {
        for (const auto& pair : _overLapRel)
        {
            HadLaneGroup* lowLG = pair.first;
            const std::vector<PtrOverlayLaneGroup>& upLGs = pair.second;

            if (!_overLapRelExpand.count(lowLG))
            {
                continue;
            }
            for (const PtrOverlayLaneGroup ptrOverlay : upLGs)
            {
                // 取一条线
                LineString3d line;
                getCenterLine(lowLG, line);

                float lenFromStart = calcLength(line.vertexes, ptrOverlay->lowerCrossPt);

                //广度遍历
                std::vector<HadLaneGroup*> lgExpands;
                if (bforward)
                {
                    float lineLenght = calcLength(line.vertexes);
                    bfsLaneGroup(lowLG, bforward, length+ lenFromStart, lgExpands);
                }
                else
                {
                    float lineLenght = calcLength(line.vertexes);

                    bfsLaneGroup(lowLG, bforward, length + (lineLenght - lenFromStart), lgExpands);
                }

                //加入_overLapRelExpand
                for (HadLaneGroup* lgExpand : lgExpands)
                {
                    if (lgExpand == lowLG)
                    {
                        continue;
                    }
                    auto iter_expandRel = _overLapRelExpand.try_emplace(lgExpand, std::set<HadLaneGroup*>{}).first;
                    for (HadLaneGroup* upperLg : _overLapRelExpand[lowLG])
                    {
                        if (!isTouchWithHadLaneGroup(lgExpand, upperLg))
                        {
                            iter_expandRel->second.insert(upperLg);
                        }
                    }
                   
                    // LA关系也加入
                    for (HadLaneGroupAssociation* laAssoc : lgExpand->associations)
                    {
                        for (HadLaneGroupAssociation::HadRelLaneGroupAssociation la : laAssoc->relLaneGroupAssociations)
                        {
                            HadLaneGroup* lgLA = la.first == lgExpand ? la.second : la.first;

                            if (lgLA)
                            {
                                auto iter_expand_asso = _overLapRelExpand.try_emplace(lgLA, std::set<HadLaneGroup*>{}).first;

                                for (HadLaneGroup* upperLg : _overLapRelExpand[lowLG])
                                {
                                    if (!isTouchWithHadLaneGroup(lgLA, upperLg) && lgLA != upperLg)
                                    {
                                        iter_expand_asso->second.insert(upperLg);
                                    }
                                }

                            }
                         
                          

                        }
                    }

                 
                }
            }

        }

        //printInfo("expandLowerRoad begin");

        //for (const auto& pair : _overLapRelExpand)
        //{
        //    HadLaneGroup* lowLG = pair.first;
        //    const std::set<HadLaneGroup*>& upLGs = pair.second;

        //    printInfo("lower lane group %I64d", lowLG->originId);
        //    for (const HadLaneGroup* lg : upLGs)
        //    {
        //        printInfo("   upper lane group %I64d", lg->originId);
        //    }
        //}

        printInfo("expandLowerRoad end");

    }

    void TransparentCompiler::writeRDS()
    {
        for (const auto& pair : _overLapRelExpand)
        {
            HadLaneGroup* lowLG = pair.first;
            const std::set<HadLaneGroup*>& upperLGs = pair.second;

            std::set<HadLaneGroup*> seconds;
            for (HadLaneGroup* uplg: upperLGs)
            {
                //当前网格
                if (uplg->owner->getId() == _pGrid->getId())
                {
                    seconds.insert(uplg);
                }
            }

            if (!seconds.empty())
            {
                RdsTransparencyRel* rdsTransRel = (RdsTransparencyRel*)createObject(_pTile, EntityType::RDS_TRANSPARENCYREL);
                rdsTransRel->first = lowLG->originId;
                for (HadLaneGroup* lg : seconds)
                {
                    RDS::RdsGroup* group = queryGroup(lg->originId, _pTile);
                    if (group)
                    {
                        rdsTransRel->second.emplace_back(group->id);
                    }
                }
              
            }
         
        }
    }

    void TransparentCompiler::getCenterLine(HadLaneGroup* pGroup, LineString3d& line)
    {
        if (!pGroup->laneBoundaries.empty())
        {
            line = pGroup->laneBoundaries[pGroup->laneBoundaries.size() / 2]->location;
            if (directionEqual(pGroup->laneBoundaries[pGroup->laneBoundaries.size() / 2], pGroup, 3))
            {
                std::reverse(line.vertexes.begin(), line.vertexes.end());
            }

        }
        else if (!pGroup->roadBoundaries.empty())
        {
            line = pGroup->roadBoundaries[pGroup->roadBoundaries.size() / 2]->location;
            if (directionEqual(pGroup->roadBoundaries[pGroup->roadBoundaries.size() / 2], pGroup, 3))
            {
                std::reverse(line.vertexes.begin(), line.vertexes.end());
            }
        }
    }

    //从LG的起点处开始搜索searchDistance距离的车道组
    void TransparentCompiler::bfsLaneGroup(HadLaneGroup* lg, bool bForward, float searchDistance, std::vector<HadLaneGroup*>& lgOut) {

        std::set<HadLaneGroup*> lgSearched;

        struct bfs_info {
            HadLaneGroup* lg;
            bool bForward;
            float distance;
        };

        std::queue<bfs_info> queueLaneGroup;
        queueLaneGroup.push(bfs_info{ lg,bForward,searchDistance });

        while (!queueLaneGroup.empty())
        {
            bfs_info bfsGroupInfo = queueLaneGroup.front();
            queueLaneGroup.pop();

            if (lgSearched.count(bfsGroupInfo.lg) || bfsGroupInfo.distance <= 0)
            {
                continue;
            }
            lgSearched.insert(bfsGroupInfo.lg);
            lgOut.emplace_back(bfsGroupInfo.lg);

            LineString3d line;
            getCenterLine((HadLaneGroup*)bfsGroupInfo.lg, line);
            float len = calcLength(line.vertexes);
            float sercherLen = bfsGroupInfo.distance-len;
            if (sercherLen < 0)
            {
                continue;
            }
            if (bfsGroupInfo.bForward)
            {

                for (auto lane : bfsGroupInfo.lg->next)
                {
                    if (!lgSearched.count((HadLaneGroup*)lane))
                    {
                        queueLaneGroup.push(bfs_info{ (HadLaneGroup*)lane,bForward,sercherLen });
                    }

                }
            }
            else
            {
                for (auto lane : bfsGroupInfo.lg->previous)
                {
                    if (!lgSearched.count((HadLaneGroup*)lane))
                    {
                        queueLaneGroup.push(bfs_info{ (HadLaneGroup*)lane,bForward,sercherLen });
                    }
                }
            }

        }

    }

   


    // 创建法线方向的多边形
    ring_2t TransparentCompiler::getRect(MapPoint3D64 start, MapPoint3D64 end, int distance)
    {
        point_2t startPt = POINT_2T(start);
        point_2t endPt = POINT_2T(end);

        ring_2t ring;
        segment_2t seg(POINT_2T(start), POINT_2T(end));
        segment_2t segleft = SEGMENT_2T_EX(seg, distance);

        segment_2t segT;
        bg::strategy::transform::rotate_transformer<boost::geometry::degree, int, 2, 2> rotate(45);
        bg::transform(startPt, endPt, rotate);

        return ring;
        
    }
}