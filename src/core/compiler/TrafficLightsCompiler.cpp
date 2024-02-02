#include "stdafx.h"
#include "TrafficLightsCompiler.h"
#include "grap_point_algorithm.h"
#include "cq_math_basic.h"
namespace OMDB
{

    MapPoint3D64 PT_MT(point_t pt)
    {
        return MapPoint3D64_make(pt.get<0>(), pt.get<1>(), pt.get<2>() / 10);
    }

    // 弧度->角度:[0~360)
    uint16 toDegree(double angle)
    {
        return (uint16)(((int32)(angle * 180 / MATH_PI) % 360 + 360) % 360);
    }

    // 正北0度顺时针->正东0度逆时针[0~360)
    int north_degree_to_east_degree(int degree)
    {
        return ((360 - degree % 360) + 90) % 360;
    }

    // 点到点的方向角度
    int bg_degree(point_t pt1, point_t pt2)
    {
        auto a = Math_getDirectionFromTwoCoordinatesNds(PT_MT(pt1).pos.toNdsPoint(), PT_MT(pt2).pos.toNdsPoint());
        return toDegree(a);
    }

    // 角度之差
    int difference_of_degreess(int dg1, int dg2)
    {
        double degree = abs(dg1 - dg2) % 360;
        degree = degree > 180 ? 360 - degree : degree;
        return degree;
    }

    // 旋转
    int degree_add(int d, int value)
    {
        d += value;
        d = d < 0 ? d + 360 : d;
        d %= 360;
        return d;
    }

#define POLE_HEIGHT_ADJUSTMENT (-50)

    void TrafficLightsCompiler::compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile)
    {

        _pGrid = pGrid;
        _nearby = nearby;
        _pTile = pTile;

        // R-Tree
        createLightsBoxRtree();
        createPoleRtree();
        createLaneGroupRtree();

        collectLightsSystem();
        createRDS();
    }

    void TrafficLightsCompiler::collectLightsSystem()
    {
      
        std::map<HadPole*, PtrLightsPoleSystem> holeSystem;
        for (auto lg : _pGrid->query(ElementType::HAD_LANE_GROUP))
        {
            HadLaneGroup* laneGroup = (HadLaneGroup*)lg;

            /* if (laneGroup->originId != 209271043941421825)
             {
                 continue;
             }*/

             // 车道组内红绿灯
            std::vector<PtrTrafficLightsInfo> ligthsInLGroup;
            for (auto obj : laneGroup->objects)
            {
                if (obj->objectType == ElementType::HAD_OBJECT_TRAFFIC_LIGHTS)
                {
                    HadTrafficLights* l = (HadTrafficLights*)obj;
                    if (l->type != 1
                        || l->columnNumber > 3
                        || _lightsExtendInfo.count(l->originId)

                        )
                    {
                        continue;

                        PtrTrafficLightsInfo p = getLightsExtendInfo(l);
                        if (p)
                        {
                            ligthsInLGroup.emplace_back(p);
                            _lightsExtendInfo.emplace(l->originId, p);
                        }

                    }
                }

                if (ligthsInLGroup.empty())
                    continue;

                // 路面高度
                MapPoint3D64* pPolygon = NULL;
                size_t nPolygonPtNum = 0;
                int roadHeight = 0;
                if ((nPolygonPtNum = getRightBoundary(laneGroup, pPolygon)) == 0)
                {
                    continue;
                }
                roadHeight = pPolygon[nPolygonPtNum - 1].z;

                // 聚类
                std::vector<LightsCollect> clusteringLights;
                clustering(ligthsInLGroup, clusteringLights, 6);

                for (auto l : clusteringLights)
                {
                    HadPole* pole = FindPole(l, roadHeight);
                    if (pole)
                    {
                        //最高点
                        auto p = std::max_element(pole->location.vertexes.begin(), pole->location.vertexes.end(),
                            [&](MapPoint3D64& p1, MapPoint3D64& p2)
                            {
                                return p1.z < p2.z;
                            }

                        );
                        MapPoint3D64 polePtMaxZ = *p;

                        HadLaneGroup* lg = IsPointOnHadLaneGroup(polePtMaxZ);
                        if (lg && !lg->inIntersection)
                        {
                            continue;
                        }
                        //是否距离车道过远
                        double distanceToLaneGroup = UINT32_MAX;
                        getNeareastLaneGroup(polePtMaxZ, distanceToLaneGroup);
                        if (distanceToLaneGroup > 8)
                        {
                            continue;
                        }


                        std::vector<LightsCollect> LightsCollectByPole;
                        spliteTShapePole(l, pole, LightsCollectByPole);	//T型杆分割
                        for (auto lightsCollect : LightsCollectByPole)
                        {
                            PtrLightsPoleSystem ptrSystem = createLightsSystem(lightsCollect, roadHeight, pole);

                            if (!ptrSystem)
                            {
                                continue;
                            }
                            //同杆2个方向
                            if (holeSystem.count(pole))
                            {
                                if (difference_of_degreess(ptrSystem->barHeading, holeSystem[pole]->barHeading) < 20)
                                {
                                    MergePtrLightsPoleSystem(holeSystem[pole], ptrSystem);
                                    continue;
                                }
                            }

                            _lightsPoleSystemArray.emplace_back(ptrSystem);
                            holeSystem.insert({ pole, ptrSystem });
                        }


                    }
                    else
                    {


                        // 做成竖杆
                        auto ptrSysWithNoPole = createLightsSystemNotFindPole(l, roadHeight);
                        if (ptrSysWithNoPole)
                        {
                            _lightsPoleSystemArray.emplace_back(ptrSysWithNoPole);
                        }

                    }

                }
            }
        }

    }

    HadPole* TrafficLightsCompiler::FindPole(LightsCollect lights, int roadHeight)
    {
        // 1. 灯箱中心点和平均方向
        point_t	lightsPoints;
        double	heading = 0;
        box_t	box = lights.front()->box;
        for (PtrTrafficLightsInfo p : lights)
        {
            bg::expand(box, p->box);
            heading += p->lights->heading;
        }
        bg::centroid(box, lightsPoints);
        heading /= lights.size();

        // 2.获取最近的杆子坐标
        bool bGetPole = false;
        MapPoint3D64 polePositon;


        HadPole* pole = getNeareastPole(PT_MT(lightsPoints), heading, box.min_corner().get<2>()/10);

        return pole;
        if (pole)
        {
            polePositon = pole->location.vertexes.front();
            polePositon.z = roadHeight;
            bGetPole = true;
        }
    }

    void TrafficLightsCompiler::spliteTShapePole(const LightsCollect& lights, HadPole* pole, std::vector<LightsCollect>& LightsCollectByPole) {

        // 对于T字型杆子,需要再次分类,根据在同一个方向判断
        struct lightsPoleDegree {
            PtrTrafficLightsInfo p;
            int degree;
        };

        std::vector<lightsPoleDegree> lightsPoleDegrees;
        LightsCollect lightsOnPole;	//在杆子上的pole
        MapPoint3D64 polePositon = pole->location.vertexes.front();

        for (PtrTrafficLightsInfo p : lights)
        {
            int distance = GrapPointAlgorithm::geoLengthD(polePositon.pos.toNdsPoint(), p->centerPoint.pos.toNdsPoint());
            if (distance < 0.3)
            {
                lightsOnPole.emplace_back(p);
            }
            else
            {
                auto a = Math_getDirectionFromTwoCoordinatesNds(polePositon.pos.toNdsPoint(), p->centerPoint.pos.toNdsPoint());
                lightsPoleDegrees.emplace_back(lightsPoleDegree{ p,toDegree(a) });
            }

        }
        if (!lightsPoleDegrees.empty())
        {
            //按同一个方向分类
            std::sort(lightsPoleDegrees.begin(), lightsPoleDegrees.end(), [&](lightsPoleDegree& a, lightsPoleDegree& b) -> bool
                { return a.degree < b.degree; });

            lightsPoleDegree pFirst = lightsPoleDegrees.front();
            LightsCollectByPole.emplace_back(LightsCollect{});
            LightsCollectByPole.back().emplace_back(pFirst.p);
            for (lightsPoleDegree pl : lightsPoleDegrees)
            {
                if (pFirst.p->lights == pl.p->lights)
                    continue;

                if (difference_of_degreess(pFirst.degree, pl.degree) < 30)
                {
                    LightsCollectByPole.back().emplace_back(pl.p);
                }
                else
                {
                    LightsCollectByPole.emplace_back(LightsCollect{});
                    LightsCollectByPole.back().emplace_back(pl.p);
                    pFirst = pl;
                }
            }
        }
        //加入杆子上的
        if (LightsCollectByPole.empty())
        {
            LightsCollectByPole.emplace_back(lightsOnPole);
        }
        else
        {
            LightsCollectByPole.front().insert(LightsCollectByPole.front().end(), lightsOnPole.begin(), lightsOnPole.end());
        }

    }
    PtrLightsPoleSystem TrafficLightsCompiler::createLightsSystem( LightsCollect& lights, int roadHeight, HadPole* pole)
    {

        // 灯箱中心点和接地杆点
        MapPoint3D64 polePositon = pole->location.vertexes.front();
        polePositon.z = roadHeight;
        point_t	centerBoxPoint;
        box_t	box = lights.front()->box;
        for (auto p : lights)
        {
            bg::expand(box, p->box);
        }
        bg::centroid(box, centerBoxPoint);


        // 灯箱排序,横杆距离大优先,竖杆上高度优先
        for (PtrTrafficLightsInfo p : lights)
        {
            float distanceToPole = GrapPointAlgorithm::geoLengthD(p->lights->location.vertexes.front().pos.toNdsPoint(),
                polePositon.pos.toNdsPoint());
            p->distanceToPole = distanceToPole;
           
        }

        std::sort(lights.begin(), lights.end(), [&](PtrTrafficLightsInfo& a, PtrTrafficLightsInfo& b) -> bool
            { 
                if (a->onBar() && b->onBar())
                {
                    return a->minHeight() > b->minHeight();
                }
                return a->distanceToPole > b->distanceToPole;
            });

      

        PtrLightsPoleSystem ptrTrafficLightsSystem = std::make_shared<LightsPoleSystem>();
        bool hasBar = false;
        for (PtrTrafficLightsInfo p : lights)
        {
            if (p->onBar())
                hasBar = true;

            ptrTrafficLightsSystem->lightsinfos.emplace_back(p);
        }
     
        ptrTrafficLightsSystem->lightsBoxCenterPoint = PT_MT(centerBoxPoint);
        ptrTrafficLightsSystem->polePositon = polePositon;
        ptrTrafficLightsSystem->roadHeight = roadHeight;
        ptrTrafficLightsSystem->maxLightsHeight = box.max_corner().get<2>() / 10;
        ptrTrafficLightsSystem->barHeading = toDegree(Math_getDirectionFromTwoCoordinatesNds(polePositon.pos.toNdsPoint(), PT_MT(centerBoxPoint).pos.toNdsPoint()));
        ptrTrafficLightsSystem->barLength = lights.front()->distanceToPole* 100;
        ptrTrafficLightsSystem->hasBar = hasBar;

        return ptrTrafficLightsSystem;
    }

    // 有红绿灯绑在同一个杆上,这时候做合并
    void TrafficLightsCompiler::MergePtrLightsPoleSystem(PtrLightsPoleSystem pole1, PtrLightsPoleSystem mergedPole)
    {

        std::vector<PtrTrafficLightsInfo> lights = pole1->lightsinfos;
        lights.insert(lights.end(), mergedPole->lightsinfos.begin(), mergedPole->lightsinfos.end());
        // 灯箱排序,横杆距离大优先,竖杆上高度优先
        for (PtrTrafficLightsInfo p : lights)
        {
            float distanceToPole = GrapPointAlgorithm::geoLengthD(p->lights->location.vertexes.front().pos.toNdsPoint(),
                pole1->polePositon.pos.toNdsPoint());
            p->distanceToPole = distanceToPole;

        }

        std::sort(lights.begin(), lights.end(), [&](PtrTrafficLightsInfo& a, PtrTrafficLightsInfo& b) -> bool
            {
                if (a->onBar() && b->onBar())
                {
                    return a->minHeight() > b->minHeight();
                }
                return a->distanceToPole > b->distanceToPole;
            });


        point_t	centerBoxPoint;
        box_t	box = lights.front()->box;
        for (auto p : lights)
        {
            bg::expand(box, p->box);
        }
        bg::centroid(box, centerBoxPoint);


        bool hasBar = false;
        pole1->lightsinfos.clear();
        for (PtrTrafficLightsInfo p : lights)
        {
            if (p->onBar())
                hasBar = true;

            pole1->lightsinfos.emplace_back(p);
        }

        pole1->lightsBoxCenterPoint = PT_MT(centerBoxPoint);
        pole1->maxLightsHeight = box.max_corner().get<2>() / 10;
        pole1->barHeading = toDegree(Math_getDirectionFromTwoCoordinatesNds(pole1->polePositon.pos.toNdsPoint(), PT_MT(centerBoxPoint).pos.toNdsPoint()));
        pole1->barLength = lights.front()->distanceToPole * 100;
        pole1->hasBar = hasBar;

    }
    PtrLightsPoleSystem TrafficLightsCompiler::createLightsSystemNotFindPole(LightsCollect lights, int roadHeight)
    {

        // 1.计算灯箱中心点和平均方向
        point_t centerBoxPoint;
        MapPoint3D64 lightMapPt;
        double heading = 0;
        std::vector<MapPoint3D64> pts;
        box_t box = lights.front()->box;
        box_t b1;
        for (PtrTrafficLightsInfo p : lights)
        {
            bg::expand(box, p->box);
            heading += p->lights->heading;
        }

        bg::centroid(box, centerBoxPoint);
        heading /= lights.size();

        // 中心点作为杆子
        MapPoint3D64 polePositon;
  
        // 不在车道内
        HadLaneGroup* lg = IsPointOnHadLaneGroup(PT_MT(centerBoxPoint));
        if (lg && !lg->inIntersection)
        {
            return nullptr;
        }

        //是否距离过远
        double distanceToLaneGroup = UINT32_MAX;
        getNeareastLaneGroup(PT_MT(centerBoxPoint), distanceToLaneGroup);
        if (distanceToLaneGroup > 8)
        {
            return nullptr;
        }


        polePositon = PT_MT(centerBoxPoint);
        polePositon.z = roadHeight;
        // 3.创建信号灯集合
     
        PtrLightsPoleSystem ptrTrafficLightsSystem = std::make_shared<LightsPoleSystem>();
        for (PtrTrafficLightsInfo p : lights)
        {
            ptrTrafficLightsSystem->lightsinfos.emplace_back(p);
        }

        ptrTrafficLightsSystem->lightsBoxCenterPoint = PT_MT(centerBoxPoint);
        ptrTrafficLightsSystem->polePositon = polePositon;
        ptrTrafficLightsSystem->roadHeight = roadHeight;
        ptrTrafficLightsSystem->maxLightsHeight = box.max_corner().get<2>() / 10;
        ptrTrafficLightsSystem->barHeading = toDegree(Math_getDirectionFromTwoCoordinatesNds(polePositon.pos.toNdsPoint(), PT_MT(centerBoxPoint).pos.toNdsPoint()));
        ptrTrafficLightsSystem->barLength = 0;
        ptrTrafficLightsSystem->hasBar = false;


        return ptrTrafficLightsSystem;
    }

    void TrafficLightsCompiler::createRDS()
    {
   
        for (PtrLightsPoleSystem lightsSys : _lightsPoleSystemArray)
        {
            RdsTrafficLight* pLight = (RdsTrafficLight*)createObject(_pTile, EntityType::RDS_TRAFFICLIGHT);

         
            for (PtrTrafficLightsInfo lightsInfo : lightsSys->lightsinfos)
            {

                RdsTrafficLight::LightBox rdsLightBox;
                rdsLightBox.bulbsCount = lightsInfo->lights->columnNumber;
                rdsLightBox.heading = north_degree_to_east_degree(lightsInfo->lights->heading) * MATH_DEGREE2RADIAN * 1e5;
                rdsLightBox.installLocation = lightsInfo->onBar() ? RdsTrafficLight::LightBox::InstallLocation::BAR : RdsTrafficLight::LightBox::InstallLocation::POLE;
                rdsLightBox.orientation = lightsInfo->isVertical() ? RdsTrafficLight::LightBox::Orientation::VERTICAL : RdsTrafficLight::LightBox::Orientation::HORIZONTAL;
                rdsLightBox.type = (RdsTrafficLight::LightBox::Type)lightsInfo->lights->type;

                pLight->lightBoxs.emplace_back(rdsLightBox);

            }

            pLight->type = lightsSys->hasBar ? RdsTrafficLight::PoleType::BAR : RdsTrafficLight::PoleType::POLE;
            pLight->barHeading = lightsSys->barHeading * MATH_DEGREE2RADIAN * 1e5;
            pLight->maxBarLenght = lightsSys->barLength;

            if (lightsSys->hasBar)
            {
                pLight->maxHeight = lightsSys->maxLightsHeight - lightsSys->roadHeight;
            }
            else
            {
                pLight->maxHeight = 770;
            }
            lightsSys->polePositon.z += POLE_HEIGHT_ADJUSTMENT;
            convert(lightsSys->polePositon, pLight->position);
        }

    }

    void TrafficLightsCompiler::clustering(const std::vector<PtrTrafficLightsInfo>& input, std::vector<LightsCollect>& output,
        int distance, int diff_degree)
    {
        /*   Box2TRTree::Ptr rtreeBox;
           std::unordered_map<PtrTrafficLightsInfo, std::set<PtrTrafficLightsInfo>> sameGroupTrafficLight;*/

      /*  auto getNearestLights = [&](PtrTrafficLightsInfo l) -> PtrTrafficLightsInfo
        {
            PtrTrafficLightsInfo nearLight = nullptr;

            rtreeBox->query(bgi::nearest(POINT_2T(l->centerPoint), 1) && bgi::satisfies([&](size_t const& id)
                { return input[id]->lights != l->lights; }),
                boost::make_function_output_iterator([&](size_t const& id)
                    { nearLight = input[id]; }));
            return nearLight;
        };*/
        // 两个灯箱是否符合在同一条线,距离间隔
        auto lightsIsOnline = [&](PtrTrafficLightsInfo p1, PtrTrafficLightsInfo p2) -> bool
        {
            // double degree_t = toDegree(Math_getDirectionFromTwoCoordinatesNds(grappedPt.pos.toNdsPoint(), centerMapPoint.pos.toNdsPoint()));
            double degree_t = bg_degree(POINT_T(p1->centerPoint), POINT_T(p2->centerPoint));
            double heading_north = north_degree_to_east_degree(p1->lights->heading);
            // 越接近90度最好

            double diff_degree_t = cq_abs(difference_of_degreess(degree_t, heading_north) - 90);
            if (diff_degree_t > diff_degree)
            {
                return false;
            }

            // 距离
            double lenght = GrapPointAlgorithm::geoLengthD(p1->centerPoint.pos.toNdsPoint(), p2->centerPoint.pos.toNdsPoint());
            if (lenght > distance)
            {
                return false;
            }
            return true;


        };
        int groupId = 1;
        auto cluster = [&](const std::vector<PtrTrafficLightsInfo>& collection, std::vector<PtrTrafficLightsInfo>& others)
        {
            if (collection.empty())
            {
                return;
            }
            PtrTrafficLightsInfo pFirst = collection.front();
           
            output.emplace_back(LightsCollect{});
            output.back().emplace_back(pFirst);
            for (PtrTrafficLightsInfo pl : collection)
            {
                if (pFirst->lights == pl->lights || pl->groupId)
                    continue;

                if (lightsIsOnline(pFirst, pl))
                {
                    pl->groupId = pFirst->groupId;
                    output.back().emplace_back(pl);
                }
                else
                {
                    others.emplace_back(pl);
                }
            }
        };

        /*    std::vector<box_2t> boxs;
            Box2TRTree::guardParam param;
            Box2TRTree::indexGetterBox originIndBox(boxs);
            rtreeBox = std::make_shared<Box2TRTree::RTree>(boost::irange<std::size_t>(0lu, boxs.size()), param, originIndBox);
           */
        for (int i = 0;i < input.size(); i++)
        {
            if (!input[i]->groupId)
            {
                input[i]->groupId = groupId++;
            }

            for (int j = 1; j <input.size() ;j++)
            {
               if (!input[j]->groupId && lightsIsOnline(input[i], input[j]))
               {
                   input[j]->groupId = input[i]->groupId;
               }
            }
        }

        for (int id = 1; id < groupId; id++)
        {
            output.emplace_back(LightsCollect{});
            for (auto p : input)
            {
                if (p->groupId == id)
                {
                    output.back().emplace_back(p);
                }
            }

        }


        
      /*  std::vector<PtrTrafficLightsInfo> inputs(input);
        do
        { 
            std::vector<PtrTrafficLightsInfo> others;
            cluster(inputs, others);
            inputs = others;
            if (inputs.empty())
            {
                break;
            }

        } while (true);*/
    }
    /*bool TrafficLightsCompiler::isOnBar(std::vector<PtrTrafficLightsInfo>& lights, int roadHeight)
    {
        bool bIsOnBar = false;
        for (PtrTrafficLightsInfo ptrLight : lights)
        {
            if (ptrLight->isOnbar(roadHeight))
            {
                bIsOnBar = true;
            }
        }
        return bIsOnBar;
    }*/
    HadLaneGroup* TrafficLightsCompiler::getNeareastLaneGroup(MapPoint3D64 centerPt, double heading, MapPoint3D64& roadBoundaryPoint)
    {

        HadLaneGroup* pLaneGroup = nullptr;
        point_t closestRoadBoundaryPoint;
        bool bOnLaneGroup = true;
        box_t lightBox;
        point_t centerPoint = POINT_T(centerPt);
        bg::envelope(centerPoint, lightBox);

        bool bBelongLanGroup = false;
        uint32 differ_degree = UINT32_MAX;
        int distance = UINT32_MAX;

        getBox2dRTree(_laneGroupTree)->query(bgi::intersects(B3_B2(lightBox)), boost::make_function_output_iterator([&](size_t const& id)
            {
                HadLaneGroup* pLGNear = _laneGroupTree.elements[id];

                MapPoint3D64* pPolygon = NULL;
                size_t nPolygonPtNum = 0;

                if (!pLGNear->width)
                {
                    return;
                }

                std::vector<MapPoint3D64> pts1(pPolygon, pPolygon + nPolygonPtNum);
                polygon_t p = POLYGON_T(pts1);

                if ((nPolygonPtNum = getRightBoundary(pLGNear, pPolygon)) == 0)
                {
                    return;
                }
                std::vector<MapPoint3D64> pts(pPolygon, pPolygon + nPolygonPtNum);
                linestring_t laneGroupRightBoundary = LINESTRING_T(pts);


                segment_t seg_t;
                bg::closest_points(centerPoint, laneGroupRightBoundary, seg_t);
                point_t nearestBoudaryPoint = seg_t.second;
                // point_t nearestBoudaryPoint = POINT_T(grappedPt);

                bool bValid = true;
                // 高度,判断上下桥
                int diffHeight = centerPoint.get<2>() - nearestBoudaryPoint.get<2>();
                // 大于3米,小于10米
                if (!(diffHeight > 3000 && diffHeight < 10000))
                {
                    return;
                }

                // 在路中间

                MapPoint3D64 mapPt1 = MapPoint3D64_make(centerPoint.get<0>(), centerPoint.get<1>(), centerPoint.get<2>());
                MapPoint3D64 mapPt2 = MapPoint3D64_make(nearestBoudaryPoint.get<0>(), nearestBoudaryPoint.get<1>(), nearestBoudaryPoint.get<2>());
                double distance = GrapPointAlgorithm::geoLengthD(mapPt1.pos.toNdsPoint(), mapPt2.pos.toNdsPoint());
                if (distance * 100 > pLGNear->width)
                {
                    // return;
                }

                // double degree_t = toDegree(Math_getDirectionFromTwoCoordinatesNds(grappedPt.pos.toNdsPoint(), centerMapPoint.pos.toNdsPoint()));
                double degree_t = bg_degree(centerPoint, nearestBoudaryPoint);
                double heading_t = north_degree_to_east_degree(heading);
                // 预测角度,灯的角度减去90度.在灯的方向左侧
                double pri_degeree = degree_add(heading_t, 90);

                double diff_degree_t = difference_of_degreess(degree_t, pri_degeree);
                if (diff_degree_t < differ_degree)
                {
                    differ_degree = diff_degree_t;
                    pLaneGroup = pLGNear;
                    closestRoadBoundaryPoint = nearestBoudaryPoint;
                } }));

        if (!pLaneGroup || differ_degree > 20)
        {
            return nullptr;
        }
        roadBoundaryPoint = MapPoint3D64_make(closestRoadBoundaryPoint.get<0>(), closestRoadBoundaryPoint.get<1>(), closestRoadBoundaryPoint.get<2>() / 10);

        return pLaneGroup;
    }

    HadLaneGroup* TrafficLightsCompiler::getNeareastLaneGroup(MapPoint3D64 centerPt,double& distance)
    {

        HadLaneGroup* pLaneGroup = nullptr;
        uint32 differ_degree = UINT32_MAX;
        distance = UINT32_MAX;

        getBox2dRTree(_laneGroupTree)->query(bgi::nearest(POINT_2T(centerPt),3), boost::make_function_output_iterator([&](size_t const& id)
            {
                HadLaneGroup* pLGNear = _laneGroupTree.elements[id];

                MapPoint3D64* pPolygon = NULL;
                size_t nPolygonPtNum = 0;

                if (!pLGNear->width)
                {
                    return;
                }

                std::vector<MapPoint3D64> pts1(pPolygon, pPolygon + nPolygonPtNum);
                polygon_t p = POLYGON_T(pts1);

                if ((nPolygonPtNum = makeLaneGroupPolygon(pLGNear, pPolygon)) == 0)
                {
                    return;
                }
                std::vector<MapPoint3D64> pts(pPolygon, pPolygon + nPolygonPtNum);
                linestring_t laneGroupRightBoundary = LINESTRING_T(pts);


                segment_t seg_t;
                bg::closest_points(POINT_T(centerPt), laneGroupRightBoundary, seg_t);
                point_t nearestBoudaryPoint = seg_t.second;
              
                bool bValid = true;
                // 高度,判断上下桥
                int diffHeight = POINT_T(centerPt).get<2>() - nearestBoudaryPoint.get<2>();
                // 大于3米,小于10米
                if (!(diffHeight > 3000 && diffHeight < 10000))
                {
                    return;
                }

                double dis = GrapPointAlgorithm::geoLengthD(centerPt.pos.toNdsPoint(), PT_MT(nearestBoudaryPoint).pos.toNdsPoint());
                if (dis < distance)
                {
                    distance = dis;
                    pLaneGroup = pLGNear;
                }
                }));

        
        return pLaneGroup;
    }




    bool TrafficLightsCompiler::getRoadHeight(MapPoint3D64 centerPt, double& height)
    {

        HadLaneGroup* pLaneGroup = nullptr;

        getBoxRTree(_laneGroupTree)->query(bgi::nearest(POINT_T(centerPt), 3),
            boost::make_function_output_iterator([&](size_t const& id)
                {
                    if (pLaneGroup)
                    {
                        return;
                    }
                    HadLaneGroup* pLGNear = _laneGroupTree.elements[id];

                    MapPoint3D64* pPolygon = NULL;
                    size_t nPolygonPtNum = 0;

                    if (!pLGNear->width)
                    {
                        return;
                    }

                    std::vector<MapPoint3D64> pts1(pPolygon, pPolygon + nPolygonPtNum);
                    polygon_t p = POLYGON_T(pts1);


                    if ((nPolygonPtNum = getRightBoundary(pLGNear, pPolygon)) == 0)
                    {
                        return;
                    }
                    std::vector<MapPoint3D64> pts(pPolygon, pPolygon + nPolygonPtNum);
                    linestring_t laneGroupRightBoundary = LINESTRING_T(pts);


                    segment_t seg_t;
                    bg::closest_points(POINT_T(centerPt), laneGroupRightBoundary, seg_t);
                    point_t nearestBoudaryPoint = seg_t.second;
                    // point_t nearestBoudaryPoint = POINT_T(grappedPt);

                    bool bValid = true;
                    // 高度,判断上下桥
                    int diffHeight = POINT_T(centerPt).get<2>() - nearestBoudaryPoint.get<2>();
                    // 大于3米,小于10米
                    if (!(diffHeight > 3000 && diffHeight < 10000))
                    {
                        return;
                    }
                    height = nearestBoudaryPoint.get<2>() / 10;
                    pLaneGroup = pLGNear;
                }));

        if (!pLaneGroup)
        {
            return false;
        }

        return true;
    }


    HadPole* TrafficLightsCompiler::getNeareastPole(MapPoint3D64 centerPoint, double heading, double mixZ)
    {
        struct PoleInfo
        {
            HadPole* pole;
            double distance;
            double differ_degree;
        };

        HadPole* pPoleSelect = nullptr;
        double distance = UINT32_MAX;
        uint32 differ_degree = UINT32_MAX;
        std::vector<PoleInfo> poles;
        getBox2dRTree(_polesTree)->query(bgi::nearest(POINT_2T(centerPoint), 5),
            boost::make_function_output_iterator([&](size_t const& id)
                {
                    HadPole* pole = _polesTree.elements[id];

                    auto pMix = std::min_element(pole->location.vertexes.begin(), pole->location.vertexes.end(),
                        [&](MapPoint3D64& p1, MapPoint3D64& p2)
                        {
                            return p1.z < p2.z;
                        }

                    );
                    auto pMax = std::max_element(pole->location.vertexes.begin(), pole->location.vertexes.end(),
                        [&](MapPoint3D64& p1, MapPoint3D64& p2)
                        {
                            return p1.z < p2.z;
                        }

                    );
                    //杆子不能比灯箱低
                    MapPoint3D64 polePtMax = *pMax;
                    if (polePtMax.z < mixZ)
                    {
                        return;
                    }

                    MapPoint3D64 polePt = *pMix;

                    // 高度,判断上下桥
                    int diffHeight = centerPoint.z - polePt.z;
                    // 大于3米,小于10米
                    if (!(diffHeight > 300 && diffHeight < 1000))
                    {
                        return;
                    }

                    // 距离
                    double distance = GrapPointAlgorithm::geoLengthD(polePt.pos.toNdsPoint(), centerPoint.pos.toNdsPoint());
                    if (distance > 20)
                    {
                        return;
                    }
                    // 距离小于0.5米认为是竖杆.
                    if (distance < 0.5)
                    {
                        poles.emplace_back(PoleInfo{ pole, distance, 0 });
                        /*     printInfo("lights find Pole distance < 0.5 [mesh:%I64d],[location::(%d,%d)]", _pGrid->getId(),
                                 polePt.pos.lon / 1000, polePt.pos.lat / 1000);*/

                        return;
                    }
                    // double degree_t = toDegree(Math_getDirectionFromTwoCoordinatesNds(grappedPt.pos.toNdsPoint(), centerMapPoint.pos.toNdsPoint()));
                    double degree_t = bg_degree(POINT_T(centerPoint), POINT_T(polePt));
                    double heading_north = north_degree_to_east_degree(heading);
                    // 越接近90度最好

                    double diff_degree_t = cq_abs(difference_of_degreess(degree_t, heading_north) - 90);
                    if (diff_degree_t < 25 || (distance < 0.8 && diff_degree_t <30))
                    {
                        poles.emplace_back(PoleInfo{ pole, distance, diff_degree_t });
                    } }));
       
        if (poles.empty())
        {
            return nullptr;
        }
        auto p = std::min_element(poles.begin(), poles.end(),
            [&](PoleInfo& p1, PoleInfo& p2)
            {
                return p1.distance < p2.distance;
            }

        );

        return (*p).pole;
    }


  /*  bool TrafficLightsCompiler::isOnBar(std::vector<PtrTrafficLightsInfo>& lights, int roadHeight)
    {
        bool bIsOnBar = false;
        for (PtrTrafficLightsInfo ptrLight : lights)
        {
            if (ptrLight->isOnbar(roadHeight))
            {
                bIsOnBar = true;
            }
        }
        return bIsOnBar;
    }*/

    HadLaneGroup* TrafficLightsCompiler::IsPointOnHadLaneGroup(MapPoint3D64 pt)
    {
        
        HadLaneGroup* pLaneGroup = nullptr;
        point_t closestRoadBoundaryPoint;
        bool bOnLaneGroup = true;
        box_t lightBox;
        point_t centerPoint = POINT_T(pt);
        bg::envelope(centerPoint, lightBox);

        bool bBelongLanGroup = false;
        uint32 differ_degree = UINT32_MAX;
        int distance = UINT32_MAX;

        getBox2dRTree(_laneGroupTree)->query(bgi::intersects(B3_B2(lightBox)), boost::make_function_output_iterator([&](size_t const& id)
            {

                if (pLaneGroup)
                {
                    return;
                }
                HadLaneGroup* pLGNear = _laneGroupTree.elements[id];

                MapPoint3D64* pPolygon = NULL;
                size_t nPolygonPtNum = 0;

                if (!pLGNear->width)
                {
                    return;
                }

                if ((nPolygonPtNum = makeLaneGroupPolygon(pLGNear, pPolygon)) == 0)
                {
                    return;
                }
                std::vector<MapPoint3D64> pts1(pPolygon, pPolygon + nPolygonPtNum);
                polygon_t p = POLYGON_T(pts1);

                if (!bg::intersects(p, centerPoint))
                {
                    return;
                }

                std::vector<MapPoint3D64> pts(pPolygon, pPolygon + nPolygonPtNum);
                linestring_t laneGroupRightBoundary = LINESTRING_T(pts);

                /*	MapPoint3D64 centerMapPoint = MapPoint3D64_make(centerPoint.get<0>(), centerPoint.get<1>(), centerPoint.get<2>() / 10);
                    MapPoint3D64 grappedPt, nearestPt;
                    size_t si, ei;
                    if (!GrapPointAlgorithm::grapOrMatchNearestPoint(centerMapPoint, pts, grappedPt, nearestPt, si, ei))
                    {
                        return;
                    }*/

                segment_t seg_t;
                bg::closest_points(centerPoint, laneGroupRightBoundary, seg_t);
                point_t nearestBoudaryPoint = seg_t.second;
                // point_t nearestBoudaryPoint = POINT_T(grappedPt);

                bool bValid = true;
                // 高度,判断上下桥 
                int diffHeight = centerPoint.get<2>() - nearestBoudaryPoint.get<2>();
                // 大于3米,小于10米
                if (!(diffHeight > 3000 && diffHeight < 10000))
                {
                    return;
                }

                pLaneGroup = pLGNear;

            }));

        return pLaneGroup;

    }


    template <typename element>
    BoxRTree::Ptr TrafficLightsCompiler::getBoxRTree(EelementBoxTree<element*>& tree)
    {
        auto& tmp = tree.rtreeBox;
        if (tmp)
            return tmp;
        BoxRTree::guardParam param;
        BoxRTree::indexGetterBox originIndBox(tree.boxs);
        auto rtreeBox = std::make_shared<BoxRTree::RTree>(boost::irange<std::size_t>(0lu, tree.boxs.size()), param, originIndBox);
        tree.rtreeBox = rtreeBox;
        return rtreeBox;
    }

    template <typename element>
    Box2TRTree::Ptr TrafficLightsCompiler::getBox2dRTree(EelementBoxTree<element*>& tree)
    {
        auto& tmp = tree.rtree2dBox;
        if (tmp)
            return tmp;
        Box2TRTree::guardParam param;
        Box2TRTree::indexGetterBox originIndBox(tree.boxs2T);
        auto rtreeBox = std::make_shared<Box2TRTree::RTree>(boost::irange<std::size_t>(0lu, tree.boxs2T.size()), param, originIndBox);
        tree.rtree2dBox = rtreeBox;
        return rtreeBox;
    }

    void TrafficLightsCompiler::createLightsBoxRtree()
    {

        std::vector<HadGrid*> grids;
        grids = _nearby;
        grids.emplace_back(_pGrid);
        for (auto grid : grids)
        {
            for (auto obj : grid->query(ElementType::HAD_OBJECT_TRAFFIC_LIGHTS))
            {
                HadTrafficLights* pl = (HadTrafficLights*)obj;
                _lightsTree.elements.emplace_back(pl);
                _lightsTree.boxs.emplace_back(BOX_T(pl->location.vertexes));
            }
        }
    }

    void TrafficLightsCompiler::createPoleRtree()
    {

        std::vector<HadGrid*> grids;
        grids = _nearby;
        grids.emplace_back(_pGrid);
        for (auto grid : grids)
        {
            for (auto obj : grid->query(ElementType::HAD_OBJECT_POLE))
            {
                HadPole* pl = (HadPole*)obj;
                box_t box = BOX_T(pl->location.vertexes);
                _polesTree.elements.emplace_back(pl);
                _polesTree.boxs.emplace_back(BOX_T(pl->location.vertexes));
                _polesTree.boxs2T.emplace_back(BOX_2T(pl->location.vertexes));
            }
        }
    }
    void TrafficLightsCompiler::createLaneGroupRtree()
    {

        std::vector<HadGrid*> grids;
        grids = _nearby;
        grids.emplace_back(_pGrid);
        for (auto grid : grids)
        {
            for (auto obj : grid->query(ElementType::HAD_LANE_GROUP))
            {
                HadLaneGroup* pl = (HadLaneGroup*)obj;
                std::vector<MapPoint3D64> v;
                v.emplace_back(pl->extent.max);
                v.emplace_back(pl->extent.min);
                box_t b3 = BOX_T(v);
                _laneGroupTree.elements.emplace_back(pl);
                _laneGroupTree.boxs.emplace_back(b3);
                _laneGroupTree.boxs2T.emplace_back(B3_B2(b3));
            }
        }
    }

    // 获取道路最右边的边界线
    size_t TrafficLightsCompiler::getRightBoundary(HadLaneGroup* pGroup, MapPoint3D64*& polygon)
    {
        if (pGroup->roadBoundaries.size() >= 2)
        {

            size_t roadBoundaryNum = pGroup->roadBoundaries.size();
            size_t nPolyline1PtNum = pGroup->roadBoundaries[roadBoundaryNum - 1]->location.vertexes.size();

            // 添加路面右边线几何点。
            std::vector<MapPoint3D64> Line1Reverse(nPolyline1PtNum);
            if (directionEqual(pGroup->roadBoundaries[roadBoundaryNum - 1], pGroup, 3))
                std::reverse_copy(pGroup->roadBoundaries[roadBoundaryNum - 1]->location.vertexes.begin(), pGroup->roadBoundaries[roadBoundaryNum - 1]->location.vertexes.end(), Line1Reverse.begin());
            else
            {
                std::copy(pGroup->roadBoundaries[roadBoundaryNum - 1]->location.vertexes.begin(), pGroup->roadBoundaries[roadBoundaryNum - 1]->location.vertexes.end(), Line1Reverse.begin());
            }

            size_t nPtNum = Line1Reverse.size();
            polygon = new MapPoint3D64[nPtNum];
            memcpy(polygon, Line1Reverse.data(), Line1Reverse.size() * sizeof(MapPoint3D64));
            return nPtNum;
        }
        if (pGroup->laneBoundaries.size() >= 2)
        {

            size_t laneBoundaryNum = pGroup->laneBoundaries.size();

            size_t nPolyline1PtNum = pGroup->laneBoundaries[laneBoundaryNum - 1]->location.vertexes.size();

            // 添加路面右边线几何点。
            std::vector<MapPoint3D64> Line1Reverse(nPolyline1PtNum);
            if (directionEqual(pGroup->laneBoundaries[laneBoundaryNum - 1], pGroup, 3))
                std::reverse_copy(pGroup->laneBoundaries[laneBoundaryNum - 1]->location.vertexes.begin(), pGroup->laneBoundaries[laneBoundaryNum - 1]->location.vertexes.end(), Line1Reverse.begin());
            else
            {
                std::copy(pGroup->laneBoundaries[laneBoundaryNum - 1]->location.vertexes.begin(), pGroup->laneBoundaries[laneBoundaryNum - 1]->location.vertexes.end(), Line1Reverse.begin());
            }

            size_t nPtNum = Line1Reverse.size();
            polygon = new MapPoint3D64[nPtNum];
            memcpy(polygon, Line1Reverse.data(), Line1Reverse.size() * sizeof(MapPoint3D64));
            return nPtNum;
        }
        return 0;
    }

    PtrTrafficLightsInfo TrafficLightsCompiler::getLightsExtendInfo(HadTrafficLights* lights)
    {

        point_t centerPoint;
        bool bOnLaneGroup = true;
        box_t lightBox = BOX_T(lights->location.vertexes);
        bg::centroid(lightBox, centerPoint);

        // 宽度
        MapPoint3D64 pMin = PT_MT(lightBox.min_corner());
        MapPoint3D64 pMax = PT_MT(lightBox.max_corner());

        PtrTrafficLightsInfo pTL = std::make_shared<TrafficLightsInfo>();
        pTL->lights = lights;
        pTL->centerPoint = PT_MT(centerPoint);
        pTL->box = lightBox;
        pTL->width = GrapPointAlgorithm::geoLengthD(pMin.pos.toNdsPoint(), pMax.pos.toNdsPoint()) * 100;
        pTL->orientation = lights->orientation;
        return pTL;
    }

}