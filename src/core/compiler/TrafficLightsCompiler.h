#pragma once
#include "Compiler.h"
namespace OMDB
{

    struct TrafficLightsInfo {

        HadTrafficLights* lights;

        //几何中心点
        MapPoint3D64 centerPoint;

        //box
        box_t box;

        // 方向 1 垂直; 2 水平
        uint8 orientation;

        //灯箱水平宽度 cm
        int width;

        double distanceToPole{ 0 };

        bool onBar() { return distanceToPole > 0.8; }

        //是否垂直
        bool isVertical() { return orientation == 1; }
       
        //最低高度cm
        int minHeight() { return box.min_corner().get<2>() / 10;}

        //最高高度
        int maxHeight() { return box.max_corner().get<2>() / 10;}

        int groupId{0};
    };

    using PtrTrafficLightsInfo = std::shared_ptr<TrafficLightsInfo>;

    // 信号灯系统
    struct LightsPoleSystem {

        std::vector<PtrTrafficLightsInfo> lightsinfos;

        //灯箱中心点
        MapPoint3D64 lightsBoxCenterPoint;

        //杆的坐标
        MapPoint3D64 polePositon;

        //杆子方向 东为正 [0,360)
        uint32 barHeading;

        //杆子长度cm
        uint32 barLength;

        //路面高度 cm
        uint32 roadHeight;

        //灯箱最大高度
        uint32 maxLightsHeight;

        bool hasBar{ false };

    };
    
    using PtrLightsPoleSystem = std::shared_ptr<LightsPoleSystem>;
    typedef  std::vector<PtrTrafficLightsInfo> LightsCollect;

    template <typename element>
    struct EelementBoxTree {
        std::vector<element> elements;
        std::vector<box_t> boxs;
        std::vector<box_2t> boxs2T;
        BoxRTree::Ptr rtreeBox{ nullptr };
        Box2TRTree::Ptr rtree2dBox{ nullptr };
    };

    class TrafficLightsCompiler : public Compiler
    {
    public:
        TrafficLightsCompiler(CompilerData& data) :Compiler(data) {};
    protected:
        virtual void compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile) override;

        // Rtree
        void createLightsBoxRtree();
        void createPoleRtree();
        void createLaneGroupRtree();

        // process
        void collectLightsSystem();

        // output
        void createRDS();
    private:
        PtrLightsPoleSystem createLightsSystem( LightsCollect& lights, int roadHeight, HadPole* pole);
        void MergePtrLightsPoleSystem(PtrLightsPoleSystem pole1, PtrLightsPoleSystem mergedPole);
        PtrLightsPoleSystem createLightsSystemNotFindPole(LightsCollect lights, int roadHeight);

        // 查找附近的灯杆
        HadPole* FindPole(LightsCollect lights, int roadHeight);
        HadPole* getNeareastPole(MapPoint3D64 centerPoint, double heading,double mixZ);

        bool isOnBar(std::vector<PtrTrafficLightsInfo>& lights, int roadHeight);
        // T型 灯杆分割灯箱
        void spliteTShapePole(const LightsCollect& lights, HadPole* pole, std::vector<LightsCollect>& splitOut);

        // 聚类
        void clustering(const std::vector<PtrTrafficLightsInfo>& in, std::vector<LightsCollect>& out, int distance = 10, int diff_degree = 20);
        

        PtrTrafficLightsInfo getLightsExtendInfo(HadTrafficLights* lights);

        HadLaneGroup* getNeareastLaneGroup(MapPoint3D64 centerPoint, double heading, MapPoint3D64& roadBoundaryPoint);
        HadLaneGroup* getNeareastLaneGroup(MapPoint3D64 centerPt, double& distance);
        //寻找点所在路面高度
        bool getRoadHeight(MapPoint3D64 centerPt, double& height);
        HadLaneGroup* IsPointOnHadLaneGroup(MapPoint3D64 pt);
        template <typename element> BoxRTree::Ptr getBoxRTree(EelementBoxTree<element*>& tree);
        template <typename element> Box2TRTree::Ptr getBox2dRTree(EelementBoxTree<element*>& tree);
        size_t getRightBoundary(HadLaneGroup* pGroup, MapPoint3D64*& polygon);
    private:

        EelementBoxTree<HadTrafficLights*> _lightsTree;
        EelementBoxTree<HadPole*> _polesTree;
        EelementBoxTree<HadLaneGroup*> _laneGroupTree;

        //灯的车道信息
        std::unordered_map<int64, PtrTrafficLightsInfo> _lightsExtendInfo;
        // 信号灯系统集合.一个系统是一组信号灯组成.
        std::vector<PtrLightsPoleSystem> _lightsPoleSystemArray;

        HadGrid* _pGrid;
        std::vector<HadGrid*> _nearby;
        RdsTile* _pTile;

    };
}

