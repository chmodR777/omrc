#pragma once
#include "Compiler.h"
namespace OMDB
{

    struct TrafficLightsInfo {

        HadTrafficLights* lights;

        //�������ĵ�
        MapPoint3D64 centerPoint;

        //box
        box_t box;

        // ���� 1 ��ֱ; 2 ˮƽ
        uint8 orientation;

        //����ˮƽ��� cm
        int width;

        double distanceToPole{ 0 };

        bool onBar() { return distanceToPole > 0.8; }

        //�Ƿ�ֱ
        bool isVertical() { return orientation == 1; }
       
        //��͸߶�cm
        int minHeight() { return box.min_corner().get<2>() / 10;}

        //��߸߶�
        int maxHeight() { return box.max_corner().get<2>() / 10;}

        int groupId{0};
    };

    using PtrTrafficLightsInfo = std::shared_ptr<TrafficLightsInfo>;

    // �źŵ�ϵͳ
    struct LightsPoleSystem {

        std::vector<PtrTrafficLightsInfo> lightsinfos;

        //�������ĵ�
        MapPoint3D64 lightsBoxCenterPoint;

        //�˵�����
        MapPoint3D64 polePositon;

        //���ӷ��� ��Ϊ�� [0,360)
        uint32 barHeading;

        //���ӳ���cm
        uint32 barLength;

        //·��߶� cm
        uint32 roadHeight;

        //�������߶�
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

        // ���Ҹ����ĵƸ�
        HadPole* FindPole(LightsCollect lights, int roadHeight);
        HadPole* getNeareastPole(MapPoint3D64 centerPoint, double heading,double mixZ);

        bool isOnBar(std::vector<PtrTrafficLightsInfo>& lights, int roadHeight);
        // T�� �Ƹ˷ָ����
        void spliteTShapePole(const LightsCollect& lights, HadPole* pole, std::vector<LightsCollect>& splitOut);

        // ����
        void clustering(const std::vector<PtrTrafficLightsInfo>& in, std::vector<LightsCollect>& out, int distance = 10, int diff_degree = 20);
        

        PtrTrafficLightsInfo getLightsExtendInfo(HadTrafficLights* lights);

        HadLaneGroup* getNeareastLaneGroup(MapPoint3D64 centerPoint, double heading, MapPoint3D64& roadBoundaryPoint);
        HadLaneGroup* getNeareastLaneGroup(MapPoint3D64 centerPt, double& distance);
        //Ѱ�ҵ�����·��߶�
        bool getRoadHeight(MapPoint3D64 centerPt, double& height);
        HadLaneGroup* IsPointOnHadLaneGroup(MapPoint3D64 pt);
        template <typename element> BoxRTree::Ptr getBoxRTree(EelementBoxTree<element*>& tree);
        template <typename element> Box2TRTree::Ptr getBox2dRTree(EelementBoxTree<element*>& tree);
        size_t getRightBoundary(HadLaneGroup* pGroup, MapPoint3D64*& polygon);
    private:

        EelementBoxTree<HadTrafficLights*> _lightsTree;
        EelementBoxTree<HadPole*> _polesTree;
        EelementBoxTree<HadLaneGroup*> _laneGroupTree;

        //�Ƶĳ�����Ϣ
        std::unordered_map<int64, PtrTrafficLightsInfo> _lightsExtendInfo;
        // �źŵ�ϵͳ����.һ��ϵͳ��һ���źŵ����.
        std::vector<PtrLightsPoleSystem> _lightsPoleSystemArray;

        HadGrid* _pGrid;
        std::vector<HadGrid*> _nearby;
        RdsTile* _pTile;

    };
}

