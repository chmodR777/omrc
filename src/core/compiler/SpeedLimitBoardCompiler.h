#pragma once
#include "Compiler.h"
#include "clipper.hpp"

namespace OMDB
{
    class SpeedLimitBoardCompiler : public Compiler
    {
	public:
        SpeedLimitBoardCompiler(CompilerData& data) :Compiler(data) {};
    protected:
        //交通看板的杆子。（1个杆子存在1个或多个牌子）
        struct Pole
        {
            RdsGroup* rdsGroup;
            RdsSpeedLimitBoard rdsSLB;
            MapPoint3D64 boardsCenterPt;
            std::vector<HadTrafficSign*> vctTS;
        };

    protected:
        virtual void compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile) override;

        bool existPole(const MapPoint3D64& grappedPt, Pole*& pExistPole);

        bool existNearbyTunnel(HadTrafficSign* pTrafficSign, HadLaneGroup* pLinkGroup);

        bool existTunnel(HadTrafficSign* pTrafficSign, HadLaneGroup* pLinkGroup);

        void createRdsSLB(HadTrafficSign* pTrafficSign, HadLaneGroup* linkGroup, LineString3d& guardrailLine, RdsGroup* pRdsGroup);

    private:
        HadRoadBoundary* getGrappedBoundary(HadLaneGroup* pGroup, MapPoint3D64& pt);
        MapPoint3D64 getCenterPtFromPolygon(std::vector<MapPoint3D64> vertexes);
        bool pointInRoad(MapPoint3D64 centerPt, HadLaneGroup* pGroup);

		void grapPoint(MapPoint3D64& vertex, std::vector<HadSkeleton*>& boundaries,
			MapPoint3D64& grappedPt, LineString3d& grappedLinestring, double& grappedDis);

        void setPairBoard(HadTrafficSign* pTrafficSign, RdsSpeedLimitBoard* pExistRdsSlb, int diffHigh, int32 speed, double angle, RdsSpeedLimitBoard::SpeedLimitType type);

        double calSLBDirection(HadTrafficSign* pTrafficSign, LineString3d& guardrailLine, HadLaneGroup* linkGroup);

        // 计算护栏平均角度
        double guardrailAverage(const LineString3d& originLine);


    private:
        static double EXIST_TUNNEL_DIFFERENCE;          //牌子附近存在有效隧道判定长度差。单位：米。
        static double PAIR_SIGN_HIGH_DIFFERENCE;        //同一杆子上两个牌子的有效判定高度差。单位：米。
        static double SAME_POLE_DISTANCE_DIFFERENCE;    //同一杆子上两个牌子的有效判定距离差。单位：米。

        //std::unordered_map<uint64, MapPoint3D64> m_mapSlbCenterPt;
        std::vector<Pole> m_vctPoles;
    };
}

