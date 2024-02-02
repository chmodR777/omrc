#pragma once
#include "Compiler.h"
namespace OMDB
{
    class Sign3dCompiler : public Compiler
    {
    protected:
        virtual void compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile) override;

        void grapPoint(MapPoint3D64& vertex, std::vector<HadSkeleton*>& boundaries, 
            MapPoint3D64& grappedPt, MapPoint3D64& siPt, MapPoint3D64& eiPt, double& grappedDis);

        void createRdsSLB(RdsTile* pTile, HadTrafficSign* pTrafficSign, MapPoint3D64 centerPt, const MapPoint3D64& siPt, const MapPoint3D64& eiPt, const MapPoint3D64& minGrappedPt, RdsGroup* pRdsGroup);

        bool existPole(RdsGroup* const currRdsGroup, const MapPoint3D64& grappedPt, RdsSpeedLimitBoard*& pExistSlb);

        // 限速标牌存在重复,重复的限速标牌所属车道组可能与本车道组不同,因此前后车道组的限速标牌也需要收集
        bool containsDuplicateSign3d(HadGrid* const grid, RdsTile* const pTile, RdsGroup* const currRdsGroup, MapPoint3D64& grappedPt);

    private:
        MapPoint3D64 Sign3dCompiler::getCenterPtFromPolygon(std::vector<MapPoint3D64> vertexes);


    private:
        //static double SIGN_3D_DISTANCE_TOLERANCE;
        static double SAME_POLE_DISTANCE_TOLERANCE;
        static const int PAIR_SIGN_HIGH_DIFFERENCE = 50;     //同一杆子上两个牌子的有效判定高度差。单位：厘米。

        std::unordered_map<uint64, MapPoint3D64> m_mapSlbCenterPt;
    };
}

