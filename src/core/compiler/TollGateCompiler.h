#pragma once
#include "Compiler.h"

namespace OMDB
{   
    class TollGateCompiler : public Compiler
    {
	public:
        TollGateCompiler(CompilerData& data) :Compiler(data) {};
    protected:
        virtual void compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile) override;

    private:
        //void _addRdsTollPassageway(int indexBoundary, MapPoint3D64 ptLaneEnd, MapPoint3D64 ptFoot, RdsTile* pTile);
        void _addRdsTollPassageway(
            const HadLaneBoundary* pBoundary,
            const HadTollGate::PtrTollLane& pTollLane,
            MapPoint3D64 ptLaneEnd,
            MapPoint3D64 ptFoot,
            RdsTile* pTile);

        void _setRdsTollGate(HadLaneGroup* pLaneGroup);

		// 计算护栏平均角度
		double boundaryAverage(const LineString3d& originLine);

        void _recalPosition();

        void _resampleCallback(Vector3 point, Vector3 dir);
        static void _resampleCallbackWrapper(Vector3 point, Vector3 dir, void* userdata) 
        {
            ((TollGateCompiler*)userdata)->_resampleCallback(point, dir);
        }

        void optimizeName(const std::wstring& tollName, std::wstring&  optimizeTollName);
        void optimizeName();
    private:
        /// @brief 求二维空间直线外点对直线的垂足。（其中高程为两点高程最大值，主要使用场景为二维）
        /// @param pt1 直线上的第一个点 
        /// @param pt2 直线上的第二个点
        /// @param pt3 直线外的点。
        /// @return 垂足点。
        MapPoint3D64 _calFootOfPerpendicular(MapPoint3D64 pt1, MapPoint3D64 pt2, MapPoint3D64 pt3);

        /// @brief 求二维空间两直线交点。（其中高程为两点高程最大值，主要使用场景为二维）
        /// @param pt1 第一条直线上的第一个点 
        /// @param pt2 第一条直线上的第二个点
        /// @param pt3 第二条直线上的第一个点
        /// @param pt4 第二条直线上的第二个点
        /// @return 交点。
        MapPoint3D64 _calIntersection(MapPoint3D64 pt1, MapPoint3D64 pt2, MapPoint3D64 pt3, MapPoint3D64 pt4);

    private:
        HadTollGate* m_pTollGate;
        RdsToll* m_pRdsToll;
        HadLane* m_pLane;
        std::vector<HadTollGate::PtrTollLane> m_TollLanes;
        std::vector<Vector3> m_vctPostion;
    };

}

