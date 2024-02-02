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

		// ���㻤��ƽ���Ƕ�
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
        /// @brief ���ά�ռ�ֱ������ֱ�ߵĴ��㡣�����и߳�Ϊ����߳����ֵ����Ҫʹ�ó���Ϊ��ά��
        /// @param pt1 ֱ���ϵĵ�һ���� 
        /// @param pt2 ֱ���ϵĵڶ�����
        /// @param pt3 ֱ����ĵ㡣
        /// @return ����㡣
        MapPoint3D64 _calFootOfPerpendicular(MapPoint3D64 pt1, MapPoint3D64 pt2, MapPoint3D64 pt3);

        /// @brief ���ά�ռ���ֱ�߽��㡣�����и߳�Ϊ����߳����ֵ����Ҫʹ�ó���Ϊ��ά��
        /// @param pt1 ��һ��ֱ���ϵĵ�һ���� 
        /// @param pt2 ��һ��ֱ���ϵĵڶ�����
        /// @param pt3 �ڶ���ֱ���ϵĵ�һ����
        /// @param pt4 �ڶ���ֱ���ϵĵڶ�����
        /// @return ���㡣
        MapPoint3D64 _calIntersection(MapPoint3D64 pt1, MapPoint3D64 pt2, MapPoint3D64 pt3, MapPoint3D64 pt4);

    private:
        HadTollGate* m_pTollGate;
        RdsToll* m_pRdsToll;
        HadLane* m_pLane;
        std::vector<HadTollGate::PtrTollLane> m_TollLanes;
        std::vector<Vector3> m_vctPostion;
    };

}

