#pragma once
#include "Compiler.h"
#include "clipper.hpp"
#include <unordered_map>

// ��ֹ�����ض���� cq_stdlib.h �е� memcpy ��Ӱ�쵽����Ĵ��루��Ҫ�� CGAL / boost ���е� "std::memcpy"��
#pragma push_macro("memcpy")
#undef memcpy
#include "boost/geometry.hpp"
#include <unordered_set>

namespace OMDB
{   
    namespace bg = boost::geometry;
    using namespace bg;
    typedef bg::model::d2::point_xy<double> BgPoint;
    typedef bg::model::linestring<BgPoint> BgLinestring;

    /// @brief �ص�����
    struct OverlapArea
    {
        MapPoint3D64 ptAboveGrap;   //�ཻ�����ĵ�ͶӰ���Ϸ��������ڳ������ϵ�ͶӰ�㡣
        MapPoint3D64 ptBelowGrap;   //�ཻ�����ĵ�ͶӰ���·��������ڳ������ϵ�ͶӰ�㡣

         ///�������������Ա
         ///...
    };

    typedef std::vector<OverlapArea> VectorOverlapArea;
    typedef std::unordered_map<const HadLaneGroup*, VectorOverlapArea> MapLgIntersection;
    typedef std::unordered_map<const HadLaneGroup*, MapLgIntersection> MapLgIntersectLg;

    typedef std::unordered_set<const HadLaneGroup*> SetLg;
    typedef std::unordered_map<const HadLaneGroup*, SetLg> MapLg2Lgs;

    struct LaneGroupPair
    {
        LaneGroupPair(HadLaneGroup* _pLg1, HadLaneGroup* _pLg2) : pLg1(_pLg1), pLg2(_pLg2) {}
        HadLaneGroup* pLg1;
        HadLaneGroup* pLg2;
    };
    
    struct LaneGroupPair_Hasher
    {
        size_t operator()(const LaneGroupPair& lgPair) const noexcept
        {
            return std::hash<int64>{}(lgPair.pLg1->originId);
            std::size_t h1 = std::hash<int64>{}(lgPair.pLg1->originId);
            std::size_t h2 = std::hash<int64>{}(lgPair.pLg2->originId);
            return h1 ^ (h2 << 1); 
        }
    };

    struct LaneGroupPair_Comparator
    {
        bool operator()(const LaneGroupPair& LgPair1, const LaneGroupPair& LgPair2) const noexcept
        {
            if (LgPair1.pLg1 == LgPair2.pLg1 && LgPair1.pLg2 == LgPair2.pLg2)
                return true;
            else if (LgPair1.pLg1 == LgPair2.pLg2 && LgPair1.pLg2 == LgPair2.pLg1)
                return true;
            else
                return false;
        }
    };

    typedef std::unordered_set<LaneGroupPair, LaneGroupPair_Hasher, LaneGroupPair_Comparator> SetLgPair;


    class GradeSeparationCompiler : public Compiler
    {
	public:
        GradeSeparationCompiler(CompilerData& data) :Compiler(data) {};
    protected:
        virtual void compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile) override;

    public:
        /// @brief ���彻�������͸��Ч�������ĸ߶Ȳ���ֵ����λ�����ס���Դ���ݸ߳�Ϊ���Σ�
        /// ֻ�����������ཻ�������ĵ�ͶӰ�����߽��ߵ�ͶӰ�㣩�߲���ڴ���ֵʱ���������ɿ��������͸�������㼶��ϵ��֮����һ������
        static const int SEMITRANSPARENT_HEIGHT_DIFF_THRESHOLD = 100;

        //������������չ����
        static const double SEMITRANSPARENT_ABOVE_LG_EXPAND_DISTANCE;
        static const double SEMITRANSPARENT_BELOW_LG_EXPAND_DISTANCE;

    private:
        /// @brief ͨ��Link���ཻ�������彻���ϵ��
        bool _generateCrossRelationByLinkSelfIntersection(HadLink* pLink);

        /// @brief ͨ���������ռ��ϵ�������彻���ϵ��
        void _generateCrossRelationByLaneGroups(HadGrid* pGrid, HadLaneGroup* pGroup);

        //�ʵ�����ѹ������
        void _expandOverlapArea();

        void _lgIntersectProcess(HadLaneGroup* pCurLg, MapPoint3D64* points1, size_t pointCount1, HadLaneGroup* pNearbyLg);

        //bool _intersect(MapPoint3D64* points1, size_t pointCount1, MapPoint3D64* points2, size_t pointCount2, ClipperLib::PolyTree& polyTree);

        //size_t _makeLaneGroupPolygon(HadLaneGroup* pGroup, MapPoint3D64*& polygon);

        void _insertLgIntersection(HadLaneGroup* pBelowLg, const MapPoint3D64& ptBelowGrap, HadLaneGroup* pAboveLg, const MapPoint3D64& ptAboveGrap);

        bool _recursivePreviousLaneGroup(const HadLaneGroup* pGroup, double dRemainDis, SetLg& setExpandLg, bool bFirst = false, MapPoint3D64 ptLocation = {0});
        bool _recursiveNextLaneGroup(const HadLaneGroup* pGroup, double dRemainDis, SetLg& setExpandLg, bool bFirst = false, MapPoint3D64 ptLocation = { 0 });

        double _getLength(const MapPoint3D64& point1, const MapPoint3D64& point2);

        void _addRdsGroup(const SetLg& setLg, std::vector<RdsRelationship::ObjectInfo>& vctRdsGroup);

        void _mapPointsToBgLine(const MapPoint3D64* points, int32 count, BgLinestring& resultOut);

    private:
        MapLgIntersectLg m_mapLgIntersectLg;
        MapLg2Lgs m_mapBelow2AboveExpandLgs;
        MapLg2Lgs m_mapBelow2BelowExpandLgs;
        SetLgPair m_setLgPair;      ///< ���ڼ�¼�����������ռ��ϵ������̡�������ʹ洢�ڴ˽ṹ�С�
    };

}

