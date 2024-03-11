#pragma once
#include <vector>
#include "navi_rds/rds.h"
#include "../om/HadGrid.h"
#include "geometry/map_point3d64.h"
#include "clipper.hpp"
#include "map_point3d64_converter.h"
#include "algorithm/linear_interpolation_triangle_surface.h"
#include "CompileSetting.h"

using namespace RDS;
#define DB_HAD_APPLY_PA_REFERENCE	 -99
#define DB_HAD_NOT_APPLY_PA_REFERENCE	 999999
#define LINK_IS_IN_TOLL_AREA (94)
#define LINK_WAY_TYPE_PA_NAME (68)
#define LINK_MULTI_DIGITIZED_PA_NAME (51)

// had_link_form属性
#define LINK_IS_IN_TUNNEL  (31)        //隧道
#define LINK_IS_IN_ROUNDABOUT (33)     //环岛
#define LINK_IS_IN_UTURN (35)          //掉头

namespace OMDB
{
	struct Blob
	{
		void* data;
		size_t size;
	};

    template <typename Container>
    class VectorIndexableGuardrail {
        using size_type = typename Container::size_type;
        using cref = const typename Container::value_type&;
        Container const& container;

    public:
        using result_type = cref;

        explicit VectorIndexableGuardrail(Container const& c)
            : container(c)
        {
        }

        result_type operator()(size_type i) const
        {
            return container[i];
        }
    };

    struct SegmentRTree {
        using guardParam = bgi::quadratic<16>;
        using indexGetterSegment = VectorIndexableGuardrail<std::vector<segment_t>>;
        using RTree = bgi::rtree<std::vector<segment_t>::size_type, guardParam, indexGetterSegment>;
        using Ptr = std::shared_ptr<RTree>;
    };

    struct BoxRTree {
        using guardParam = bgi::quadratic<16>;
        using indexGetterBox = VectorIndexableGuardrail<std::vector<box_t>>;
        using RTree = bgi::rtree<std::vector<box_t>::size_type, guardParam, indexGetterBox>;
        using Ptr = std::shared_ptr<RTree>;
    };

    struct Box2TRTree {
        using guardParam = bgi::quadratic<16>;
        using indexGetterBox = VectorIndexableGuardrail<std::vector<box_2t>>;
        using RTree = bgi::rtree<std::vector<box_2t>::size_type, guardParam, indexGetterBox>;
        using Ptr = std::shared_ptr<RTree>;
    };

    struct laneGroupInfo
    {
        HadLaneGroup* _laneGroup;
        box_t _groupBox;
        box_t _groupBox2d;
        ring_2t _groupPoly;
        linestring_t _groupPoints;
        point_t _centerPoint;
        vector_t _parallelDir;
        segment_t _verticalSegment;
        vector_t _leftVerUnitDir;
        vector_t _rightVerUnitDir;
        linestring_t refLine;
        linestring_t raisedRefLine;
        linestring_t raisedRefLine_relative;
        double _startRaiseHeight;
        double _endRaiseHeight;
        bool _isConvergence{ false };
        bool _isCirclePoly{ false };

        LineString3d _originMapLineL;
        LineString3d _originMapLineR;
        linestring_t _originLineL;
        linestring_t _originLineR;
        bool isReversedL{ false };
        bool isReversedR{ false };
        linestring_t _raisedLineL;
        linestring_t _raisedLineR;
        double floor_height{ 100000 };
        int groupId;
        int64 id() {
            return _laneGroup ? _laneGroup->originId : 0;
        }
    };
    using PtrLaneGroupInfo = std::shared_ptr<laneGroupInfo>;

	struct rdsIntersectionInfo
	{
		RdsIntersection* _intersection;
		box_2t _intersectionBox2T;
		ring_2t _intersectionPoly2T;
		linestring_t _intersectionPoints;
        Polygon3d _originIntersctPoly;
        int64 _originId;
        std::vector<OMDB::LineString3d> curbLines;
        RdsGroup* rdsGroup{ nullptr };
	};

    struct rdsRoadInfo
    {
        int64 originId;
        RdsRoad* _road;
		box_2t _roadBox2T;
		ring_2t _roadPoly2T;
        linestring_t _roadPoints;
        std::vector<LineString3d> _originRoadLines;
    };

	struct rdsLineInfo
	{
		box_2t _lineBox2T;
		linestring_t _linePoints;
		std::vector<MapPoint3D64> _originPoints;
	};

	struct CompilerData
	{
		// RDS车道线数据
        Box2TRTree::Ptr m_rtreeLineBox; 
        std::vector<box_2t> m_rdsLineBoxes;
		std::vector<rdsLineInfo> m_rdsLines;
        SegmentRTree::Ptr rtreeGridLines;
        std::vector<segment_t> gridLines;

		// RDS路口数据
        Box2TRTree::Ptr m_rtreeIntersectionBox;
        std::vector<box_2t> m_rdsIntersectionBoxes;
		std::vector<rdsIntersectionInfo> m_rdsIntersections;
        std::vector<Triangle> m_intersectionTriangles;
        std::vector<OMDB::LineString3d> greenbeltLines;

        // RDS路面数据
        Box2TRTree::Ptr m_rtreeRoadBox;
        std::vector<box_2t> m_rdsRoadBoxes;
        std::vector<rdsRoadInfo> m_rdsRoads;
        std::vector<Triangle> m_roadTriangles;
        std::unordered_map<int64, rdsRoadInfo> m_originIdToRdsRoadInfo;

        // RDSStopLine数据和构建路面、路口的端线
        Box2TRTree::Ptr m_rtreeStopLineBox; 
        std::vector<box_2t> m_rdsStopLineBoxes;
        std::vector<rdsLineInfo> m_rdsStopLines;    // RDS StopLine
        SegmentRTree::Ptr rtreeGridIntersectionStopLines;
        std::vector<segment_t> gridIntectionStopLines;  // 路口端线
        std::unordered_map<size_t, int64> stopLineToOriginIdMaps;
        std::vector<segment_t> gridRoadStopLines;   // 路面端线

        //车道组Rtree
        Box2TRTree::Ptr m_rtreeLaneGroup;
        std::vector<box_2t> m_laneGroupBoxes;
        std::vector<HadLaneGroup*> gridLaneGroups;

        //保存锯齿状车道组（为了和后面路牙编译同步边界）
        std::vector<HadLaneGroup*> hullGroups;
        std::vector<bool> isLeftHulls;
	};

    class Compiler
    {
    public:
        virtual ~Compiler() = default;
        Compiler(CompilerData& data) :compilerData(data) {};
		void Compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile);

        static Vector3d calDirection(MapPoint3D64 start, MapPoint3D64 end);
        static float calcLength(MapPoint3D64& pt1, MapPoint3D64& pt2);
        static float calcLength(const std::vector<MapPoint3D64> polyline);
        static float calcLength(const std::vector<MapPoint3D64> polyline, const MapPoint3D64& pt);// pt 距离polyline起点的距离，先抓点，抓点失败返回距起点的直线距离

        static bool _intersect(MapPoint3D64* points1, size_t pointCount1, MapPoint3D64* points2, size_t pointCount2, ClipperLib::PolyTree& polyTree);

        forceinline linestring_t getConvertLine(const std::vector<MapPoint3D64>& pts)
        {
            auto mapPoints = pts;
            coordinatesTransform.convert(mapPoints.data(), mapPoints.size());
            return LINESTRING_T(mapPoints);
        }

    protected:
        virtual void compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile) = 0;

		void convert(const MapPoint3D64& point, Point3d& pt);
		void convert(const OMDB::LineString3d& src, RDS::LineString3d& dst);
		void convert(const OMDB::MultiPoint3d& src, RDS::MultiPoint3d& dst);
		void convert(const OMDB::MultiLineString3d& src, RDS::MultiLineString3d& dst);
		void convert(const OMDB::Polygon3d& src, RDS::Polygon3d& dst);
		void convert(const OMDB::MultiPolygon3d& src, RDS::MultiPolygon3d& dst);

        void convert(const RDS::Point3d& src, MapPoint3D64& dst);
		void convert(const RDS::Polygon3d& src, OMDB::Polygon3d& dst);
        void convert(const RDS::LineString3d& src, OMDB::LineString3d& dst);
        void convert(const RDS::MultiLineString3d& src, OMDB::MultiLineString3d& dst);

        static bool isStraightIntersection(HadIntersection* const hadIntersct);
        static bool isTunnelArea(HadLaneGroup* pLinkGroup);
        static bool isInTollArea(HadLaneGroup* pLinkGroup);
        static bool isMultiDigitized(HadLaneGroup* pLinkGroup);
        static bool isSemiTransparentGroup(HadLaneGroup* pLaneGroup);
        static bool isNotSemiTransparentGroup(HadLaneGroup* pLaneGroup);
        static bool isExistFillArea(HadLaneGroup* pLaneGroup);
        static bool isExistAssociation(HadLaneGroup* pLaneGroup);
        static bool isProDataLevel(HadLaneGroup* pLaneGroup);
        static bool isProDataLevel(const std::vector<HadLaneGroup*>& pLaneGroups);

        static bool isUTurn(HadLaneGroup* pLinkGroup);
        static bool isRoundabout(HadLaneGroup* pLinkGroup);
        static bool isCreateRdsForIntersection(HadGrid* const pGrid, const std::vector<HadLaneGroup*> pGroups);

        static bool isTouchWithHadLaneGroup(HadLaneGroup* laneGroup1, HadLaneGroup* laneGroup2);
       
//         RdsTile* getOrCreateTile(int32 id,RdsDatabase* pDatabase);
        RdsObject* createObject(RdsTile* pTile, EntityType type);
        RdsGroup* queryGroup(int64 originId,RdsTile* pTile);
        void deleteGroup(RdsGroup* pGroup, RdsTile* pTile);
        void deleteObject(RdsObject* pObject, RdsTile* pTile);

        void connectForwardRoadBoundary(HadLaneGroup* pGroup, HadRoadBoundary* pBoundary,
            std::deque<HadRoadBoundary*>& connectedBoundarys, std::deque<HadLaneGroup*>& connectedLaneGroups, int connectNum);

        void connectBackwardRoadBoundary(HadLaneGroup* pGroup, HadRoadBoundary* pBoundary,
            std::deque<HadRoadBoundary*>& connectedBoundarys, std::deque<HadLaneGroup*>& connectedLaneGroups, int connectNum);

		void connectForwardLaneBoundary(HadLaneGroup* pGroup, HadLaneBoundary* pBoundary,
			std::deque<HadLaneBoundary*>& connectedBoundarys, std::deque<HadLaneGroup*>& connectedLaneGroups, int connectNum);

		void connectBackwardLaneBoundary(HadLaneGroup* pGroup, HadLaneBoundary* pBoundary,
			std::deque<HadLaneBoundary*>& connectedBoundarys, std::deque<HadLaneGroup*>& connectedLaneGroups, int connectNum);

        void getConnectedGroups(HadLaneGroup* pLaneGroup, std::vector<std::vector<HadLaneGroup*>>& allConnectedGroups, int connectNum);

		// 车道组连接
		void getConnectGroup(HadLaneGroup* pLaneGroup, HadLaneGroup* pTopoGroup,
			std::vector<HadLaneGroup*>& connectedGroups, bool forward, int connectNum);

        //车道组是否有上层车道组压盖
        bool checkOverlayOnLaneGroup(HadLaneGroup* pLaneGroup);

        //道路Link是否有压盖
        bool checkOverlayOnLink(HadLink* pLink);

		/// 判断边界左右是否与side相同
        static bool sideEqual(HadRoadBoundary* const pBoundary, HadLaneGroup* const pGroup, int side);

		/// 判断边界方向是否与direction相同
        static bool directionEqual(HadLaneBoundary* const pBoundary, HadLaneGroup* const pGroup, int direction);

        /// 判断边界方向是否与direction相同
        static bool directionEqual(HadRoadBoundary* const pBoundary, HadLaneGroup* const pGroup, int direction);

		static HadRoadBoundaryNode* getStartNode(HadRoadBoundary* const pBoundary, HadLaneGroup* const pGroup);
		static HadRoadBoundaryNode* getEndNode(HadRoadBoundary* const pBoundary, HadLaneGroup* const pGroup);
		static HadLaneBoundaryNode* getStartNode(HadLaneBoundary* const pBoundary, HadLaneGroup* const pGroup);
		static HadLaneBoundaryNode* getEndNode(HadLaneBoundary* const pBoundary, HadLaneGroup* const pGroup);

        static LineString3d getBoundaryLocation(HadLaneBoundary* const pBoundary, HadLaneGroup* const pGroup);

        static LineString3d getBoundaryLocation(HadRoadBoundary* const pBoundary, HadLaneGroup* const pGroup);

        static void projectLineOnRoadSurface(const LineString3d& leftRdBoundary, const LineString3d& rightRdBoundary, const std::vector<MapPoint3D64>& line, std::vector<MapPoint3D64>& lineOnRoadSurface, double tolerance = 0);
        static void projectPointOnRoadSurface(const LineString3d& leftRdBoundary, const LineString3d& rightRdBoundary,const MapPoint3D64& point,MapPoint3D64& pointOnRoadSurface, double tolerance = 0);

        static void densityRoadSurface(const std::vector<MapPoint3D64>& line, std::vector<MapPoint3D64>& lineOnRoadSurface, double tolerance = 100000);//100m


        /// @brief 得到两点间的水平距离。非真实距离，仅用于距离比较。
        int64 _squaredHDistance(const MapPoint3D64& pt1, const MapPoint3D64& pt2);

        MapPoint3D64 _nearestLineStringEnd(const MapPoint3D64& pt, const LineString3d& lineString);

        void getLaneGroupBoundary(HadLaneGroup* pGroup, LineString3d& leftSide, LineString3d& rightSide, bool isUseHullGroup = true);

        void makeLaneGroupPolygon(const std::vector<MapPoint3D64>& leftSide, const std::vector<MapPoint3D64>& rightSide, Polygon3d& polygon);

        virtual size_t makeLaneGroupPolygon(HadLaneGroup* pGroup, MapPoint3D64*& polygon);

        BoundingBox2d makeBoundingBox2d(const std::vector<MapPoint3D64>& vertexes);

        Box2TRTree::Ptr getLineBox2TRTree();                        // 获取编译后的车道线RTree
        SegmentRTree::Ptr getLineSegmentRTree();                    // 获取编译后的车道线Segment RTree
        Box2TRTree::Ptr getIntersectionBox2TRTree();                // 获取编译后的路口RTree
        Box2TRTree::Ptr getRoadBox2TRTree();                        // 获取编译后的路面RTree
        Box2TRTree::Ptr getStopLineBox2TRTree();                    // 获取编译后的stopline对象RTree
        SegmentRTree::Ptr getIntersectionStopLineSegmentRTree();    // 获取编译后的路口stopline对象RTree
        Box2TRTree::Ptr getLaneGroupBox2DRTree();                   // 获取车道组的对象RTree

        //获取网
        point_t getNearestPoint(const std::vector<point_t>& points, const point_t& originPoint);

        // 3d
        void getClosestSeg(
            const point_t& point,
            std::vector<point_t>& line,
            std::vector<point_t>::iterator& it_min1,
            std::vector<point_t>::iterator& it_min2);

        // MapPoint3D
        void getClosestSeg(
            const MapPoint3D64& point,
            std::vector<MapPoint3D64>& line,
            std::vector<MapPoint3D64>::iterator& it_min1,
            std::vector<MapPoint3D64>::iterator& it_min2);

        // 2d
        void getClosestSeg(
            const point_2t& point,
            std::vector<point_2t>& line,
            std::vector<point_2t>::iterator& it_min1,
            std::vector<point_2t>::iterator& it_min2);

        static std::vector<HadSkeleton*> getLaneGroupSkeletons(std::vector<HadSkeleton*>& skeletons);
    private:
        static bool isNotSemiTransparentLaneGroup(HadLaneGroup* skeleton);
        static bool isNotSemiTransparentRoadBoundary(HadRoadBoundary* skeleton);
        static bool isNotSemiTransparentLaneBoundary(HadLaneBoundary* skeleton);

    protected:
        MapPoint3D64Converter coordinatesTransform;
        CompilerData& compilerData;
    };
}
