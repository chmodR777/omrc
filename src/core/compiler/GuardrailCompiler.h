#pragma once
#include <unordered_map>
#include <set>
#include <unordered_map>
#include "Compiler.h"
#include "algorithm/geography_algorithm.h"
#include "algorithm/grap_point_algorithm.h"

namespace OMDB
{
    struct guardrailPoint
    {
        guardrailPoint(
            const MapPoint3D64& originPoint,
            double fractionDistance,
            int id)
            : _originPoint(originPoint),
            _fractionDistance(fractionDistance),
            _id(id)
        {
        }

        int _id;
        MapPoint3D64 _originPoint;
        double _fractionDistance;
    };

    struct guardrailVertex
    {
        guardrailVertex(
            const MapPoint3D64& coordinate,
            int id)
            : _originPoint(coordinate),
            _id(id)
        {
            _point2d = POINT_2T(coordinate);
        }

        int _id;
        MapPoint3D64 _originPoint;
        point_2t _point2d;
    };

    struct forkGroupInfo
    {
        vector_t _baseLine;
        vector_t _baseDir;
        point_t _basePoint;
        box_t _basePointExpandBox;
        ring_2t _basePointExpandPoly;
        ring_2t _basePointExpandPoly2;
    };

    struct kneePoint
    {
        point_2t _point;
        segment_2t _prevSeg;
        segment_2t _nextSeg;
    };

    struct goreDiversionInfo
    {
        RdsDiversion* _rdsDiversion;
        Polygon3d _diversionPoly;
        vector_t _baseLine;
        vector_t _baseDir;
        point_t _basePoint;
        ring_2t _originRing;

        //update
        point_2t _endPoint;
        bool _isDeleteGuardrail{ false };
        ring_2t _deleteGuardrailPoly;
    };

    struct singleDiversionInfo
    {
        RdsDiversion* _rdsDiversion;
        Polygon3d _diversionPoly;
        ring_2t _originRing;
        ring_2t _expandRing;
        box_t _diversionBox;
        point_t _centerPoint;
        bool _isWillDelete{ false };
        bool _isCirclePoly{ false };
    };

    struct GuardrailIndices {
        SegmentRTree::Ptr rtreeBarrierSegs;
        BoxRTree::Ptr rtreeDiversionBox;
        BoxRTree::Ptr rtreeSingleBox;
        BoxRTree::Ptr rtreeLaneGroupBox;
    };

    struct guardrailEdge
    {
        guardrailEdge(
            guardrailVertex& vertexOne,
            guardrailVertex& vertexTwo,
            int id)
            : _vertices({ &vertexOne, &vertexTwo }),
            _id(id)
        {
            _seg2d = segment_2t(vertexOne._point2d, vertexTwo._point2d);
            _line2d.push_back(vertexOne._point2d);
            _line2d.push_back(vertexTwo._point2d);
            next = nullptr;
        }

        int _id;
        std::array<guardrailVertex*, 2> _vertices;
        segment_2t _seg2d;
        linestring_2t _line2d;
        guardrailEdge* next;
    };

    class GuardrailCompiler : public Compiler
    {
	public:
        GuardrailCompiler(CompilerData& data) :Compiler(data) {};
    protected:
        virtual void compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile) override;

        void connectNextRoadBoundary(HadLaneGroup* pGroup, HadRoadBoundary* pBoundary, LineString3d& originLine);

        bool isMiddleRoadBoundary(HadLaneGroup* pGroup, HadRoadBoundary* pBoundary, LineString3d& segLine);

        double getLineInPolyPercentage(linestring_2t& tmpLine, std::vector<ring_2t>& polys);

        void createGuardrail(
            std::vector<RdsObject*>& rdsObjects,
            RdsTile* pTile,
            std::vector<LineString3d>& lines);

        std::vector<guardrailPoint> buildGuardrailPoints(
            const std::vector<MapPoint3D64>& originPoints);

        LineString3d getNewGuardrailByFraction(
            double startFractionDis,
            double endFractionDis,
            std::vector<guardrailPoint> originGuardrails,
            std::vector<MapPoint3D64>& originPoints);

        void setBoundaryGuardrailSegments(
            const LineString3d& originLine, 
            std::vector<segment_2t>& guardrailSegments);

        bool existGuardrailSegment(
            const point_2t& pa, const point_2t& pb, 
            const rtree_type_2segment& guardrailRtree,
            const std::vector<segment_2t>& guardrailSegments);

        bool isExistBarrierObjects(
            const LineString3d& line,
            const SegmentRTree::Ptr& rtreeSegs,
            const std::vector<segment_t>& allBarrierSegs);

        std::vector<LineString3d> modifyGuardrailByRdsTriDiversion(
            const std::vector<goreDiversionInfo>& triangleDiversions,
            const std::vector<segment_2t>& guardrailSegments,
            const LineString3d& originLine);

        bool modifyGuardrailByRdsSingleDiversion(
            const laneGroupInfo& groupInfo,
            const std::vector<singleDiversionInfo>& singleDiversions,
            const LineString3d& originLine, 
            const size_t& index,
            std::vector<LineString3d>& resultLines);

        bool isCirclePoly(ring_2t& polyRing);

        std::vector<forkGroupInfo> getForkGroups(
            const std::vector<HadLaneGroup*>& groups);

        box_t expandPoint(
            point_t p,
            int64 len);
        
        box_t expandBoxZValue(
            box_t b,
            int64 zVal);        

        void updateGoreDiversionInfo(
            goreDiversionInfo& goreDiversion, ring_2t _basePointExpandPoly);

        bool isKneePoint(
            const point_2t& currentPoint,
            const point_2t& prevPoint,
            const point_2t& nextPoint);

        ring_2t getExpandPolyByVector(
            const point_2t& basePoint,
            const vector_2t& dirH,
            const vector_2t& dirV,
            const double width,
            const double length);

        ring_2t getExpandPolyByVector_twoway(
            const point_2t& basePoint,
            const vector_2t& dirH,
            const vector_2t& dirV,
            const double width,
            const double length);

        bool getExpandPolyByOffset(
            const linestring_2t& line,
            const int64& offsetSize,
            ring_2t& resultRing);

        bool getExpandPolyByOffset(
            const ring_2t& line,
            const int64& offsetSize,
            ring_2t& resultRing);

        std::vector<HadLaneGroup*> getGridAllGroups(
            HadGrid* const pGrid);

        void getPrevGroups(
            HadLaneGroup* const currentGroup,
            std::vector<HadLaneGroup*>& resultGroups,
            const int& num,
            int& count);

        void getNextGroups(
            HadLaneGroup* const currentGroup,
            std::vector<HadLaneGroup*>& resultGroups,
            const int& num,
            int& count);

        void setLaneGroupData(
            const std::vector<HadLaneGroup*>& groups);

        void setBarrierData(
            HadGrid* const pGrid);

        void setBarrierObjects(
            HadLaneGroup* pGroup, 
            HadObject* pObject,
            std::vector<LineString3d>& tmpLines);

        void setDiversionData(
            RdsTile* const pTile);

		void setLaRoadFaceData(
			RdsTile* const pTile);

        void setLaDiversionData();

        void setGoreDiversionData();

        void setSingleDiversionData();

        bool isLaDiversion(RdsDiversion* rdsDiversion);

        void removeDuplicatePoint(std::vector<LineString3d> roundGuardrails, std::vector<LineString3d>& filteredLines);

		void buildLaPolygon(std::vector<LineString3d>& laEdges, Polygon3d& polygon);

        point_t getNearestPoint(
            const std::vector<point_t>& points,
            const point_t& originPoint);

        rtree_type_2segment getBoundarySegmentRTree(
            const std::vector<segment_2t>& segs);

        SegmentRTree::Ptr getBarrierSegmentRTree();
        BoxRTree::Ptr getDiversionBoxRTree();
        BoxRTree::Ptr getSingleBoxRTree();
        BoxRTree::Ptr getLaneGroupBoxRTree();

    private:
        std::vector<HadLaneGroup*> allGroups;
        std::vector<forkGroupInfo> allForkGroups;
        std::vector<laneGroupInfo> allGroupDatas;
        std::vector<box_t> allGroupBoxes;
        std::unordered_map<int64, laneGroupInfo> allGroupMaps;

        std::vector<LineString3d> allBarrierLines;
        std::vector<segment_t> allBarrierSegs;

        std::vector<std::shared_ptr<RdsDiversion>> gridLaDiversions;
        std::vector<RdsDiversion*> gridDiversions;
        std::vector<RdsDiversion*> gridDeleteDiversions;
        std::vector<Polygon3d> gridDiversionPolys;
        std::vector<box_t> gridDiversionBoxes;

        std::vector<goreDiversionInfo> allGoreDiversions;
        std::vector<singleDiversionInfo> allSingleDiversions;
        std::vector<box_t> gridSingleDiversionBoxes;

        GuardrailIndices guardrailInd;

        bg::strategy::buffer::join_miter join_strategy;
        bg::strategy::buffer::end_flat end_strategy;
        bg::strategy::buffer::point_square circle_strategy;
        bg::strategy::buffer::side_straight side_strategy;

    };
}

