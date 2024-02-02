#pragma once
#include "Compiler.h"

namespace OMDB
{
    class MarkingCompiler : public Compiler
    {
    public:
        MarkingCompiler(CompilerData& data) :Compiler(data) {};
        struct MarkingObj
        {
            float dis;
            RDS::RdsMarking::MarkingType type;
        };

        struct MarkingLane
        {
            segment_t seg;
            segment_t verSeg;
            MapPoint3D64 beginPoint;
            MapPoint3D64 endPoint;
            size_t lineId{ 0 };
            size_t segId{ 0 };
            size_t id{ 0 };
            point_t segDir{ 0, 0, 0 };
            point_t verSegDir{ 0, 0, 0 };
            vector_t segUnitDir{ 0.0, 0.0, 0.0 };
            vector_t verSegUnitDir{ 0.0, 0.0, 0.0 };
            double segLength{ 0.0 };
            double verSegLength{ 0.0 };
        };

        struct MarkingLaneData
        {
            std::vector<MarkingLane> firstDatas;
            std::vector<MarkingLane> secondDatas;
            bool isSingleBoundary() { return firstDatas.empty() || secondDatas.empty() ? true : false; };
        };

        static MarkingLane buildMarkingLane(
            const size_t& id,
            const point_t& searchPoint,
            const std::vector<std::vector<MapPoint3D64>>& allLines,
            const std::vector<segment_t>& segments,
            const std::vector<std::vector<size_t>>& sizes);

        static void getObjectWidthAndDir(
            const std::vector<std::vector<MapPoint3D64>>& allLines,
            const rtree_type_segment& rtree,
            const point_t& p_1,
            const int withinRefLanes,
            const bool isIntersection,
            const std::vector<MapPoint3D64>& polyPoints,
            const std::vector<segment_t>& segments,
            const std::vector<std::vector<size_t>>& sizes,
            point_t& pos,
            Vector3& dir,
            double& laneWidth);

        static void generateContourByPosAndDir(
            MapPoint3D64 pos,
            Vector3 dir,
            Vector3 up,
            float width,
            float height,
            std::vector<MapPoint3D64>& outPoints);

        // 计算两个向量的最小夹角
        static uint16 minimalDegree(point_t dir1, point_t dir2);

    protected:
        virtual void compile(
            HadGrid* const pGrid, 
            const std::vector<HadGrid*>& nearby, 
            RdsTile* pTile) override;

        void getMarkingRate(
            RDS::RdsMarking::MarkingType markingType,
            float& width,
            float& height);

        void addRdsMarking(
            RdsTile* pTile,
            HadArrow* hadArrow,
            RDS::RdsMarking::MarkingType type);

		void processCrossGrid(HadGrid* const pGrid,
			HadArrow* hadArrow,
			std::vector<std::vector<MapPoint3D64>>&  allLines);

        bool outOfRoadFace(
            const HadArrow* hadArrow, 
            const std::vector<HadGrid*>& nearby,
            std::vector<MapPoint3D64> outPts);

		void createConnectedBoundary(
            HadLaneGroup* pCurrentGroup,
            std::vector<HadLaneGroup*> connectedLaneGroups,
			std::vector<MapPoint3D64>& firstBoundaryVertexes,
			std::vector<MapPoint3D64>& secondBoundaryVertexes);

		int getObjectPos(
			point_t& pos,
			const HadArrow* hadArrow);

        inline point_t getArrowCenterPoint(
            const linestring_t& pLine)
        {
            point_t p(0, 0, 0);
            for (size_t i = 0; i < 4; ++i)
                bg::add_point(p, pLine[i]);
            bg::divide_value(p, 4.0);
            return p;
        }

        bool isIntersectionArrow(
            const std::vector<MapPoint3D64>& pArrowPoly);
        
    };
}
 
