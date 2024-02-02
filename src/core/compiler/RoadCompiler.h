#pragma once
#include "Compiler.h"
#include "GuardrailCompiler.h"
#include <map_point3d64_converter.h>

namespace OMDB
{
    class RoadCompiler : public Compiler
    {
	public:
		RoadCompiler(CompilerData& data) :Compiler(data) {};

    protected:
		virtual void compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile) override;

		void createRoadFaceAtNextStraightRoad(HadLaneGroup* pCurrentGroup, RDS::RdsTile* pTile);
		void connectRoadFaceAtNextStraightRoad(HadLaneGroup* pCurrentGroup, LineString3d& leftSide, LineString3d& rightSide);
		void connectRoadFaceAtPreviousStraightRoad(HadLaneGroup* pCurrentGroup, LineString3d& leftSide, LineString3d& rightSide);

		void connectIntersectionRoad(
			LineString3d& leftSide, 
			LineString3d& rightSide, 
			bool isNext);

        void createRoadFaceAtForkRoadDownStream(HadLaneGroup* pCurrentGroup, RDS::RdsTile* pTile);
		void connectRoadFaceAtForkRoadDownStream(HadLaneGroup* pCurrentGroup, 
			LineString3d& leftSide, LineString3d& rightSide, 
			HadLaneGroup*& pLeftLaneGroup, HadLaneGroup*& pRightLaneGroup,
			LineString3d& forkRoadLeftSideDownStreamStopLine, 
			LineString3d& forkRoadRightSideDownStreamStopLine);
		void connectRoadFaceAtForkRoadUpStream(HadLaneGroup* pCurrentGroup,
			LineString3d& leftSide, LineString3d& rightSide,
			HadLaneGroup*& pLeftLaneGroup, HadLaneGroup*& pRightLaneGroup,
			LineString3d& forkRoadLeftSideUpStreamStopLine,
			LineString3d& forkRoadRightSideUpStreamStopLine);
		void addMiddlePoint(LineString3d& firstStopLine, LineString3d& secondStopLine);

		RDS::RdsRoad* createRoadFace(RdsTile* pTile, HadLaneGroup* pGroup);
        RDS::RdsRoad* createRoadFace(RdsTile* pTile, HadLaneGroup* pGroup, LineString3d& leftSide, LineString3d& rightSide);
		MapPoint3D64 getRoadBoundaryFirstPointRelGroup(HadLaneGroup* pGroup, HadRoadBoundary* pBoundary);
		MapPoint3D64 getRoadBoundaryFirstPointRelGroup(HadLaneGroup* pGroup, HadLaneBoundary* pBoundary);
		MapPoint3D64 getRoadBoundaryLastPointRelGroup(HadLaneGroup* pGroup, HadRoadBoundary* pBoundary);
		MapPoint3D64 getRoadBoundaryLastPointRelGroup(HadLaneGroup* pGroup, HadLaneBoundary* pBoundary);

		void saveRdsRoad(RdsRoad* pRoad, std::vector<LineString3d>& originRoadLines, bool isRoundabout);
    };
}

