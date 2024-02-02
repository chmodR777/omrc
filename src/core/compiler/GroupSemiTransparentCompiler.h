#pragma once
#include "Compiler.h"
namespace OMDB
{
	/// <summary>
    /// 车道组半透明
    /// </summary>
    class GroupSemiTransparentCompiler : public Compiler
    {
	public:
        GroupSemiTransparentCompiler(CompilerData& data) :Compiler(data) {};
    protected:
        bool isCrossGrid(HadLaneGroup* pGroup);
		bool containsCurrentGrid(std::vector<HadSkeleton*>& groups, HadGrid* const pGrid);
        virtual void compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile) override;

        void getConnectedGroups(HadLaneGroup* pLaneGroup, std::vector<std::vector<HadLaneGroup*>>& allConnectedGroups, MultiPoint3d& semiTransparentPoints);
        // 车道组连接
        void connectGroup(HadLaneGroup* pLaneGroup, HadLaneGroup* pTopoGroup, 
            std::vector<HadLaneGroup*>& connectedGroups, double& leftBoundaryDistance, double& rightBoundaryDistance, bool forward);
        // 车道组集裁减
        void clipGroups(RdsTile* pTile, HadLaneGroup* pLaneGroup, 
            std::vector<HadLaneGroup*>& connectedGroups, MultiPoint3d& semiTransparentPoints);
        // 车道组裁减
        bool clipGroup(RdsTile* pTile, HadLaneGroup* pLaneGroup, HadLaneGroup* connectedGroup, MultiPoint3d& semiTransparentPoints,
            double& leftBoundaryDistance, double& rightBoundaryDistance, LineString3d& leftSide, LineString3d& rightSide);
        // 设置半透明点
        void setupSemiTransparentPoints(RdsGroup* pGroup, MultiPoint3d& semiTransparentPoints, int32 semiTransparentLength);
		// 删除半透明处不渲染的对象
		void deleteSemiTransparentObjects(RdsTile* pTile, HadLaneGroup* pLaneGroup, RdsGroup* pGroup, MultiPoint3d& semiTransparentPoints);
        bool containsMultiDigitized(HadLaneGroup* g, std::set<HadLaneGroup*>& groups, HadLaneGroup* semiTransparentGroup);
        double getLaRoadFaceDistance(MultiLineString3d& refLine, MultiPoint3d& semiTransparentPoints);
        bool isRoadFaceCoveredByDiversion(RdsGroup* pGroup, Polygon3d& laPolygon);
        void buildRoadFacePolygon(MultiLineString3d refLine, Polygon3d& polygon);

        void clipGroup(RdsTile* pTile, RdsGroup* pGroup, RdsGroup* newGroup, LineString3d& leftSide, LineString3d& rightSide);

        // 裁减各对象
        void clipPier(RdsTile* pTile, RdsGroup* pGroup, RdsGroup* newGroup, RdsPier* pPier, Polygon3d& polygon);
        void clipSpeedLimitBoard(RdsTile* pTile, RdsGroup* pGroup, RdsGroup* newGroup, RdsSpeedLimitBoard* pSpeedLimitBoard, Polygon3d& polygon);
        void clipToll(RdsTile* pTile, RdsGroup* pGroup, RdsGroup* newGroup, RdsToll* pToll, Polygon3d& polygon);

        void clipLine(RdsTile* pTile, RdsGroup* pGroup, RdsGroup* newGroup, RdsLine* pLine, LineString3d& leftSide, LineString3d& rightSide);
        void clipGuardrail(RdsTile* pTile, RdsGroup* pGroup, RdsGroup* newGroup, RdsGuardrail* pGuardrail, LineString3d& leftSide, LineString3d& rightSide);
        void clipRoad(RdsTile* pTile, RdsGroup* pGroup, RdsGroup* newGroup, RdsRoad* pRoad, LineString3d& leftSide, LineString3d& rightSide);
        void clipGreenbelt(RdsTile* pTile, RdsGroup* pGroup, RdsGroup* newGroup, RdsGreenbelt* pGreenbelt, LineString3d& leftSide, LineString3d& rightSide);
        void clipTunnel(RdsTile* pTile, RdsGroup* pGroup, RdsGroup* newGroup, RdsTunnel* pTunnel, LineString3d& leftSide, LineString3d& rightSide);

        void clipText(RdsTile* pTile, RdsGroup* pGroup, RdsGroup* newGroup, RdsText* pText, Polygon3d& polygon);
        void clipMarking(RdsTile* pTile, RdsGroup* pGroup, RdsGroup* newGroup, RdsMarking* pMarking, Polygon3d& polygon);
        void clipDiversion(RdsTile* pTile, RdsGroup* pGroup, RdsGroup* newGroup, RdsDiversion* pDiversion, Polygon3d& polygon);
        void clipIntersection(RdsTile* pTile, RdsGroup* pGroup, RdsGroup* newGroup, RdsIntersection* pIntersection, Polygon3d& polygon);

        bool clipLineString(LineString3d& refLine, LineString3d& src, LineString3d& dst);
        bool hasIntersect(std::vector<MapPoint3D64>& points1, std::vector<MapPoint3D64>& points2);

		// 合并车道组边界
		void mergeGroupBoundary(std::vector<HadLaneGroup*>& connectedGroups, LineString3d& leftSide, LineString3d& rightSide);
        // 构建半透明点
        MultiPoint3d constructSemiTransparentPoints(HadLaneGroup* pLaneGroup);

        std::vector<HadLaneGroup*> getTopoGroups(HadLaneGroup* pLaneGroup, bool forward);
        double calcBoundaryDistance(std::vector<MapPoint3D64>& points);
        bool isSelfIntersect(LineString3d& lineString);
	private:
        static double SEMI_TRANSPARENT_CLIP_DISTANCE_TOLERANCE;
        static double SEMI_TRANSPARENT_DISTANCE_TOLERANCE;
        static double BOUNDARY_DISTANCE_TOLERANCE;
        static double BOUNDARY_ZVALUE_TOLERANCE;

        // 是否是主路半透明车道组
        bool m_mainRoadSemiTransparent = false;
    };
}

