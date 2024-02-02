 #pragma once
 #include "Compiler.h"
#include "../framework/Spinlock.h"
#include "clipper.hpp"
 namespace OMDB
 {
     class DiversionCompiler : public Compiler
     {
         friend class DiversionCreator;
         using HadRelLaneGroupAssociation = HadLaneGroupAssociation::HadRelLaneGroupAssociation;
     public:
         class Diversion;
         DiversionCompiler(CompilerData& data) :Compiler(data), m_surfaceRTree2T(nullptr){};
     protected:
         virtual void compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile) override;
         std::set<HadLaneGroup*> getCrossGridGroups(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby);
         void processCrossGridLine(HadGrid* const pGrid, std::set<HadLaneGroup*>& crossGridGroups, std::vector<rdsLineInfo>& allLines);
         void processCrossGridRoad(std::set<HadLaneGroup*>& crossGridGroups);

         void compileLaneGroupAssociations(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* const pTile,
             std::vector<HadRelLaneGroupAssociation*>& allLaneGroupAssociations, 
             std::vector<rdsLineInfo>& allLines, rtree_type_segment& rtree);
		 void compileRoadFace(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* const pTile,
			 std::vector<HadRelLaneGroupAssociation*>& allLaneGroupAssociations, 
             std::vector<rdsLineInfo>& allLines, rtree_type_segment& rtree);
		 bool containsCurrentGrid(HadGrid* const pGrid, std::vector<HadRelLaneGroupAssociation*>& laneGroupAssociations);
		 bool containsCurrentGrid(HadGrid* const pGrid, HadFillArea* const pFillArea);
         // 按拓扑连接LA关系
         void connectLaneGroupAssociations(HadRelLaneGroupAssociation* lgAssociation, 
             HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, bool* applyDiversion,
             std::vector<HadRelLaneGroupAssociation*>& allLaneGroupAssociations, std::vector<HadRelLaneGroupAssociation*>& laneGroupAssociations);
         // 是否为X型的两个UTurnLA相连
         bool isXUTurnLA(HadRelLaneGroupAssociation* lgAssociation, HadRelLaneGroupAssociation* pLA);
         // 查找LA关系迭代器
		 std::vector<HadRelLaneGroupAssociation*>::iterator findLgAssociationIter(
             HadRelLaneGroupAssociation* lgAssociation, HadLaneGroup* firstLaneGroup, HadLaneGroup* secondLaneGroup,
             HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, std::vector<HadRelLaneGroupAssociation*>& allLaneGroupAssociations, bool* applyDiversion);
         // 合并导流区
         void mergeFillArea(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby);
         void compileFillArea(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* const pTile, 
             std::vector<rdsLineInfo>& allLines, rtree_type_segment& rtree);
         void compileDiversion(RdsTile* const pTile, Diversion& diversion, 
             std::vector<rdsLineInfo>& allLines, rtree_type_segment& rtree);
         bool getNearestSegments(const point_t& currPt, const segment_t& segment, const std::vector<segment_t>& segments,
             const std::set<size_t>& segmentIds, std::vector<size_t>& nearestSegmentIds, point_t& nearestPt);
		 // 获取导流区面所在的坐标点,排除excludeIndex点
		 static void getPolygonPoints(std::vector<MapPoint3D64>& polygonVertexes, std::vector<MapPoint3D64>& points, int excludeIndex);
         void adjustPoint(int ptIndex, MapPoint3D64& pt, std::vector<MapPoint3D64>& polygonVertexes, point_t& toPt);
         // 导流带拉到最近的车道线
         void adjustDiversion(std::vector<MapPoint3D64>& vertexes, std::vector<rdsLineInfo>& allLines, rtree_type_segment& rtree);
         void adjustDiversionZValue(const std::vector<HadLaneGroup*>& laneGroups, std::vector<MapPoint3D64>& vertexes, std::vector<Triangle>& polyTriangles);
         // 过滤即可以调头又可以直行的UTurnLA
         void filterUTurnLA(std::vector<HadRelLaneGroupAssociation*>& laneGroupAssociations);
         // 通过导流区是否与其他导流区合并设置Diversion标识
         void applyDiversionForFillArea(HadFillArea* const pFillArea,const std::vector<HadGrid*>& grids);
         void getNearbyFillAreas(HadFillArea* const pFillArea, const std::vector<HadGrid*>& grids, std::vector<HadFillArea*>& nearbyFillAreas);
		 // 根据起始、终点构建box,并返回BOX两个点的距离
		 inline ring_t getSegmentBox(const segment_t& pabSeg, double offset);

         // 导流区是否与车道组相交
         bool hasIntersect(HadFillArea* const pFillArea, const HadLaneGroup* laneGroup);
		 // 导流区是否在车道组上
		 bool onLaneGroup(HadFillArea* const pFillArea, const HadLaneGroup* laneGroup);
         // 通过LA是否压面导流区设置Diversion标识
         void applyDiversionForFillArea(HadRelLaneGroupAssociation* const lgAssociation, HadLaneGroup* const pLaneGroup, 
             std::vector<std::pair<LineString3d, double>>& laneDistance, ClipperLib::Path& laPath, std::vector<HadFillArea*>& laPathApplyedFillAreas);
         // 过滤LA关系生成的超大导流带
         void filterLargeLaDiversion(ClipperLib::Path& laPath, LineString3d& centerLine, std::vector<HadFillArea*>& laPathApplyedFillAreas);
         void groupLaneGroupAssociations(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, std::vector<HadRelLaneGroupAssociation*>& allLaneGroupAssociations, bool* applyDiversion);
         std::string getLaneGroupAssociationId(HadRelLaneGroupAssociation* lgAssociation);

     private:
         static double LA_LANE_DISTANCE_TOLERANCE;
         static double ADJUST_DIVERSION_DISTANCE_TOLERANCE;
         static double ADJUST_DIVERSION_IGNORE_DISTANCE_TOLERANCE;

		 std::vector<segment_t> m_segments;
		 std::vector<std::vector<size_t>> m_sizes;

         std::vector<Triangle> m_surfaceTriangles;
         rtree_type_2box* m_surfaceRTree2T;
     };
 }
 
