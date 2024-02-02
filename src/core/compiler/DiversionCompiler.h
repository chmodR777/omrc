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
         // ����������LA��ϵ
         void connectLaneGroupAssociations(HadRelLaneGroupAssociation* lgAssociation, 
             HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, bool* applyDiversion,
             std::vector<HadRelLaneGroupAssociation*>& allLaneGroupAssociations, std::vector<HadRelLaneGroupAssociation*>& laneGroupAssociations);
         // �Ƿ�ΪX�͵�����UTurnLA����
         bool isXUTurnLA(HadRelLaneGroupAssociation* lgAssociation, HadRelLaneGroupAssociation* pLA);
         // ����LA��ϵ������
		 std::vector<HadRelLaneGroupAssociation*>::iterator findLgAssociationIter(
             HadRelLaneGroupAssociation* lgAssociation, HadLaneGroup* firstLaneGroup, HadLaneGroup* secondLaneGroup,
             HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, std::vector<HadRelLaneGroupAssociation*>& allLaneGroupAssociations, bool* applyDiversion);
         // �ϲ�������
         void mergeFillArea(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby);
         void compileFillArea(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* const pTile, 
             std::vector<rdsLineInfo>& allLines, rtree_type_segment& rtree);
         void compileDiversion(RdsTile* const pTile, Diversion& diversion, 
             std::vector<rdsLineInfo>& allLines, rtree_type_segment& rtree);
         bool getNearestSegments(const point_t& currPt, const segment_t& segment, const std::vector<segment_t>& segments,
             const std::set<size_t>& segmentIds, std::vector<size_t>& nearestSegmentIds, point_t& nearestPt);
		 // ��ȡ�����������ڵ������,�ų�excludeIndex��
		 static void getPolygonPoints(std::vector<MapPoint3D64>& polygonVertexes, std::vector<MapPoint3D64>& points, int excludeIndex);
         void adjustPoint(int ptIndex, MapPoint3D64& pt, std::vector<MapPoint3D64>& polygonVertexes, point_t& toPt);
         // ��������������ĳ�����
         void adjustDiversion(std::vector<MapPoint3D64>& vertexes, std::vector<rdsLineInfo>& allLines, rtree_type_segment& rtree);
         void adjustDiversionZValue(const std::vector<HadLaneGroup*>& laneGroups, std::vector<MapPoint3D64>& vertexes, std::vector<Triangle>& polyTriangles);
         // ���˼����Ե�ͷ�ֿ���ֱ�е�UTurnLA
         void filterUTurnLA(std::vector<HadRelLaneGroupAssociation*>& laneGroupAssociations);
         // ͨ���������Ƿ��������������ϲ�����Diversion��ʶ
         void applyDiversionForFillArea(HadFillArea* const pFillArea,const std::vector<HadGrid*>& grids);
         void getNearbyFillAreas(HadFillArea* const pFillArea, const std::vector<HadGrid*>& grids, std::vector<HadFillArea*>& nearbyFillAreas);
		 // ������ʼ���յ㹹��box,������BOX������ľ���
		 inline ring_t getSegmentBox(const segment_t& pabSeg, double offset);

         // �������Ƿ��복�����ཻ
         bool hasIntersect(HadFillArea* const pFillArea, const HadLaneGroup* laneGroup);
		 // �������Ƿ��ڳ�������
		 bool onLaneGroup(HadFillArea* const pFillArea, const HadLaneGroup* laneGroup);
         // ͨ��LA�Ƿ�ѹ�浼��������Diversion��ʶ
         void applyDiversionForFillArea(HadRelLaneGroupAssociation* const lgAssociation, HadLaneGroup* const pLaneGroup, 
             std::vector<std::pair<LineString3d, double>>& laneDistance, ClipperLib::Path& laPath, std::vector<HadFillArea*>& laPathApplyedFillAreas);
         // ����LA��ϵ���ɵĳ�������
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
 
