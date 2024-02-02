#include "stdafx.h"
#include <mutex>
#include <algorithm>
#include "DiversionCompiler.h"
#include "../framework/SpatialSeacher.h"
#include "math3d/vector_math.h"
#include "clipper.hpp"
#include "polyline_intersector.h"
#include "algorithm/grap_point_algorithm.h"
#include "algorithm/poly2tri.h"
#include "algorithm/linear_interpolation_triangle_surface.h"

#define PROJECT_TOLERANCE 100
#define HAD_GRID_NDS_LEVEL	13
#define MAX_HEIGHT 3000
namespace OMDB
{
	class DiversionCreator;
	class DiversionCompiler::Diversion
	{
		friend class DiversionCreator;
		friend class DiversionCompiler;
	public:
		Diversion(HadGrid* grid, HadFillArea* fillArea): m_grid(grid), m_fillArea(fillArea) {
			if (m_fillArea != nullptr) {
				for (auto laneGroup : m_fillArea->laneGroups) {
					m_laneGroups.push_back(laneGroup);
				}
			}
		}
		~Diversion();

	private:
		// 所属网格
		HadGrid* m_grid;
		// 数据侧导流带
		HadFillArea* m_fillArea;
		// 导流带关联的车道组
		std::vector<HadLaneGroup*> m_laneGroups;
		// 周边LA关系
		std::vector<HadRelLaneGroupAssociation*> m_lgAssociations;
		// 导流带可能生成多个小的导流带,每一个都有小导流带都有对应的边
		std::vector<std::vector<LineString3d>> m_edgesList;
		// 存储各个小导流带扩张边界之后对应的边
		std::vector<std::vector<LineString3d>> m_expandEdgesList;
		// 存储LA关系生成的各个小导流带边及距离
		std::vector<std::vector<std::pair<LineString3d, double>>> m_laLaneDistance;
		// 是否是三角岛
		bool isGore() { return !m_lgAssociations.empty() || (m_fillArea != nullptr && m_fillArea->isGore);}
	};

	DiversionCompiler::Diversion::~Diversion()
	{
	}

	class DiversionCreator
	{
		using HadRelLaneGroupAssociation = HadLaneGroupAssociation::HadRelLaneGroupAssociation;
	public:
		DiversionCreator(
			DiversionCompiler* compiler,
			DiversionCompiler::Diversion* diversion):m_compiler(compiler),m_diversion(diversion) {}
		void generateRoadFace(std::vector<std::pair<LineString3d, LineString3d>>& laEdges);
		void generateContour(std::vector<Polygon3d>& polygons);
		void generateReferenceLine(std::vector<LineString3d>& centerReferenceLines);
		bool mergeDiversion(DiversionCompiler::Diversion* nearbyDiversion);
	private:
		// 使用待合并的导流带拆边
		void splitEdges(Polygon3d& polygon, Polygon3d& nearbyPolygon, 
			std::vector<LineString3d>& edges, LineString3d*& fitEdge, bool& insertFitPt);
		// 判断polygon面上的pt能不能拟合到nearbyPolygon上的点,如果能拟合上去,意味着该点属于重合边上的点
		bool fitPointInLine(int currIndex, Polygon3d& polygon, Polygon3d& nearbyPolygon, 
			std::vector<std::pair<MapPoint3D64, MapPoint3D64>>& fitPts, bool& insertFitPt);
		void adjustPointsSequence(Polygon3d& polygon, Polygon3d& nearbyPolygon, bool& insertFitPt);

		// 用导流带拆边
		void splitEdges(std::vector<MapPoint3D64>& polygonVertexes, 
			bool addMinAngleEdge, std::vector<LineString3d>& edges);
		void splitBigEdge(std::vector<LineString3d>& edges, int edgeIndex, double edgeDistance);

		// 三角岛,用LA关系连边
		void connectEdges();
		// 是否首尾都是相连的
		bool isSelfConnected(LineString3d& firstLaneEdge, LineString3d& secondLaneEdge);
		// 收集附近的车道边界
		void collectBoundaryLaneGroups(
			std::map<int64, HadLaneBoundary*>& nearbyBoundarys,
			std::map<HadLaneBoundary*, HadLaneGroup*>& boundaryLaneGroups);
		// 按拓扑连接车道边界
		void connectLaneBoundarys(HadLaneBoundary* boundary, HadLaneBoundary* filterBoundary, 
			std::map<int64, HadLaneBoundary*>& nearbyBoundarys, std::vector<HadLaneBoundary*>& connectedBoundarys);
		// 按拓扑连接公共车道边界
		void connectCommonLaneBoundarys(std::vector<HadLaneBoundary*>& firstLaneBoundarys, 
			std::vector<HadLaneBoundary*>& secondLaneBoundarys, std::map<HadLaneBoundary*, HadLaneGroup*>& boundaryLaneGroups);
		// 连接车道边界
		LineString3d connectEdge(std::map<HadLaneBoundary*, HadLaneGroup*>& boundaryLaneGroups, std::vector<HadLaneBoundary*>& laneBoundarys);
		// 修复车道边界长度
		void fixLaneDistance(LineString3d& firstLaneEdge, LineString3d& secondLaneEdge);
		// 适配两边车道边界长度
		void createNearbyLaneBoundary(
			std::vector<MapPoint3D64>& points, 
			std::vector<MapPoint3D64>& nearbyPoints, 
			std::vector<MapPoint3D64>& laneBoundaryPoints);
		// 是否连接到intersection
		bool connectIntersection(LineString3d& laneEdge, LineString3d& oppositeLaneEdge, MapPoint3D64& intersectPt, std::vector<HadLaneBoundary*>& laneBoundarys);
		void connectOverlapping(LineString3d& laneEdge, LineString3d& oppositeLaneEdge, MapPoint3D64& intersectPt, std::vector<HadLaneBoundary*>& laneBoundarys);
		// 三角岛,延伸到护栏
		void connectGuardrail(LineString3d& laneEdge, LineString3d& oppositeLaneEdge, MapPoint3D64& intersectPt, std::vector<HadLaneBoundary*>& laneBoundarys);
		void getConnectedBoundary(LineString3d& laneEdge, LineString3d& oppositeLaneEdge, MapPoint3D64& intersectPt, 
			std::vector<HadLaneBoundary*>& laneBoundarys, HadLaneBoundary*& connectedLaneBoundary, HadRoadBoundary*& connectedRoadBoundary);
		void connectToGuardrail(LineString3d& laneEdge, HadLaneBoundary* laneBoundary, HadRoadBoundary* roadBoundary, bool isNext);
		// 扩张边界
		void expandEdges();
		// 三角岛用LA关系建面
		void buildEdges(std::vector<Polygon3d>& polygons);
		void buildEdge(std::vector<LineString3d>& edges, Polygon3d& polygon);
		void expandLaneDistancePair(std::vector<std::pair<LineString3d*, double>>& laneDistance, std::vector<LineString3d*>& edges);
		void sortFillAreaLaneDistance(std::vector<std::pair<LineString3d*, double>>& laneDistance, 
			std::vector<HadLaneGroup*>& laneGroups, std::vector<HadRelLaneGroupAssociation*>& overedLgAssociations);
		// 计算两条线段的最小夹角
		double getAngle(std::vector<MapPoint3D64>& laneVertexes, std::vector<MapPoint3D64>& nearbyLaneBoundaryPoints);
		// 延长车道边界长度,参与中心参考线计算
		void extendLaneDistance(std::pair<LineString3d*, double>& firstLaneDistancePair, std::pair<LineString3d*, double>& secondLaneDistancePair);
		// 按百分比采集线上的点
		void grapPointByPercentage(LineString3d* edge, std::vector<MapPoint3D64>& vertexes, int vertexSize, double totalDistance = 0);
		// 计算边的长度
		double calcLaneDistance(std::vector<MapPoint3D64>& points);
		// 获取最近的车道边界
		void getNearbyLaneBoundary(std::vector<HadLaneGroup*>& laneGroups, std::vector<MapPoint3D64>& laneVertexes, HadLaneBoundary*& nearbyLaneBoundary);
		double getNearbyLaneBoundary(std::vector<MapPoint3D64>& laneVertexes, std::vector<HadLaneBoundary*>& laneBoundaries, HadLaneBoundary*& nearbyLaneBoundary);
		std::vector<double> getSectionsForPoints(std::vector<MapPoint3D64>& points, double& boundaryLength);
		std::vector<double> getWeightsForPoints(std::vector<MapPoint3D64>& points, double& boundaryLength);
		bool getPointOnLineByWeight(std::vector<MapPoint3D64>& points, std::vector<double>& weights,
			double& weight, MapPoint3D64& point, size_t& si, double& distanceToVertex);
		MapPoint3D64 getLaneBoundaryVector(std::vector<MapPoint3D64>& vertexes);
		uint16 minimalDegree(double angle);
		uint16 toDegree(double angle);

	private:
		static int ROAD_BOUNDARY_PA_NAME;
		static int LINK_SEPARATION_LEFT_PA_NAME;
		static int LINK_SEPARATION_RIGHT_PA_NAME;
		static double LANE_BOUNDARY_DISTANCE_TOLERANCE;
		static double OVERLAPPING_DISTANCE_TOLERANCE;
		static double MAX_OVERLAPPING_DISTANCE_TOLERANCE;
		static double CONNECT_TO_GUARDRAIL_DISTANCE_TOLERANCE;

		DiversionCompiler* m_compiler;
		DiversionCompiler::Diversion* m_diversion;
	};

	int DiversionCreator::ROAD_BOUNDARY_PA_NAME = 7;
	int DiversionCreator::LINK_SEPARATION_LEFT_PA_NAME = 18;
	int DiversionCreator::LINK_SEPARATION_RIGHT_PA_NAME = 19;
	double DiversionCreator::OVERLAPPING_DISTANCE_TOLERANCE = 5000;
	double DiversionCreator::MAX_OVERLAPPING_DISTANCE_TOLERANCE = 8000;
	double DiversionCreator::LANE_BOUNDARY_DISTANCE_TOLERANCE = 20000;
	double DiversionCreator::CONNECT_TO_GUARDRAIL_DISTANCE_TOLERANCE = 30000;

	forceinline double getIntersectionPoint(
		std::vector<MapPoint3D64>& firstEdgeVertexes,
		std::vector<MapPoint3D64>& secondEdgeVertexes,
		MapPoint3D64& intersectionPt)
	{
		double minDistance = DBL_MAX;
		for (auto& firstVertex : firstEdgeVertexes) {
			int secondIdx = GrapPointAlgorithm::findNearestPoint(secondEdgeVertexes, firstVertex);
			auto& secondVertex = secondEdgeVertexes[secondIdx];
			if (firstVertex == secondVertex) {
				intersectionPt = firstVertex;
				return 0;
			}

			double distance = firstVertex.pos.distance(secondVertex.pos);
			if (distance < minDistance) {
				intersectionPt = firstVertex;
				minDistance = distance;
			}
		}
		return minDistance;
	}

	void DiversionCreator::generateRoadFace(std::vector<std::pair<LineString3d, LineString3d>>& laEdges)
	{
		// 收集附近的车道边界
		std::map<int64, HadLaneBoundary*> nearbyBoundarys;
		std::map<HadLaneBoundary*, HadLaneGroup*> boundaryLaneGroups;
		collectBoundaryLaneGroups(nearbyBoundarys, boundaryLaneGroups);

		// 数据侧的多个LA关系可能生成一条道路,这里需要选出还未生成路面的LA关系
		auto pickupLinkGroupAssociation = [&](std::vector<HadRelLaneGroupAssociation*> lgAssociations) -> HadRelLaneGroupAssociation* {
			HadRelLaneGroupAssociation* retLgAssociation = nullptr;
			double minDistance = DBL_MAX;
			for (auto lgAssociation : lgAssociations) {
				int64 firstOriginId = lgAssociation->firstLaneBoundary->originId;
				int64 secondOriginId = lgAssociation->secondLaneBoundary->originId;
				if (nearbyBoundarys.find(firstOriginId) != nearbyBoundarys.end()
					&& nearbyBoundarys.find(secondOriginId) != nearbyBoundarys.end()) {
					MapPoint3D64 intersectionPt = {};
					auto& firstLaneEdge = lgAssociation->firstLaneBoundary->location;
					auto& secondLaneEdge = lgAssociation->secondLaneBoundary->location;
					double distance = getIntersectionPoint(firstLaneEdge.vertexes, secondLaneEdge.vertexes, intersectionPt);
					if (distance < minDistance) {
						retLgAssociation = lgAssociation;
						minDistance = distance;
					}
				}
			}
			return retLgAssociation;
		};

		HadRelLaneGroupAssociation* lgAssociation = nullptr;
		while ((lgAssociation = pickupLinkGroupAssociation(m_diversion->m_lgAssociations)) != nullptr)
		{
			std::vector<LineString3d> laEdge;
			HadLaneBoundary* firstLaneBoundary = lgAssociation->firstLaneBoundary;
			HadLaneBoundary* secondLaneBoundary = lgAssociation->secondLaneBoundary;
			if (firstLaneBoundary->originId > secondLaneBoundary->originId) {
				std::swap(firstLaneBoundary, secondLaneBoundary);
			}

			std::vector<HadLaneBoundary*> firstLaneBoundarys;
			std::vector<HadLaneBoundary*> secondLaneBoundarys;
			connectLaneBoundarys(firstLaneBoundary, secondLaneBoundary, nearbyBoundarys, firstLaneBoundarys);
			connectLaneBoundarys(secondLaneBoundary, firstLaneBoundary, nearbyBoundarys, secondLaneBoundarys);
			// connectCommonLaneBoundarys(firstLaneBoundarys, secondLaneBoundarys, boundaryLaneGroups);

			LineString3d firstLaneEdge = connectEdge(boundaryLaneGroups, firstLaneBoundarys);
			LineString3d secondLaneEdge = connectEdge(boundaryLaneGroups, secondLaneBoundarys);

			MapPoint3D64 firstLaneVec = getLaneBoundaryVector(firstLaneEdge.vertexes);
			MapPoint3D64 secondLaneVec = getLaneBoundaryVector(secondLaneEdge.vertexes);
			if (dot(firstLaneVec, secondLaneVec) > 0) {
				fixLaneDistance(firstLaneEdge, secondLaneEdge);

				// 首尾不都相连时进行扩张
				if (!isSelfConnected(firstLaneEdge, secondLaneEdge)) {
					MapPoint3D64 intersectionPt = {};
					getIntersectionPoint(firstLaneEdge.vertexes, secondLaneEdge.vertexes, intersectionPt);
					bool containsIntersection = connectIntersection(firstLaneEdge, secondLaneEdge, intersectionPt, firstLaneBoundarys)
						|| connectIntersection(secondLaneEdge, firstLaneEdge, intersectionPt, secondLaneBoundarys);

					connectOverlapping(firstLaneEdge, secondLaneEdge, intersectionPt, firstLaneBoundarys);
					connectOverlapping(secondLaneEdge, firstLaneEdge, intersectionPt, secondLaneBoundarys);
					if (!containsIntersection && !isSelfConnected(firstLaneEdge, secondLaneEdge)) {
						connectGuardrail(firstLaneEdge, secondLaneEdge, intersectionPt, firstLaneBoundarys);
						connectGuardrail(secondLaneEdge, firstLaneEdge, intersectionPt, secondLaneBoundarys);
					}
				}

				std::reverse(secondLaneEdge.vertexes.begin(), secondLaneEdge.vertexes.end());
				laEdge.push_back(firstLaneEdge);
				laEdge.push_back(secondLaneEdge);
			}
			else {
				// 先扩展第一条边
				if (!isSelfConnected(firstLaneEdge, secondLaneEdge)) {
					MapPoint3D64 intersectionPt = {};
					std::reverse(secondLaneEdge.vertexes.begin(), secondLaneEdge.vertexes.end());
					getIntersectionPoint(firstLaneEdge.vertexes, secondLaneEdge.vertexes, intersectionPt);
					bool containsIntersection = connectIntersection(firstLaneEdge, secondLaneEdge, intersectionPt, firstLaneBoundarys)
						|| connectIntersection(secondLaneEdge, firstLaneEdge, intersectionPt, secondLaneBoundarys);

					connectOverlapping(firstLaneEdge, secondLaneEdge, intersectionPt, firstLaneBoundarys);
					if (!containsIntersection && !isSelfConnected(firstLaneEdge, secondLaneEdge)) {
						connectGuardrail(firstLaneEdge, secondLaneEdge, intersectionPt, firstLaneBoundarys);
					}
					std::reverse(secondLaneEdge.vertexes.begin(), secondLaneEdge.vertexes.end());
				}

				// 再扩展另一条边
				if (!isSelfConnected(firstLaneEdge, secondLaneEdge)) {
					MapPoint3D64 intersectionPt = {};
					std::reverse(firstLaneEdge.vertexes.begin(), firstLaneEdge.vertexes.end());
					getIntersectionPoint(firstLaneEdge.vertexes, secondLaneEdge.vertexes, intersectionPt);
					bool containsIntersection = connectIntersection(firstLaneEdge, secondLaneEdge, intersectionPt, firstLaneBoundarys)
						|| connectIntersection(secondLaneEdge, firstLaneEdge, intersectionPt, secondLaneBoundarys);

					connectOverlapping(secondLaneEdge, firstLaneEdge, intersectionPt, secondLaneBoundarys);
					if (!containsIntersection && !isSelfConnected(firstLaneEdge, secondLaneEdge)) {
						connectGuardrail(secondLaneEdge, firstLaneEdge, intersectionPt, secondLaneBoundarys);
					}
					std::reverse(firstLaneEdge.vertexes.begin(), firstLaneEdge.vertexes.end());
				}

				laEdge.push_back(firstLaneEdge);
				laEdge.push_back(secondLaneEdge);
			}

			MapPoint3D64 prevVertex = {};
			std::vector<MapPoint3D64> laEdgeVertexes;
			for (auto& edge : laEdge) {
				for (auto& vertex : edge.vertexes) {
					if (!floatEqual(vertex.pos.distanceSquare(prevVertex.pos), 0)) {
						laEdgeVertexes.push_back(vertex);
						prevVertex = vertex;
					}
				}
			}

			// 掉头的地方可能连成一条边,此时需要拆边
			std::vector<LineString3d> edges;
			splitEdges(laEdgeVertexes, false, edges);
			if (edges.size() == 2) {
				LineString3d& leftSide = edges[0];
				LineString3d& rightSide = edges[1];
				std::reverse(rightSide.vertexes.begin(), rightSide.vertexes.end());
				laEdges.push_back(std::make_pair(leftSide, rightSide));
			}
			else
			{
				LineString3d& leftSide = firstLaneEdge;
				LineString3d& rightSide = secondLaneEdge;
				std::reverse(rightSide.vertexes.begin(), rightSide.vertexes.end());
				laEdges.push_back(std::make_pair(leftSide, rightSide));
			}
		}
	}

	void DiversionCreator::generateContour(std::vector<Polygon3d>& polygons)
	{
		if (!m_diversion->m_lgAssociations.empty()) {
			connectEdges();
		} else {
			std::vector<LineString3d> edges;
			splitEdges(m_diversion->m_fillArea->polygon.vertexes, true, edges);
			m_diversion->m_edgesList.push_back(edges);
		}

		expandEdges();
		
		return buildEdges(polygons);
	}

	void DiversionCreator::generateReferenceLine(std::vector<LineString3d>& centerReferenceLines)
	{
		auto generateCenterPoint = [](MapPoint3D64& firstPoint, MapPoint3D64& secondPoint)->MapPoint3D64 {
			MapPoint3D64 pt = { 0 };
			pt.pos.lon += firstPoint.pos.lon;
			pt.pos.lat += firstPoint.pos.lat;
			pt.z += firstPoint.z;

			pt.pos.lon += secondPoint.pos.lon;
			pt.pos.lat += secondPoint.pos.lat;
			pt.z += secondPoint.z;

			pt.pos.lon /= 2;
			pt.pos.lat /= 2;
			pt.z /= (int32)2;
			return pt;
		};

		for (auto& edges : m_diversion->m_expandEdgesList) {
			std::vector<std::pair<LineString3d*, double>> laneDistance;
			for (auto& edge : edges) {
				double edgeDistance = calcLaneDistance(edge.vertexes);
				laneDistance.push_back(std::pair<LineString3d*, double>(&edge, edgeDistance));
			}

			// 优先选择与车道边界线平行的边
			std::vector<HadRelLaneGroupAssociation*> overedLgAssociations;
			if (m_diversion->m_fillArea != nullptr) {
				overedLgAssociations = m_diversion->m_fillArea->overedLgAssociations;
			}
			sortFillAreaLaneDistance(laneDistance, m_diversion->m_laneGroups, overedLgAssociations);

			// 可能是一个圆弧导流区
			if (laneDistance.size() < 2)
				continue;
			
			auto laneDistanceIter = laneDistance.begin();
			auto& firstLaneDistancePair = *laneDistanceIter;
			auto& secondLaneDistancePair = *(laneDistanceIter + 1);
			if (m_diversion->isGore()) {
				extendLaneDistance(firstLaneDistancePair, secondLaneDistancePair);
			}

			std::vector<MapPoint3D64> firstLaneVertexes;
			std::vector<MapPoint3D64> secondLaneVertexes;
			int firstLaneVertexeSize = firstLaneDistancePair.first->vertexes.size();
			int secondLaneVertexeSize = secondLaneDistancePair.first->vertexes.size();
			int vertexSize = max(firstLaneVertexeSize, secondLaneVertexeSize);
			grapPointByPercentage(firstLaneDistancePair.first, firstLaneVertexes, vertexSize, firstLaneDistancePair.second);
			grapPointByPercentage(secondLaneDistancePair.first, secondLaneVertexes, vertexSize, secondLaneDistancePair.second);

			HadLaneBoundary* nearbyLaneBoundary = nullptr;
			getNearbyLaneBoundary(m_diversion->m_laneGroups, firstLaneVertexes, nearbyLaneBoundary);
			if (nearbyLaneBoundary == nullptr) {
				getNearbyLaneBoundary(m_diversion->m_laneGroups, secondLaneVertexes, nearbyLaneBoundary);
			}

			if (nearbyLaneBoundary != nullptr) {
				MapPoint3D64 firstLaneVec = getLaneBoundaryVector(firstLaneVertexes);
				MapPoint3D64 secondLaneVec = getLaneBoundaryVector(secondLaneVertexes);
				MapPoint3D64 firstLaneBoundaryVec = getLaneBoundaryVector(nearbyLaneBoundary->location.vertexes);
				auto vectorDotProduct1 = dot(firstLaneBoundaryVec, firstLaneVec);
				auto vectorDotProduct2 = dot(firstLaneBoundaryVec, secondLaneVec);
				if (vectorDotProduct1 > 0 && vectorDotProduct2 <= 0) {
					std::reverse(secondLaneVertexes.begin(), secondLaneVertexes.end());
				}
				else if (vectorDotProduct2 > 0 && vectorDotProduct1 <= 0) {
					std::reverse(firstLaneVertexes.begin(), firstLaneVertexes.end());
				}
				else {
					int64 fillAreaId = m_diversion->m_fillArea ? m_diversion->m_fillArea->originId : 0;
// 					printf("VectorDotProduct error %lld::%lld\n", m_diversion->m_grid->originId, fillAreaId);
				}
			} else if (m_diversion->m_laneGroups.empty()){
				if (GrapPointAlgorithm::isOnLeft(firstLaneVertexes, secondLaneVertexes)) {
					std::reverse(firstLaneVertexes.begin(), firstLaneVertexes.end());
				} else {
					std::reverse(secondLaneVertexes.begin(), secondLaneVertexes.end());
				}
			}

			// 按比例生成对应参看线点
			LineString3d centerReferenceLine;
			for (int idx = 0; idx < firstLaneVertexes.size(); idx++) {
				MapPoint3D64 centerPoint = generateCenterPoint(firstLaneVertexes[idx], secondLaneVertexes[idx]);
				centerReferenceLine.vertexes.push_back(centerPoint);
			}
			centerReferenceLines.push_back(centerReferenceLine);
		}
	}

	bool DiversionCreator::mergeDiversion(DiversionCompiler::Diversion* nearbyDiversion)
	{
		auto polygon = m_diversion->m_fillArea->polygon;
		auto nearbyPolygon = nearbyDiversion->m_fillArea->polygon;
		if (polygon.vertexes.empty() || nearbyPolygon.vertexes.empty())
			return false;

		bool insertFitPt = false;
		LineString3d* fitEdge = nullptr;
		std::vector<LineString3d> edges;
		splitEdges(polygon, nearbyPolygon, edges, fitEdge, insertFitPt);

		LineString3d* nearbyFitEdge = nullptr;
		std::vector<LineString3d> nearbyEdges;
		splitEdges(nearbyPolygon, polygon, nearbyEdges, nearbyFitEdge, insertFitPt);
		if (insertFitPt && nearbyFitEdge != nullptr) {
			fitEdge = nullptr;
			splitEdges(polygon, nearbyPolygon, edges, fitEdge, insertFitPt);
		}

		std::vector<LineString3d> mergedEdges;
		if (fitEdge != nullptr && nearbyFitEdge != nullptr) {
			auto edgeIdx = std::find_if(edges.begin(), edges.end(),
				[&](auto& e) {return &e == fitEdge; }) - edges.begin();
			auto nearbyEdgeIdx = std::find_if(nearbyEdges.begin(), nearbyEdges.end(),
				[&](auto& e) {return &e == nearbyFitEdge; }) - nearbyEdges.begin();
			for (int i = 0; i < edges.size(); i++) {
				auto& edge = edges[i];
				if (i != edgeIdx) {
					mergedEdges.push_back(edge);
				}
				else
				{
					for (int j = nearbyEdgeIdx + 1; j < nearbyEdges.size(); j++) {
						auto& nearbyEdge = nearbyEdges[j];
						mergedEdges.push_back(nearbyEdge);
					}
					for (int j = 0; j < nearbyEdgeIdx; j++) {
						auto& nearbyEdge = nearbyEdges[j];
						mergedEdges.push_back(nearbyEdge);
					}
				}
			}

			// 数据侧导流区有重复
			if (mergedEdges.empty()) {
				return false;
			}

			// 平滑连接点
			for (int i = 0; i < mergedEdges.size(); i++) 
			{
				auto& currEdge = mergedEdges[i];
				auto& nextEdge = (i != mergedEdges.size() - 1) ? mergedEdges[i + 1] : mergedEdges[0];
				auto& currEndPt = currEdge.vertexes.back();
				auto& nextStartPt = nextEdge.vertexes.front();
				currEndPt = nextStartPt = min(currEndPt, nextStartPt);
			}

			// 合并导流带
			Polygon3d newPolygon;
			ClipperLib::Path newPolygonPath;
			buildEdge(mergedEdges, newPolygon);
			for (auto& vertex : newPolygon.vertexes)
				newPolygonPath << ClipperLib::IntPoint(vertex.pos.lon, vertex.pos.lat, vertex.z);
			if (ClipperLib::Area(newPolygonPath) == 0) {
				return false; // 合并导流区后剩一条线
			}

			// 合并车道组
			m_diversion->m_fillArea->polygon = newPolygon;
			auto& laneGroups = m_diversion->m_fillArea->laneGroups;
			m_diversion->m_fillArea->isGore |= nearbyDiversion->m_fillArea->isGore;
			auto& overedLgAssociations = m_diversion->m_fillArea->overedLgAssociations;
			for (auto& overedLgAssociation : nearbyDiversion->m_fillArea->overedLgAssociations) {
				if (std::find(overedLgAssociations.begin(), overedLgAssociations.end(), overedLgAssociation) == overedLgAssociations.end())
					overedLgAssociations.push_back(overedLgAssociation);
			}
			for (auto& nearbyLaneGroup : nearbyDiversion->m_fillArea->laneGroups) {
				if (std::find(laneGroups.begin(), laneGroups.end(), nearbyLaneGroup) == laneGroups.end())
					laneGroups.push_back(nearbyLaneGroup);
			}
			m_diversion->m_laneGroups = laneGroups;
			return true;
		}
		else if (fitEdge != nullptr || nearbyFitEdge != nullptr)
		{
			//printf("d");
		}
		return false;
	}

	MapPoint3D64 DiversionCreator::getLaneBoundaryVector(std::vector<MapPoint3D64>& vertexes) {
		MapPoint3D64 vec = {};
		for (int idx = 0; idx < vertexes.size() - 1; idx++) {
			auto& currPt = vertexes[idx];
			auto& nextPt = vertexes[idx + 1];
			MapPoint3D64 tmpVec = nextPt - currPt;
			vec = vec + tmpVec;
		}
		return vec;
	}

	void DiversionCreator::extendLaneDistance(std::pair<LineString3d*, double>& firstLaneDistancePair, std::pair<LineString3d*, double>& secondLaneDistancePair)
	{
		auto extendPoint = [](MapPoint3D64& endPt, MapPoint3D64& prevPt, double extendDistance)->MapPoint3D64 {
			DVector2 v1;
			v1.x = endPt.pos.lon - prevPt.pos.lon;
			v1.y = endPt.pos.lat - prevPt.pos.lat;
			if (v1.x == 0 && v1.y == 0) {
				return endPt;
			}
			v1.normalize();

			DVector2 p = v1 * extendDistance;
			p.x += endPt.pos.lon;
			p.y += endPt.pos.lat;

			MapPoint3D64 extendedPt;
			extendedPt.pos.lon = p.x;
			extendedPt.pos.lat = p.y;

			extendedPt.z = endPt.z + (endPt.z - prevPt.z) / v1.length();
			return extendedPt;
		};

		if (firstLaneDistancePair.second < secondLaneDistancePair.second)
			std::swap(firstLaneDistancePair, secondLaneDistancePair);
		double extendDistance = firstLaneDistancePair.second - secondLaneDistancePair.second;
		double extendRate = extendDistance / secondLaneDistancePair.second;
		if (extendRate > 0.1) {
			MapPoint3D64 intersectPt = {};
			getIntersectionPoint(secondLaneDistancePair.first->vertexes, firstLaneDistancePair.first->vertexes, intersectPt);
			int vertexIdx = GrapPointAlgorithm::findNearestPoint(secondLaneDistancePair.first->vertexes, intersectPt);
			auto& vertex = secondLaneDistancePair.first->vertexes[vertexIdx];
			if (vertex == *secondLaneDistancePair.first->vertexes.begin()) {
				int endPtIdx = secondLaneDistancePair.first->vertexes.size() - 1;
				MapPoint3D64& endPt = secondLaneDistancePair.first->vertexes[endPtIdx];
				MapPoint3D64& prevPt = secondLaneDistancePair.first->vertexes[endPtIdx - 1];

				MapPoint3D64 extendedPt = extendPoint(endPt, prevPt, extendDistance);
				secondLaneDistancePair.second += extendedPt.pos.distance(endPt.pos);
				secondLaneDistancePair.first->vertexes.push_back(extendedPt);
			}
			else if (vertex == *secondLaneDistancePair.first->vertexes.rbegin()) {
				MapPoint3D64& endPt = secondLaneDistancePair.first->vertexes[0];
				MapPoint3D64& prevPt = secondLaneDistancePair.first->vertexes[1];

				MapPoint3D64 extendedPt = extendPoint(endPt, prevPt, extendDistance);
				secondLaneDistancePair.second += extendedPt.pos.distance(endPt.pos);
				secondLaneDistancePair.first->vertexes.emplace(secondLaneDistancePair.first->vertexes.begin(), extendedPt);
			}
		}
	}

	void DiversionCreator::grapPointByPercentage(LineString3d* edge, std::vector<MapPoint3D64>& vertexes, int vertexSize, double totalDistance)
	{
		if (totalDistance == 0) {
			totalDistance = calcLaneDistance(edge->vertexes);
		}
		std::deque<MapPoint3D64> edgeVertexes(edge->vertexes.begin(), edge->vertexes.end());
		double currDistance = 0;
		while (edgeVertexes.size() > 1) {
			auto edgeIter = edgeVertexes.begin();
			auto& currPt = *edgeIter;
			auto& nextPt = *(edgeIter + 1);
			double distance = currPt.pos.distance(nextPt.pos);
			double percentageDistance = totalDistance * (vertexes.size() + 1) / vertexSize;
			if ((currDistance + distance) <= percentageDistance) {
				currDistance += distance;
				edgeVertexes.pop_front();
			}
			else
			{
				DVector2 v1;
				v1.x = nextPt.pos.lon - currPt.pos.lon;
				v1.y = nextPt.pos.lat - currPt.pos.lat;
				v1.normalize();

				double diffDistance = percentageDistance - currDistance;
				DVector2 p = v1 * diffDistance;
				p.x += currPt.pos.lon;
				p.y += currPt.pos.lat;

				MapPoint3D64 grappedPt;
				grappedPt.pos.lon = p.x;
				grappedPt.pos.lat = p.y;

				grappedPt.z = currPt.z + (nextPt.z - currPt.z) * diffDistance / distance;
				vertexes.push_back(grappedPt);

				currDistance += diffDistance;
				edgeVertexes.pop_front();
				edgeVertexes.push_front(grappedPt);
			}

			if (edgeVertexes.size() == 1 && vertexes.size() < vertexSize) {
				vertexes.push_back(nextPt);
			}
			if (currDistance > totalDistance) {
				break;
			}
		}

		vertexes.insert(vertexes.begin(), *edge->vertexes.begin());
		vertexes.back() = edge->vertexes.back();
	}

	double DiversionCreator::calcLaneDistance(std::vector<MapPoint3D64>& points)
	{
		double distance = 0;
		for (int idx = 0; idx < points.size() - 1; idx++) {
			auto& currPt = points[idx];
			auto& nextPt = points[idx + 1];
			distance += currPt.pos.distance(nextPt.pos);
		}
		return distance;
	}

	void DiversionCreator::getNearbyLaneBoundary(std::vector<HadLaneGroup*>& laneGroups, std::vector<MapPoint3D64>& laneVertexes, HadLaneBoundary*& nearbyLaneBoundary) {
		double minDistance = DBL_MAX;
		for (HadLaneGroup* laneGroup : laneGroups) {
			HadLaneBoundary* laneBoundary = nullptr;
			double distance = getNearbyLaneBoundary(laneVertexes, laneGroup->laneBoundaries, laneBoundary);
			if (distance < minDistance) {
				nearbyLaneBoundary = laneBoundary;
				minDistance = distance;
			}
		}
	}

	double DiversionCreator::getNearbyLaneBoundary(std::vector<MapPoint3D64>& laneVertexes, std::vector<HadLaneBoundary*>& laneBoundaries, HadLaneBoundary*& nearbyLaneBoundary)
	{
		double minDistance = DBL_MAX;
		std::vector<double> avgDistances(laneBoundaries.size());
		std::vector<int> grappedPoints(laneBoundaries.size());
		std::vector<int> tmpMinSi(laneBoundaries.size(), INT_MAX);
		std::vector<int> tmpMaxEi(laneBoundaries.size(), INT_MIN);
		for (int i = 0; i < laneBoundaries.size(); i++) {
			HadLaneBoundary* nearbyBoundary = laneBoundaries[i];
			for_each(laneVertexes.begin(), laneVertexes.end(),
				[&](const MapPoint3D64& point)->void {
					size_t si, ei;
					MapPoint3D64 minGrappedPt = {};
					GrapPointAlgorithm::grapOrMatchNearestPoint(point, nearbyBoundary->location.vertexes, minGrappedPt, si, ei);
					if (minGrappedPt.pos.lon != 0 && minGrappedPt.pos.lat != 0) {
						Vector2 v;
						v.x = (float)(point.pos.lon - minGrappedPt.pos.lon);
						v.y = (float)(point.pos.lat - minGrappedPt.pos.lat);
						double distance = v.length();
						avgDistances[i] += distance;
						grappedPoints[i] += 1;
						tmpMinSi[i] = min(tmpMinSi[i], (int)si);
						tmpMaxEi[i] = max(tmpMaxEi[i], (int)ei);
					}
				}
			);
		}

		for (int i = 0; i < laneBoundaries.size(); i++) {
			if (grappedPoints[i] == 0) {
				continue;
			}

			avgDistances[i] /= grappedPoints[i];
			if (avgDistances[i] < minDistance) {
				minDistance = avgDistances[i];
				nearbyLaneBoundary = laneBoundaries[i];
			}
		}
		return minDistance;
	}

	forceinline std::vector<double> DiversionCreator::getSectionsForPoints(std::vector<MapPoint3D64>& points, double& boundaryLength)
	{
		std::vector<double> sections;
		boundaryLength = 0.0;
		for (size_t i = 0; i < points.size() - 1; i++)
		{
			// double sl = points[i].pos.distance(points[i + 1].pos);
			double sl = GrapPointAlgorithm::geoLengthD(points[i].pos.toNdsPoint(), points[i + 1].pos.toNdsPoint());
			boundaryLength += sl;
			sections.push_back(sl);
		}
		sections.push_back(0);
		for (size_t i = 0; i < points.size(); i++)
			sections[i] = sections[i] / boundaryLength;
		return sections;
	}

	forceinline std::vector<double> DiversionCreator::getWeightsForPoints(std::vector<MapPoint3D64>& points, double& boundaryLength)
	{
		std::vector<double> weights;
		std::vector<double> sections = getSectionsForPoints(points, boundaryLength);
		double tmpWeight = 0.0;
		for (size_t i = 0; i < sections.size() - 1; i++)
		{
			tmpWeight += sections[i];
			weights.push_back(tmpWeight);
		}
		weights.push_back(0);
		return weights;
	}

	bool DiversionCreator::getPointOnLineByWeight(std::vector<MapPoint3D64>& points, std::vector<double>& weights, 
		double& weight, MapPoint3D64& point, size_t& si, double& distanceToVertex) {
		if (weight < 0.0 || weight > 1.0)
			return false;
		double pSection = 0.0;
		size_t si_t = 0;
		find_if(weights.begin(), weights.end(), [&](double w) {
			pSection = w;
			if (pSection > weight)
			{
				if (pSection > 1.0)
					pSection = 1.0;
				if (weight == 1)
					si_t++;
				return true;
			}
			si_t++;
			return false;
			});
		si_t = min(si_t, points.size() - 1);
		if (pSection > weight)
		{
			double diffOffset = pSection - weight;
			double l;
			if (si_t == 0)
			{
				l = 1 - diffOffset / weights[si_t];
			}
			else
			{
				l = 1 - diffOffset / (weights[si_t] - weights[si_t - 1]);
			}
			DVector3 v1 = dvec3(points[si_t].pos.lon, points[si_t].pos.lat, points[si_t].z);
			DVector3 v2 = dvec3(points[si_t + 1].pos.lon, points[si_t + 1].pos.lat, points[si_t + 1].z);
			DVector3 vs = v2 - v1;
			vs = vs * l;
			vs = vs + v1;
			MapPoint3D64 samplePoint{ MapPoint64{(INT64)vs.x,(INT64)vs.y},(INT64)vs.z };
			point = samplePoint;
			distanceToVertex = l;
		}
		else {
			point = points[si_t];
			distanceToVertex = 0;
		}
		si = si_t;
		return true;
	}

	void DiversionCreator::splitEdges(Polygon3d& polygon, Polygon3d& nearbyPolygon, 
		std::vector<LineString3d>& edges, LineString3d*& fitEdge, bool& insertFitPt)
	{
		edges.clear();
		LineString3d edge;
		insertFitPt = false;
		bool containsFitPoint = false;
		std::vector<size_t> fitIndices;
		std::vector<std::pair<MapPoint3D64, MapPoint3D64>> fitPts;

		adjustPointsSequence(polygon, nearbyPolygon, insertFitPt);
		for (int idx = 0; idx < polygon.vertexes.size(); idx++) {
			auto& currPt = polygon.vertexes[idx];
			if (fitPointInLine(idx, polygon, nearbyPolygon, fitPts, insertFitPt)) {
				if (!containsFitPoint && idx != 0) {
					if (!edge.vertexes.empty()) {
						edge.vertexes.push_back(currPt);
						edges.push_back(edge);
						edge = {};
					}
				}

				containsFitPoint = true;
				edge.vertexes.push_back(currPt);
				if (idx == polygon.vertexes.size() - 1) {
					if (edge.vertexes.size() > 1) {
						edges.push_back(edge);
						fitIndices.push_back(edges.size() - 1);
					}
				}
			} else {
				if (containsFitPoint) {
					if (edge.vertexes.size() > 1) {
						edges.push_back(edge);
						fitIndices.push_back(edges.size() - 1);
						auto& backPt = edge.vertexes.back();

						edge = {};
						edge.vertexes.push_back(backPt);
					} else if (edge.vertexes.size() == 1 && !fitPts.empty()){
						fitPts.pop_back();
					}
				}

				containsFitPoint = false;
				edge.vertexes.push_back(currPt);
				if (idx == polygon.vertexes.size() - 1) {
					if (edge.vertexes.size() > 1) {
						edges.push_back(edge);
					}
				}
			}
		}

		// 只处理一条边重合的
		if (fitIndices.size() == 1) {
			fitEdge = &edges[fitIndices.front()];
		}
	}

	void DiversionCreator::adjustPointsSequence(Polygon3d& polygon, Polygon3d& nearbyPolygon, bool& insertFitPt)
	{
		int loopCount = 0;
		insertFitPt = false;
		bool endPtFit = false;
		do {
			loopCount++;
			endPtFit = false;
			int idx = polygon.vertexes.size() - 1;
			std::vector<std::pair<MapPoint3D64, MapPoint3D64>> fitPts;
			if (fitPointInLine(idx, polygon, nearbyPolygon, fitPts, insertFitPt)) {
				endPtFit = true;
				auto endIter = polygon.vertexes.end() - 1;
				auto beginPt = *(endIter - 1);
				polygon.vertexes.pop_back();
				polygon.vertexes.insert(polygon.vertexes.begin(), beginPt);
			}
		} while (endPtFit && loopCount < polygon.vertexes.size());
	}

	bool DiversionCreator::fitPointInLine(int currIndex, Polygon3d& polygon, Polygon3d& nearbyPolygon, 
		std::vector<std::pair<MapPoint3D64, MapPoint3D64>>& fitPts, bool& insertFitPt)
	{
		size_t si, ei;
		const float ZVALUE_EPSILON = 300.f;  // ≈30cm
		const float POINT_IN_LINE_EPSILON = 600.f;  // ≈60cm
		const float FIT_POINT_IN_LINE_EPSILON = 1000.f;  // ≈1m
		auto checkFitAngle = [&](MapPoint3D64& pt, MapPoint3D64& grappedPt)->bool {
			if (fitPts.empty())
				return true;

			auto& prevFitPair = fitPts.back();
			NdsPoint prevNdsPt = prevFitPair.first.pos.toNdsPoint();
			NdsPoint prevGrappedNdsPt = prevFitPair.second.pos.toNdsPoint();

			NdsPoint ndsPt = pt.pos.toNdsPoint();
			NdsPoint grappedNdsPt = grappedPt.pos.toNdsPoint();
			
			// 计算前后fitPt平行度,夹角小于15°阈值认为是平行的
			float direction = toDegree(Math_getDirectionFromTwoCoordinatesNds(ndsPt, prevNdsPt));
			float grappedDirection = toDegree(Math_getDirectionFromTwoCoordinatesNds(grappedNdsPt, prevGrappedNdsPt));
			auto directionDelta = fabs(direction - grappedDirection);
			auto minAngle = minimalDegree(directionDelta);
			return minAngle < 15;
		};

		// 首先判断抓点是否fit
		MapPoint3D64 minGrappedPt = {};
		MapPoint3D64 currPt = polygon.vertexes[currIndex];
		if (GrapPointAlgorithm::grapOrMatchNearestPoint(currPt, nearbyPolygon.vertexes, minGrappedPt, si, ei) &&
			floatEqualWithEpsilon(currPt.pos.distance(minGrappedPt.pos), 0, POINT_IN_LINE_EPSILON) && 
			fabs(currPt.z - minGrappedPt.z) < ZVALUE_EPSILON && checkFitAngle(currPt, minGrappedPt)) {
			int nearbyPointIdx = GrapPointAlgorithm::findNearestPoint(nearbyPolygon.vertexes, minGrappedPt);
			auto& nearestPoint = nearbyPolygon.vertexes[nearbyPointIdx];
			if (!floatEqualWithEpsilon(minGrappedPt.pos.distance(nearestPoint.pos), 0, POINT_IN_LINE_EPSILON)) {
				auto insertIter = mapPoint3D64_iterator(nearbyPolygon.vertexes, nearbyPolygon.vertexes[ei]);
				nearbyPolygon.vertexes.insert(insertIter, minGrappedPt);
				insertFitPt = true;
			}

			// 判断是否最近点
			MapPoint3D64 intersectPt = {};
			std::vector<MapPoint3D64> points;
			DiversionCompiler::getPolygonPoints(polygon.vertexes, points, currIndex);
			if (PolylineIntersector::intersect(currPt, minGrappedPt, points, 10, intersectPt, si, ei)) {
				if (minGrappedPt.pos.distance(intersectPt.pos) < minGrappedPt.pos.distance(currPt.pos)) {
					return false;
				}
			} 
			MapPoint3D64 polygonPt = {};
			if (GrapPointAlgorithm::grapOrMatchNearestPoint(minGrappedPt, polygon.vertexes, polygonPt, si, ei)) {
				if (PolylineIntersector::intersect(currPt, polygonPt, points, 10, intersectPt, si, ei)) {
					if (polygonPt.pos.distance(intersectPt.pos) < polygonPt.pos.distance(currPt.pos)) {
						return false;
					}
				}
			}
			fitPts.push_back(std::make_pair(currPt, minGrappedPt));
			return true;
		}

		// 可能抓到对侧点,此时需要选择最近点
		int nearbyPointIdx = GrapPointAlgorithm::findNearestPoint(nearbyPolygon.vertexes, currPt);
		auto nearestPoint = nearbyPolygon.vertexes[nearbyPointIdx];
		auto currDistance = currPt.pos.distance(nearestPoint.pos);
		if (currPt.pos.distance(minGrappedPt.pos) > currDistance)
			minGrappedPt = nearestPoint;
		if (minGrappedPt.pos.lon == 0 && minGrappedPt.pos.lat == 0)
			minGrappedPt = nearestPoint;

		// 最近点距超过fit距离,不需要接着判断
		if (!floatEqualWithEpsilon(currPt.pos.distance(nearestPoint.pos), 0, FIT_POINT_IN_LINE_EPSILON)) {
			return false;
		}

		// 当前点不是对侧最近点时,不需要接着判断
		int prevIndex = currIndex != 0 ? currIndex - 1 : polygon.vertexes.size() - 2;
		int nextIndex = currIndex != polygon.vertexes.size() - 1 ? currIndex + 1 : 1;
		MapPoint3D64 prevPt = polygon.vertexes[prevIndex];
		MapPoint3D64 nextPt = polygon.vertexes[nextIndex];
		auto prevDistance = prevPt.pos.distance(nearestPoint.pos);
		auto nextDistance = nextPt.pos.distance(nearestPoint.pos);
		if (prevDistance < currDistance || nextDistance < currDistance) {
			return false;
		}

		// 判断是否最近点
		MapPoint3D64 intersectPt = {};
		std::vector<MapPoint3D64> points;
		DiversionCompiler::getPolygonPoints(polygon.vertexes, points, currIndex);
		if (PolylineIntersector::intersect(currPt, minGrappedPt, points, 10, intersectPt, si, ei)) {
			if (minGrappedPt.pos.distance(intersectPt.pos) < minGrappedPt.pos.distance(currPt.pos)) {
				return false;
			}
		}
		MapPoint3D64 polygonPt = {};
		if (GrapPointAlgorithm::grapOrMatchNearestPoint(minGrappedPt, polygon.vertexes, polygonPt, si, ei)) {
			if (PolylineIntersector::intersect(currPt, polygonPt, points, 10, intersectPt, si, ei)) {
				if (polygonPt.pos.distance(intersectPt.pos) < polygonPt.pos.distance(currPt.pos)) {
					return false;
				}
			}
		}

		// 用nearbyPolygon上的抓点或最近点判断在本方polygon上抓点是否fit
		if (GrapPointAlgorithm::grapOrMatchNearestPoint(nearestPoint, polygon.vertexes, minGrappedPt, si, ei) &&
			floatEqualWithEpsilon(nearestPoint.pos.distance(minGrappedPt.pos), 0, POINT_IN_LINE_EPSILON) && 
			fabs(nearestPoint.z - minGrappedPt.z) < ZVALUE_EPSILON && checkFitAngle(currPt, minGrappedPt)) {
			fitPts.push_back(std::make_pair(currPt, nearestPoint));
			return true;
		}

		return false;
	}

	void DiversionCreator::splitEdges(std::vector<MapPoint3D64>& polygonVertexes, 
			bool addMinAngleEdge, std::vector<LineString3d>& edges) {
		auto makeSingleEdge = [](MapPoint3D64& vertex)->LineString3d {
			LineString3d edge;
			edge.vertexes.push_back(vertex);
			edge.vertexes.push_back(vertex);
			return edge;
		};

		edges.clear();
		LineString3d edge;
		int maxEdgeIndex = -1;
		double totalDistance = 0, maxEdgeDistance = 0;
		float firstDirection = 0, prevDirection = 0;
		int endIdx = polygonVertexes.size() - 1;
		for (int idx = 0; idx < endIdx; idx++) {
			auto& currPt = polygonVertexes[idx];
			auto& nextPt = polygonVertexes[idx + 1];
			totalDistance += currPt.pos.distance(nextPt.pos);

			NdsPoint ndsStartPt = currPt.pos.toNdsPoint();
			NdsPoint ndsEndPt = nextPt.pos.toNdsPoint();
			float direction = toDegree(Math_getDirectionFromTwoCoordinatesNds(ndsStartPt, ndsEndPt));
			if (idx == 0) {
				edge.vertexes.push_back(currPt);
				firstDirection = direction;
				prevDirection = direction;
				continue;
			}

			auto directionDelta = fabs(direction - prevDirection);
			auto minAngle = minimalDegree(directionDelta);
			if (directionDelta > 30 && directionDelta < 330) {
				edge.vertexes.push_back(currPt);
				edges.push_back(edge);
				double edgeDistance = calcLaneDistance(edge.vertexes);
				if (edgeDistance > maxEdgeDistance) {
					maxEdgeDistance = edgeDistance;
					maxEdgeIndex = edges.size() - 1;
				}
				if (addMinAngleEdge && minAngle < 30) {
					edges.push_back(makeSingleEdge(currPt));
				}

				edge = {};
				edge.vertexes.push_back(currPt);
			}
			else
			{
				edge.vertexes.push_back(currPt);
			}

			if (idx == endIdx - 1) {
				edge.vertexes.push_back(nextPt);
				edges.push_back(edge);
				double edgeDistance = calcLaneDistance(edge.vertexes);
				if (edgeDistance > maxEdgeDistance) {
					maxEdgeDistance = edgeDistance;
					maxEdgeIndex = edges.size() - 1;
				}
			}
			prevDirection = direction;
		}

		// 首尾角度小的需要连接起来
		auto directionDelta = fabs(firstDirection - prevDirection);
		auto minAngle = minimalDegree(directionDelta);
		if (directionDelta > 30 && directionDelta < 330 
				&& addMinAngleEdge && minAngle < 30) {
			auto& endPt = polygonVertexes[endIdx];
			edges.push_back(makeSingleEdge(endPt));
		}

		if (edges.size() > 1 && (directionDelta < 30 || directionDelta > 330)) {
			LineString3d& startLine = edges.front();
			LineString3d& endLine = edges.back();
			MapPoint3D64 prevVertex = startLine.vertexes.front();
			for_each(endLine.vertexes.rbegin(), endLine.vertexes.rend(),
				[&](MapPoint3D64& vertex)->void {
					float lengthSquared = vertex.pos.distanceSquare(prevVertex.pos);
					if (!floatEqual(lengthSquared, 0)) {
						startLine.vertexes.insert(startLine.vertexes.begin(), vertex);
						prevVertex = vertex;
					}
				}
			);
			edges.pop_back();
			double edgeDistance = calcLaneDistance(startLine.vertexes);
			if (edgeDistance > maxEdgeDistance) {
				maxEdgeDistance = edgeDistance;
				maxEdgeIndex = 0;
			}
		}

		// 拆分圆弧形的导流带
		if (maxEdgeDistance / totalDistance > 0.8) {
			splitBigEdge(edges, maxEdgeIndex, maxEdgeDistance);
		}
	}

	void DiversionCreator::splitBigEdge(std::vector<LineString3d>& edges, int edgeIndex, double edgeDistance)
	{
		auto getEdgeIter = [](std::vector<LineString3d>& edges, LineString3d& edge) {
			return std::find_if(edges.begin(), edges.end(), [&](LineString3d& e) {
				return &e == &edge;
			});
		};

		LineString3d edge;
		double distance = 0;
		std::vector<LineString3d> splitedEdges;
		LineString3d& bigEdge = edges[edgeIndex];
		int endIdx = bigEdge.vertexes.size() - 1;
		for (int idx = 0; idx < endIdx; idx++) {
			auto& currPt = bigEdge.vertexes[idx];
			auto& nextPt = bigEdge.vertexes[idx + 1];
			distance += currPt.pos.distance(nextPt.pos);
			if (distance / edgeDistance > 0.5) {
				edge.vertexes.push_back(currPt);
				splitedEdges.push_back(edge);
				
				edge.vertexes.clear();
				distance = 0;
			}

			edge.vertexes.push_back(currPt);
			if (idx == endIdx - 1) {
				edge.vertexes.push_back(nextPt);
				splitedEdges.push_back(edge);
			}
		}

		auto insertIter = getEdgeIter(edges, bigEdge);
		for (auto& splitedEdge : splitedEdges) {
			insertIter = edges.insert(insertIter + 1, splitedEdge);
		}
		
		auto edgeIter = getEdgeIter(edges, edges[edgeIndex]);
		edges.erase(edgeIter);
	}

	void DiversionCreator::connectEdges()
	{
		// 收集附近的车道边界
		std::map<int64, HadLaneBoundary*> nearbyBoundarys;
		std::map<HadLaneBoundary*, HadLaneGroup*> boundaryLaneGroups;
		collectBoundaryLaneGroups(nearbyBoundarys, boundaryLaneGroups);

		// 数据侧的导流带可能生成多个小的导流带,这里需要选出还未生成小导流带的LA关系
		auto pickupLinkGroupAssociation = [&](std::vector<HadRelLaneGroupAssociation*> lgAssociations) -> HadRelLaneGroupAssociation* {
			HadRelLaneGroupAssociation* retLgAssociation = nullptr;
			double minDistance = DBL_MAX;
			for (auto lgAssociation : lgAssociations) {
				int64 firstOriginId = lgAssociation->firstLaneBoundary->originId;
				int64 secondOriginId = lgAssociation->secondLaneBoundary->originId;
				if (nearbyBoundarys.find(firstOriginId) != nearbyBoundarys.end()
					&& nearbyBoundarys.find(secondOriginId) != nearbyBoundarys.end()) {
					MapPoint3D64 intersectionPt = {};
					auto& firstLaneEdge = lgAssociation->firstLaneBoundary->location;
					auto& secondLaneEdge = lgAssociation->secondLaneBoundary->location;
					double distance = getIntersectionPoint(firstLaneEdge.vertexes, secondLaneEdge.vertexes, intersectionPt);
					if (distance < minDistance) {
						retLgAssociation = lgAssociation;
						minDistance = distance;
					}
				}
			}
			return retLgAssociation;
		};

		HadRelLaneGroupAssociation* lgAssociation = nullptr;
		while ((lgAssociation = pickupLinkGroupAssociation(m_diversion->m_lgAssociations)) != nullptr) 
		{
			std::vector<LineString3d> laEdges;
			HadLaneBoundary* firstLaneBoundary = lgAssociation->firstLaneBoundary;
			HadLaneBoundary* secondLaneBoundary = lgAssociation->secondLaneBoundary;
			if (firstLaneBoundary->originId > secondLaneBoundary->originId) {
				std::swap(firstLaneBoundary, secondLaneBoundary);
			}

			std::vector<HadLaneBoundary*> firstLaneBoundarys;
			std::vector<HadLaneBoundary*> secondLaneBoundarys;
			connectLaneBoundarys(firstLaneBoundary, secondLaneBoundary, nearbyBoundarys, firstLaneBoundarys);
			connectLaneBoundarys(secondLaneBoundary, firstLaneBoundary, nearbyBoundarys, secondLaneBoundarys);

			LineString3d firstLaneEdge = connectEdge(boundaryLaneGroups, firstLaneBoundarys);
			LineString3d secondLaneEdge = connectEdge(boundaryLaneGroups, secondLaneBoundarys);

			MapPoint3D64 firstLaneVec = getLaneBoundaryVector(firstLaneEdge.vertexes);
			MapPoint3D64 secondLaneVec = getLaneBoundaryVector(secondLaneEdge.vertexes);
			if (dot(firstLaneVec, secondLaneVec) > 0) {
				fixLaneDistance(firstLaneEdge, secondLaneEdge);

				// 首尾不都相连时进行扩张
				if (!isSelfConnected(firstLaneEdge, secondLaneEdge)) {
					MapPoint3D64 intersectionPt = {};
					getIntersectionPoint(firstLaneEdge.vertexes, secondLaneEdge.vertexes, intersectionPt);
					bool containsIntersection = connectIntersection(firstLaneEdge, secondLaneEdge, intersectionPt, firstLaneBoundarys)
						|| connectIntersection(secondLaneEdge, firstLaneEdge, intersectionPt, secondLaneBoundarys);

					connectOverlapping(firstLaneEdge, secondLaneEdge, intersectionPt, firstLaneBoundarys);
					connectOverlapping(secondLaneEdge, firstLaneEdge, intersectionPt, secondLaneBoundarys);
					if (!containsIntersection && !isSelfConnected(firstLaneEdge, secondLaneEdge)) {
						connectGuardrail(firstLaneEdge, secondLaneEdge, intersectionPt, firstLaneBoundarys);
						connectGuardrail(secondLaneEdge, firstLaneEdge, intersectionPt, secondLaneBoundarys);
					}
				}

				std::reverse(secondLaneEdge.vertexes.begin(), secondLaneEdge.vertexes.end());
				laEdges.push_back(firstLaneEdge);
				laEdges.push_back(secondLaneEdge);
			} else {
				// 先扩展第一条边
				if (!isSelfConnected(firstLaneEdge, secondLaneEdge)) {
					MapPoint3D64 intersectionPt = {};
					std::reverse(secondLaneEdge.vertexes.begin(), secondLaneEdge.vertexes.end());
					getIntersectionPoint(firstLaneEdge.vertexes, secondLaneEdge.vertexes, intersectionPt);
					bool containsIntersection = connectIntersection(firstLaneEdge, secondLaneEdge, intersectionPt, firstLaneBoundarys)
						|| connectIntersection(secondLaneEdge, firstLaneEdge, intersectionPt, secondLaneBoundarys);

					connectOverlapping(firstLaneEdge, secondLaneEdge, intersectionPt, firstLaneBoundarys);
					if (!containsIntersection && !isSelfConnected(firstLaneEdge, secondLaneEdge)) {
						connectGuardrail(firstLaneEdge, secondLaneEdge, intersectionPt, firstLaneBoundarys);
					}
					std::reverse(secondLaneEdge.vertexes.begin(), secondLaneEdge.vertexes.end());
				}

				// 再扩展另一条边
				if (!isSelfConnected(firstLaneEdge, secondLaneEdge)) {
					MapPoint3D64 intersectionPt = {};
					std::reverse(firstLaneEdge.vertexes.begin(), firstLaneEdge.vertexes.end());
					getIntersectionPoint(firstLaneEdge.vertexes, secondLaneEdge.vertexes, intersectionPt);
					bool containsIntersection = connectIntersection(firstLaneEdge, secondLaneEdge, intersectionPt, firstLaneBoundarys)
						|| connectIntersection(secondLaneEdge, firstLaneEdge, intersectionPt, secondLaneBoundarys);

					connectOverlapping(secondLaneEdge, firstLaneEdge, intersectionPt, secondLaneBoundarys);
					if (!containsIntersection && !isSelfConnected(firstLaneEdge, secondLaneEdge)) {
						connectGuardrail(secondLaneEdge, firstLaneEdge, intersectionPt, secondLaneBoundarys);
					}
					std::reverse(firstLaneEdge.vertexes.begin(), firstLaneEdge.vertexes.end());
				}

				laEdges.push_back(firstLaneEdge);
				laEdges.push_back(secondLaneEdge);
			}

			Polygon3d polygon;
			buildEdge(laEdges, polygon);

			std::vector<LineString3d> edges;
			splitEdges(polygon.vertexes, true, edges);
			m_diversion->m_edgesList.push_back(edges);

			std::vector<std::pair<LineString3d, double>> laneDistance;
			for (auto& laEdge : laEdges) {
				double edgeDistance = calcLaneDistance(laEdge.vertexes);
				laneDistance.push_back(std::pair<LineString3d, double>(laEdge, edgeDistance));
			}
			m_diversion->m_laLaneDistance.push_back(laneDistance);
		}
	}

	bool DiversionCreator::isSelfConnected(LineString3d& firstLaneEdge, LineString3d& secondLaneEdge)
	{
		auto& firstLaneStartPt = firstLaneEdge.vertexes.front();
		auto& firstLaneEndPt = firstLaneEdge.vertexes.back();
		auto& secondLaneStartPt = secondLaneEdge.vertexes.front();
		auto& secondLaneEndPt = secondLaneEdge.vertexes.back();
		if (!((floatEqual(firstLaneStartPt.pos.distance(secondLaneStartPt.pos), 0) &&
			floatEqual(firstLaneEndPt.pos.distance(secondLaneEndPt.pos), 0)) ||
			(floatEqual(firstLaneStartPt.pos.distance(secondLaneEndPt.pos), 0) &&
				floatEqual(firstLaneEndPt.pos.distance(secondLaneStartPt.pos), 0)))) {
			return false;
		}

		return true;
	}

	void DiversionCreator::collectBoundaryLaneGroups(
		std::map<int64, HadLaneBoundary*>& nearbyBoundarys,
		std::map<HadLaneBoundary*, HadLaneGroup*>& boundaryLaneGroups)
	{
		for (auto lgAssociation : m_diversion->m_lgAssociations) {
			int64 firstOriginId = lgAssociation->firstLaneBoundary->originId;
			auto firstIter = nearbyBoundarys.find(firstOriginId);
			if (firstIter != nearbyBoundarys.end() && firstIter->second != lgAssociation->firstLaneBoundary) {
				throw std::runtime_error{ "DiversionCreator::connectEdges exist cycle: " + firstOriginId };
			}
			nearbyBoundarys.emplace(firstOriginId, lgAssociation->firstLaneBoundary);
			boundaryLaneGroups.emplace(lgAssociation->firstLaneBoundary, lgAssociation->first);

			int64 secondOriginId = lgAssociation->secondLaneBoundary->originId;
			auto secondIter = nearbyBoundarys.find(secondOriginId);
			if (secondIter != nearbyBoundarys.end() && secondIter->second != lgAssociation->secondLaneBoundary) {
				throw std::runtime_error{ "DiversionCreator::connectEdges exist cycle: " + secondOriginId };
			}
			nearbyBoundarys.emplace(secondOriginId, lgAssociation->secondLaneBoundary);
			boundaryLaneGroups.emplace(lgAssociation->secondLaneBoundary, lgAssociation->second);
		}
	}

	void DiversionCreator::connectLaneBoundarys(HadLaneBoundary* boundary, HadLaneBoundary* filterBoundary, 
		std::map<int64, HadLaneBoundary*>& nearbyBoundarys, std::vector<HadLaneBoundary*>& connectedBoundarys)
	{
		std::set<HadLaneBoundary*> filterBoundarys{ filterBoundary };
		auto getFilterBoundary = [&](HadLaneBoundary* tmpBoundary)->HadLaneBoundary* {
			for (auto lgAssociation : m_diversion->m_lgAssociations) {
				if (lgAssociation->firstLaneBoundary == tmpBoundary) {
					return lgAssociation->secondLaneBoundary;
				}

				if (lgAssociation->secondLaneBoundary == tmpBoundary) {
					return lgAssociation->firstLaneBoundary;
				}
			}
			return nullptr;
		};

		// previous
		HadLaneBoundary* previousBoundary = boundary;
		while (previousBoundary->previous.size() >= 1) {
			bool containsPreviousBoundary = false;
			for (auto& pBoundary : previousBoundary->previous) {
				HadLaneBoundary* tmpBoundary = (HadLaneBoundary*)pBoundary;
				if (filterBoundarys.find(tmpBoundary) != filterBoundarys.end()) {
					continue;
				}

				if (nearbyBoundarys.find(tmpBoundary->originId) != nearbyBoundarys.end()) {
					filterBoundarys.emplace(getFilterBoundary(tmpBoundary));
					previousBoundary = tmpBoundary;
					containsPreviousBoundary = true;
					connectedBoundarys.emplace(connectedBoundarys.begin(), previousBoundary);
					nearbyBoundarys.erase(previousBoundary->originId);
				}
			}
			if (!containsPreviousBoundary) {
				break;
			}
		}

		// current
		connectedBoundarys.push_back(boundary);
		nearbyBoundarys.erase(boundary->originId);

		// next
		HadLaneBoundary* nextBoundary = boundary;
		while (nextBoundary->next.size() >= 1) {
			bool containsNextBoundary = false;
			for (auto& pBoundary : nextBoundary->next) {
				HadLaneBoundary* tmpBoundary = (HadLaneBoundary*)pBoundary;
				if (filterBoundarys.find(tmpBoundary) != filterBoundarys.end()) {
					continue;
				}

				if (nearbyBoundarys.find(tmpBoundary->originId) != nearbyBoundarys.end()) {
					filterBoundarys.emplace(getFilterBoundary(tmpBoundary));
					nextBoundary = tmpBoundary;
					containsNextBoundary = true;
					connectedBoundarys.push_back(nextBoundary);
					nearbyBoundarys.erase(nextBoundary->originId);
				}
			}
			if (!containsNextBoundary) {
				break;
			}
		}
	}

	void DiversionCreator::connectCommonLaneBoundarys(std::vector<HadLaneBoundary*>& firstLaneBoundarys, 
		std::vector<HadLaneBoundary*>& secondLaneBoundarys, std::map<HadLaneBoundary*, HadLaneGroup*>& boundaryLaneGroups)
	{
		auto findLgAssociationIter = [&](HadLaneBoundary* firstLaneBoundary, HadLaneBoundary* secondLaneBoundary)->
			std::vector<HadRelLaneGroupAssociation*>::iterator {
			for (auto iter = m_diversion->m_lgAssociations.begin(); iter != m_diversion->m_lgAssociations.end(); iter++) {
				auto pLA = *iter;
				if ((pLA->firstLaneBoundary == firstLaneBoundary && pLA->secondLaneBoundary == secondLaneBoundary) ||
					(pLA->firstLaneBoundary == secondLaneBoundary && pLA->secondLaneBoundary == firstLaneBoundary)) {
					return iter;
				}
			}
			return m_diversion->m_lgAssociations.end();
		};

		auto getCommonLaneBoundary = [](std::vector<HadSkeleton*>& lhs, std::vector<HadSkeleton*>& rhs)->HadLaneBoundary* {
			std::set<HadSkeleton*> skeletons;
			for (auto lh : lhs) {
				for (auto rh : rhs) {
					if (lh == rh)
						skeletons.emplace(lh);
				}
			}
			if (skeletons.size() == 1) {
				return (HadLaneBoundary*) *skeletons.begin();
			}
			return nullptr;
		};

		auto connectCommonLaneBoundary = [&](HadLaneBoundary* firstLaneBoundary, HadLaneBoundary* secondLaneBoundary, std::vector<HadLaneBoundary*>& laneBoundarys) {
			auto lgAssociationIter = findLgAssociationIter(firstLaneBoundary, secondLaneBoundary);
			if (lgAssociationIter == m_diversion->m_lgAssociations.end())
				return;

			bool addFirst = true;
			auto addIter = std::find(laneBoundarys.begin(), laneBoundarys.end(), firstLaneBoundary);
			if (addIter == laneBoundarys.end()) {
				addFirst = false;
				addIter = std::find(laneBoundarys.begin(), laneBoundarys.end(), secondLaneBoundary);
				if (addIter == laneBoundarys.end()) {
					return;
				}
			}
			
			auto lgAssociation = *lgAssociationIter;
			if (lgAssociation->directType == 2)
			{
				auto previousBoundary = getCommonLaneBoundary(firstLaneBoundary->previous, secondLaneBoundary->previous);
				if (previousBoundary != nullptr && previousBoundary->linkGroups.size() == 1) {
					auto firstLaneIter = std::find(previousBoundary->next.begin(), previousBoundary->next.end(), firstLaneBoundary);
					auto secondLaneIter = std::find(previousBoundary->next.begin(), previousBoundary->next.end(), secondLaneBoundary);
					if (!(firstLaneIter != previousBoundary->next.end() && secondLaneIter != previousBoundary->next.end())) {
						boundaryLaneGroups.emplace(previousBoundary, previousBoundary->linkGroups.begin()->second);
						laneBoundarys.insert(addIter, previousBoundary);
					}
				}

				auto nextBoundary = getCommonLaneBoundary(firstLaneBoundary->next, secondLaneBoundary->next);
				if (nextBoundary != nullptr && nextBoundary->linkGroups.size() == 1) {
					auto firstLaneIter =  std::find(nextBoundary->previous.begin(), nextBoundary->previous.end(), firstLaneBoundary);
					auto secondLaneIter = std::find(nextBoundary->previous.begin(), nextBoundary->previous.end(), secondLaneBoundary);
					if (!(firstLaneIter != nextBoundary->previous.end() && secondLaneIter != nextBoundary->previous.end())) {
						boundaryLaneGroups.emplace(nextBoundary, nextBoundary->linkGroups.begin()->second);
						laneBoundarys.push_back(nextBoundary);
					}
				}
			}
			else if (lgAssociation->directType == 3)
			{
				auto previousBoundary = getCommonLaneBoundary(firstLaneBoundary->previous, secondLaneBoundary->next);
				if (previousBoundary != nullptr && previousBoundary->linkGroups.size() == 1) {
					boundaryLaneGroups.emplace(previousBoundary, previousBoundary->linkGroups.begin()->second);
					if (addFirst) {
						laneBoundarys.insert(addIter, previousBoundary);
					} else {
						laneBoundarys.push_back(previousBoundary);
					}
				}

				auto nextBoundary = getCommonLaneBoundary(firstLaneBoundary->next, secondLaneBoundary->previous);
				if (nextBoundary != nullptr && nextBoundary->linkGroups.size() == 1) {
					boundaryLaneGroups.emplace(nextBoundary, nextBoundary->linkGroups.begin()->second);
					if (addFirst) {
						laneBoundarys.push_back(nextBoundary);
					} else {
						laneBoundarys.insert(addIter, nextBoundary);
					}
				}
			}
		};

		int size = firstLaneBoundarys.size();
		if (firstLaneBoundarys.empty() || size != secondLaneBoundarys.size())
			return;

		if (size == 1) { // 只有一组LA关系时
			connectCommonLaneBoundary(firstLaneBoundarys[0], secondLaneBoundarys[0], firstLaneBoundarys);
			return;
		}

		auto headFirstLaneBoundary = firstLaneBoundarys[0];
		auto headSecondLaneBoundary = secondLaneBoundarys[0];
		auto tailFirstLaneBoundary = firstLaneBoundarys[size - 1];
		auto tailSecondLaneBoundary = secondLaneBoundarys[size - 1];
		if (findLgAssociationIter(headFirstLaneBoundary, headSecondLaneBoundary) != m_diversion->m_lgAssociations.end()) 
		{
			connectCommonLaneBoundary(headFirstLaneBoundary, headSecondLaneBoundary, firstLaneBoundarys);
			connectCommonLaneBoundary(tailFirstLaneBoundary, tailSecondLaneBoundary, secondLaneBoundarys);
		}
		else if (findLgAssociationIter(headFirstLaneBoundary, tailSecondLaneBoundary) != m_diversion->m_lgAssociations.end())
		{
			connectCommonLaneBoundary(headFirstLaneBoundary, tailSecondLaneBoundary, firstLaneBoundarys);
			connectCommonLaneBoundary(tailFirstLaneBoundary, headSecondLaneBoundary, secondLaneBoundarys);
		}
	}

	LineString3d DiversionCreator::connectEdge(std::map<HadLaneBoundary*, HadLaneGroup*>& boundaryLaneGroups,
		std::vector<HadLaneBoundary*>& laneBoundarys)
	{
		LineString3d edge;
		MapPoint3D64 prevVertex = {};
		for (HadLaneBoundary* laneBoundary : laneBoundarys) {
			auto location = laneBoundary->location;
			HadLaneGroup* laneGroup = boundaryLaneGroups[laneBoundary];

			// 车道线线投影到路面
			auto& coordinatesTransform = m_compiler->coordinatesTransform;
			auto& surfaceTriangles = m_compiler->m_surfaceTriangles;
			auto& surfaceRTree2T = *m_compiler->m_surfaceRTree2T;
			std::vector<MapPoint3D64> lineOnRoadSurface;
			LinearInterpolationTriangleSurface::interpolationLine(
				coordinatesTransform, surfaceTriangles, surfaceRTree2T, location.vertexes, lineOnRoadSurface);
			location.vertexes = lineOnRoadSurface;

			for_each(location.vertexes.begin(), location.vertexes.end(), [&](MapPoint3D64 vertex)->void {
				float lengthSquared = vertex.pos.distanceSquare(prevVertex.pos);
				if (!floatEqual(lengthSquared, 0)) {
					edge.vertexes.push_back(vertex);
					prevVertex = vertex;
				}
			});
		}
		return edge;
	}

	void DiversionCreator::fixLaneDistance(LineString3d& firstLaneEdge, LineString3d& secondLaneEdge)
	{
		// 车道边界线长度可能差别很大,此时需要裁减长的那根边界线
		double firstLaneDistance = calcLaneDistance(firstLaneEdge.vertexes);
		double secondLaneDistance = calcLaneDistance(secondLaneEdge.vertexes);
		if (abs(firstLaneDistance - secondLaneDistance) <= CONNECT_TO_GUARDRAIL_DISTANCE_TOLERANCE)
			return;

		if (secondLaneDistance && (firstLaneDistance / secondLaneDistance > 1.5)) {
			std::vector<MapPoint3D64> firstLaneBoundaryPoints;
			std::reverse(firstLaneEdge.vertexes.begin(), firstLaneEdge.vertexes.end());
			createNearbyLaneBoundary(firstLaneEdge.vertexes, secondLaneEdge.vertexes, firstLaneBoundaryPoints);
			if (firstLaneBoundaryPoints.size() > 1) {
				std::reverse(firstLaneBoundaryPoints.begin(), firstLaneBoundaryPoints.end());
				firstLaneEdge.vertexes = firstLaneBoundaryPoints;
			}
			else {
				std::reverse(firstLaneEdge.vertexes.begin(), firstLaneEdge.vertexes.end());
			}
		}
		if (firstLaneDistance && (secondLaneDistance / firstLaneDistance > 1.5)) {
			std::vector<MapPoint3D64> secondLaneBoundaryPoints;
			std::reverse(secondLaneEdge.vertexes.begin(), secondLaneEdge.vertexes.end());
			createNearbyLaneBoundary(secondLaneEdge.vertexes, firstLaneEdge.vertexes, secondLaneBoundaryPoints);
			if (secondLaneBoundaryPoints.size() > 1) {
				std::reverse(secondLaneBoundaryPoints.begin(), secondLaneBoundaryPoints.end());
				secondLaneEdge.vertexes = secondLaneBoundaryPoints;
			}
			else {
				std::reverse(secondLaneEdge.vertexes.begin(), secondLaneEdge.vertexes.end());
			}
		}
	}

	void DiversionCreator::createNearbyLaneBoundary(std::vector<MapPoint3D64>& points, 
		std::vector<MapPoint3D64>& nearbyPoints, std::vector<MapPoint3D64>& laneBoundaryPoints)
	{
		auto grapPoint = [](MapPoint3D64& point, std::vector<MapPoint3D64>& nearbyPoints, MapPoint3D64& nearbyGrappedPt, size_t& si, size_t& ei) -> bool {
			if (GrapPointAlgorithm::grapOrMatchNearestPoint(point, nearbyPoints, nearbyGrappedPt, si, ei) &&
				point.pos.distance(nearbyGrappedPt.pos) < LANE_BOUNDARY_DISTANCE_TOLERANCE) {
				return true;
			}
			return false;
		};

		int startIdx = -1, endIdx = -1;
		for (int idx = 0; idx < points.size(); idx++) {
			size_t si, ei;
			MapPoint3D64& point = points[idx];
			MapPoint3D64 nearbyGrappedPt = {};
			if (grapPoint(point, nearbyPoints, nearbyGrappedPt, si, ei)) {
				startIdx = idx;
				break;
			}
		};

		for (int idx = points.size() - 1; idx >= 0; idx--) {
			size_t si, ei;
			MapPoint3D64& point = points[idx];
			MapPoint3D64 nearbyGrappedPt = {};
			if (grapPoint(point, nearbyPoints, nearbyGrappedPt, si, ei)) {
				endIdx = idx;
				break;
			}
		};

		MapPoint3D64 prevVertex = {};
		if (startIdx != -1 && endIdx != -1) {
			for (int idx = startIdx; idx <= endIdx; idx++) {
				MapPoint3D64& point = points[idx];
				if (!floatEqual(point.pos.distanceSquare(prevVertex.pos), 0)) {
					laneBoundaryPoints.push_back(point);
					prevVertex = point;
				}
			}
		}
		else if (startIdx != -1) {
			if (!floatEqual(points[startIdx].pos.distanceSquare(prevVertex.pos), 0)) {
				laneBoundaryPoints.push_back(points[startIdx]);
				prevVertex = points[startIdx];
			}
		}
		else if (endIdx != -1) {
			if (!floatEqual(points[endIdx].pos.distanceSquare(prevVertex.pos), 0)) {
				laneBoundaryPoints.push_back(points[endIdx]);
				prevVertex = points[endIdx];
			}
		}

		// 补充尾部抓点
		for (int idx = 0; idx < nearbyPoints.size(); idx++) {
			size_t si, ei;
			MapPoint3D64 firstGrappedPt = {};
			MapPoint3D64 firstNearbyPoint = nearbyPoints[idx];
			if (grapPoint(firstNearbyPoint, points, firstGrappedPt, si, ei)) {
				if (si >= endIdx) {
					if (!floatEqual(firstGrappedPt.pos.distanceSquare(prevVertex.pos), 0)) {
						laneBoundaryPoints.push_back(firstGrappedPt);
						prevVertex = firstGrappedPt;
					}
				}
				break;
			}
		}

		// 补充首部抓点
		for (int idx = nearbyPoints.size() - 1; idx >= 0; idx--) {
			size_t si, ei;
			MapPoint3D64 lastGrappedPt = {};
			MapPoint3D64 lastNearbyPoint = nearbyPoints[idx];
			if (grapPoint(lastNearbyPoint, points, lastGrappedPt, si, ei)) {
				if (ei <= startIdx) {
					if (!floatEqual(lastGrappedPt.pos.distanceSquare(prevVertex.pos), 0)) {
						laneBoundaryPoints.emplace(laneBoundaryPoints.begin(), lastGrappedPt);
						prevVertex = lastGrappedPt;
					}
				}
				break;
			}
		}

		if (laneBoundaryPoints.size() == 1)
			laneBoundaryPoints.clear();
	}

	bool DiversionCreator::connectIntersection(LineString3d& laneEdge, LineString3d& oppositeLaneEdge, MapPoint3D64& intersectPt, std::vector<HadLaneBoundary*>& laneBoundarys)
	{
		UNREFERENCED_PARAMETER(oppositeLaneEdge);
		auto inIntersection = [](HadLaneBoundary* laneBoundary)->bool {
			for (auto pGroup : laneBoundary->linkGroups) {
				if (pGroup.second->inIntersection)
					return true;
			}
			return false;
		};

		int vertexIdx = GrapPointAlgorithm::findNearestPoint(laneEdge.vertexes, intersectPt);
		auto& vertex = laneEdge.vertexes[vertexIdx];
		auto& frontLaneEdgePt = laneEdge.vertexes.front();
		auto& backLaneEdgePt = laneEdge.vertexes.back();
		if (vertex == frontLaneEdgePt) {
			HadLaneBoundary* currLaneBoundary = laneBoundarys[laneBoundarys.size() - 1];
			if (inIntersection(currLaneBoundary))
				return true;
			for (auto skeleton : currLaneBoundary->next) {
				if (inIntersection((HadLaneBoundary*)skeleton))
					return true;
			}
		}
		else if (vertex == backLaneEdgePt) {
			HadLaneBoundary* currLaneBoundary = laneBoundarys[0];
			if (inIntersection(currLaneBoundary))
				return true;
			for (auto skeleton : currLaneBoundary->previous) {
				if (inIntersection((HadLaneBoundary*)skeleton))
					return true;
			}
		}
		return false;
	}

	void DiversionCreator::connectOverlapping(LineString3d& laneEdge, LineString3d& oppositeLaneEdge, MapPoint3D64& intersectPt, std::vector<HadLaneBoundary*>& laneBoundarys)
	{
		auto filterBoundary = [&](HadLaneGroup* pGroup)->bool {
			if (pGroup && pGroup->inIntersection)
				return true;
			if (pGroup && DiversionCompiler::isSemiTransparentGroup(pGroup))
				return true;
			return false;
		};

		// 车道边界与道路边界是否重叠
		double laneBoundaryDistance = 0, roadBoundaryDistance = 0;
		auto isOverlapping = [&](HadLaneBoundary* laneBoundary, HadRoadBoundary* roadBoundary)->bool {
			if (laneBoundary->linkGroups.size() == 1 && filterBoundary(laneBoundary->linkGroups.begin()->second))
				return false;
			if (roadBoundary->linkGroups.size() == 1 && filterBoundary(roadBoundary->linkGroups.begin()->second))
				return false;
			if (laneBoundary->filterOverlappingBoundary)
				return false;

			auto& laneBoundaryVertexes = laneBoundary->location.vertexes;
			auto& roadBoundaryVertexes = roadBoundary->location.vertexes;
			int pointNum = min(laneBoundaryVertexes.size(), roadBoundaryVertexes.size());
			laneBoundaryDistance += calcLaneDistance(laneBoundaryVertexes);
			roadBoundaryDistance += calcLaneDistance(roadBoundaryVertexes);
			if (laneBoundaryDistance < OVERLAPPING_DISTANCE_TOLERANCE &&
				roadBoundaryDistance < MAX_OVERLAPPING_DISTANCE_TOLERANCE) {
				return true;
			}

			double grappedDistance = 0;
			for (int idx = 0; idx < pointNum; idx++) {
				size_t si, ei;
				auto& lanePt = laneBoundaryVertexes[idx];
				MapPoint3D64 grappedPt = {};
				GrapPointAlgorithm::grapOrMatchNearestPoint(lanePt, roadBoundaryVertexes, grappedPt, si, ei);
				grappedDistance += grappedPt.pos.distance(lanePt.pos);
				if (grappedDistance > 300) {
					return false;
				}
			}

			if (laneBoundaryDistance > MAX_OVERLAPPING_DISTANCE_TOLERANCE ||
				roadBoundaryDistance > MAX_OVERLAPPING_DISTANCE_TOLERANCE) {
				return false;
			}

			return true;
		};

		// 有一小段重叠的车道边界和道路边界时直接合并该车道边界
		HadLaneBoundary* connectedLaneBoundary = nullptr;
		HadRoadBoundary* connectedRoadBoundary = nullptr;
		getConnectedBoundary(laneEdge, oppositeLaneEdge, intersectPt, laneBoundarys, connectedLaneBoundary, connectedRoadBoundary);
		int vertexIdx = GrapPointAlgorithm::findNearestPoint(laneEdge.vertexes, intersectPt);
		auto& vertex = laneEdge.vertexes[vertexIdx];
		if (vertex == *laneEdge.vertexes.begin()) {
			if (connectedLaneBoundary != nullptr && connectedRoadBoundary != nullptr) {
				if (isOverlapping(connectedLaneBoundary, connectedRoadBoundary)) {
					for (int idx = 0; idx < connectedLaneBoundary->location.vertexes.size(); idx++) {
						auto& lanePoint = connectedLaneBoundary->location.vertexes[idx];
						laneEdge.vertexes.push_back(lanePoint);
					}
					laneBoundarys.push_back(connectedLaneBoundary);
					// connectOverlapping(laneEdge, oppositeLaneEdge, intersectPt, laneBoundarys);
				}
			}
		}
		else if (vertex == *laneEdge.vertexes.rbegin()) {
			if (connectedLaneBoundary != nullptr && connectedRoadBoundary != nullptr) {
				if (isOverlapping(connectedLaneBoundary, connectedRoadBoundary)) {
					for (int idx = connectedLaneBoundary->location.vertexes.size() - 1; idx >= 0; idx--) {
						auto& lanePoint = connectedLaneBoundary->location.vertexes[idx];
						laneEdge.vertexes.emplace(laneEdge.vertexes.begin(), lanePoint);
					}
					laneBoundarys.emplace(laneBoundarys.begin(), connectedLaneBoundary);
					// connectOverlapping(laneEdge, oppositeLaneEdge, intersectPt, laneBoundarys);
				}
			}
		}
	}

	void DiversionCreator::connectGuardrail(LineString3d& laneEdge, LineString3d& oppositeLaneEdge, MapPoint3D64& intersectPt, std::vector<HadLaneBoundary*>& laneBoundarys)
	{
		HadLaneBoundary* connectedLaneBoundary = nullptr;
		HadRoadBoundary* connectedRoadBoundary = nullptr;
		getConnectedBoundary(laneEdge, oppositeLaneEdge, intersectPt, laneBoundarys, connectedLaneBoundary, connectedRoadBoundary);
		int vertexIdx = GrapPointAlgorithm::findNearestPoint(laneEdge.vertexes, intersectPt);
		auto& vertex = laneEdge.vertexes[vertexIdx];
		if (vertex == *laneEdge.vertexes.begin()) {
			HadLaneBoundary* currLaneBoundary = laneBoundarys[laneBoundarys.size() - 1];
			if (connectedLaneBoundary != nullptr && connectedRoadBoundary != nullptr) {
				connectToGuardrail(laneEdge, connectedLaneBoundary, connectedRoadBoundary, true);
			}
		}
		else if (vertex == *laneEdge.vertexes.rbegin()) {
			HadLaneBoundary* currLaneBoundary = laneBoundarys[0];
			if (connectedLaneBoundary != nullptr && connectedRoadBoundary != nullptr) {
				connectToGuardrail(laneEdge, connectedLaneBoundary, connectedRoadBoundary, false);
			}
		}
	}

	void DiversionCreator::getConnectedBoundary(LineString3d& laneEdge, LineString3d& oppositeLaneEdge, MapPoint3D64& intersectPt, 
		std::vector<HadLaneBoundary*>& laneBoundarys, HadLaneBoundary*& connectedLaneBoundary, HadRoadBoundary*& connectedRoadBoundary) {
		auto getNearestRoadBoundary = [&oppositeLaneEdge, &intersectPt](HadLaneBoundary* laneBoundary, MapPoint3D64 laneEdgePt)->HadRoadBoundary* {
			HadRoadBoundary* roadBoundary = nullptr;
			if (laneBoundary->linkGroups.size() == 1) {
				auto laneGroup = laneBoundary->linkGroups[laneBoundary->linkGroups.begin()->first];
				if (laneGroup->roadBoundaries.size() != 2) {
					return roadBoundary;
				}

				MapPoint3D64 lanePoint = laneBoundary->location.vertexes.front();
				MapPoint3D64 lanePoint1 = laneBoundary->location.vertexes.back();
				MapPoint3D64 firstLeftPoint = laneGroup->roadBoundaries[0]->location.vertexes.front();
				MapPoint3D64 secondLeftPoint = laneGroup->roadBoundaries[1]->location.vertexes.front();
				if (lanePoint.pos.distanceSquare(laneEdgePt.pos) > lanePoint1.pos.distanceSquare(laneEdgePt.pos)) {
					lanePoint = laneBoundary->location.vertexes.back();
					firstLeftPoint = laneGroup->roadBoundaries[0]->location.vertexes.back();
					secondLeftPoint = laneGroup->roadBoundaries[1]->location.vertexes.back();
				}

				int oppositeVertexIdx = GrapPointAlgorithm::findNearestPoint(oppositeLaneEdge.vertexes, intersectPt);
				auto oppositeVertex = oppositeLaneEdge.vertexes[oppositeVertexIdx];
				if (oppositeVertex == oppositeLaneEdge.vertexes.front()) {
					oppositeVertex = oppositeLaneEdge.vertexes.back();
				} else if (oppositeVertex == oppositeLaneEdge.vertexes.back()) {
					oppositeVertex = oppositeLaneEdge.vertexes.front();
				}

				auto firstLeftPointDis = firstLeftPoint.pos.distanceSquare(lanePoint.pos);
				firstLeftPointDis += firstLeftPoint.pos.distanceSquare(oppositeVertex.pos);

				auto secondLeftPointDis = secondLeftPoint.pos.distanceSquare(lanePoint.pos);
				secondLeftPointDis += secondLeftPoint.pos.distanceSquare(oppositeVertex.pos);
				if (firstLeftPointDis < secondLeftPointDis) {
					roadBoundary = laneGroup->roadBoundaries[0];
				} else {
					roadBoundary = laneGroup->roadBoundaries[1];
				}
			}
			return roadBoundary;
		};

		MapPoint3D64 connectedPt = {};
		HadLaneBoundary* currLaneBoundary = nullptr;
		HadRoadBoundary* currRoadBoundary = nullptr;
		int vertexIdx = GrapPointAlgorithm::findNearestPoint(laneEdge.vertexes, intersectPt);
		auto& vertex = laneEdge.vertexes[vertexIdx];
		auto& frontLaneEdgePt = laneEdge.vertexes.front();
		auto& backLaneEdgePt = laneEdge.vertexes.back();
		if (vertex == frontLaneEdgePt) {
			connectedPt = backLaneEdgePt;
			currLaneBoundary = laneBoundarys[laneBoundarys.size() - 1];
			currRoadBoundary = getNearestRoadBoundary(currLaneBoundary, backLaneEdgePt);
			if (currRoadBoundary != nullptr && currRoadBoundary->next.size() == 1) {
				connectedRoadBoundary = (HadRoadBoundary*)currRoadBoundary->next[0];
			}
			if (currLaneBoundary->next.size() == 1) {
				connectedLaneBoundary = (HadLaneBoundary*)currLaneBoundary->next[0];
			}
			else if (connectedRoadBoundary != nullptr && currLaneBoundary->next.size() > 1) {
				std::vector<HadLaneBoundary*> nextLaneBoundaries;
				for (auto nextBoundary : currLaneBoundary->next) {
					nextLaneBoundaries.push_back((HadLaneBoundary*)nextBoundary);
				}
				getNearbyLaneBoundary(connectedRoadBoundary->location.vertexes, nextLaneBoundaries, connectedLaneBoundary);
			}
			if (connectedRoadBoundary == nullptr && connectedLaneBoundary != nullptr) {
				connectedRoadBoundary = getNearestRoadBoundary(connectedLaneBoundary, backLaneEdgePt);
			}
		}
		else if (vertex == backLaneEdgePt) {
			connectedPt = frontLaneEdgePt;
			currLaneBoundary = laneBoundarys[0];
			currRoadBoundary = getNearestRoadBoundary(currLaneBoundary, frontLaneEdgePt);
			if (currRoadBoundary != nullptr && currRoadBoundary->previous.size() == 1) {
				connectedRoadBoundary = (HadRoadBoundary*)currRoadBoundary->previous[0];
			}
			if (currLaneBoundary->previous.size() == 1) {
				connectedLaneBoundary = (HadLaneBoundary*)currLaneBoundary->previous[0];
			}
			else if (connectedRoadBoundary != nullptr && currLaneBoundary->previous.size() > 1) {
				std::vector<HadLaneBoundary*> previousLaneBoundaries;
				for (auto previousBoundary : currLaneBoundary->previous) {
					previousLaneBoundaries.push_back((HadLaneBoundary*)previousBoundary);
				}
				getNearbyLaneBoundary(connectedRoadBoundary->location.vertexes, previousLaneBoundaries, connectedLaneBoundary);
			}
			if (connectedRoadBoundary == nullptr && connectedLaneBoundary != nullptr) {
				connectedRoadBoundary = getNearestRoadBoundary(connectedLaneBoundary, frontLaneEdgePt);
			}
		}
		// 检查车道连接夹角,路口处很多都是左右转车道,这些不能连
		if (currLaneBoundary != nullptr &&connectedLaneBoundary != nullptr) {
			auto getConnectVertexes = [&connectedPt](std::vector<MapPoint3D64>& vertexes)->std::vector<MapPoint3D64> {
				std::vector<MapPoint3D64> connectedVertexes;
				if (vertexes.size() < 2)
					return connectedVertexes;
				int vertexIdx = GrapPointAlgorithm::findNearestPoint(vertexes, connectedPt);
				for (int idx = vertexIdx - 1; idx >= 0; idx--) {
					if (vertexes[idx] != connectedPt) {
						connectedVertexes.push_back(vertexes[idx]);
						break;
					}
				}
				connectedVertexes.push_back(vertexes[vertexIdx]);
				for (int idx = vertexIdx + 1; idx < vertexes.size(); idx++) {
					if (vertexes[idx] != connectedPt) {
						connectedVertexes.push_back(vertexes[idx]);
						break;
					}
				}
				return connectedVertexes;
			};

			auto currVertexes = getConnectVertexes(currLaneBoundary->location.vertexes);
			auto connectedVertexes = getConnectVertexes(connectedLaneBoundary->location.vertexes);
			auto connectedLaneAngle = getAngle(currVertexes, connectedVertexes);
			if (connectedLaneAngle > 30) {
				connectedLaneBoundary = nullptr;
				connectedRoadBoundary = nullptr;
			}
		}
	}

	void DiversionCreator::connectToGuardrail(LineString3d& laneEdge, HadLaneBoundary* laneBoundary, HadRoadBoundary* roadBoundary, bool isNext)
	{
		MapPoint3D64* guardrailPt = nullptr;
		auto& lanePt = isNext ? laneEdge.vertexes.back() : laneEdge.vertexes.front();
		if (roadBoundary->boundaryType == BoundaryType::GURADRAIL || roadBoundary->boundaryType == BoundaryType::WALL) {
			int guardrailIdx = isNext ? 0 : roadBoundary->location.vertexes.size() - 1;
			guardrailPt = &roadBoundary->location.vertexes[guardrailIdx];
		} else if (!roadBoundary->attributes.empty()) {
			//基于道路边界PA获取护栏起点
			for (auto& attributeItem : roadBoundary->attributes) {
				if (attributeItem->name == ROAD_BOUNDARY_PA_NAME) {
					if (attributeItem->value == (int)BoundaryType::GURADRAIL || attributeItem->value == (int)BoundaryType::WALL) {
						if (isNext) {
							guardrailPt = &attributeItem->points.postions[0];
							break;
						} else {
							guardrailPt = &attributeItem->points.postions[1];
						}
					}
				}
			}
		}

		// 连接到道路边界
		size_t si, ei;
		MapPoint3D64 grappedRoadBoundaryPt = {};
		auto& roadBoundaryVertexes = roadBoundary->location.vertexes;
		GrapPointAlgorithm::grapOrMatchNearestPoint(lanePt, roadBoundaryVertexes, grappedRoadBoundaryPt, si, ei);
		double grappedRoadBoundaryDistance = grappedRoadBoundaryPt.pos.distance(lanePt.pos);
		if (guardrailPt == nullptr) {
			guardrailPt = &grappedRoadBoundaryPt;
		}

		if (guardrailPt != nullptr) {
			// 连接到护栏距离阈值,超过距离的不扩展
			double distance = (*guardrailPt).pos.distance(lanePt.pos);
			if (grappedRoadBoundaryDistance < distance) {
				distance = grappedRoadBoundaryDistance;
				guardrailPt = &grappedRoadBoundaryPt;
			}
			if (distance > CONNECT_TO_GUARDRAIL_DISTANCE_TOLERANCE) {
				return;
			}

			std::vector<MapPoint3D64> laneBoundaryVertexes = laneBoundary->location.vertexes;
			if (!laneBoundary->linkGroups.empty() && laneBoundary->linkGroups.begin()->second->roadBoundaries.size() == 2)
			{
				LineString3d first = laneBoundary->linkGroups.begin()->second->roadBoundaries[0]->location;
				LineString3d second = laneBoundary->linkGroups.begin()->second->roadBoundaries[1]->location;

				std::vector<MapPoint3D64> lineOnRoadSurface;
				DiversionCompiler::projectLineOnRoadSurface(first, second, laneBoundaryVertexes, lineOnRoadSurface, PROJECT_TOLERANCE);
				laneBoundaryVertexes = lineOnRoadSurface;
			}

			int guardrailIdx = GrapPointAlgorithm::findNearestPoint(laneBoundaryVertexes, *guardrailPt);
			MapPoint3D64& nearestLaneBoundaryPt = laneBoundaryVertexes[guardrailIdx];
			MapPoint3D64 grappedLaneBoundaryPt = {};
			if (isNext) {
				for (int idx = 1; idx <= guardrailIdx; idx++) {
					auto& lanePoint = laneBoundaryVertexes[idx];
					laneEdge.vertexes.push_back(lanePoint);
				}
				if (GrapPointAlgorithm::grapOrMatchNearestPoint(*guardrailPt, laneBoundaryVertexes, grappedLaneBoundaryPt, si, ei)) {
					if (guardrailPt->pos.distance(grappedLaneBoundaryPt.pos) < guardrailPt->pos.distance(nearestLaneBoundaryPt.pos))
						laneEdge.vertexes.push_back(grappedLaneBoundaryPt);
				}
				laneEdge.vertexes.push_back(*guardrailPt);
			}
			else {
				for (int idx = laneBoundaryVertexes.size() - 1; idx >= guardrailIdx; idx--) {
					auto& lanePoint = laneBoundaryVertexes[idx];
					laneEdge.vertexes.emplace(laneEdge.vertexes.begin(), lanePoint);
				}
				if (GrapPointAlgorithm::grapOrMatchNearestPoint(*guardrailPt, laneBoundaryVertexes, grappedLaneBoundaryPt, si, ei)) {
					if (guardrailPt->pos.distance(grappedLaneBoundaryPt.pos) < guardrailPt->pos.distance(nearestLaneBoundaryPt.pos))
						laneEdge.vertexes.emplace(laneEdge.vertexes.begin(), grappedLaneBoundaryPt);
				}
				laneEdge.vertexes.emplace(laneEdge.vertexes.begin(), *guardrailPt);
			}

		}
	}

	void DiversionCreator::expandEdges()
	{
		auto calcLaneDistancePtr = [](LineString3d* edge)->double {
			double distance = 0;
			for (int idx = 0; idx < edge->vertexes.size() - 1; idx++) {
				auto& currPt = edge->vertexes[idx];
				auto& nextPt = edge->vertexes[idx + 1];
				distance += currPt.pos.distance(nextPt.pos);
			}
			return distance;
		};

		auto reverseEdges = [](std::vector<LineString3d>& edges) {
			std::reverse(edges.begin(), edges.end());
			for (auto& edge : edges) {
				std::reverse(edge.vertexes.begin(), edge.vertexes.end());
			}
		};

		for (auto& edges : m_diversion->m_edgesList) {
			std::vector<std::pair<LineString3d*, double>> laneDistance;
			std::vector<LineString3d> tempEdges = edges;
			std::vector<LineString3d*> ptrEdges;
			for (auto& tempEdge : tempEdges) {
				ptrEdges.push_back(&tempEdge);
				double tempEdgeDistance = calcLaneDistancePtr(&tempEdge);
				laneDistance.push_back(std::pair<LineString3d*, double>(&tempEdge, tempEdgeDistance));
			}

			// 扩张边界
			if (!m_diversion->isGore()) {
				expandLaneDistancePair(laneDistance, ptrEdges);
			}
			std::vector<LineString3d> expandEdges;
			for (auto ptrEdge : ptrEdges) {
				double ptrEdgeDistance = calcLaneDistancePtr(ptrEdge);
				if (ptrEdgeDistance != 0) {
					expandEdges.push_back(*ptrEdge);
				}
			}

			// 逆序
			ClipperLib::Path fillAreaPath;
			MapPoint3D64 prevVertex = {};
			for (auto& edge : edges) {
				for (auto& vertex : edge.vertexes) {
					if (!floatEqual(vertex.pos.distanceSquare(prevVertex.pos), 0)) {
						fillAreaPath << ClipperLib::IntPoint(vertex.pos.lon, vertex.pos.lat, vertex.z);
						prevVertex = vertex;
					}
				}
			}
			MapPoint3D64& startPt = edges[0].vertexes.front();
			MapPoint3D64& endPt = edges[edges.size() - 1].vertexes.back();
			if (!floatEqual(startPt.pos.distanceSquare(endPt.pos), 0)) {
				fillAreaPath << ClipperLib::IntPoint(startPt.pos.lon, startPt.pos.lat, startPt.z);
			}

			if (!ClipperLib::Orientation(fillAreaPath)) {
				if (m_diversion->m_fillArea != nullptr) {
					std::reverse(m_diversion->m_fillArea->polygon.vertexes.begin(), m_diversion->m_fillArea->polygon.vertexes.end());
				}
				reverseEdges(expandEdges);
				reverseEdges(edges);
			}
			m_diversion->m_expandEdgesList.push_back(expandEdges);
		}
	}

	void DiversionCreator::expandLaneDistancePair(std::vector<std::pair<LineString3d*, double>>& laneDistance, std::vector<LineString3d*>& edges)
	{
		// 获取边在laneDistance的迭代器
		auto getLaneDistanceIter = [&](LineString3d* edge)->std::vector<std::pair<LineString3d*, double>>::iterator {
			for (auto& iter = laneDistance.begin(); iter != laneDistance.end(); iter++) {
				if (iter->first == edge) {
					return iter;
				}
			}
			return laneDistance.end();
		};

		// 正向扩张prevIter,把nextIter追加到prevIter后面
		auto expandLaneDistanceForward = [&](std::vector<std::pair<LineString3d*, double>>::iterator& prevIter,
			std::vector<std::pair<LineString3d*, double>>::iterator& nextIter) {
				LineString3d* prevEdge = prevIter->first;
				LineString3d* nextEdge = nextIter->first;
				MapPoint3D64 prevVertex = prevEdge->vertexes.back();
				for_each(nextEdge->vertexes.begin(), nextEdge->vertexes.end(),
					[&](MapPoint3D64& vertex)->void {
						if (!floatEqual(vertex.pos.distanceSquare(prevVertex.pos), 0)) {
							prevEdge->vertexes.push_back(vertex);
							prevVertex = vertex;
						}
					}
				);

				edges.erase(std::find(edges.begin(), edges.end(), nextEdge));
				getLaneDistanceIter(prevEdge)->second += getLaneDistanceIter(nextEdge)->second;
				laneDistance.erase(getLaneDistanceIter(nextEdge));
		};

		// 反向扩张prevIter,把nextIter追加到prevIter前面
		auto expandLaneDistanceBackward = [&](std::vector<std::pair<LineString3d*, double>>::iterator& prevIter,
			std::vector<std::pair<LineString3d*, double>>::iterator& nextIter) {
				LineString3d* prevEdge = prevIter->first;
				LineString3d* nextEdge = nextIter->first;
				MapPoint3D64 prevVertex = prevEdge->vertexes.front();
				for_each(nextEdge->vertexes.rbegin(), nextEdge->vertexes.rend(),
					[&](MapPoint3D64& vertex)->void {
						if (!floatEqual(vertex.pos.distanceSquare(prevVertex.pos), 0)) {
							prevEdge->vertexes.insert(prevEdge->vertexes.begin(), vertex);
							prevVertex = vertex;
						}
					}
				);

				edges.erase(std::find(edges.begin(), edges.end(), nextEdge));
				getLaneDistanceIter(prevEdge)->second += getLaneDistanceIter(nextEdge)->second;
				laneDistance.erase(getLaneDistanceIter(nextEdge));
		};

		// 优先选择与车道边界线平行的边
		std::vector<HadRelLaneGroupAssociation*> overedLgAssociations;
		if (m_diversion->m_fillArea != nullptr) {
			overedLgAssociations = m_diversion->m_fillArea->overedLgAssociations;
		}
		sortFillAreaLaneDistance(laneDistance, m_diversion->m_laneGroups, overedLgAssociations);

		if (laneDistance.size() < 2)
			return;

		auto firstLaneDistancePairIter = laneDistance.begin();
		auto secondLaneDistancePairIter = firstLaneDistancePairIter + 1;
		if (firstLaneDistancePairIter->second < secondLaneDistancePairIter->second) {
			std::swap(firstLaneDistancePairIter, secondLaneDistancePairIter);
		}

		// 两条边之间可以出现一条连接边
		auto firstIdx = std::find(edges.begin(), edges.end(), firstLaneDistancePairIter->first) - edges.begin();
		auto secondIdx = std::find(edges.begin(), edges.end(), secondLaneDistancePairIter->first) - edges.begin();
		int roundDifference = min(firstIdx, secondIdx) + edges.size() - max(firstIdx, secondIdx);
		if (std::abs(firstIdx - secondIdx) <= 2 && roundDifference <= 2) {
			return;
		}

		// 每次扩张一次,扩张距离短的那条边
		if (firstIdx < secondIdx) { // 距离长的在前
			if ((secondIdx - firstIdx) > 2) { // 向前扩张
				LineString3d* prevEdge = edges[secondIdx - 1];
				auto& prevIter = getLaneDistanceIter(prevEdge);
				expandLaneDistanceForward(prevIter, secondLaneDistancePairIter);
			}
			else { // 向后扩张
				if (secondIdx != edges.size() - 1) {
					LineString3d* nextEdge = edges[secondIdx + 1];
					auto& nextIter = getLaneDistanceIter(nextEdge);
					expandLaneDistanceForward(secondLaneDistancePairIter, nextIter);
				}
				else {
					LineString3d* prevEdge = edges[0];
					auto& prevIter = getLaneDistanceIter(prevEdge);
					expandLaneDistanceBackward(prevIter, secondLaneDistancePairIter);
				}
			}
		}
		else { // 距离长的在后
			if ((firstIdx - secondIdx) > 2) { // 向后扩张
				LineString3d* nextEdge = edges[secondIdx + 1];
				auto& nextIter = getLaneDistanceIter(nextEdge);
				expandLaneDistanceForward(secondLaneDistancePairIter, nextIter);
			}
			else { // 向前扩张
				if (secondIdx != 0) {
					LineString3d* prevEdge = edges[secondIdx - 1];
					auto& prevIter = getLaneDistanceIter(prevEdge);
					expandLaneDistanceForward(prevIter, secondLaneDistancePairIter);
				}
				else {
					LineString3d* nextEdge = edges[edges.size() - 1];
					auto& nextIter = getLaneDistanceIter(nextEdge);
					expandLaneDistanceBackward(secondLaneDistancePairIter, nextIter);
				}
			}
		}
		// 接着扩张下一次
		expandLaneDistancePair(laneDistance, edges);
	}

	void DiversionCreator::sortFillAreaLaneDistance(std::vector<std::pair<LineString3d*, double>>& laneDistance,
		std::vector<HadLaneGroup*>& laneGroups, std::vector<HadRelLaneGroupAssociation*>& overedLgAssociations)
	{
		// 获取边对应车道边界与边等长的坐标点
		std::map<LineString3d*, bool> allNearbyLaneBoundaryOveredLgAssociation;
		std::map<LineString3d*, std::vector<MapPoint3D64>> allLaneBoundaryPoints;
		std::map<LineString3d*, std::vector<MapPoint3D64>> allNearbyLaneBoundaryPoints;
		auto getNearbyLaneBoundaryPoints = [&](std::pair<LineString3d*, double>& pair, std::vector<MapPoint3D64>& lanePoints,
			std::vector<MapPoint3D64>& nearbyLaneBoundaryPoints, bool& nearbyLaneBoundaryOveredLgAssociation) {
			lanePoints = allLaneBoundaryPoints[pair.first];
			nearbyLaneBoundaryPoints = allNearbyLaneBoundaryPoints[pair.first];
			nearbyLaneBoundaryOveredLgAssociation = allNearbyLaneBoundaryOveredLgAssociation[pair.first];
			if (pair.second != 0 && lanePoints.empty() && nearbyLaneBoundaryPoints.empty()) {
				HadLaneBoundary* nearbyLaneBoundary = nullptr;
				getNearbyLaneBoundary(laneGroups, pair.first->vertexes, nearbyLaneBoundary);
				if (nearbyLaneBoundary != nullptr) {
					auto nearbyLaneBoundaryVertexes = nearbyLaneBoundary->location.vertexes;
					MapPoint3D64 firstLaneVec = getLaneBoundaryVector(pair.first->vertexes);
					MapPoint3D64 secondLaneVec = getLaneBoundaryVector(nearbyLaneBoundaryVertexes);
					if (dot(firstLaneVec, secondLaneVec) > 0) {
						std::reverse(nearbyLaneBoundaryVertexes.begin(), nearbyLaneBoundaryVertexes.end());
					}

					createNearbyLaneBoundary(pair.first->vertexes, nearbyLaneBoundaryVertexes, lanePoints);
					if (lanePoints.empty()) {
						lanePoints = pair.first->vertexes;
					}
					allLaneBoundaryPoints.erase(pair.first);
					allLaneBoundaryPoints.emplace(pair.first, lanePoints);

					createNearbyLaneBoundary(nearbyLaneBoundaryVertexes, pair.first->vertexes, nearbyLaneBoundaryPoints);
					if (nearbyLaneBoundaryPoints.empty()) {
						nearbyLaneBoundaryPoints = nearbyLaneBoundaryVertexes;
					}
					allNearbyLaneBoundaryPoints.erase(pair.first);
					allNearbyLaneBoundaryPoints.emplace(pair.first, nearbyLaneBoundaryPoints);

					//夹角小于10°阈值认为是平行的
					double nearbyLaneAngle = getAngle(lanePoints, nearbyLaneBoundaryPoints);
					if (nearbyLaneAngle < 10 && !overedLgAssociations.empty()) {
						// 检查是否被LA边压盖,压盖的边需要排到前边
						for (auto overedLgAssociation : overedLgAssociations) {
							HadLaneBoundary* firstLaneBoundary = overedLgAssociation->firstLaneBoundary;
							HadLaneBoundary* secondLaneBoundary = overedLgAssociation->secondLaneBoundary;
							if (firstLaneBoundary == nearbyLaneBoundary || secondLaneBoundary == nearbyLaneBoundary) {
								nearbyLaneBoundaryOveredLgAssociation = true;
								allNearbyLaneBoundaryOveredLgAssociation.erase(pair.first);
								allNearbyLaneBoundaryOveredLgAssociation.emplace(pair.first, true);
								break;
							}
						}
					}
				}
			}
		};

		// 第一次排序,优先把平行的排到前面
		std::sort(laneDistance.begin(), laneDistance.end(),
			[&](std::pair<LineString3d*, double>& first, std::pair<LineString3d*, double>& second)->bool {
				bool firstOveredLgAssociation;
				std::vector<MapPoint3D64> firstLanePoints;
				std::vector<MapPoint3D64> firstNearbyLaneBoundaryPoints;
				getNearbyLaneBoundaryPoints(first, firstLanePoints, firstNearbyLaneBoundaryPoints, firstOveredLgAssociation);

				bool secondOveredLgAssociation;
				std::vector<MapPoint3D64> secondLanePoints;
				std::vector<MapPoint3D64> secondNearbyLaneBoundaryPoints;
				getNearbyLaneBoundaryPoints(second, secondLanePoints, secondNearbyLaneBoundaryPoints, secondOveredLgAssociation);

				double firstNearbyLaneAngle = getAngle(firstLanePoints, firstNearbyLaneBoundaryPoints);
				double secondNearbyLaneAngle = getAngle(secondLanePoints, secondNearbyLaneBoundaryPoints);

				// 被LA车道线边界压面的或者与车道线边界靠的非常近,夹角小于10°阈值认为是平行的
				if ((firstNearbyLaneAngle < 10 || firstOveredLgAssociation) && (secondNearbyLaneAngle < 10 || secondOveredLgAssociation)) {
					if (firstOveredLgAssociation && secondOveredLgAssociation) {
						return second.second < first.second;
					}
					else if (firstOveredLgAssociation) {
						return true;
					}
					else if (secondOveredLgAssociation) {
						return false;
					}

					return second.second < first.second;
				}
				else if (firstNearbyLaneAngle < 10 || firstOveredLgAssociation) {
					return true;
				}
				else if (secondNearbyLaneAngle < 10 || secondOveredLgAssociation) {
					return false;
				}

				return second.second < first.second;
			}
		);

		// 第二次排序,把长度太小的平行线往后排
		bool continueSort = false;
		do 
		{
			continueSort = false;
			for (int idx = 0; idx < laneDistance.size() - 1; idx++) {
				std::pair<LineString3d*, double>& first = laneDistance[idx];
				std::pair<LineString3d*, double>& second = laneDistance[idx + 1];
				if (second.second != 0 && first.second / second.second < 0.5) {
					std::swap(first, second);
					continueSort = true;
				}
			}
		} while (continueSort);
	}

	double DiversionCreator::getAngle(std::vector<MapPoint3D64>& laneVertexes, std::vector<MapPoint3D64>& nearbyLaneBoundaryPoints)
	{
		// 临接车道坐标为空直接返回
		if (nearbyLaneBoundaryPoints.empty()) {
			return DBL_MAX;
		}

		MapPoint3D64 laneVec = getLaneBoundaryVector(laneVertexes);
		MapPoint3D64 nearbyLaneVec = getLaneBoundaryVector(nearbyLaneBoundaryPoints);
		double angle1 = std::atan2(laneVec.pos.lat, laneVec.pos.lon);
		angle1 = int(angle1 * 180 / MATH_PI);
		double angle2 = std::atan2(nearbyLaneVec.pos.lat, nearbyLaneVec.pos.lon);
		angle2 = int(angle2 * 180 / MATH_PI);
		double angle = DBL_MAX;
		if (angle1 * angle2 >= 0) {
			angle = abs(angle1 - angle2);
		}
		else {
			angle = abs(angle1) + abs(angle2);
			if (angle > 180) {
				angle = 360 - angle;
			}
		}
		return minimalDegree(angle);
	}

	void DiversionCreator::buildEdges(std::vector<Polygon3d>& polygons)
	{
		for (auto& edges : m_diversion->m_edgesList) {
			Polygon3d polygon;
			buildEdge(edges, polygon);
			polygons.push_back(polygon);
		}
	}

	void DiversionCreator::buildEdge(std::vector<LineString3d>& edges, Polygon3d& polygon)
	{
		polygon.vertexes.clear();
		MapPoint3D64 prevVertex = {};
		for (auto& edge : edges) {
			for (auto& vertex : edge.vertexes) {
				if (!floatEqual(vertex.pos.distanceSquare(prevVertex.pos), 0)) {
					polygon.vertexes.push_back(vertex);
					prevVertex = vertex;
				}
			}
		}

		// 闭环
		MapPoint3D64& startPt = polygon.vertexes.front();
		MapPoint3D64& endPt = polygon.vertexes.back();
		if (!floatEqual(startPt.pos.distanceSquare(endPt.pos), 0)) {
			polygon.vertexes.push_back(startPt);
		}
	}

	uint16 DiversionCreator::minimalDegree(double angle)
	{
		angle = (int32)angle % 180;
		if (angle > 90) {
			angle = 180 - angle;
		}
		return angle;
	}

	uint16 DiversionCreator::toDegree(double angle)
	{
		return (uint16)(((int32)(angle * 180 / MATH_PI) % 360 + 360) % 360);
	}

	void DiversionCompiler::compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile)
	{
		bool applyDiversion = true;
		std::vector<HadRelLaneGroupAssociation*> allLaneGroupAssociations;
		groupLaneGroupAssociations(pGrid, nearby, allLaneGroupAssociations, &applyDiversion);

		std::vector<HadGrid*> nearbyGrids = { pGrid };
		for_each(nearby.begin(), nearby.end(), [&](HadGrid* g)->void {nearbyGrids.push_back(g); });

		// 过滤即可以调头又可以直行的LA
		filterUTurnLA(allLaneGroupAssociations);

		// 合并导流区
		mergeFillArea(pGrid, nearbyGrids);

		// 路口三角化结果
		m_surfaceTriangles.insert(m_surfaceTriangles.end(), compilerData.m_intersectionTriangles.begin(), compilerData.m_intersectionTriangles.end());

		// 路面三角化结果
		m_surfaceTriangles.insert(m_surfaceTriangles.end(), compilerData.m_roadTriangles.begin(), compilerData.m_roadTriangles.end());

		// 跨网格三角化路面
		auto crossGridGroups = getCrossGridGroups(pGrid, nearbyGrids);
		processCrossGridRoad(crossGridGroups);

		// surface rtree
		parameters surfaceParam;
		std::vector<box_2t> surfaceTriangleBoxes;
		LinearInterpolationTriangleSurface::getTriangleBoxes(m_surfaceTriangles, surfaceTriangleBoxes);
		index_getter_2box surfaceOriginInd(surfaceTriangleBoxes);
		rtree_type_2box surfaceRTree2T(boost::irange<std::size_t>(0lu, surfaceTriangleBoxes.size()), surfaceParam, surfaceOriginInd);
		m_surfaceRTree2T = &surfaceRTree2T;

		// 车道线索引
		auto allLines = compilerData.m_rdsLines;
		processCrossGridLine(pGrid, crossGridGroups, allLines);
		for (size_t i = 0; i < allLines.size(); ++i)
		{
			auto& rdsLineInfo = allLines[i];
			auto& originPoints = rdsLineInfo._originPoints;
			for (size_t j = 0; j < originPoints.size() - 1; ++j)
			{
				std::vector<size_t> tmpSize;
				segment_t tmpSeg(POINT_T(originPoints[j]), POINT_T(originPoints[j + 1]));
				m_segments.push_back(tmpSeg);
				m_sizes.push_back(std::vector<size_t>{i, j});
			}
		}
		parameters param;
		index_getter_segment originInd(m_segments);
		rtree_type_segment rtree(boost::irange<std::size_t>(0lu, m_segments.size()), param, originInd);

		// 从LA关系构建导流带
		compileLaneGroupAssociations(pGrid, nearbyGrids, pTile, allLaneGroupAssociations, allLines, rtree);

		// 从导流区构建导流带
		compileFillArea(pGrid, nearbyGrids, pTile, allLines, rtree);

		bool notApplyDiversion = false;
		std::vector<HadRelLaneGroupAssociation*> roadLaneGroupAssociations;
		groupLaneGroupAssociations(pGrid, nearby, roadLaneGroupAssociations, &notApplyDiversion);

		// 使用LA关系补面
		compileRoadFace(pGrid, nearbyGrids, pTile, roadLaneGroupAssociations, allLines, rtree);
	}

	std::set<HadLaneGroup*> DiversionCompiler::getCrossGridGroups(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby)
	{
		// 从导流区里找跨网格车道组
		std::set<HadLaneGroup*> crossGridGroups;
		auto addCrossGridGroup = [&](HadLaneGroup* pLaneGroup) {
			if (pLaneGroup != nullptr) {
				for (auto laneBoundary : pLaneGroup->laneBoundaries) {
					if (pGrid != laneBoundary->owner) {
						crossGridGroups.emplace(pLaneGroup);
						break;
					}
				}
			}
		};

		for (auto& tmpFillAreaObj : pGrid->query(ElementType::HAD_OBJECT_FILL_AREA)) {
			HadFillArea* hadFillArea = (HadFillArea*)tmpFillAreaObj;
			for each (HadLaneGroup * pLaneGroup  in hadFillArea->laneGroups) {
				addCrossGridGroup(pLaneGroup);
			}
		}

		// 从LA关系里找跨网格车道组
		std::vector<HadRelLaneGroupAssociation*> allLaneGroupAssociations;
		groupLaneGroupAssociations(pGrid, nearby, allLaneGroupAssociations, nullptr);
		// connectLaneGroupAssociations可能会增加跨网格LA关系,此处需防止迭代器失效
		auto tmpAllLaneGroupAssociations = allLaneGroupAssociations;
		for (auto lgAssociation : tmpAllLaneGroupAssociations)
		{
			std::vector<HadRelLaneGroupAssociation*> laneGroupAssociations;
			connectLaneGroupAssociations(lgAssociation, pGrid, nearby, nullptr, allLaneGroupAssociations, laneGroupAssociations);
			for (HadRelLaneGroupAssociation* tmpLgAssociation : laneGroupAssociations) {

				addCrossGridGroup(tmpLgAssociation->first);
				addCrossGridGroup(tmpLgAssociation->second);
			}
		}

		return crossGridGroups;
	}

	void DiversionCompiler::processCrossGridLine(HadGrid* const pGrid, std::set<HadLaneGroup*>& crossGridGroups, std::vector<rdsLineInfo>& allLines)
	{
		// 添加跨网格车道线
		for each (HadLaneGroup * pLaneGroup  in crossGridGroups) {
			for (auto laneBoundary : pLaneGroup->laneBoundaries) {
				if (pGrid != laneBoundary->owner) {
					LineString3d location = laneBoundary->location;
					if (directionEqual(laneBoundary, pLaneGroup, 3)) {
						std::reverse(location.vertexes.begin(), location.vertexes.end());
					}
					if (location.vertexes.size() < 2)
						continue;

					// 车道线线投影到路面
					std::vector<MapPoint3D64> lineOnRoadSurface;
					LinearInterpolationTriangleSurface::interpolationLine(
						coordinatesTransform, m_surfaceTriangles, *m_surfaceRTree2T, location.vertexes, lineOnRoadSurface);
					location.vertexes = lineOnRoadSurface;

					rdsLineInfo tmp;
					tmp._line = nullptr;
					tmp._originPoints = location.vertexes;
					std::vector<MapPoint3D64> tmpLine = location.vertexes;
					coordinatesTransform.convert(tmpLine.data(), tmpLine.size());
					tmp._lineBox2T = BOX_2T(tmpLine);
					tmp._linePoints = LINESTRING_T(tmpLine);
					allLines.push_back(tmp);
				}
			}
		}
	}

	void DiversionCompiler::processCrossGridRoad(std::set<HadLaneGroup*>& crossGridGroups)
	{
		// 添加跨网格三角化面
		for each (HadLaneGroup * pLaneGroup  in crossGridGroups) {
			LineString3d leftSide, rightSide;
			getLaneGroupBoundary(pLaneGroup, leftSide, rightSide);
			if (!leftSide.vertexes.empty() && !rightSide.vertexes.empty()) {
				std::vector<Triangle> roadTriangles;
				LinearInterpolationTriangleSurface::triangularizeStroke(coordinatesTransform, leftSide.vertexes, rightSide.vertexes, roadTriangles);
				m_surfaceTriangles.insert(m_surfaceTriangles.end(), roadTriangles.begin(), roadTriangles.end());
			}
		}
	}

	void DiversionCompiler::compileLaneGroupAssociations(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* const pTile,
		std::vector<HadRelLaneGroupAssociation*>& allLaneGroupAssociations, std::vector<rdsLineInfo>& allLines, rtree_type_segment& rtree)
	{
		// 数据侧的多个LA关系可能生成一个导流带,这里需要选出还未生成导流带的LA关系
		auto pickupLaneGroupAssociation = [&](std::vector<HadRelLaneGroupAssociation*> laneGroupAssociations) -> HadRelLaneGroupAssociation* {
			HadRelLaneGroupAssociation* retLgAssociation = nullptr;
			if (!laneGroupAssociations.empty()) {
				retLgAssociation = laneGroupAssociations.front();
			}
			return retLgAssociation;
		};

		bool applyDiversion = true;
		HadRelLaneGroupAssociation* lgAssociation = nullptr;
		while ((lgAssociation = pickupLaneGroupAssociation(allLaneGroupAssociations)) != nullptr)
		{
			std::vector<HadRelLaneGroupAssociation*> laneGroupAssociations;
			connectLaneGroupAssociations(lgAssociation, pGrid, nearby, &applyDiversion, allLaneGroupAssociations, laneGroupAssociations);

			Diversion diversion(pGrid, nullptr);
			for (HadRelLaneGroupAssociation* tmpLgAssociation : laneGroupAssociations) {
				diversion.m_lgAssociations.push_back(tmpLgAssociation);
				if (std::find(diversion.m_laneGroups.begin(), diversion.m_laneGroups.end(), tmpLgAssociation->first) == diversion.m_laneGroups.end()) {
					diversion.m_laneGroups.push_back(tmpLgAssociation->first);
				}
				if (std::find(diversion.m_laneGroups.begin(), diversion.m_laneGroups.end(), tmpLgAssociation->second) == diversion.m_laneGroups.end()) {
					diversion.m_laneGroups.push_back(tmpLgAssociation->second);
				}
			}

			if (containsCurrentGrid(pGrid, laneGroupAssociations)) {
				compileDiversion(pTile, diversion, allLines, rtree);
			}

			for (HadRelLaneGroupAssociation* tmpLgAssociation : laneGroupAssociations) {
				auto tmpLgAssociationIter = std::find(allLaneGroupAssociations.begin(), allLaneGroupAssociations.end(), tmpLgAssociation);
				if (tmpLgAssociationIter != allLaneGroupAssociations.end()) {
					allLaneGroupAssociations.erase(tmpLgAssociationIter);
				}
			}
		}
	}

	void DiversionCompiler::compileRoadFace(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* const pTile, 
		std::vector<HadRelLaneGroupAssociation*>& allLaneGroupAssociations, std::vector<rdsLineInfo>& allLines, rtree_type_segment& rtree)
	{
		// 数据侧的多个LA关系可能生成一条道路,这里需要选出还未生成路面的LA关系
		auto pickupLaneGroupAssociation = [&](std::vector<HadRelLaneGroupAssociation*> laneGroupAssociations) -> HadRelLaneGroupAssociation* {
			HadRelLaneGroupAssociation* retLgAssociation = nullptr;
			if (!laneGroupAssociations.empty()) {
				retLgAssociation = laneGroupAssociations.front();
			}
			return retLgAssociation;
		};

		bool applyDiversion = false;
		HadRelLaneGroupAssociation* lgAssociation = nullptr;
		while ((lgAssociation = pickupLaneGroupAssociation(allLaneGroupAssociations)) != nullptr)
		{
			std::vector<HadRelLaneGroupAssociation*> laneGroupAssociations;
			connectLaneGroupAssociations(lgAssociation, pGrid, nearby, &applyDiversion, allLaneGroupAssociations, laneGroupAssociations);

			Diversion diversion(pGrid, nullptr);
			bool containsCrossGridGroup = false;
			bool containsSemiTransparentGroup = false;
			for (HadRelLaneGroupAssociation* tmpLgAssociation : laneGroupAssociations) {
				diversion.m_lgAssociations.push_back(tmpLgAssociation);
				if (std::find(diversion.m_laneGroups.begin(), diversion.m_laneGroups.end(), tmpLgAssociation->first) == diversion.m_laneGroups.end()) {
					diversion.m_laneGroups.push_back(tmpLgAssociation->first);
					if (!containsSemiTransparentGroup && isSemiTransparentGroup(tmpLgAssociation->first))
						containsSemiTransparentGroup = true;
					if (tmpLgAssociation->first->owner != pGrid)
						containsCrossGridGroup = true;
				}
				if (std::find(diversion.m_laneGroups.begin(), diversion.m_laneGroups.end(), tmpLgAssociation->second) == diversion.m_laneGroups.end()) {
					diversion.m_laneGroups.push_back(tmpLgAssociation->second);
					if (!containsSemiTransparentGroup && isSemiTransparentGroup(tmpLgAssociation->second))
						containsSemiTransparentGroup = true;
					if (tmpLgAssociation->second->owner != pGrid)
						containsCrossGridGroup = true;
				}
			}

			//判断是否为普通路
			if (CompileSetting::instance()->isNotCompileUrbanData)
			{
				if (!isProDataLevel(diversion.m_laneGroups))
				{
					for (HadRelLaneGroupAssociation* tmpLgAssociation : laneGroupAssociations) {
						auto tmpLgAssociationIter = std::find(allLaneGroupAssociations.begin(), allLaneGroupAssociations.end(), tmpLgAssociation);
						if (tmpLgAssociationIter != allLaneGroupAssociations.end()) {
							allLaneGroupAssociations.erase(tmpLgAssociationIter);
						}
					}
					continue;
				}
			}
			
			// https://jira.navinfo.com/browse/HQNAVI-2658
			bool skipCrossGridSemiTransparentGroup = containsCrossGridGroup && containsSemiTransparentGroup;
			if (!skipCrossGridSemiTransparentGroup && containsCurrentGrid(pGrid, laneGroupAssociations)) {
				DiversionCreator diversionCreator(this, &diversion);
				std::vector<std::pair<LineString3d, LineString3d>> laEdges;
				diversionCreator.generateRoadFace(laEdges);
				for (auto& laEdge : laEdges) {
					adjustDiversion(laEdge.first.vertexes, allLines, rtree);
					adjustDiversion(laEdge.second.vertexes, allLines, rtree);
					RdsRoad* pRoad = (RdsRoad*)createObject(pTile, EntityType::RDS_ROAD);
					RDS::LineString3d lineLeftSide;
					convert(laEdge.first, lineLeftSide);
					RDS::LineString3d lineRightSize;
					convert(laEdge.second, lineRightSize);
					pRoad->contour.lines.resize(2);
					pRoad->contour.lines[0] = lineLeftSide;
					pRoad->contour.lines[1] = lineRightSize;
					pRoad->roadType = RdsRoad::RoadType::LA;
					for (auto laneGroup : diversion.m_laneGroups) {
						pRoad->groups.push_back(laneGroup);
						RdsGroup* pRdsGroup = queryGroup(laneGroup->originId, pTile);
						if (pRdsGroup)
							pRdsGroup->objects.push_back(pRoad);
					}
				}
			}

			for (HadRelLaneGroupAssociation* tmpLgAssociation : laneGroupAssociations) {
				auto tmpLgAssociationIter = std::find(allLaneGroupAssociations.begin(), allLaneGroupAssociations.end(), tmpLgAssociation);
				if (tmpLgAssociationIter != allLaneGroupAssociations.end()) {
					allLaneGroupAssociations.erase(tmpLgAssociationIter);
				}
			}
		}
	}

	bool DiversionCompiler::containsCurrentGrid(HadGrid* const pGrid, std::vector<HadRelLaneGroupAssociation*>& laneGroupAssociations)
	{
		for (HadRelLaneGroupAssociation* tmpLgAssociation : laneGroupAssociations) {
			if (tmpLgAssociation->first->owner == pGrid || tmpLgAssociation->second->owner == pGrid) {
				return true;
			}

			for (auto overedFillArea : tmpLgAssociation->overedFillAreas) {
				if (containsCurrentGrid(pGrid, overedFillArea)) {
					return true;
				}
			}
		}

		return false;
	}

	bool DiversionCompiler::containsCurrentGrid(HadGrid* const pGrid, HadFillArea* const pFillArea)
	{
		if (pFillArea && pFillArea->owner == pGrid) {
			return true;
		}

		if (pFillArea && pFillArea->mergedFillArea) {
			if (containsCurrentGrid(pGrid, pFillArea->mergedFillArea)) {
				return true;
			}
		}

		return false;
	}

	void DiversionCompiler::connectLaneGroupAssociations(HadRelLaneGroupAssociation* lgAssociation, 
		HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, bool* applyDiversion,
		std::vector<HadRelLaneGroupAssociation*>& allLaneGroupAssociations, std::vector<HadRelLaneGroupAssociation*>& laneGroupAssociations)
	{
		// previous
		HadRelLaneGroupAssociation* previousLgAssociation = lgAssociation;
		while (previousLgAssociation->first->previous.size() >= 1) {
			bool containsPreviousLgAssociation = false;
			for (auto& pFirstLg : previousLgAssociation->first->previous) {
				HadLaneGroup* firstLaneGroup = (HadLaneGroup*)pFirstLg;
				std::vector<HadSkeleton*>& previous = previousLgAssociation->directType == 2
					? previousLgAssociation->second->previous : previousLgAssociation->second->next;
				for (auto& pSecondLg : previous) {
					HadLaneGroup* secondLaneGroup = (HadLaneGroup*)pSecondLg;
					if (firstLaneGroup == secondLaneGroup) {
						break;
					}

					auto laIter = findLgAssociationIter(previousLgAssociation, firstLaneGroup, secondLaneGroup, pGrid, nearby, allLaneGroupAssociations, applyDiversion);
					if (lgAssociation != (*laIter) && laIter != allLaneGroupAssociations.end() && !isXUTurnLA(previousLgAssociation, *laIter)) {
						previousLgAssociation = (*laIter);
						if (previousLgAssociation->first == secondLaneGroup && previousLgAssociation->second == firstLaneGroup) {
							// 保持往前寻找,不能会陷入死循环
							std::swap(previousLgAssociation->first, previousLgAssociation->second);
							std::swap(previousLgAssociation->firstLaneBoundary, previousLgAssociation->secondLaneBoundary);
						}
						containsPreviousLgAssociation = true;
						laneGroupAssociations.emplace(laneGroupAssociations.begin(), previousLgAssociation);
					}
				}
				if (containsPreviousLgAssociation) {
					break;
				}
			}
			if (!containsPreviousLgAssociation) {
				break;
			}
		}

		// current
		laneGroupAssociations.push_back(lgAssociation);

		// next
		HadRelLaneGroupAssociation* nextLgAssociation = lgAssociation;
		while (nextLgAssociation->first->next.size() >= 1) {
			bool containsNextLgAssociation = false;
			for (auto& pFirstLg : nextLgAssociation->first->next) {
				HadLaneGroup* firstLaneGroup = (HadLaneGroup*)pFirstLg;
				std::vector<HadSkeleton*>& next = nextLgAssociation->directType == 2
					? nextLgAssociation->second->next : nextLgAssociation->second->previous;
				for (auto& pSecondLg : next) {
					HadLaneGroup* secondLaneGroup = (HadLaneGroup*)pSecondLg;
					if (firstLaneGroup == secondLaneGroup) {
						break;
					}

					auto laIter = findLgAssociationIter(nextLgAssociation, firstLaneGroup, secondLaneGroup, pGrid, nearby, allLaneGroupAssociations, applyDiversion);
					if (lgAssociation != (*laIter) && laIter != allLaneGroupAssociations.end() && !isXUTurnLA(nextLgAssociation, *laIter)) {
						nextLgAssociation = (*laIter);
						if (nextLgAssociation->first == secondLaneGroup && nextLgAssociation->second == firstLaneGroup) {
							// 保持往后寻找,不能会陷入死循环
							std::swap(nextLgAssociation->first, nextLgAssociation->second);
							std::swap(nextLgAssociation->firstLaneBoundary, nextLgAssociation->secondLaneBoundary);
						}
						containsNextLgAssociation = true;
						laneGroupAssociations.push_back(nextLgAssociation);
					}
				}
				if (containsNextLgAssociation) {
					break;
				}
			}
			if (!containsNextLgAssociation) {
				break;
			}
		};
	}

	bool DiversionCompiler::isXUTurnLA(HadRelLaneGroupAssociation* lgAssociation, HadRelLaneGroupAssociation* pLA)
	{
		// 通过车道边界拓扑关系判断是否为车道边界相交LA关系
		auto isUTurnLA = [](HadRelLaneGroupAssociation* la)->bool {
			std::vector<HadSkeleton*>& firstPrevious = la->firstLaneBoundary->previous;
			std::vector<HadSkeleton*>& firstNext = la->firstLaneBoundary->next;
			std::vector<HadSkeleton*>& secondPrevious = la->secondLaneBoundary->previous;
			std::vector<HadSkeleton*>& secondNext = la->secondLaneBoundary->next;
			if (std::find(firstPrevious.begin(), firstPrevious.end(), la->secondLaneBoundary) != firstPrevious.end() ||
				std::find(firstNext.begin(), firstNext.end(), la->secondLaneBoundary) != firstNext.end() ||
				std::find(secondPrevious.begin(), secondPrevious.end(), la->firstLaneBoundary) != secondPrevious.end() ||
				std::find(secondNext.begin(), secondNext.end(), la->firstLaneBoundary) != secondNext.end()) {
				return true;
			}

			return false;
		};

		// 检查是否为X型的
		if (isUTurnLA(lgAssociation) && isUTurnLA(pLA)) {
			MapPoint3D64 intersectionPt1 = {};
			auto& firstLaneEdge = lgAssociation->firstLaneBoundary->location;
			auto& secondLaneEdge = lgAssociation->secondLaneBoundary->location;
			getIntersectionPoint(firstLaneEdge.vertexes, secondLaneEdge.vertexes, intersectionPt1);

			MapPoint3D64 intersectionPt2 = {};
			auto& pLAFirstLaneEdge = pLA->firstLaneBoundary->location;
			auto& pLASecondLaneEdge = pLA->secondLaneBoundary->location;
			getIntersectionPoint(pLAFirstLaneEdge.vertexes, pLASecondLaneEdge.vertexes, intersectionPt2);
			if (floatEqual(intersectionPt1.pos.distanceSquare(intersectionPt2.pos), 0)) {
				return true;
			}
		}

		return false;
	}

	std::vector<HadLaneGroupAssociation::HadRelLaneGroupAssociation*>::iterator
		DiversionCompiler::findLgAssociationIter(
			HadRelLaneGroupAssociation* lgAssociation, HadLaneGroup* firstLaneGroup, HadLaneGroup* secondLaneGroup,
			HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, std::vector<HadRelLaneGroupAssociation*>& allLaneGroupAssociations, bool* pApplyDiversion)
	{
		auto begin = allLaneGroupAssociations.begin();
		auto end = allLaneGroupAssociations.end();
		std::vector<HadRelLaneGroupAssociation*>::iterator iter = std::find_if(begin, end, [&](HadRelLaneGroupAssociation* pLA)->bool {
			return (pLA->first == firstLaneGroup && pLA->second == secondLaneGroup) ||
				(pLA->first == secondLaneGroup && pLA->second == firstLaneGroup);
			});
		if (iter != allLaneGroupAssociations.end()) {
			return iter;
		}

		bool findNearbyLa = false;
		for (HadGrid* nearbyGrid : nearby) {
			if (pGrid == nearbyGrid) {
				continue;
			}

			for (auto la : nearbyGrid->query(ElementType::HAD_LG_ASSOCIATIOIN)) {
				HadLaneGroupAssociation* pLaneGroupAssociation = (HadLaneGroupAssociation*)la;
				for (auto& relLgAssociation : pLaneGroupAssociation->relLaneGroupAssociations) {
					HadRelLaneGroupAssociation* pLA = &relLgAssociation;
					if (pLA->firstLaneBoundary == nullptr || pLA->secondLaneBoundary == nullptr) {
						continue;
					}

					if (pApplyDiversion != nullptr) 
					{
						if (*pApplyDiversion) {
							if (pLA->applyDiversion) // 过滤LA关系
								continue;
							if ((pLA->first == firstLaneGroup && pLA->second == secondLaneGroup) ||
								(pLA->first == secondLaneGroup && pLA->second == firstLaneGroup)) {
								if (std::find(allLaneGroupAssociations.begin(), allLaneGroupAssociations.end(), pLA) == allLaneGroupAssociations.end()) {
									if (!isXUTurnLA(lgAssociation, pLA)) {
										allLaneGroupAssociations.push_back(pLA);
									}
								}
								findNearbyLa = true;
								break;
							}
						}
						else if (pLA->applyDiversion && pLA->groupId && pLA->overedFillAreas.empty())
						{
							if ((pLA->first == firstLaneGroup && pLA->second == secondLaneGroup) ||
								(pLA->first == secondLaneGroup && pLA->second == firstLaneGroup)) {
								if (std::find(allLaneGroupAssociations.begin(), allLaneGroupAssociations.end(), pLA) == allLaneGroupAssociations.end()) {
									if (!isXUTurnLA(lgAssociation, pLA)) {
										allLaneGroupAssociations.push_back(pLA);
									}
								}
								findNearbyLa = true;
								break;
							}
						}
					}
					else 
					{
						if ((pLA->first == firstLaneGroup && pLA->second == secondLaneGroup) ||
							(pLA->first == secondLaneGroup && pLA->second == firstLaneGroup)) {
							if (std::find(allLaneGroupAssociations.begin(), allLaneGroupAssociations.end(), pLA) == allLaneGroupAssociations.end()) {
								if (!isXUTurnLA(lgAssociation, pLA)) {
									allLaneGroupAssociations.push_back(pLA);
								}
							}
							findNearbyLa = true;
							break;
						}
					}
				}

				if (findNearbyLa)
					break;
			}
			if (findNearbyLa)
				break;
		}

		if (findNearbyLa) {
			auto newBegin = allLaneGroupAssociations.begin();
			auto newEnd = allLaneGroupAssociations.end();
			return std::find_if(newBegin, newEnd, [&](HadRelLaneGroupAssociation* pLA)->bool {
				return (pLA->first == firstLaneGroup && pLA->second == secondLaneGroup) ||
					(pLA->first == secondLaneGroup && pLA->second == firstLaneGroup);
				});
		}

		return allLaneGroupAssociations.end();
	}

	void DiversionCompiler::mergeFillArea(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby)
	{
		for (auto obj : pGrid->query(ElementType::HAD_OBJECT_FILL_AREA))
		{
			HadFillArea* pFillArea = (HadFillArea*)obj;
			if (pFillArea->applyDiversion) {
				continue;
			}

			bool merged = false;
			bool hasMerged = false;
			do {
				merged = false;
				Diversion diversion(pGrid, pFillArea);
				DiversionCreator diversionCreator(this, &diversion);

				std::vector<HadFillArea*> nearbyFillAreas;
				getNearbyFillAreas(pFillArea, nearby, nearbyFillAreas);
				for (auto nearbyFillArea : nearbyFillAreas) {
					if (nearbyFillArea->applyDiversion) {
						continue;
					}

					Diversion nearbyDiversion(nearbyFillArea->owner, nearbyFillArea);
					if (diversionCreator.mergeDiversion(&nearbyDiversion)) {
						pFillArea->extent = makeBoundingBox2d(pFillArea->polygon.vertexes);
						nearbyFillArea->mergedFillArea = pFillArea;
						nearbyFillArea->applyDiversion = true;
						hasMerged = true;
						merged = true;
						break;
					}
				}
			} while (merged);

			// 坐标点排序,渲染需要
			if (hasMerged) {
				int minPtIndex = -1;
				MapPoint3D64 minPt = {{LLONG_MAX, LLONG_MAX},INT_MAX};
				auto& polygonVertexes = pFillArea->polygon.vertexes;
				std::deque<MapPoint3D64> vertexes{ polygonVertexes.begin(), polygonVertexes.end()};
				for (int idx = 0; idx < vertexes.size(); idx++) {
					auto& vertex = vertexes[idx];
					if (vertex < minPt) {
						minPtIndex = idx;
						minPt = vertex;
					}
				}

				vertexes.pop_back();
				for (int idx = 0; idx < minPtIndex; idx++) {
					MapPoint3D64 front = vertexes.front();
					vertexes.pop_front();
					vertexes.push_back(front);
				}
				vertexes.push_back(minPt);
				polygonVertexes.assign(vertexes.begin(), vertexes.end());
			}
		}
	}

	void DiversionCompiler::compileFillArea(HadGrid* const pGrid,const std::vector<HadGrid*>& nearby, RdsTile* const pTile, 
		std::vector<rdsLineInfo>& allLines, rtree_type_segment& rtree)
	{
		for (auto obj : pGrid->query(ElementType::HAD_OBJECT_FILL_AREA))
		{
			HadFillArea* pFillArea = (HadFillArea*)obj;
			applyDiversionForFillArea(pFillArea, nearby);
			if (pFillArea->applyDiversion) {
				continue;
			}

			Diversion diversion(pGrid, pFillArea);
			compileDiversion(pTile, diversion, allLines, rtree);
		}
	}

	void DiversionCompiler::compileDiversion(RdsTile* const pTile, Diversion& diversion, 
		std::vector<rdsLineInfo>& allLines, rtree_type_segment& rtree)
	{
		std::vector<Polygon3d> polygons;
		std::vector<LineString3d> centerReferenceLines;
		DiversionCreator diversionCreator(this, &diversion);
		diversionCreator.generateContour(polygons);
		diversionCreator.generateReferenceLine(centerReferenceLines);
		if (polygons.size() != centerReferenceLines.size()) {
			int64 fillAreaId = diversion.m_fillArea ? diversion.m_fillArea->originId : 0;
			return;
		}

		for (int idx = 0; idx < polygons.size(); idx++) {
			auto& polygon = polygons[idx];
			auto& centerLine = centerReferenceLines[idx];
			std::vector<HadLaneGroup*> laneGroups;
			std::vector<std::pair<LineString3d, double>> laLaneDistance;
			if (diversion.m_fillArea != nullptr) {
				for (HadLaneGroup* laneGroup : diversion.m_fillArea->laneGroups) {
					if (std::find(laneGroups.begin(), laneGroups.end(), laneGroup) == laneGroups.end()) {
						laneGroups.push_back(laneGroup);
					}
				}
			}
			else
			{
				// 只有LA关系生成导流带需使用边长度判断
				laLaneDistance = diversion.m_laLaneDistance[idx];
			}

			ClipperLib::Path laPath;
			for (auto& vertex : polygon.vertexes) {
				laPath << ClipperLib::IntPoint(vertex.pos.lon, vertex.pos.lat, vertex.z);
			}
			bool lgAssociationCrossGrid = false;
			std::vector<HadFillArea*> laPathApplyedFillAreas;
			for (HadRelLaneGroupAssociation* tmpLgAssociation : diversion.m_lgAssociations) {
				// 设置已应用于构建导流带标识
				tmpLgAssociation->applyDiversion = true;
				HadLaneGroup* first = tmpLgAssociation->first;
				HadLaneGroup* second = tmpLgAssociation->second;
				if (std::find(laneGroups.begin(), laneGroups.end(), first) == laneGroups.end()) {
					applyDiversionForFillArea(tmpLgAssociation, first, laLaneDistance, laPath, laPathApplyedFillAreas);
					laneGroups.push_back(first);
					if (first->crossGrid) {
						lgAssociationCrossGrid = true;
					}
				}
				if (std::find(laneGroups.begin(), laneGroups.end(), second) == laneGroups.end()) {
					applyDiversionForFillArea(tmpLgAssociation, second, laLaneDistance, laPath, laPathApplyedFillAreas);
					laneGroups.push_back(second);
					if (second->crossGrid) {
						lgAssociationCrossGrid = true;
					}
				}

				// 补充相交处的前后车道组
				std::vector<HadSkeleton*>& firstPrevious = first->previous;
				std::vector<HadSkeleton*>& firstNext = first->next;
				std::vector<HadSkeleton*>& secondPrevious = second->previous;
				std::vector<HadSkeleton*>& secondNext = second->next;
				if (firstPrevious.size() == 1 && secondPrevious.size() == 1 && firstPrevious[0] == secondPrevious[0]) {
					if (std::find(laneGroups.begin(), laneGroups.end(), firstPrevious[0]) == laneGroups.end()) {
						laneGroups.push_back((HadLaneGroup*)firstPrevious[0]);
					}
				}
				if (firstNext.size() == 1 && secondNext.size() == 1 && firstNext[0] == secondNext[0]) {
					if (std::find(laneGroups.begin(), laneGroups.end(), firstNext[0]) == laneGroups.end()) {
						laneGroups.push_back((HadLaneGroup*)firstNext[0]);
					}
				}
			}

			// LA关系构建的导流带面积比较小或很大,不生成导流带
			filterLargeLaDiversion(laPath, centerLine, laPathApplyedFillAreas);
			if (diversion.m_fillArea == nullptr && laPathApplyedFillAreas.empty()) {
				for (HadRelLaneGroupAssociation* tmpLgAssociation : diversion.m_lgAssociations) {
					// 清空压盖导流区列表,后面会应用于LA补面
					tmpLgAssociation->overedFillAreas.clear();
				}
				continue;
			}

			// 过滤LA压面处理过的导流区
			if (!laPathApplyedFillAreas.empty()) {
				for (auto laPathApplyedFillArea : laPathApplyedFillAreas) {
					laPathApplyedFillArea->applyDiversion = true;
					for (HadLaneGroup* laneGroup : laPathApplyedFillArea->laneGroups) {
						if (std::find(laneGroups.begin(), laneGroups.end(), laneGroup) == laneGroups.end()) {
							laneGroups.push_back(laneGroup);
						}
					}
				}
			}

			//判断是否为普通路
			if (CompileSetting::instance()->isNotCompileUrbanData)
			{
				if (!isProDataLevel(laneGroups))
					continue;
			}

			RdsDiversion* pDiversion = (RdsDiversion*)createObject(pTile, EntityType::RDS_DIVERSION);
			if (!diversion.m_lgAssociations.empty()) {
				pDiversion->isLaDiversion = true;
			}
			if (diversion.isGore()) {
				pDiversion->diversionType = RdsDiversion::DiversionType::GORE;
			}
			else {
				pDiversion->diversionType = RdsDiversion::DiversionType::SINGLE;
			}

			// 拉到车道线
			adjustDiversion(polygon.vertexes, allLines, rtree);

			// 计算导流带三角化及索引
			std::vector<Triangle> polyTriangles;
			Poly2Tri::triangularize(coordinatesTransform, polygon.vertexes, pDiversion->polyTriIndex, polyTriangles);

			// 拉升局部ZValue
			adjustDiversionZValue(laneGroups, polygon.vertexes, polyTriangles);

			convert(polygon, pDiversion->contour);
			convert(centerLine, pDiversion->centerRefereceLine);
			for (HadLaneGroup* laneGroup : laneGroups) {
				pDiversion->groups.push_back(laneGroup);
				RdsGroup* pRdsGroup = queryGroup(laneGroup->originId, pTile);
				if (pRdsGroup) {
					pRdsGroup->objects.push_back(pDiversion);
				}
			}
		}
	}

	double DiversionCompiler::ADJUST_DIVERSION_DISTANCE_TOLERANCE = 500; // ≈0.5m
	double DiversionCompiler::ADJUST_DIVERSION_IGNORE_DISTANCE_TOLERANCE = 50; // ≈0.05m
	bool DiversionCompiler::getNearestSegments(const point_t& currPt, const segment_t& segment, const std::vector<segment_t>& segments,
		const std::set<size_t>& segmentIds, std::vector<size_t>& nearestSegmentIds, point_t& nearestPt)
	{
		nearestSegmentIds.clear();
		if (segment.first == segment.second)
			return false;

		bool existNearestSegPt = false;
		bool existNearestGrappedPt = false;
		point_t nearestSegPt{}, nearestGrappedPt{};
		double nearestSegPtDis = DBL_MAX, nearestGrappedPtDis = DBL_MAX;
		auto setupNearestPt = [&](const point_t& segPt, const segment_t& tmpSegment) {
			if (segPt == currPt) {
				auto pt2d = P3_P2(segPt);
				auto d1 = bg::distance(P3_P2(tmpSegment.first), pt2d);
				if (d1 < ADJUST_DIVERSION_DISTANCE_TOLERANCE && d1 < nearestSegPtDis) {
					existNearestSegPt = true;
					nearestSegPt = tmpSegment.first;
					nearestSegPtDis = d1;
				}

				auto d2 = bg::distance(P3_P2(tmpSegment.second), pt2d);
				if (d2 < ADJUST_DIVERSION_DISTANCE_TOLERANCE && d2 < nearestSegPtDis) {
					existNearestSegPt = true;
					nearestSegPt = tmpSegment.second;
					nearestSegPtDis = d2;
				}

				point_t grapedPt;
				if (GRAP_POINT(segPt, tmpSegment, grapedPt, ADJUST_DIVERSION_DISTANCE_TOLERANCE)) {
					auto dis = bg::distance(pt2d, P3_P2(grapedPt));
					if (dis < ADJUST_DIVERSION_DISTANCE_TOLERANCE && dis < nearestGrappedPtDis) {
						existNearestGrappedPt = true;
						nearestGrappedPt = grapedPt;
						nearestGrappedPtDis = dis;
					}
				}
			}
		};

		segment_2t tmpSeg;
		segment_2t currSeg = S3_S2(segment);
		std::vector<rdsLineInfo*> adjustedLines;
		std::vector<std::pair<size_t, double>> segDisPair;
		for (auto tmpSegId : segmentIds) {
			auto& tmpSegment = segments[tmpSegId];
			auto tmpSegment2t = S3_S2(tmpSegment);
			if (tmpSegment2t.first == tmpSegment2t.second)
				continue;

			//auto& tmpRdsLineInfo = allLines[m_sizes[tmpSegId][0]];
			//if (tmpRdsLineInfo._line && tmpRdsLineInfo._line->lineType == RdsLine::LineType::eOther)
				//continue;

			point_2t grapedPt;
			bool currPtGraped = false;
			std::vector<segment_2t> grapedSegs;
			if (GRAP_POINT(currSeg.first, tmpSegment2t, grapedPt, ADJUST_DIVERSION_DISTANCE_TOLERANCE)) {
				grapedSegs.push_back(segment_2t(currSeg.first, grapedPt));
				currPtGraped = true;
			}
			if (GRAP_POINT(currSeg.second, tmpSegment2t, grapedPt, ADJUST_DIVERSION_DISTANCE_TOLERANCE)) {
				grapedSegs.push_back(segment_2t(currSeg.second, grapedPt));
				currPtGraped = true;
			}
			if (grapedSegs.size() < 2) {
				if (GRAP_POINT(tmpSegment2t.first, currSeg, grapedPt, ADJUST_DIVERSION_DISTANCE_TOLERANCE)) {
					grapedSegs.push_back(segment_2t(tmpSegment2t.first, grapedPt));
				}
				if (GRAP_POINT(tmpSegment2t.second, currSeg, grapedPt, ADJUST_DIVERSION_DISTANCE_TOLERANCE)) {
					grapedSegs.push_back(segment_2t(tmpSegment2t.second, grapedPt));
				}
			}
			if (grapedSegs.size() < 2) {
				continue;
			}

			if (currPtGraped) {
				setupNearestPt(segment.first, tmpSegment);
				setupNearestPt(segment.second, tmpSegment);
			}
			for (int idx = 0; idx < grapedSegs.size() - 1; idx++) {
				auto& cSeg = grapedSegs[idx];
				auto& nSeg = grapedSegs[idx + 1];
				if (!bg::intersects(cSeg, nSeg)) {
					auto d1 = bg::distance(cSeg.first, cSeg.second);
					auto d2 = bg::distance(nSeg.first, nSeg.second);
					if (d1 < ADJUST_DIVERSION_DISTANCE_TOLERANCE || d2 < ADJUST_DIVERSION_DISTANCE_TOLERANCE) {
						segDisPair.push_back(std::make_pair(tmpSegId, (d1 + d2) / 2));
						break;
					}
				}
			}
		}

		if (!segDisPair.empty()) {
			std::sort(segDisPair.begin(), segDisPair.end(), [](auto& p1, auto& p2) {
				return p1.second < p2.second;
			});
			for_each(segDisPair.begin(), segDisPair.end(), [&](auto& p) {
				nearestSegmentIds.push_back(p.first);
			});
		}
		if (existNearestGrappedPt) {
			nearestPt = nearestGrappedPt;
		} else if (existNearestSegPt) {
			nearestPt = nearestSegPt;
		}
		return existNearestSegPt || existNearestGrappedPt;
	}

	void DiversionCompiler::getPolygonPoints(std::vector<MapPoint3D64>& polygonVertexes, std::vector<MapPoint3D64>& points, int excludeIndex)
	{
		// 处理排除点为首尾点
		MapPoint3D64 prevVertex = {};
		if (excludeIndex == 0 || excludeIndex == polygonVertexes.size() - 1) {
			for (auto iter = polygonVertexes.begin() + 1; iter < polygonVertexes.end() - 1; iter++) {
				auto& point = *iter;
				if (!floatEqual(point.pos.distanceSquare(prevVertex.pos), 0)) {
					points.push_back(point);
					prevVertex = point;
				}
			}
			return;
		}

		auto excludeIter = mapPoint3D64_iterator(polygonVertexes, polygonVertexes[excludeIndex]);
		for (auto iter = excludeIter + 1; iter != polygonVertexes.end(); iter++) {
			auto& point = *iter;
			if (!floatEqual(point.pos.distanceSquare(prevVertex.pos), 0)) {
				points.push_back(point);
				prevVertex = point;
			}
		}
		for (auto iter = polygonVertexes.begin() + 1; iter != excludeIter; iter++) {
			auto& point = *iter;
			if (!floatEqual(point.pos.distanceSquare(prevVertex.pos), 0)) {
				points.push_back(point);
				prevVertex = point;
			}
		}
	}

	void DiversionCompiler::adjustPoint(int ptIndex, MapPoint3D64& pt, std::vector<MapPoint3D64>& polygonVertexes, point_t& toPt)
	{
		MapPoint3D64 tmpPos = MapPoint3D64_make(toPt.get<0>(), toPt.get<1>(), toPt.get<2>());
		tmpPos.z /= 10;
		auto ptDistance = pt.pos.distance(tmpPos.pos);
		if (ptDistance < ADJUST_DIVERSION_IGNORE_DISTANCE_TOLERANCE) { // 本身很近时直接返回 
			pt = tmpPos;
			return;
		}

		std::vector<MapPoint3D64> points;
		getPolygonPoints(polygonVertexes, points, ptIndex);
		auto pointsLine = LINESTRING_2T(points);

		linestring_2t clipper;
		auto pt2d = POINT_2T(pt);
		segment_2t tmpSeg{ pt2d, P3_P2(toPt) };
		auto tmpSegEx = SEGMENT_2T_EX(tmpSeg, ADJUST_DIVERSION_DISTANCE_TOLERANCE / 2);
		clipper.push_back(tmpSegEx.first);
		clipper.push_back(tmpSegEx.second);

		std::vector<point_2t> intersectPoints;
		bg::intersection(pointsLine, clipper, intersectPoints);
		for (auto& intersectPoint : intersectPoints) {
			auto intersectDistance = bg::distance(POINT_2T(tmpPos), intersectPoint);
			if (intersectDistance < ptDistance)
				return;
		}
		// 拉到车道线
		pt = tmpPos;
	}

	void DiversionCompiler::adjustDiversion(std::vector<MapPoint3D64>& vertexes, std::vector<rdsLineInfo>& allLines, rtree_type_segment& rtree)
	{
		UNREFERENCED_PARAMETER(allLines);
		auto getMinZValue = [](point_t& pt, const segment_t& segment)->double {
			segment_t verSeg;
			double zValue = 0;
			bg::closest_points(pt, segment, verSeg);
			double firstZVal = std::abs(verSeg.first.get<2>() - pt.get<2>());
			if (firstZVal > zValue)
				zValue = firstZVal;
			double secondZVal = std::abs(verSeg.second.get<2>() - pt.get<2>());
			if (secondZVal > zValue)
				zValue = secondZVal;
			return zValue;
		};

		auto querySegments = [&](point_t& pt, segment_t& pcSeg, rtree_type_segment& rtree, const std::vector<segment_t>& segments)->std::set<size_t> {
			auto segRing = getSegmentBox(pcSeg, ADJUST_DIVERSION_DISTANCE_TOLERANCE);
			ring_2t segRing2T = R3_R2(segRing);
			bg::correct(segRing2T);

			box_t outBox;
			std::set<size_t> segmentIds;
			bg::envelope(segRing, outBox);
			rtree.query(bgi::intersects(outBox),
				boost::make_function_output_iterator([&](size_t const& id) {
					auto& segment = segments[id];
					if (bg::intersects(S3_S2(segment), segRing2T)) {
						if (getMinZValue(pt, segment) < MAX_HEIGHT) {
							segmentIds.emplace(id);
						}
					}
				})
			);
			return segmentIds;
		};

		auto getLineSegments = [&](MapPoint3D64& pt, segment_t& pcSeg, const std::vector<segment_t>& segments, std::vector<size_t>& nearestSegIds) {
			point_t nearestPt{};
			auto tPt = POINT_T(pt);
			std::vector<size_t> nearestIds;
			std::set<size_t> segmentIds = querySegments(tPt, pcSeg, rtree, segments);
			getNearestSegments(tPt, pcSeg, segments, segmentIds, nearestIds, nearestPt);
			for (auto nearestId : nearestIds) {
				auto beginIter = nearestSegIds.begin();
				auto endIter = nearestSegIds.end();
				if (std::find(beginIter, endIter, nearestId) == endIter)
					nearestSegIds.push_back(nearestId);
			}
		};

		auto adjustPointToSegment = [&](int ptIndex, MapPoint3D64& pt, segment_t& pcSeg, const std::vector<segment_t>& segments) {
			point_t nearestPt{};
			auto tPt = POINT_T(pt);
			std::vector<size_t> nearestIds;
			std::set<size_t> segmentIds = querySegments(tPt, pcSeg, rtree, segments);
			if (getNearestSegments(tPt, pcSeg, segments, segmentIds, nearestIds, nearestPt)) {
				adjustPoint(ptIndex, pt, vertexes, nearestPt);
			}
		};

		// 获取导流带边对应的车道线段
		std::vector<size_t> nearestSegIds;
		for (int idx = 1; idx < vertexes.size(); idx++) {
			auto& prevPt = vertexes[idx - 1];
			auto& currPt = vertexes[idx];
			auto pPt = POINT_T(prevPt);
			auto cPt = POINT_T(currPt);
			segment_t pcSeg = segment_t(pPt, cPt);
			if (idx - 1 == 0) { // 首点
				getLineSegments(prevPt, pcSeg, m_segments, nearestSegIds);
			}

			getLineSegments(currPt, pcSeg, m_segments, nearestSegIds);
		}

		// 把车道线上的点插到导流带上
		std::vector<point_t> lineSegPts;
		for (auto& nearestSegId : nearestSegIds) {
			segment_t& lineSeg = m_segments[nearestSegId];
			lineSegPts.push_back(lineSeg.first);
			lineSegPts.push_back(lineSeg.second);
		}
		auto ip_line = std::unique(lineSegPts.begin(), lineSegPts.end(), point_t_compare);
		lineSegPts.resize(std::distance(lineSegPts.begin(), ip_line));
		for (auto lineSegIdx = 0; lineSegIdx < lineSegPts.size(); lineSegIdx++) {
			auto& linePt = lineSegPts[lineSegIdx];
			MapPoint3D64 tmpPos = MapPoint3D64_make(linePt.get<0>(), linePt.get<1>(), linePt.get<2>());
			tmpPos.z /= 10;
			size_t si, ei;
			MapPoint3D64 minGrappedPt = {};
			if (GrapPointAlgorithm::grapOrMatchNearestPoint(tmpPos, vertexes, minGrappedPt, si, ei)) {
				auto minDistance = minGrappedPt.pos.distance(tmpPos.pos);
				if (minDistance < ADJUST_DIVERSION_IGNORE_DISTANCE_TOLERANCE) // 本身很近无需插点
					continue;
				auto siDis = minGrappedPt.pos.distance(vertexes[si].pos);
				if (siDis < ADJUST_DIVERSION_IGNORE_DISTANCE_TOLERANCE) // 离首点很近无需插点
					continue;
				auto eiDis = minGrappedPt.pos.distance(vertexes[ei].pos);
				if (eiDis < ADJUST_DIVERSION_IGNORE_DISTANCE_TOLERANCE) // 离尾点很近无需插点
					continue;

				if (minDistance < ADJUST_DIVERSION_DISTANCE_TOLERANCE) {
					auto insertIter = mapPoint3D64_iterator(vertexes, vertexes[ei]);
					vertexes.insert(insertIter, minGrappedPt);
				}
			}
		}

		int nSize = vertexes.size();
		bool frontEqualBack = false;
		auto& frontPt = vertexes.front();
		auto& backPt = vertexes.back();
		if (frontPt.pos == backPt.pos) {
			frontEqualBack = true;
			nSize--;
		}

		// 把导流带拉到车道线上
		for (int idx = 1; idx < nSize; idx++) {
			auto& prevPt = vertexes[idx - 1];
			auto& currPt = vertexes[idx];
			auto pPt = POINT_T(prevPt);
			auto cPt = POINT_T(currPt);
			segment_t pcSeg = segment_t(pPt, cPt);
			if (idx - 1 == 0) { // 首点
				adjustPointToSegment(idx - 1, prevPt, pcSeg, m_segments);
			}

			adjustPointToSegment(idx, currPt, pcSeg, m_segments);
		}
		if (frontEqualBack) {
			backPt = frontPt;
		}
	}

	void DiversionCompiler::adjustDiversionZValue(const std::vector<HadLaneGroup*>& laneGroups, std::vector<MapPoint3D64>& vertexes, std::vector<Triangle>& polyTriangles)
	{
		auto pDiversionPoly = vertexes;
		coordinatesTransform.convert(pDiversionPoly.data(), pDiversionPoly.size());
		auto tmpDiversionRing = RING_2T(pDiversionPoly);
		bg::correct(tmpDiversionRing);

		std::set<size_t> rdsIntersectionIds;
		auto& gridIntersectionDatas = compilerData.m_rdsIntersections;
		getIntersectionBox2TRTree()->query(bgi::intersects(BOX_2T(pDiversionPoly)),
			boost::make_function_output_iterator([&](size_t const& id) {
				auto tmpPoints = gridIntersectionDatas[id]._intersectionPoints;
				auto tmpRing = gridIntersectionDatas[id]._intersectionPoly2T;
				bg::correct(tmpRing);
				if (bg::intersects(tmpDiversionRing, tmpRing))
				{
					point_t tmpCenterPoint, tmpFillAreaCenterPoint;
					bg::centroid(BOX_T(pDiversionPoly), tmpCenterPoint);
					bg::centroid(BOX_T(tmpPoints), tmpFillAreaCenterPoint);
					if (std::abs(tmpCenterPoint.get<2>() - tmpFillAreaCenterPoint.get<2>()) < 3000)
						rdsIntersectionIds.emplace(id);
				}
			}));

		std::vector<Triangle> surfaceTriangles;
		for (auto rdsIntersectionId : rdsIntersectionIds)
		{
			auto& intersectionInfo = compilerData.m_rdsIntersections[rdsIntersectionId];
			if (intersectionInfo._intersectionPoints.size() < 3)
				continue;

			std::vector<ring_2t> dRings;
			std::vector<Triangle> intersectionTriangles;
			LinearInterpolationTriangleSurface::triangularize(coordinatesTransform, 
				intersectionInfo._originIntersctPoly.vertexes, PolygonDirection_unknown, intersectionTriangles);
			surfaceTriangles.insert(surfaceTriangles.end(), intersectionTriangles.begin(), intersectionTriangles.end());
			for (auto& tri : intersectionTriangles)
				dRings.push_back(tri.trianglePoly);
			//printf("");
		}
		// 跨网格时RdsRoad数据不存在,改用车道组
		for (auto& laneGroup : laneGroups)
		{
			if (laneGroup->inIntersection)
				continue;

			LineString3d leftSide, rightSide;
			getLaneGroupBoundary(laneGroup, leftSide, rightSide);
			if (leftSide.vertexes.empty() || rightSide.vertexes.empty())
				continue;

			std::vector<Triangle> roadTriangles;
			LinearInterpolationTriangleSurface::triangularizeStroke(coordinatesTransform, leftSide.vertexes, rightSide.vertexes, roadTriangles);
			surfaceTriangles.insert(surfaceTriangles.end(), roadTriangles.begin(), roadTriangles.end());
		}

		if (polyTriangles.empty()) {
			LinearInterpolationTriangleSurface::triangularizeWMT(coordinatesTransform, vertexes, polyTriangles);
		}

		std::vector<MapPoint3D64> lineOnSurface;
		LinearInterpolationTriangleSurface::interpolationPolygon(coordinatesTransform, surfaceTriangles, polyTriangles, vertexes, lineOnSurface);
		vertexes = lineOnSurface;
	}

	void DiversionCompiler::filterUTurnLA(std::vector<HadRelLaneGroupAssociation*>& laneGroupAssociations)
	{
		auto findDirectTypeIter = [](std::vector<HadRelLaneGroupAssociation*>& las, int directType)->
			std::vector<HadRelLaneGroupAssociation*>::iterator {
			return std::find_if(las.begin(), las.end(), [&](HadRelLaneGroupAssociation* pLA)->bool {
				return pLA->directType == directType;
			});
		};

		auto findLgAssociationIter = [&](HadRelLaneGroupAssociation* la)->
			std::vector<HadRelLaneGroupAssociation*>::iterator {
			auto begin = laneGroupAssociations.begin();
			auto end = laneGroupAssociations.end();
			return std::find_if(begin, end, [&](HadRelLaneGroupAssociation* pLA)->bool {
				return pLA == la;
				});
		};

		std::map<int64, std::vector<HadRelLaneGroupAssociation*>> lgAssociations;
		for (auto& lgAssociation : laneGroupAssociations) {
			int64 firstOriginId = lgAssociation->firstLaneBoundary->originId;
			auto firstIter = lgAssociations.find(firstOriginId);
			if (firstIter == lgAssociations.end()) {
				std::vector<HadRelLaneGroupAssociation*> vec;
				vec.push_back(lgAssociation);
				lgAssociations.emplace(firstOriginId, vec);
			}
			else {
				firstIter->second.push_back(lgAssociation);
			}

			int64 secondOriginId = lgAssociation->secondLaneBoundary->originId;
			auto secondIter = lgAssociations.find(secondOriginId);
			if (secondIter == lgAssociations.end()) {
				std::vector<HadRelLaneGroupAssociation*> vec;
				vec.push_back(lgAssociation);
				lgAssociations.emplace(secondOriginId, vec);
			}
			else {
				secondIter->second.push_back(lgAssociation);
			}
		}

		for (auto& iter : lgAssociations) {
			auto diffIter = findDirectTypeIter(iter.second, 3);
			auto sameIter = findDirectTypeIter(iter.second, 2);
			if (diffIter != iter.second.end() && sameIter != iter.second.end()) {
				(*diffIter)->applyDiversion = true;
				(*diffIter)->firstLaneBoundary->filterOverlappingBoundary = true;
				(*diffIter)->secondLaneBoundary->filterOverlappingBoundary = true;

				(*sameIter)->applyDiversion = true;
				(*sameIter)->firstLaneBoundary->filterOverlappingBoundary = true;
				(*sameIter)->secondLaneBoundary->filterOverlappingBoundary = true;
				auto diffLaIter = findLgAssociationIter(*diffIter);
				if (diffLaIter != laneGroupAssociations.end())
				{
					laneGroupAssociations.erase(diffLaIter);
				}
				auto sameLaIter = findLgAssociationIter(*sameIter);
				if (sameLaIter != laneGroupAssociations.end())
				{
					// 清空压盖导流区列表,后面会应用于LA补面
					(*sameLaIter)->overedFillAreas.clear();
					laneGroupAssociations.erase(sameLaIter);
				}
			}
		}
	}

	void DiversionCompiler::applyDiversionForFillArea(HadFillArea* const pFillArea, const std::vector<HadGrid*>& grids)
	{
		// 过滤掉不相交且不在车道组上的导流带
		BoundingBox2d currBbox = BoundingBox2d::expand(pFillArea->extent, 1e4);
		std::vector<HadLaneGroup*> nearbyLaneGroups = SpatialSeacher::seachNearby(grids, currBbox);
		bool hasIntersectLaneGroup = false;
		for (auto nearbyLaneGroup : nearbyLaneGroups) {
			if (pFillArea->applyDiversion) {
				return;
			}

			if (hasIntersect(pFillArea, nearbyLaneGroup) || onLaneGroup(pFillArea, nearbyLaneGroup)) {
				hasIntersectLaneGroup = true;
			}
		}

		if (!hasIntersectLaneGroup) {
			pFillArea->applyDiversion = true;
		}
	}

	void DiversionCompiler::getNearbyFillAreas(HadFillArea* const pFillArea, const std::vector<HadGrid*>& grids, std::vector<HadFillArea*>& nearbyFillAreas)
	{
		BoundingBox2d currBbox = BoundingBox2d::expand(pFillArea->extent, 1e4);
		NdsRect ndsRect = BoundingBox2d::toNdsRect(currBbox);

		NdsGridIdIterator gridIdIterator;
		NdsGridIdIterator_constructWithNdsRect(&gridIdIterator, ndsRect, HAD_GRID_NDS_LEVEL);
		NdsGridId gridId;
		while ((gridId = NdsGridIdIterator_next(&gridIdIterator)) != invalidNdsGridId)
		{
			int32 meshId = NdsGridId_toMeshId(gridId);
			auto iter = std::find_if(grids.begin(), grids.end(), [&](HadGrid* pGrid)->bool {
				if (pGrid->getId() == meshId)
				{
					return true;
				}
				else
				{
					return false;
				}
				});
			if (iter == grids.end())
				continue;

			HadGrid* grid = *iter;
			if (grid->getBoundingbox2d().overlap(currBbox, 0) ||
				grid->getBoundingbox2d().contain(currBbox, 0))
			{
				std::vector<HadElement*>& objects = grid->query(ElementType::HAD_OBJECT_FILL_AREA);
				for (auto& o : objects)
				{
					HadFillArea* nearbyFillArea = (HadFillArea*)o;
					if (pFillArea == nearbyFillArea) {
						continue;
					}

					// 不相交且不包含的过滤掉
					BoundingBox2d nearbyBbox = nearbyFillArea->extent;
					if (!(currBbox.overlap(nearbyBbox, 0) ||
						currBbox.contain(nearbyBbox, 0) ||
						nearbyBbox.contain(currBbox, 0))) {
						continue;
					}

					if (std::find(nearbyFillAreas.begin(), nearbyFillAreas.end(), nearbyFillArea) == nearbyFillAreas.end()) {
						nearbyFillAreas.push_back(nearbyFillArea);
					}
				}
			}
		}
	}

	inline ring_t DiversionCompiler::getSegmentBox(const segment_t& pabSeg, double offset)
	{
		auto& startPt = pabSeg.first;
		auto& endPt = pabSeg.second;
		auto va = S2_V2(S3_S2(pabSeg));
		auto v_n = V2_N(va);
		auto va_n = vector_2t(v_n.get<1>(), -v_n.get<0>());
		auto vb_n = vector_2t(-v_n.get<1>(), v_n.get<0>());

		ring_t tmpRing;
		tmpRing.push_back(point_t(startPt.get<0>() + va_n.get<0>() * offset, startPt.get<1>() + va_n.get<1>() * offset, startPt.get<2>()));
		tmpRing.push_back(point_t(endPt.get<0>() + va_n.get<0>() * offset, endPt.get<1>() + va_n.get<1>() * offset, endPt.get<2>()));
		tmpRing.push_back(point_t(endPt.get<0>() + vb_n.get<0>() * offset, endPt.get<1>() + vb_n.get<1>() * offset, endPt.get<2>()));
		tmpRing.push_back(point_t(startPt.get<0>() + vb_n.get<0>() * offset, startPt.get<1>() + vb_n.get<1>() * offset, startPt.get<2>()));
		return tmpRing;
	}

	bool DiversionCompiler::hasIntersect(HadFillArea* const pFillArea, const HadLaneGroup* laneGroup)
	{
		static const double OFFSET_IN_METER = 1.5;
		const float METER_PER_LAT_UNIT = 1.11f;
		const float LAT_UNIT_PER_METER = 1 / METER_PER_LAT_UNIT;
		if (pFillArea->polygon.vertexes.empty()) {
			return false;
		}

		ClipperLib::Path fillAreaPath;
		MapPoint64 basePoint = pFillArea->polygon.vertexes[0].pos;
		for (int i = 0; i < pFillArea->polygon.vertexes.size(); i++) {
			MapPoint64 point = pFillArea->polygon.vertexes[i].pos;
			fillAreaPath << ClipperLib::IntPoint(point.lon - basePoint.lon, point.lat - basePoint.lat, 0.0);
		}

		ClipperLib::Paths roadpaths;
		auto& laneBoundarys = laneGroup->laneBoundaries;
		for each (auto laneBoundary in laneBoundarys) {
			ClipperLib::Path laneBoundaryPath;
			std::vector<MapPoint3D64> laneBoundaryVertexes = laneBoundary->location.vertexes;
			for (int i = 0; i < laneBoundaryVertexes.size(); i++) {
				laneBoundaryPath << ClipperLib::IntPoint(laneBoundaryVertexes[i].pos.lon - basePoint.lon, laneBoundaryVertexes[i].pos.lat - basePoint.lat, 0.0);
			}

			roadpaths << laneBoundaryPath;
		}

		auto& roadBoundarys = laneGroup->roadBoundaries;
		for each (auto roadBoundary in roadBoundarys) {
			ClipperLib::Path roadBoundary_path;
			std::vector<MapPoint3D64> roadBoundaryVertexes = roadBoundary->location.vertexes;
			for (int i = 0; i < roadBoundaryVertexes.size(); i++) {
				roadBoundary_path << ClipperLib::IntPoint(roadBoundaryVertexes[i].pos.lon - basePoint.lon, roadBoundaryVertexes[i].pos.lat - basePoint.lat, 0.0);
			}

			roadpaths << roadBoundary_path;
		}


		//导流区扩大
		ClipperLib::ClipperOffset offseter;
		offseter.AddPath(fillAreaPath, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
		ClipperLib::Paths extObjePaths;
		offseter.Execute(extObjePaths, OFFSET_IN_METER * LAT_UNIT_PER_METER * 300);
		ClipperLib::Path& extObjPath = extObjePaths[0];

		//加入车道，道路边界
		ClipperLib::Clipper clipper;
		clipper.AddPaths(roadpaths, ClipperLib::ptSubject, false);
		clipper.AddPath(extObjPath, ClipperLib::ptClip, true);
		extObjPath << ClipperLib::IntPoint(extObjPath.at(0).X, extObjPath.at(0).Y, 0.0);


		ClipperLib::PolyTree polyTree;
		clipper.Execute(ClipperLib::ctIntersection, polyTree, ClipperLib::pftEvenOdd);
		if (polyTree.Total() == 0) {
			return false;
		}

		return true;
	}

	bool DiversionCompiler::onLaneGroup(HadFillArea* const pFillArea, const HadLaneGroup* laneGroup)
	{
		ClipperLib::Path laneBoundaryPath;
		if (pFillArea->polygon.vertexes.empty()) {
			return false;
		}

		auto& laneBoundarys = laneGroup->laneBoundaries;
		if (laneBoundarys.size() < 2)
		{
			return false;
		}

		//最左、最右车道合成多边形
		MapPoint64 basePoint = pFillArea->polygon.vertexes[0].pos;
		auto firstLaneBoundaryVertexes = laneBoundarys[0]->location.vertexes;
		for (int i = 0; i < firstLaneBoundaryVertexes.size(); i++) {
			laneBoundaryPath << ClipperLib::IntPoint(firstLaneBoundaryVertexes[i].pos.lon - basePoint.lon, firstLaneBoundaryVertexes[i].pos.lat - basePoint.lat, 0.0);
		}

		auto lastLaneBoundaryVertexes = laneBoundarys[laneBoundarys.size() - 1]->location.vertexes;
		for (int i = lastLaneBoundaryVertexes.size() - 1; i >= 0; i--) {
			laneBoundaryPath << ClipperLib::IntPoint(lastLaneBoundaryVertexes[i].pos.lon - basePoint.lon, lastLaneBoundaryVertexes[i].pos.lat - basePoint.lat, 0.0);
		}
		laneBoundaryPath << laneBoundaryPath[0];

		//判断导流区点是否在多边形内
		for (int i = 0; i < pFillArea->polygon.vertexes.size(); i++) {
			MapPoint64 point = pFillArea->polygon.vertexes[i].pos;
			if (ClipperLib::PointInPolygon(ClipperLib::IntPoint(point.lon - basePoint.lon, point.lat - basePoint.lat, 0.0), laneBoundaryPath)) {
				return true;
			}
		}

		return false;
	}

	double DiversionCompiler::LA_LANE_DISTANCE_TOLERANCE = 10000; // ≈10m
	void DiversionCompiler::applyDiversionForFillArea(HadRelLaneGroupAssociation* const lgAssociation, HadLaneGroup* const pLaneGroup,
		std::vector<std::pair<LineString3d, double>>& laneDistance, ClipperLib::Path& laPath, std::vector<HadFillArea*>& laPathApplyedFillAreas) {
		auto applyDiversionFillArea = [&](HadFillArea* pFillArea) {
			if (std::find(laPathApplyedFillAreas.begin(), laPathApplyedFillAreas.end(), pFillArea) != laPathApplyedFillAreas.end()) {
				return;
			}

			ClipperLib::Path fillAreaPath;
			for (auto& vertex : pFillArea->polygon.vertexes) {
				fillAreaPath << ClipperLib::IntPoint(vertex.pos.lon, vertex.pos.lat, vertex.z);
			}

			ClipperLib::Clipper clipper;
			ClipperLib::PolyTree polyTree;
			clipper.AddPath(fillAreaPath, ClipperLib::ptSubject, true);
			clipper.AddPath(laPath, ClipperLib::ptClip, true);
			clipper.Execute(ClipperLib::ctIntersection, polyTree, ClipperLib::pftEvenOdd);
			if (polyTree.Total())
			{
				double totalPolyArea = 0;
				double laArea = std::abs(ClipperLib::Area(laPath));
				double fillArea = std::abs(ClipperLib::Area(fillAreaPath));
				for (auto polyNode : polyTree.Childs) {
					totalPolyArea += std::abs(ClipperLib::Area(polyNode->Contour));
				}

				double polyLaAreaRate = totalPolyArea / laArea;
				double polyFillAreaRate = totalPolyArea / fillArea;
				if (polyFillAreaRate > 0.85 && polyLaAreaRate > 0.3) {
					laPathApplyedFillAreas.push_back(pFillArea);
				}
			}
		};

		// 判断LA关系生成的两条边长度
		if (laneDistance.size() == 2) {
			auto& firstLaneDistance = laneDistance[0].second;
			auto& secondLaneDistance = laneDistance[1].second;
			if (fabs(firstLaneDistance - secondLaneDistance) > LA_LANE_DISTANCE_TOLERANCE) {
				return;
			}
		}

		for (HadFillArea* overedFillArea : lgAssociation->overedFillAreas) {
			if (!overedFillArea->applyDiversion) {
				applyDiversionFillArea(overedFillArea);
			}
		}

		for (auto pObject : pLaneGroup->objects) {
			if (pObject->objectType == ElementType::HAD_OBJECT_FILL_AREA) {
				HadFillArea* pFillArea = (HadFillArea*)pObject;
				HadFillArea* mergedFillArea = pFillArea->mergedFillArea;
				if (mergedFillArea && !mergedFillArea->applyDiversion) {
					applyDiversionFillArea(mergedFillArea);
				}

				if (pFillArea->applyDiversion) {
					continue;
				}

				applyDiversionFillArea(pFillArea);
			}
		}
	}

	void DiversionCompiler::filterLargeLaDiversion(
		ClipperLib::Path& laPath, LineString3d& centerLine, std::vector<HadFillArea*>& laPathApplyedFillAreas)
	{
		if (laPath.empty() || laPathApplyedFillAreas.empty()) {
			return;
		}

		double distance = 0;
		double CENTER_LINE_DISTANCE_TOLERANCE = 50000; // ≈50m
		for (int idx = 0; idx < centerLine.vertexes.size() - 1; idx++) {
			auto& currPt = centerLine.vertexes[idx];
			auto& nextPt = centerLine.vertexes[idx + 1];
			distance += currPt.pos.distance(nextPt.pos);
		}
		// 本身是比较小的导流带,此时不做过滤逻辑
		if (distance < CENTER_LINE_DISTANCE_TOLERANCE) {
			return;
		}

		double fillArea{0};
		double laArea = std::abs(ClipperLib::Area(laPath));
		for (HadFillArea* pFillArea : laPathApplyedFillAreas) {
			ClipperLib::Path fillAreaPath;
			for (auto& vertex : pFillArea->polygon.vertexes) {
				fillAreaPath << ClipperLib::IntPoint(vertex.pos.lon, vertex.pos.lat, vertex.z);
			}

			fillArea += std::abs(ClipperLib::Area(fillAreaPath));
		}

		// LA生成的面比原始面大很多时清除LA面
		double polyLaAreaRate = laArea / fillArea;
		if (polyLaAreaRate > 2) {
			laPathApplyedFillAreas.clear();
		}
	}

	void DiversionCompiler::groupLaneGroupAssociations(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby,
		std::vector<HadRelLaneGroupAssociation*>& allLaneGroupAssociations, bool* pApplyDiversion)
	{
		BoundingBox2d gridBbox = BoundingBox2d::expand(pGrid->getBoundingbox2d(), 1e4);
		auto containsGridLaneGroup = [&](HadLaneGroup* const laneGroup) {
			if (laneGroup != nullptr && laneGroup->owner == pGrid) {
				return true;
			}

			// 不相交且不包含的过滤掉
			if (laneGroup != nullptr) {
				BoundingBox2d laneGroupBbox = laneGroup->extent.boundingbox2d();
				if (gridBbox.overlap(laneGroupBbox, 0) ||
					gridBbox.contain(laneGroupBbox, 0) ||
					laneGroupBbox.contain(gridBbox, 0)) {
					return true;
				}
			}

			return false;
		};

		for (auto la : pGrid->query(ElementType::HAD_LG_ASSOCIATIOIN)) {
			HadLaneGroupAssociation* pLaneGroupAssociation = (HadLaneGroupAssociation*)la;
			for (auto& relLgAssociation : pLaneGroupAssociation->relLaneGroupAssociations) {
				HadRelLaneGroupAssociation* pLA = &relLgAssociation;
				if (pLA->firstLaneBoundary == nullptr || pLA->secondLaneBoundary == nullptr) {
					continue;
				}

				if (pApplyDiversion != nullptr)
				{
					if (*pApplyDiversion) {
						if (pLA->applyDiversion) // 过滤LA关系
							continue;
						if (std::find(allLaneGroupAssociations.begin(), allLaneGroupAssociations.end(), pLA) == allLaneGroupAssociations.end()) {
							allLaneGroupAssociations.push_back(pLA);
						}
					}
					else if (pLA->applyDiversion && pLA->groupId && pLA->overedFillAreas.empty())
					{
						if (std::find(allLaneGroupAssociations.begin(), allLaneGroupAssociations.end(), pLA) == allLaneGroupAssociations.end()) {
							allLaneGroupAssociations.push_back(pLA);
						}
					}
				}
				else
				{
					if (std::find(allLaneGroupAssociations.begin(), allLaneGroupAssociations.end(), pLA) == allLaneGroupAssociations.end()) {
						allLaneGroupAssociations.push_back(pLA);
					}
				}
			}
		}

		// process nearby
		for (auto nearbyGrid : nearby) {
			if (pGrid == nearbyGrid) {
				continue;
			}

			for (auto la : nearbyGrid->query(ElementType::HAD_LG_ASSOCIATIOIN)) {
				HadLaneGroupAssociation* pLaneGroupAssociation = (HadLaneGroupAssociation*)la;
				for (auto& relLgAssociation : pLaneGroupAssociation->relLaneGroupAssociations) {
					HadRelLaneGroupAssociation* pLA = &relLgAssociation;
					if (containsGridLaneGroup(pLA->first) || containsGridLaneGroup(pLA->second)) {
						if (pLA->firstLaneBoundary == nullptr || pLA->secondLaneBoundary == nullptr) {
							continue;
						}

						if (pApplyDiversion != nullptr)
						{
							if (*pApplyDiversion) {
								if (pLA->applyDiversion) // 过滤LA关系
									continue;
								if (std::find(allLaneGroupAssociations.begin(), allLaneGroupAssociations.end(), pLA) == allLaneGroupAssociations.end()) {
									allLaneGroupAssociations.push_back(pLA);
								}
							}
							else if (pLA->applyDiversion && pLA->groupId && pLA->overedFillAreas.empty())
							{
								if (std::find(allLaneGroupAssociations.begin(), allLaneGroupAssociations.end(), pLA) == allLaneGroupAssociations.end()) {
									allLaneGroupAssociations.push_back(pLA);
								}
							}
						}
						else
						{
							if (std::find(allLaneGroupAssociations.begin(), allLaneGroupAssociations.end(), pLA) == allLaneGroupAssociations.end()) {
								allLaneGroupAssociations.push_back(pLA);
							}
						}
					}
				}
			}
		}
	}

	std::string DiversionCompiler::getLaneGroupAssociationId(HadRelLaneGroupAssociation* lgAssociation)
	{
		std::string str;
		int64 litOriginId = lgAssociation->first->originId;
		int64 bigOriginId = lgAssociation->second->originId;
		if (litOriginId > bigOriginId) {
			std::swap(litOriginId, bigOriginId);
		}
		str += std::to_string(litOriginId);
		str += "_";
		str += std::to_string(bigOriginId);
		return str;
	}

}