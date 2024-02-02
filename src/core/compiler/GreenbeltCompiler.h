#pragma once
#include "Compiler.h"
namespace OMDB
{
	/// <summary>
	/// ÂÌ»¯´ø
	/// </summary>
	class GreenbeltCompiler : public Compiler
	{
	public:
		GreenbeltCompiler(CompilerData& data) :Compiler(data) {};
	protected:
		virtual void compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile) override;

		void matchNearbyRoadBoundary(HadGrid* const grid, const std::vector<HadGrid*>& nearby,
			std::map<int64, std::set<int64>>& forwardGroupTable, 
			std::map<int64, std::set<int64>>& backwardGroupTable);
		void findNearbyRoadBoundary(const std::vector<HadGrid*>& nearby, 
			HadLaneGroup* pLinkGroup, HadLaneGroup* pNearbyLinkGroup,
			HadRoadBoundary* &pLinkGroupRoadBoundary, HadRoadBoundary* &pNearbyLinkGroupRoadBoundary, 
			std::map<int64, std::set<int64>>& forwardGroupTable, std::map<int64, std::set<int64>>& backwardGroupTable);
		void createNearbyRoadBoundary(std::vector<MapPoint3D64>& points, std::vector<MapPoint3D64>& nearbyPoints, std::vector<MapPoint3D64> &roadBoundaryPoints);
		void createTopologyRoadBoundary(HadRoadBoundary* boundary, std::vector<MapPoint3D64>& boundaryVertexes, std::vector<MapPoint3D64> &nearbyBoundaryVertexes,
			HadGrid* const grid, std::map<int64, std::set<int64>> &forwardGroupTable, std::map<int64, std::set<int64>> &backwardGroupTable, std::set<int64> &visitedIds);
		std::set<int64> findNearbyGroups(int64 originId, std::map<int64, std::set<int64>>& forwardGroupTable, std::map<int64, std::set<int64>>& backwardGroupTable);
		bool isNearbyLinkGroups(const std::vector<HadGrid*>& nearby, 
			HadLaneGroup* pLinkGroup, HadRoadBoundary* boundary, MapPoint3D64& point, 
			HadLaneGroup* pNearbyLinkGroup, HadRoadBoundary* nearbyBoundary, MapPoint3D64& nearbyPoint);
		bool existMiddleLaneGroup(MapPoint3D64& pt, HadLaneGroup* pLinkGroup, HadRoadBoundary* boundary,
			HadLaneGroup* pNearbyLinkGroup, HadRoadBoundary* nearbyBoundary, HadLaneGroup* middleLangGroup);
		void makePolygon(std::vector<MapPoint3D64>& boundaryVertexes, std::vector<MapPoint3D64>& nearbyBoundaryVertexes, Polygon3d& polygon);
		void createConnectedRoadBoundary(HadLaneGroup* pLinkGroup, HadRoadBoundary* boundary, std::vector<MapPoint3D64>& boundaryVertexes);
		forceinline bool filterRoadBoundary(HadRoadBoundary* pBoundary, HadRoadBoundary* pRoadBoundary);
		bool hasIntersect(std::vector<MapPoint3D64>& points1, std::vector<MapPoint3D64>& points2);
		BoundingBox2d makeBoundingBox2d(MapPoint3D64& point, MapPoint3D64& nearbyPoint);
		void expandNearbyLinkGroups(HadLaneGroup* pLinkGroup, const std::vector<HadGrid*>& nearby, std::vector<HadLaneGroup*>& pNearbyLinkGroups);
		double calcBoundaryDistance(std::vector<MapPoint3D64>& points);
	private:
		static double ROAD_BOUNDARY_ZVALUE_TOLERANCE;
		static double ROAD_BOUNDARY_DISCARD_TOLERANCE;
		static double ROAD_BOUNDARY_DISTANCE_TOLERANCE;
		static double MAX_ROAD_BOUNDARY_DISTANCE_TOLERANCE;
		static double ROAD_BOUNDARY_EXTRA_DISTANCE_TOLERANCE;
	};
}

