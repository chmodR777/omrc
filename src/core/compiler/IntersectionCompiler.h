#pragma once
#include "Compiler.h"

namespace OMDB
{
	enum class LaneInfoType
	{
		ROAD_BOUNDARY,
		LANE_BOUNDARY
	};

	struct LaneInfo
	{
		int64 _originLaneId;
		LineString3d _originLocation;
		linestring_2t _line2d;
		LaneInfoType _type;
		double _length;
	};

	struct NodeInfo
	{
		int64 _originNodeId;
		MapPoint3D64 _originPosition;
		std::vector<LaneInfo> _line;
		point_t _point3d;
		point_2t _point2d;
	};

	class IntersectionCompiler : public Compiler
	{
	protected:
		virtual void compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile) override;

		void saveRdsIntersection(RdsIntersection* pIntersection, Polygon3d& originIntersctPoly, int64& originId);

		// 合并相连的停止线
		void combineNearbyStopLine(ring_2t& tmpRing, rtree_2type& rtree, std::vector<NodeInfo>& originNodes);

		// 获取最近k个NodeInfo信息
		std::vector<NodeInfo> getNearestNodeInfos(const point_2t& point, size_t k,
			rtree_2type& rtree, std::vector<NodeInfo>& originNodes);

		// box范围是否停止线
		inline bool isBoxStopLine(ring_2t& outBox, double paPbLength);

		// 根据起始、终点构建box,并返回BOX两个点的距离
		inline double getRingBox(const point_2t& startPt, const point_2t& endPt, ring_2t& outBox);

		// 取环上的点
		inline point_2t& getRingPt(ring_2t& ring, int index, int offset = 0);

		bool getExpandPolyByOffset(
			const ring_2t& line,
			const int64& offsetSize,
			ring_2t& resultRing);
		
		std::vector<MapPoint3D64> offsetPointForBoundary(
			const point_t& pa,
			const point_t& pb);

		std::vector<MapPoint3D64> stopLinePointForBoundary(
			const point_t& pa,
			const point_t& pb);

		void setIntersectionInfo(
			HadIntersection* const pIntersection);

		bool setLaneAndNodeInfo(
			HadLaneGroup* const pGroup);

		void setOutIntersectionNodeInfo();

		void updateStopLineData(
			HadLaneGroup* const pGroup);

		void clearInfoData();

	public:
		IntersectionCompiler(CompilerData& data) :Compiler(data) {};
		inline bool isSmoothPoint(
			const point_2t& currentPoint,
			const point_2t& prevPoint,
			const point_2t& nextPoint)
		{
			vector_2t d1 = S2_V2(prevPoint, currentPoint);
			vector_2t d2 = S2_V2(currentPoint, nextPoint);
			auto tmpValue = bg::dot_product(V2_N(d1), V2_N(d2));
			if (tmpValue > 0.995)
				return true;
			return false;
		}

		inline void getNearestLaneInfo(std::vector<LaneInfo>& laneInfos, LaneInfo& laneInfo);

		HadIntersection* m_hadInts;
		std::vector<linestring_2t> newIntersectionBoundaries;
		std::vector<linestring_2t> intersectionLaneLines;
		std::vector<linestring_2t> intersectionOnlyLaneLines;
		std::vector<point_2t> intersectionLaneLinePoints;
		std::vector<MapPoint3D64> originPoints;
		std::map<int64, NodeInfo> allNodes;
		std::vector<point_2t> refLinkNodes;
		std::vector<point_2t> roadBoundaryPts;
		std::vector<std::vector<MapPoint3D64>> originStopLineNearbySegments;
		std::vector<segment_t> stopLineNearbySegments;
		std::vector<std::vector<MapPoint3D64>> originStopLineSegments;
		std::vector<segment_t> stopLineSegments;
		std::vector<segment_t> connectStopLineSegments;
		std::vector<std::vector<MapPoint3D64>> frontPoints;
		std::vector<std::vector<MapPoint3D64>> backPoints;
		std::vector<MapPoint3D64> outIntersectionPoints;

		bg::strategy::buffer::join_miter join_strategy;
		bg::strategy::buffer::end_flat end_strategy;
		bg::strategy::buffer::point_square circle_strategy;
		bg::strategy::buffer::side_straight side_strategy;
	};
}

