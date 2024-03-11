#pragma once
#include "Compiler.h"
#include "map_point3d64_converter.h"

namespace OMDB
{
	struct gridQuad;
	struct gridVertex
	{
		gridVertex(
			const point_t& coordinate,
			int id)
			: _originPoint(coordinate),
			_id(id)
		{
		}

		int _id;
		point_t _originPoint;
		std::array<gridQuad*, 4> _quads = std::array<gridQuad*, 4>{nullptr, nullptr, nullptr, nullptr};
		int64 _height{ 0 };
	};

	struct gridQuad
	{
		gridQuad(
			gridVertex& vertexOne,
			gridVertex& vertexTwo,
			gridVertex& vertexThree,
			gridVertex& vertexForth,
			int id)
			: _vertices({ &vertexOne, &vertexTwo, &vertexThree, &vertexForth }),
			_id(id)
		{
		}

		int _id;
		std::array<gridVertex*, 4> _vertices;
		bool isBaseHeight{ false };
		int64 _height{0};
		box_2t _quadBox;
	};

	struct SlopeAdjustTask;
	using PtrSlopeAdjustTask = std::shared_ptr<SlopeAdjustTask>;
    class GroupCompiler : public Compiler
    {
	public:
		GroupCompiler(CompilerData& data) :Compiler(data) {};
    protected:
        virtual void compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile) override;

		void adjustRoad(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby);
		void setLaneGroupData(HadGrid* const pGrid);
		BoxRTree::Ptr getLaneGroupBoxRTree();
		BoxRTree::Ptr getLaneGroupBox2DRTree();
		void updateInterpolatedLine();
		void updateRefLine();

		void interpolateLane(PtrLaneGroupInfo& groupInfo);
		void interpolateObject(PtrLaneGroupInfo& groupInfo);
		void interpolateObject();
		void interpolateBoundary(PtrLaneGroupInfo& groupInfo);
		void interpolateLaneBoundary(PtrLaneGroupInfo& groupInfo);
		void interpolatePoints(std::vector<MapPoint3D64>& point, linestring_t& refLine);
		void interpolatePoints(std::vector<MapPoint3D64>& points, multi_linestring_t& refLine);
		bool adjustLine(
			const linestring_t& line,
			const double& height,
			const double slopeRatio,
			linestring_t& outLine,
			double& outHeight);
		void adjustLaneGroup(PtrLaneGroupInfo& groupInfo, double adjustHeight, const double slopeRatio, bool bFromStart);
		bool calcAdjustLineWillBeFinished(PtrLaneGroupInfo& groupInfo, double adjustHeight, const double slopeRatio, bool bFromStart, double& lastHeight);
		void grabLowTunnel(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby);
		void divideToGroup();
		void mergeGroup();
		void calcAdjustLaeGroup(const std::vector<PtrLaneGroupInfo>& topy_group, PtrLaneGroupInfo startGroup, std::vector<PtrLaneGroupInfo>& adjustGroup, std::vector<PtrSlopeAdjustTask>& adjustTasks);
		void adjust();
		bool overLapGroup(HadLaneGroup* lane1, HadLaneGroup* lane2, int32& height);
		
		size_t makeLaneGroupPolygon(HadLaneGroup* pGroup, MapPoint3D64*& polygon) override;

		//判断是否符合抽稀
		bool isSimplyfyLine(const PtrLaneGroupInfo& currentGroupInfo);

		//路面凹包插值和直线度较好的车道组边界抽稀
		void interpolateConvexHullPoints(HadGrid* const pGrid);

		void setCurrentLaneGroupData(HadGrid* const pGrid);

		// 构建网格的Rtree
		void constructLaneGroupRtree( HadGrid* const pGrid);

		//绝对高度转相对高度
		void modifyGridHeight(
			HadGrid* const pGrid, 
			const std::vector<HadGrid*>& nearby);

		void interpolateHeightForTriangle(
			const Eigen::VectorXd& a,
			const Eigen::VectorXd& b,
			const Eigen::VectorXd& c,
			const Eigen::VectorXd& normal,
			const Eigen::VectorXd& h,
			Eigen::VectorXd& location);

		//从数据加载抽稀放到编译阶段处理
		void simplifierLine(
			HadGrid* const pGrid, 
			const std::vector<HadGrid*>& nearby);

	private:
		std::vector<PtrLaneGroupInfo> allGroupDatas_;
		std::vector<box_t> allGroupBoxes;
		std::vector<box_t> allGroupBoxes2D;
		std::unordered_map<int64, PtrLaneGroupInfo> allGroupMaps_;
		std::set<int64> currentRaisedGroupIds;
		BoxRTree::Ptr rtreeLaneGroupBox;
		std::vector<PtrLaneGroupInfo> _lowfloorTunnels;
		std::unordered_map<int64, PtrLaneGroupInfo> _lowfloorTunnels2Maps;
		std::vector<std::vector<PtrLaneGroupInfo>> _groupedTunnels;
		FastTable<int64, std::vector<PtrLaneGroupInfo>> _groupId2TunnelsMaps;
		FastTable<int64, int32> _tunnel2GroupIdMaps;
		FastTable<int64, PtrLaneGroupInfo> _adjustLaneGroupMaps;
		HadGrid* m_pGrid;
		std::unordered_map<int64, PtrLaneGroupInfo> currentGridGroupMaps;
    };

	struct SlopeAdjustTask
	{
		PtrLaneGroupInfo laneInfo;
		double startHeight;
		double slope;
		bool bFromStartAdjust;
		PtrLaneGroupInfo laneInfo_From;
		void init(PtrLaneGroupInfo laneInfo_, double startHeight_,
		double slope_,
		bool bFromStartAdjust_,
		PtrLaneGroupInfo laneInfo_From_ = nullptr)
		{
			laneInfo = laneInfo_;
			startHeight = startHeight_;
			slope = slope_;
			bFromStartAdjust = bFromStartAdjust_;
			laneInfo_From = laneInfo_From_;

		}
	};
}
