#pragma once

#include "Compiler.h"
#include <map_point3d64_converter.h>
#include "geometry/map_rect64.h"
namespace OMDB
{

#define TUNNEL_THICKNESS (300)	 // mm 毫米
#define TUNNEL_HEIGHT (100000)	 // mm 毫米
#define TUNNEL_HEIGHT_MIX (100) // mm 毫米

	// 压盖关系
	enum TunnelLinkGlandType
	{
		enum_tunnelLink_gland_none,	 // 无压盖
		enum_tunnelLink_gland_total, // 全部
		enum_tunnelLink_gland_part	 // 部分
	};
	struct TunnelLinkGlandHeightInfo
	{

		void init(const MapPoint3D64 &startPoint, const MapPoint3D64 &endPoint, const int32 height)
		{
			this->startPoint = startPoint;
			this->endPoint = endPoint;
			this->height = height;
		}

		void init(const size_t startIndex, const size_t endIndex, const int32 height)
		{
			this->startIndex = startIndex;
			this->endIndex = endIndex;
			this->height = height;
		}

		size_t startIndex;
		size_t endIndex;
		MapPoint3D64 startPoint;
		MapPoint3D64 endPoint;
		int32 height;
	};

	using HadRelLaneGroupAssociation = HadLaneGroupAssociation::HadRelLaneGroupAssociation;

	// Y形隧道的方向
	enum TunnelBranchingType
	{
		// 未知
		enum_tunnel_unknow,
		// 向前分叉
		enum_tunnel_fork,
		// 合并进入
		enum_tunnel_merge
	};

	struct TunnelLink
	{
		void initTunnelLink(HadLaneGroup *const link)
		{
			_laneGroup = link;

			groupId = 0;
			height = TUNNEL_HEIGHT;
			thickness = TUNNEL_THICKNESS;
			glandType = enum_tunnelLink_gland_none;
			_middleLane = nullptr;
			_middleLaneBoundary = nullptr;
			isYBranchInterface = false;
		}

		HadLaneGroup *_laneGroup;

		// 后面的
		std::vector<TunnelLink *> _next;
		// 前面的
		std::vector<TunnelLink *> _previous;
		// Y型隧道方向
		std::vector<TunnelBranchingType> branchTypes;
		// 有LG关系
		std::unordered_map<TunnelLink *, HadRelLaneGroupAssociation> _laRelTunnel1;

		// 区间高度压盖关系
		TunnelLinkGlandType glandType;
		std::vector<TunnelLinkGlandHeightInfo> linkGlandHeightInfos;

		// 形状几何
		LineString3d _leftLine;
		LineString3d _middleLine;
		LineString3d _rightLine;
		HadLane *_middleLane;
		HadLaneBoundary *_middleLaneBoundary;

		int groupId;

		// 高度
		double height;
		// 厚度
		double thickness;
		// 形状盒
		MapRect64 box;
		// 扩大形状盒
		MapRect64 boxEx;

		//是分叉的接口处
		bool isYBranchInterface;
		// 是否有压盖
		bool isgGland()
		{
			return height != TUNNEL_HEIGHT;
		};
		int gridId()
		{
			if (_laneGroup)
			{
				return _laneGroup->owner->getId();
			}
			return 0;
		}
		bool nextisSingleBranch()
		{
			if (_laneGroup && _laneGroup->next.size() == 1)
			{
				if (_laneGroup->next[0]->previous.size() > 1)
				{
					return false;
				}
				return true;
			}

			return false;
		}
		bool previousisSingleBranch()
		{
			if (_laneGroup && _laneGroup->previous.size() == 1)
			{
				if (_laneGroup->previous[0]->next.size() > 1)
				{
					return false;
				}
				return true;
			}

			return false;
		}

		bool bNPSingleBranch(bool bForward = true)
		{
			return bForward ? nextisSingleBranch() : previousisSingleBranch();
		}
		TunnelLink *nextNo1TunnelLink()
		{
			if (!_next.empty())
			{
				return _next[0];
			}
			return nullptr;
		}
		TunnelLink *previousNo1TunnelLink()
		{
			if (!_previous.empty())
			{
				return _previous[0];
			}
			return nullptr;
		}

		TunnelLink *NPNo1TunnelLink(bool bForward = true)
		{
			return bForward ? nextNo1TunnelLink() : previousNo1TunnelLink();
		}

		// 以下只给输出统计用
		////隧道屏蔽标识,是指不渲染
		bool s_bdisabledFlag = false;
		////重建隧道标识
		bool s_bReCreatedObjectFlag = false;

		// 是否是Y型一部分
		bool s_isBelongY = false;
	};

	// 隧道组
	struct TunnelGroup
	{
		std::set<TunnelLink *> tunnnels;
		std::set<TunnelLink *> headlinks;

		// 有分叉
		bool hasFork = false;

		uint32 groupId = 0;
		// 统一厚度
		double mixthickness()
		{
			// 最低高度
			double thickNess = TUNNEL_THICKNESS;

			for (TunnelLink *link : tunnnels)
			{
				thickNess = min(thickNess, link->thickness);
			}
			return thickNess;
		};

		double mixHeight()
		{
			// 最低高度
			double height = TUNNEL_HEIGHT;

			for (TunnelLink *link : tunnnels)
			{
				height = min(height, link->height);
			}
			return height;
		}

		bool hasInGrid(int meshid)
		{

			for (auto tunnelLink : tunnnels)
			{
				if (tunnelLink->gridId() == meshid)
				{
					return true;
				}
			}
			return false;
		}

	};

	enum tunnel_type
	{
		tunnel_type_N = 0,
		tunnel_type_Y = 1,
		tunnel_type_N_S = 2, // 起点是分叉
		tunnel_type_N_E = 3, // 终点是分叉
		tunnel_type_N_B = 4, // 都是分叉
	};

	struct TunnelBase
	{
	public:
		virtual tunnel_type tunnelType()
		{
			return tunnel_type_N;
		};
		virtual double mixHeight() { return 0; };
		virtual double mixThickNess() { return 0; };
		uint32 groupId = 0;
	};

	// N型隧道
	struct NTunnel : TunnelBase
	{
	private:
		// 是否是Y隧道一部分
		bool belongYTunnel = false; // true 就必须以新数据格式绘制
		// 路面有断开
		bool bRoadBoundryDisconnect = false;

		bool yBranchFromStart = false;
		bool yBranchFromEnd = false;
	public:
	
		std::vector<TunnelLink *> tunnnels;

		void foreachTunnel(std::function<void(TunnelLink *link)> const &f)
		{
			for (TunnelLink *link : tunnnels)
			{
				f(link);
			}
		};

		tunnel_type tunnelType() override
		{
			if (yBranchFromStart && yBranchFromEnd)
			{
				return tunnel_type_N_B;
			}
			if (yBranchFromStart)
			{
				return tunnel_type_N_S;
			}
			if (yBranchFromEnd)
			{
				return tunnel_type_N_E;
			}
			else
			{
				return tunnel_type_N;
			}
		};

		double mixHeight() override
		{

			// 最低高度
			double height = TUNNEL_HEIGHT;

			for (TunnelLink *link : tunnnels)
			{
				height = min(height, link->height);
			}
			return height;
		};

		double mixThickNess() override
		{
			// 最低高度
			double thickNess = TUNNEL_THICKNESS;

			for (TunnelLink *link : tunnnels)
			{
				thickNess = min(thickNess, link->thickness);
			}
			return thickNess;
		}

		// 是否有压盖
		bool isgGland()
		{
			return mixHeight() != TUNNEL_HEIGHT;
		}

		// 是否在同一个网格
		bool InSameGrid()
		{
			if (!tunnnels.empty())
			{
				int grid = tunnnels.at(0)->gridId();
				for (auto tunnelLink : tunnnels)
				{
					if (tunnelLink->gridId() != grid)
					{
						return false;
					}
				}
				return true;
			}

			return true;
		}

		bool hasInGrid(int meshid)
		{

			for (auto tunnelLink : tunnnels)
			{
				if (tunnelLink->gridId() == meshid)
				{
					return true;
				}
			}
			return false;
		}

		// 屏蔽，渲染不需要画
		bool needBeDisable(TunnelGroup *group = nullptr)
		{
			if (group && group->groupId == groupId)
			{
				return mixHeight() < TUNNEL_HEIGHT_MIX || group->mixthickness() == 0;
			}
			else
			{
				return mixHeight() < TUNNEL_HEIGHT_MIX || mixThickNess() == 0;
			}
		}

		// 需要重建
		bool needBeCreated(TunnelGroup *group = nullptr)
		{
			if (needBeDisable(group))
			{
				return false;
			}
			if (group && group->groupId == groupId)
			{
				return belongYTunnel || group->mixthickness() < TUNNEL_THICKNESS;
			}
			else
			{
				return belongYTunnel || mixThickNess() < TUNNEL_THICKNESS;
			}
		}

		void setBelongYTunnel()
		{
			belongYTunnel = true;
		}

		bool isBelongYTunnel()
		{
			return belongYTunnel;
		}

		void setRoadBoundryDisconnect()
		{
			bRoadBoundryDisconnect = true;
		}

		bool isRoadBoundryDisconnect()
		{
			return bRoadBoundryDisconnect;
		}

		void setYBranchFromStart()
		{
			yBranchFromStart = true;
			if (!tunnnels.empty())
			{
				tunnnels.front()->isYBranchInterface = true;
			}
		}
		bool startisYBranch() { return yBranchFromStart; }
		bool endIsYbranch() { return yBranchFromEnd; }
		void setYBranchFromEnd()
		{
			yBranchFromEnd = true;
			if (!tunnnels.empty())
			{
				tunnnels.back()->isYBranchInterface = true;
			}
		}

		// 该函数假定tunnnels为顺序
		bool hasPreviousYTunnelFork()
		{
			if (!tunnnels.empty())
			{
				for (auto p : tunnnels.front()->_previous)
					for (auto& branchType : p->branchTypes)
						if (branchType == enum_tunnel_fork)
							return true;
			}
			return false;
		}
		// 该函数假定tunnnels为顺序
		bool hasPreviousYTunnelMerge()
		{
			if (!tunnnels.empty())
			{
				for (auto p : tunnnels.front()->_previous)
					for (auto& branchType : p->branchTypes)
						if (branchType == enum_tunnel_merge)
							return true;
			}
			return false;
		}
		// 该函数假定tunnnels为顺序
		bool hasNextYTunnelFork()
		{
			if (!tunnnels.empty())
			{
				for (auto p : tunnnels.back()->_next)
					for (auto& branchType : p->branchTypes)
						if (branchType == enum_tunnel_fork)
							return true;
			}
			return false;
		}
		// 该函数假定tunnnels为顺序
		bool hasNextYTunnelMerge()
		{
			if (!tunnnels.empty())
			{
				for (auto p : tunnnels.back()->_next)
					for (auto& branchType : p->branchTypes)
						if (branchType == enum_tunnel_merge)
							return true;
			}
			return false;
		}
	};

	// Y形隧道-最大扩增单位，把LA也包含进来
	struct YTunnelMixExtend
	{
		// Y形隧道 - 最小单位
		struct YTunnelMix
		{

			// 主干
			TunnelLink *main;
			// 左边分叉，主干往分叉方向看
			TunnelLink *left;
			// 右边分叉，主干往分叉方向看
			TunnelLink *right;

			// 主干
			HadLaneGroup *main_laneGroup;
			// 左边分叉，主干往分叉方向看
			HadLaneGroup *left_laneGroup;
			// 右边分叉，主干往分叉方向看
			HadLaneGroup *right_laneGroup;

			// 方向 fork merge
			TunnelBranchingType branchType;

			void init()
			{
				main = left = right = nullptr;
				main_laneGroup = left_laneGroup = right_laneGroup;
				branchType = enum_tunnel_unknow;
			}
		};
		std::vector<TunnelLink *> main;
		std::vector<TunnelLink *> leftExpand;
		std::vector<TunnelLink *> rightExpand;

		YTunnelMix *mixYtunnel;
		// 形状几何
		LineString3d _leftLine;
		LineString3d _middleLine;
		LineString3d _rightLine;

		int BranchType()
		{
			return mixYtunnel->branchType;
		}

		void init()
		{
			mixYtunnel = NULL;
		}
		void foreachTunnel(std::function<void(TunnelLink *link)> const &callback)
		{
			for (TunnelLink *link : leftExpand)
			{
				callback(link);
			}
			for (TunnelLink *link : rightExpand)
			{
				callback(link);
			}
			for (TunnelLink *link : main)
			{
				callback(link);
			}
		}

		TunnelLink *leftEnd()
		{
			if (leftExpand.empty())
			{
				return mixYtunnel->left;
			}
			else
			{
				return *leftExpand.rbegin();
			}
		}

		/*HadLaneGroup* leftEndLaneGroup()
		{
			if (leftExpand.empty())
			{
				return mixYtunnel->left_laneGroup;
			}
			else {
				return (*leftExpand.rbegin())->_laneGroup;
			}
		}*/

		TunnelLink *rightEnd()
		{
			if (rightExpand.empty())
			{
				return mixYtunnel->right;
			}
			else
			{
				return *rightExpand.rbegin();
			}
		}
		/*HadLaneGroup* rightEndLaneGroup()
		{
			if (leftExpand.empty())
			{
				return mixYtunnel->right_laneGroup;
			}
			else {
				return (*rightExpand.rbegin())->_laneGroup;
			}
		}*/

		TunnelLink *mainEnd()
		{
			if (main.empty())
			{
				return mixYtunnel->main;
			}
			else
			{
				return *main.rbegin();
			}
		}
	};

	using YTunnelMix = YTunnelMixExtend::YTunnelMix;

	// Y型隧道-最大单位
	struct YTunnel
	{
		uint32 groupId = 0;

		// 左边分叉，主干往分叉方向看
		std::vector<TunnelLink *> left;
		// 右边分叉，主干往分叉方向看
		std::vector<TunnelLink *> right;

		// 主干
		YTunnelMixExtend *main;

		// 分叉点，从左到右（主干往分叉方向）
		MapPoint3D64 forkPoint[4];

		void init()
		{
			_mainleftOutLaneGroup = _mainrightOutLaneGroup = nullptr;
			main = nullptr;
		}

		int BranchType()
		{
			return main->BranchType();
		}

		void foreachTunnel(std::function<void(TunnelLink *link)> const &callback)
		{
			for (TunnelLink *link : left)
			{
				callback(link);
			}
			for (TunnelLink *link : right)
			{
				callback(link);
			}
			if (main)
			{
				main->foreachTunnel(callback);
			}
		}

		bool hasInGrid(int meshid)
		{
			bool bInGrid = false;
			foreachTunnel([&](TunnelLink *link)
						  {
							  if (link->gridId() == meshid)
							  {
								  bInGrid = true;
							  }
						  });
			return bInGrid;
		}

		double mixThickNess()
		{

			double thickness = TUNNEL_THICKNESS;
			foreachTunnel([&](TunnelLink *link)
						  {
							  thickness = min(thickness, link->thickness);
						  });
			return thickness;
		}

		double mixHeight()
		{

			double height = TUNNEL_HEIGHT;
			foreachTunnel([&](TunnelLink *link)
						  {
							  height = min(height, link->height);
						  });
			return height;
		}

		// 是否需要屏蔽，指渲染不需要画
		bool needBeDisable(TunnelGroup *group = nullptr)
		{
			if (group && group->groupId == groupId)
			{
				return mixHeight() < TUNNEL_HEIGHT_MIX || group->mixthickness() == 0;
			}
			else
			{
				return mixHeight() < TUNNEL_HEIGHT_MIX || mixThickNess() == 0;
			}
		}

		// 主左边分叉第一条,主干往分叉方向看
		HadLaneGroup *_mainleftOutLaneGroup;
		// 主右边分叉第一条,主干往分叉方向看
		HadLaneGroup *_mainrightOutLaneGroup;
	};

};

namespace OMDB
{
	class TunnelCompiler : public Compiler
	{
	public:
		TunnelCompiler(CompilerData& data) :Compiler(data) {};
	protected:
		void compile(HadGrid *const pGrid, const std::vector<HadGrid *> &nearby, RdsTile *pTile) override;
		void grabTunnel(HadGrid *const pGrid, const std::vector<HadGrid *> &nearby);
		void calcLineAndBox();
		void calcThicknessAndHeight(const std::vector<HadGrid*>& nearByGrid);
		void makeTopology();
		void divideToGroup();
		void createTunnel();

		void splitYTunnel();
		void splitToGrid();
		void storeToGrid();


		size_t makeLaneGroupPolygon(HadLaneGroup* pGroup, MapPoint3D64*& polygon) override;

	private:
		void createNLinkGroup(TunnelGroup *tunnelGroup);
		void createYLinkGroup(TunnelGroup *tunnelGroup);

		bool createRdsTunnel(NTunnel *nTunnel);
		bool createRdsTunnel(YTunnel *yTunnel);
		void setYTunnelMixExtendTunnelGeoLine(YTunnelMixExtend *yTunnelMixExtend);
		void getTunnelsShape(std::vector<TunnelLink *> tunnnels, bool isReverse, MultiLineString3d &lineString3DVector);
		void getTunnelsMiddleLines(std::vector<TunnelLink *> tunnnels, bool isReverse, LineString3d &lineString3DVector);

		YTunnelMix *createYTunnelMix(TunnelLink *tunnelLink, FastTable<TunnelLink *, TunnelLink *> &laTable);
		YTunnelMixExtend *createYTunnelMixExtend(YTunnelMix *yTunnelMix, FastTable<TunnelLink *, TunnelLink *> &table);
		YTunnel *createYTunnel(YTunnelMixExtend *yTunnelMixExtend, FastTable<TunnelLink *, TunnelLink *> &table);
		void findNext(TunnelLink *tunnelLink, bool bForward, std::vector<TunnelLink *> &outLink, bool includeFirst, FastTable<TunnelLink *, TunnelLink *> &table);
		void getYTunnelMixExtendLeftRight(TunnelLink *left, TunnelLink *right, bool isForward, std::vector<TunnelLink *> &leftOuts, std::vector<TunnelLink *> &rightOut, FastTable<TunnelLink *, TunnelLink *> &table);

		bool hasOverLap(TunnelLink *tunnelLink, MapPoint3D64 *pCurPolygon, size_t nCurPolygonPtNum, HadLaneGroup *pNearbyLg, int32 &overlapHeight, std::vector<TunnelLinkGlandHeightInfo> &heightInfo);
		bool hasIntersectExtend(MapPoint3D64 *pCurPolygon, size_t nCurPolygonPtNum, MapPoint3D64 *pNearPolygon, size_t nNearPolygonPtNum);
		int getDistance(MapPoint3D64 *pCurPolygon, size_t nCurPolygonPtNum, MapPoint3D64 *pNearPolygon, size_t nNearPolygonPtNum);
		int getDistance2(MapPoint3D64 *pCurPolygon, size_t nCurPolygonPtNum, MapPoint3D64 *pNearPolygon, size_t nNearPolygonPtNum);
		bool getClosePoint(MapPoint3D64* pCurPolygon, size_t nCurPolygonPtNum, MapPoint3D64* pNearPolygon, size_t nNearPolygonPtNum, MapPoint3D64& pt1Closest, MapPoint3D64& pt2Closest);
		void getCourHeightInfo(std::vector<TunnelLink*>& tunnelLinks, LineString3d& line, std::vector<RdsTunnel::HeightInfo>& heighInfo,bool bKeepTail = true);
		bool isTouchWithHadLaneGroup(HadLaneGroup *laneGroup1, HadLaneGroup *laneGroup2);
		bool DilutionTunnel(std::vector<MapPoint3D64>& points);
		void getNerbyHeightInfo(TunnelLink* tunnnel, bool bfromSelf, bool bNext, double& higninfo);
		void getNearbyHeightInfo(std::vector<TunnelLink*>& tunnnels, double& higninfo);
		void mergeHighInfo(std::vector<RdsTunnel::HeightInfo>& heightInfos, std::vector<MapPoint3D64>& middelPoint, std::vector<RdsTunnel::HeightInfo>& mergeHeighInfo);
		void mergeHighInfo(std::vector<TunnelLinkGlandHeightInfo>& heighInfo);

		bool isLaneGroupOnLeft(HadLaneGroup *leftLaneGroup, HadLaneGroup *rightLaneGroup);
		bool isLALaneGroup(HadLaneGroup *l1, HadLaneGroup *l2, HadRelLaneGroupAssociation &laneGroupAsso);
		TunnelLink *getTunnelLinkByLaneGroup(HadLaneGroup *hadLaneGroup);
		TunnelBranchingType laBranchType(HadRelLaneGroupAssociation &la);

		void getLaneGroupLines(HadLaneGroup *laneGroup, MultiLineString3d &lines);
		void getLaneGroupLines(HadLaneGroup *laneGroup, MultiLineString3d &lines, HadLane *&middleLane, HadLaneBoundary *&middleLaneBoudary);

		template <typename className>
		className *ALLOC_ELEMENT();
		point_t getNearestPoint(const std::vector<point_t>& points, const point_t& originPoint);
	private:
		BatchedAllocator m_batchedAllocator;
		std::vector<TunnelLink *> m_tunnelLinkArray;
		FastTable<HadLaneGroup *, TunnelLink *> m_laneGroup2Tunnel;
		FastTable<int32, TunnelGroup *> m_tunnelGroups;
		std::vector<NTunnel *> m_NTunnelArray;
		std::vector<YTunnel *> m_YTunnelArray;
		HadGrid *m_pGrid;
		RdsTile *m_pTile;
	};

};
