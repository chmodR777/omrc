#pragma once
#include <vector>
#include "../om/HadElement.h"
#include "../om/HadGrid.h"
namespace OMDB
{
	/// <summary>
	/// 拓扑关系生成器
	/// </summary>
	class TopoBuilder
	{
	public:
		static void buildTopo(const std::vector<HadGrid*>& grids);
		static void buildTopo(HadLink* pCurrent,HadLink* pNext);
		static void buildTopo(HadLaneGroup* pCurrent, HadLaneGroup* pNext);

		static bool directionEqual(HadLaneBoundary* const pBoundary, HadLaneGroup* const pGroup, int direction);
		static bool directionEqual(HadRoadBoundary* const pBoundary, HadLaneGroup* const pGroup, int direction);
		static HadRoadBoundaryNode* getStartNode(HadRoadBoundary* const pBoundary, HadLaneGroup* const pGroup);
		static HadRoadBoundaryNode* getEndNode(HadRoadBoundary* const pBoundary, HadLaneGroup* const pGroup);
		static HadLaneBoundaryNode* getStartNode(HadLaneBoundary* const pBoundary, HadLaneGroup* const pGroup);
		static HadLaneBoundaryNode* getEndNode(HadLaneBoundary* const pBoundary, HadLaneGroup* const pGroup);

		static void buildTopoCrossGrid(HadGrid* center,const std::vector<HadGrid*>* nearby);

		static void buildTopo(HadGrid* const grids);
		static void buildTopo(HadLink* pLink, bool crossGridOnly);
	private:
		static std::vector<HadLaneGroup*> getGroups(HadLink* pLink, std::vector<HadLaneGroup*>& groups, int direct);
	};
}

