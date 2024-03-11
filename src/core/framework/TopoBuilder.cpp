#include "stdafx.h"
#include "TopoBuilder.h"
#include <algorithm>

namespace OMDB
{	
	void TopoBuilder::buildTopo(HadGrid* const pGrid)
	{
		// 左右侧排序
		std::vector<HadElement*> laneGroups = pGrid->query(ElementType::HAD_LANE_GROUP);
		for (auto& pElement : laneGroups)
		{
			HadLaneGroup* pGroup = (HadLaneGroup*)pElement;
			int64 lgId = pGroup->originId;
			std::sort(pGroup->roadBoundaries.begin(), pGroup->roadBoundaries.end(),
				[=](const HadRoadBoundary* first, const HadRoadBoundary* second)->bool {
					return first->relLgs.find(lgId)->second.side > second->relLgs.find(lgId)->second.side;
				}
			);
			std::sort(pGroup->laneBoundaries.begin(), pGroup->laneBoundaries.end(),
				[=](const HadLaneBoundary* first, const HadLaneBoundary* second)->bool {
					return first->getSeqNumberOnLG(lgId) < second->getSeqNumberOnLG(lgId); });
		}
		// Link
		std::vector<HadElement*> links = pGrid->query(ElementType::HAD_LINK);

		// LaneGroup
		for (auto& pElement : links)
		{
			HadLink* pLink = (HadLink*)pElement;
			buildTopo(pLink, false);
		}

		auto buildDirect23Topo = [](HadLink* pCurrent, HadLink* pNext) {
			if (pCurrent->direct == 2 && pNext->direct == 2) {
				if (pCurrent->startNode == pNext->endNode) {
					buildTopo(pNext, pCurrent);
				}
				else if (pCurrent->endNode == pNext->startNode) {
					buildTopo(pCurrent, pNext);
				}
			}
			else if (pCurrent->direct == 2 && pNext->direct == 3) {
				if (pCurrent->endNode == pNext->endNode) {
					buildTopo(pCurrent, pNext);
				}
				else if (pCurrent->startNode == pNext->startNode) {
					buildTopo(pNext, pCurrent);
				}
			}
			else if (pCurrent->direct == 3 && pNext->direct == 2) {
				if (pCurrent->endNode == pNext->endNode) {
					buildTopo(pNext, pCurrent);
				}
				else if (pCurrent->startNode == pNext->startNode) {
					buildTopo(pCurrent, pNext);
				}
			}
			else if (pCurrent->direct == 3 && pNext->direct == 3) {
				if (pCurrent->startNode == pNext->endNode) {
					buildTopo(pCurrent, pNext);
				}
				else if (pCurrent->endNode == pNext->startNode) {
					buildTopo(pNext, pCurrent);
				}
			}
		};

		for (int m = 0; m < links.size(); m++)
		{
			HadLink* pCurrent = (HadLink*)links[m];
			auto currentDirect = pCurrent->direct;
			auto currentGroups = pCurrent->groups;
			for (int n = m + 1; n < links.size(); n++)
			{
				HadLink* pNext = (HadLink*)links[n];
				auto nextDirect = pNext->direct;
				auto nextGroups = pNext->groups;
				if (pCurrent->direct != 1 && pNext->direct != 1) {
					buildDirect23Topo(pCurrent, pNext);
				}
				else if (pCurrent->direct == 1 && pNext->direct == 1) {
					pCurrent->direct = 2;
					pNext->direct = 2;
					pCurrent->groups = getGroups(pCurrent, currentGroups, pCurrent->direct);
					pNext->groups = getGroups(pNext, nextGroups, pNext->direct);
					buildDirect23Topo(pCurrent, pNext);

					pCurrent->direct = 2;
					pNext->direct = 3;
					pCurrent->groups = getGroups(pCurrent, currentGroups, pCurrent->direct);
					pNext->groups = getGroups(pNext, nextGroups, pNext->direct);
					buildDirect23Topo(pCurrent, pNext);

					pCurrent->direct = 3;
					pNext->direct = 2;
					pCurrent->groups = getGroups(pCurrent, currentGroups, pCurrent->direct);
					pNext->groups = getGroups(pNext, nextGroups, pNext->direct);
					buildDirect23Topo(pCurrent, pNext);

					pCurrent->direct = 3;
					pNext->direct = 3;
					pCurrent->groups = getGroups(pCurrent, currentGroups, pCurrent->direct);
					pNext->groups = getGroups(pNext, nextGroups, pNext->direct);
					buildDirect23Topo(pCurrent, pNext);
				}
				else if (pCurrent->direct == 1 && (pNext->direct == 2 || pNext->direct == 3)) {
					pCurrent->direct = 2;
					pCurrent->groups = getGroups(pCurrent, currentGroups, pCurrent->direct);
					buildDirect23Topo(pCurrent, pNext);

					pCurrent->direct = 3;
					pCurrent->groups = getGroups(pCurrent, currentGroups, pCurrent->direct);
					buildDirect23Topo(pCurrent, pNext);
				}
				else if ((pCurrent->direct == 2 || pCurrent->direct == 3) && pNext->direct == 1) {
					pNext->direct = 2;
					pNext->groups = getGroups(pNext, nextGroups, pNext->direct);
					buildDirect23Topo(pCurrent, pNext);

					pNext->direct = 3;
					pNext->groups = getGroups(pNext, nextGroups, pNext->direct);
					buildDirect23Topo(pCurrent, pNext);
				}

				pNext->direct = nextDirect;
				pNext->groups = nextGroups;
				pCurrent->direct = currentDirect;
				pCurrent->groups = currentGroups;

			}
		}

		// RoadBoundary
		std::vector<HadElement*> roadBoundarys = pGrid->query(ElementType::HAD_ROAD_BOUNDARY);
		for (int m = 0; m < roadBoundarys.size(); m++)
		{
			HadRoadBoundary* pCurrent = (HadRoadBoundary*)roadBoundarys[m];
			for (int n = m + 1; n < roadBoundarys.size(); n++)
			{
				HadRoadBoundary* pNext = (HadRoadBoundary*)roadBoundarys[n];
				if (pCurrent->startNode == pNext->endNode)
				{
					if (std::find(pCurrent->previous.begin(), pCurrent->previous.end(), pNext) == pCurrent->previous.end()) {
						pCurrent->previous.push_back(pNext);
					}
					if (std::find(pNext->next.begin(), pNext->next.end(), pCurrent) == pNext->next.end()) {
						pNext->next.push_back(pCurrent);
						pNext->danglingEndNode = false;
					}
				}
				else if (pCurrent->endNode == pNext->startNode)
				{
					if (std::find(pCurrent->next.begin(), pCurrent->next.end(), pNext) == pCurrent->next.end()) {
						pCurrent->next.push_back(pNext);
						pCurrent->danglingEndNode = false;
					}
					if (std::find(pNext->previous.begin(), pNext->previous.end(), pCurrent) == pNext->previous.end()) {
						pNext->previous.push_back(pCurrent);
					}
				}
				if (pCurrent->endNode == pNext->endNode)
				{
					pCurrent->danglingEndNode = false;
					pNext->danglingEndNode = false;
				}
			}
		}
		// LaneBoundary
		std::vector<HadElement*> laneBoundarys = pGrid->query(ElementType::HAD_LANE_BOUNDARY);
		for (int m = 0; m < laneBoundarys.size(); m++)
		{
			HadLaneBoundary* pCurrent = (HadLaneBoundary*)laneBoundarys[m];
			for (int n = m + 1; n < laneBoundarys.size(); n++)
			{
				HadLaneBoundary* pNext = (HadLaneBoundary*)laneBoundarys[n];
				if (pCurrent->startNode == pNext->endNode)
				{
					if (std::find(pCurrent->previous.begin(), pCurrent->previous.end(), pNext) == pCurrent->previous.end()) {
						pCurrent->previous.push_back(pNext);
					}
					if (std::find(pNext->next.begin(), pNext->next.end(), pCurrent) == pNext->next.end()) {
						pNext->next.push_back(pCurrent);
					}
				}
				else if (pCurrent->endNode == pNext->startNode)
				{
					if (std::find(pCurrent->next.begin(), pCurrent->next.end(), pNext) == pCurrent->next.end()) {
						pCurrent->next.push_back(pNext);
					}
					if (std::find(pNext->previous.begin(), pNext->previous.end(), pCurrent) == pNext->previous.end()) {
						pNext->previous.push_back(pCurrent);
					}
				}
			}
		}
	}

	void TopoBuilder::buildTopo(HadLink* pLink, bool crossGridOnly)
	{
		if (pLink->groups.size() < 2)
			return;

		std::sort(pLink->groups.begin(), pLink->groups.end(),
			[&](const HadLaneGroup* first, const HadLaneGroup* second)->bool {
				auto relLinkFirst = first->relLinks.find(pLink);
				auto relLInkSecond = second->relLinks.find(pLink);
				if (relLinkFirst->second.start != relLInkSecond->second.start)
					return relLinkFirst->second.start < relLInkSecond->second.start;
				return relLinkFirst->second.end < relLInkSecond->second.end;
			}
		);

		auto buildDirect2Topo = [](HadLink* pLink, std::vector<HadLaneGroup*>& groups, bool crossGridOnly) {
			if (groups.size() < 2)
				return;

			for (int i = 0; i < groups.size() - 1; i++) {
				HadLaneGroup* pCurrent = groups[i];
				HadLaneGroup* pNext = groups[i + 1];
				bool crossGrid = pCurrent->owner != pNext->owner;
				if (crossGridOnly && !crossGrid) {
					continue;
				}

				auto rc = pCurrent->relLinks.find(pLink);
				auto rn = pNext->relLinks.find(pLink);
				if (std::abs(rc->second.end - rn->second.start) < 0.01)
				{
					buildTopo(pCurrent, pNext);
					if (std::find(pCurrent->next.begin(), pCurrent->next.end(), pNext) == pCurrent->next.end())
					{
						pCurrent->next.push_back(pNext);
					}
					if (std::find(pNext->previous.begin(), pNext->previous.end(), pCurrent) == pNext->previous.end())
					{
						pNext->previous.push_back(pCurrent);
					}
				}
			}
		};

		auto buildDirect3Topo = [](HadLink* pLink, std::vector<HadLaneGroup*>& groups, bool crossGridOnly) {
			if (groups.size() < 2)
				return;

			for (int i = groups.size() - 1; i > 0; i--) {
				HadLaneGroup* pCurrent = groups[i];
				HadLaneGroup* pNext = groups[i - 1];
				bool crossGrid = pCurrent->owner != pNext->owner;
				if (crossGridOnly && !crossGrid) {
					continue;
				}

				auto rc = pCurrent->relLinks.find(pLink);
				auto rn = pNext->relLinks.find(pLink);
				if (std::abs(rn->second.end - rc->second.start) < 0.01)
				{
					buildTopo(pCurrent, pNext);
					if (std::find(pCurrent->next.begin(), pCurrent->next.end(), pNext) == pCurrent->next.end())
					{
						pCurrent->next.push_back(pNext);
					}
					if (std::find(pNext->previous.begin(), pNext->previous.end(), pCurrent) == pNext->previous.end())
					{
						pNext->previous.push_back(pCurrent);
					}
				}
			}
		};

		// TODO 路口关系?
		if (pLink->direct == 2) {
			buildDirect2Topo(pLink, pLink->groups, crossGridOnly);
		}
		else if (pLink->direct == 3) {
			buildDirect3Topo(pLink, pLink->groups, crossGridOnly);
		}
		else if (pLink->direct == 1) {
			buildDirect2Topo(pLink, getGroups(pLink, pLink->groups, 2), crossGridOnly);
			buildDirect3Topo(pLink, getGroups(pLink, pLink->groups, 3), crossGridOnly);
		}
	}

    void TopoBuilder::buildTopo(const std::vector<HadGrid*>& grids)
    {
		for (auto& pMesh : grids)
		{
			buildTopo(pMesh);
		}
    }

	void TopoBuilder::buildTopo(HadLink* pCurrent, HadLink* pNext)
	{
		// 双方向通行的link的车道组是并排的,车道组之间没有拓扑关系
		if (pCurrent->direct == 1 || pNext->direct == 1) {
			return;
		}

		if (std::find(pCurrent->next.begin(), pCurrent->next.end(), pNext) == pCurrent->next.end())
		{
			pCurrent->next.push_back(pNext);
		}
		if (std::find(pNext->previous.begin(), pNext->previous.end(), pCurrent) == pNext->previous.end())
		{
			pNext->previous.push_back(pCurrent);
		}

		// 没有车道组的返回
		if (pCurrent->groups.empty() || pNext->groups.empty())
			return;

		HadLaneGroup* pCurrentLastGroup = nullptr;
		HadLaneGroup* pNextFirstGroup = nullptr;
		if (pCurrent->direct == 2 && pNext->direct == 2) {
			pCurrentLastGroup = *pCurrent->groups.rbegin();
			pNextFirstGroup = pNext->groups[0];
		}
		else if (pCurrent->direct == 2 && pNext->direct == 3) {
			pCurrentLastGroup = *pCurrent->groups.rbegin();
			pNextFirstGroup = *pNext->groups.rbegin();
		}
		else if (pCurrent->direct == 3 && pNext->direct == 2) {
			pCurrentLastGroup = pCurrent->groups[0];
			pNextFirstGroup = pNext->groups[0];
		}
		else if (pCurrent->direct == 3 && pNext->direct == 3) {
			pCurrentLastGroup = pCurrent->groups[0];
			pNextFirstGroup = *pNext->groups.rbegin();
		}

		// 车道组构建关系
		buildTopo(pCurrentLastGroup, pNextFirstGroup);
		if (pCurrent->owner != pNext->owner &&
			pCurrent->crossGrid && pNext->crossGrid &&
			pCurrentLastGroup != nullptr && pNextFirstGroup != nullptr) {
			pCurrentLastGroup->crossGrid = true;
			pNextFirstGroup->crossGrid = true;
		}

	}

	void TopoBuilder::buildTopo(HadLaneGroup* pCurrent, HadLaneGroup* pNext)
	{
		// 路口时可能是同一个车道组
		if (pCurrent == pNext)
			return;

		if (std::find(pCurrent->next.begin(), pCurrent->next.end(), pNext) != pCurrent->next.end() ||
			std::find(pNext->previous.begin(), pNext->previous.end(), pCurrent) != pNext->previous.end()
			)
		{
			return;
		}

		//laneGroup
		bool isBuildLaneGroupTopo = false;
		const float CONNECT_POINT_EPSILON = 500.f;  // ≈0.5m
		for (auto& iItem : pCurrent->roadBoundaries)
		{
			auto endNode = getEndNode(iItem, pCurrent);
			if (endNode == nullptr)
				continue;
			auto& endPt = endNode->position;
			for (auto& jItem : pNext->roadBoundaries)
			{
				auto startNode = getStartNode(jItem, pNext);
				if (startNode == nullptr)
					continue;
				auto& startPt = startNode->position;
				if (endNode->originId == startNode->originId ||
					endPt.pos.distance(startPt.pos) < CONNECT_POINT_EPSILON) {
					isBuildLaneGroupTopo = true;
					break;
				}
			}
			if (isBuildLaneGroupTopo)
				break;
		}
		if (!isBuildLaneGroupTopo) {
			for (auto& iItem : pCurrent->laneBoundaries)
			{
				auto endNode = getEndNode(iItem, pCurrent);
				if (endNode == nullptr)
					continue;
				auto& endPt = endNode->position;
				for (auto& jItem : pNext->laneBoundaries)
				{
					auto startNode = getStartNode(jItem, pNext);
					if (startNode == nullptr)
						continue;
					auto& startPt = startNode->position;
					if (endNode->originId == startNode->originId ||
						endPt.pos.distance(startPt.pos) < CONNECT_POINT_EPSILON) {
						isBuildLaneGroupTopo = true;
						break;
					}
				}
				if (isBuildLaneGroupTopo)
					break;
			}
		}

		// 道路边界或车道边界首尾相连
		if (isBuildLaneGroupTopo)
		{
			if (std::find(pCurrent->next.begin(), pCurrent->next.end(), pNext) == pCurrent->next.end())
			{
				pCurrent->next.push_back(pNext);
			}
			if (std::find(pNext->previous.begin(), pNext->previous.end(), pCurrent) == pNext->previous.end())
			{
				pNext->previous.push_back(pCurrent);
			}
		}

		// Lane
		for (int i = 0; i < pCurrent->lanes.size(); i++)
		{
			HadLane* pCurrentLane = pCurrent->lanes[i];
			auto endNode = pCurrentLane->endNode;
			if (endNode == nullptr)
				continue;
			for (int j = 0; j < pNext->lanes.size(); j++)
			{
				HadLane* pNextLane = pNext->lanes[j];
				auto startNode = pNextLane->startNode;
				if (startNode == nullptr)
					continue;
				if (endNode->originId == startNode->originId)
				{
					if (std::find(pCurrentLane->next.begin(), pCurrentLane->next.end(), pNextLane) == pCurrentLane->next.end())
					{
						pCurrentLane->next.push_back(pNextLane);
					}

					if (std::find(pNextLane->previous.begin(), pNextLane->previous.end(), pCurrentLane) == pNextLane->previous.end())
					{
						pNextLane->previous.push_back(pCurrentLane);
					}
				}
			}
		}

		// LaneBoundary
		for (int i = 0; i < pCurrent->laneBoundaries.size(); i++)
		{
			HadLaneBoundary* pCurrentLaneBoundary = pCurrent->laneBoundaries[i];
			auto endNode = pCurrentLaneBoundary->endNode;
			if (endNode == nullptr)
				continue;
			for (int j = 0; j < pNext->laneBoundaries.size(); j++)
			{
				HadLaneBoundary* pNextLaneBoundary = pNext->laneBoundaries[j];
				auto startNode = pNextLaneBoundary->startNode;
				if (startNode == nullptr)
					continue;
				if (endNode->originId == startNode->originId)
				{
					if (std::find(pCurrentLaneBoundary->next.begin(), pCurrentLaneBoundary->next.end(), pNextLaneBoundary) == pCurrentLaneBoundary->next.end())
					{
						pCurrentLaneBoundary->next.push_back(pNextLaneBoundary);
					}

					if (std::find(pNextLaneBoundary->previous.begin(), pNextLaneBoundary->previous.end(), pCurrentLaneBoundary) == pNextLaneBoundary->previous.end())
					{
						pNextLaneBoundary->previous.push_back(pCurrentLaneBoundary);
					}
				}
			}
		}

		// RoadBoundary
		if (pCurrent->relLinks.size() > 1 || pNext->relLinks.size() > 1)
		{
			for (int i = 0; i < pCurrent->roadBoundaries.size(); i++)
			{
				HadRoadBoundary* pCurrentRoadBoundary = pCurrent->roadBoundaries[i];
				auto endNode = pCurrentRoadBoundary->endNode;
				if (endNode == nullptr)
					continue;
				for (int j = 0; j < pNext->roadBoundaries.size(); j++)
				{
					HadRoadBoundary* pNextRoadBoundary = pNext->roadBoundaries[j];
					auto startNode = pNextRoadBoundary->startNode;
					if (startNode == nullptr)
						continue;
					if (endNode->originId == startNode->originId)
					{
						if (std::find(pCurrentRoadBoundary->next.begin(), pCurrentRoadBoundary->next.end(), pNextRoadBoundary) == pCurrentRoadBoundary->next.end())
						{
							pCurrentRoadBoundary->next.push_back(pNextRoadBoundary);
							pCurrentRoadBoundary->danglingEndNode = false;
						}

						if (std::find(pNextRoadBoundary->previous.begin(), pNextRoadBoundary->previous.end(), pCurrentRoadBoundary) == pNextRoadBoundary->previous.end())
						{
							pNextRoadBoundary->previous.push_back(pCurrentRoadBoundary);
						}
					}
					if (pNextRoadBoundary->endNode && endNode->originId == pNextRoadBoundary->endNode->originId)
					{
						pCurrentRoadBoundary->danglingEndNode = false;
						pNextRoadBoundary->danglingEndNode = false;
					}
				}
			}

		}
		else
		{
			if (!isBuildLaneGroupTopo || pCurrent->roadBoundaries.size() != 2 || pNext->roadBoundaries.size() != 2)
				return;

			auto& leftBoundary = pCurrent->roadBoundaries[0];
			auto& rightBoundary = pCurrent->roadBoundaries[1];
			auto& nextLeftBoundary = pNext->roadBoundaries[0];
			auto& nextRightBoundary = pNext->roadBoundaries[1];
			if (std::find(leftBoundary->next.begin(), leftBoundary->next.end(), nextLeftBoundary) == leftBoundary->next.end()) {
				leftBoundary->next.push_back(nextLeftBoundary);
			}
			if (std::find(rightBoundary->next.begin(), rightBoundary->next.end(), nextRightBoundary) == rightBoundary->next.end()) {
				rightBoundary->next.push_back(nextRightBoundary);
			}
			if (std::find(nextLeftBoundary->previous.begin(), nextLeftBoundary->previous.end(), leftBoundary) == nextLeftBoundary->previous.end()) {
				nextLeftBoundary->previous.push_back(leftBoundary);
			}
			if (std::find(nextRightBoundary->previous.begin(), nextRightBoundary->previous.end(), rightBoundary) == nextRightBoundary->previous.end()) {
				nextRightBoundary->previous.push_back(rightBoundary);
			}
		}
	}

	bool TopoBuilder::directionEqual(HadLaneBoundary* const pBoundary, HadLaneGroup* const pGroup, int direction)
	{
		if (pBoundary == nullptr || pGroup == nullptr)
			return false;
		if (pBoundary->relLgs.count(pGroup->originId) && pBoundary->relLgs[pGroup->originId].lgMarkDirect == direction)
			return true;
		return false;
	}

	bool TopoBuilder::directionEqual(HadRoadBoundary* const pBoundary, HadLaneGroup* const pGroup, int direction)
	{
		if (pBoundary == nullptr || pGroup == nullptr)
			return false;
		if (pBoundary->relLgs.count(pGroup->originId) && pBoundary->relLgs[pGroup->originId].direction == direction)
			return true;
		return false;
	}

	HadRoadBoundaryNode* TopoBuilder::getStartNode(HadRoadBoundary* const pBoundary, HadLaneGroup* const pGroup)
	{
		auto startNode = pBoundary->startNode;
		if (directionEqual(pBoundary, pGroup, 3))
			startNode = pBoundary->endNode;
		return startNode;
	}

	HadRoadBoundaryNode* TopoBuilder::getEndNode(HadRoadBoundary* const pBoundary, HadLaneGroup* const pGroup)
	{
		auto endNode = pBoundary->endNode;
		if (directionEqual(pBoundary, pGroup, 3))
			endNode = pBoundary->startNode;
		return endNode;
	}

	HadLaneBoundaryNode* TopoBuilder::getStartNode(HadLaneBoundary* const pBoundary, HadLaneGroup* const pGroup)
	{
		auto startNode = pBoundary->startNode;
		if (directionEqual(pBoundary, pGroup, 3))
			startNode = pBoundary->endNode;
		return startNode;
	}

	HadLaneBoundaryNode* TopoBuilder::getEndNode(HadLaneBoundary* const pBoundary, HadLaneGroup* const pGroup)
	{
		auto endNode = pBoundary->endNode;
		if (directionEqual(pBoundary, pGroup, 3))
			endNode = pBoundary->startNode;
		return endNode;
	}

	std::vector<HadLaneGroup*> TopoBuilder::getGroups(HadLink* pLink, std::vector<HadLaneGroup*>& groups, int direct)
	{
		std::vector<HadLaneGroup*> grps;
		for (auto group : groups) {
			for (auto& relLinkPair : group->relLinks) {
				if (relLinkPair.first == pLink) {
					if (relLinkPair.second.direct == direct) {
						if (std::find(grps.begin(), grps.end(), group) == grps.end()) {
							grps.push_back(group);
							break;
						}
					}
				}
			}
		}

		std::sort(grps.begin(), grps.end(),
			[&](const HadLaneGroup* first, const HadLaneGroup* second)->bool {
				auto relLinkFirst = first->relLinks.find(pLink);
				auto relLInkSecond = second->relLinks.find(pLink);
				if (relLinkFirst->second.start != relLInkSecond->second.start)
					return relLinkFirst->second.start < relLInkSecond->second.start;
				return relLinkFirst->second.end < relLInkSecond->second.end;
			}
		);
		return grps;
	}

	void TopoBuilder::buildTopoCrossGrid(HadGrid* center, const std::vector<HadGrid*>* nearby)
	{
		auto isCrossGrid = [](const HadLink* pLink)->bool {
			if (pLink->crossGrid)
				return true;
			for (auto group : pLink->groups) {
				if (group->crossGrid)
					return true;
			}
			return false;
		};

		// 处理跨网格情况
		std::vector<HadLink*> links;
		auto addCrossGridLink = [&](HadGrid* pGrid) {
			std::vector<HadElement*>& tmp = pGrid->query(ElementType::HAD_LINK);
			for_each(tmp.begin(), tmp.end(), [&](HadElement* pElement)->void
				{
					HadLink* pLink = (HadLink*)pElement;
					bool crossGrid = isCrossGrid(pLink);
					if (!crossGrid) {
						for (auto p : pLink->previous) {
							if (isCrossGrid((HadLink*)p)) {
								crossGrid = true;
								break;
							}
						}
					}
					if (!crossGrid) {
						for (auto n : pLink->next) {
							if (isCrossGrid((HadLink*)n)) {
								crossGrid = true;
								break;
							}
						}
					}
					if (crossGrid) {
						buildTopo(pLink, true);
						links.push_back(pLink);
					}
				}
			);
		};

		addCrossGridLink(center);
		for_each(nearby->begin(), nearby->end(), [&](HadGrid* pGrid)->void
			{
				addCrossGridLink(pGrid);
			}
		);

		auto buildDirect23Topo = [](HadLink* pCurrent, HadLink* pNext) {
			if (pCurrent->direct == 2 && pNext->direct == 2) {
				if (pCurrent->startNode->originId == pNext->endNode->originId) {
					buildTopo(pNext, pCurrent);
				}
				else if (pCurrent->endNode->originId == pNext->startNode->originId) {
					buildTopo(pCurrent, pNext);
				}
			}
			else if (pCurrent->direct == 2 && pNext->direct == 3) {
				if (pCurrent->endNode->originId == pNext->endNode->originId) {
					buildTopo(pCurrent, pNext);
				}
				else if (pCurrent->startNode->originId == pNext->startNode->originId) {
					buildTopo(pNext, pCurrent);
				}
			}
			else if (pCurrent->direct == 3 && pNext->direct == 2) {
				if (pCurrent->endNode->originId == pNext->endNode->originId) {
					buildTopo(pNext, pCurrent);
				}
				else if (pCurrent->startNode->originId == pNext->startNode->originId) {
					buildTopo(pCurrent, pNext);
				}
			}
			else if (pCurrent->direct == 3 && pNext->direct == 3) {
				if (pCurrent->startNode->originId == pNext->endNode->originId) {
					buildTopo(pCurrent, pNext);
				}
				else if (pCurrent->endNode->originId == pNext->startNode->originId) {
					buildTopo(pNext, pCurrent);
				}
			}
		};

		// Link
		for (int m = 0; m < links.size(); m++)
		{
			HadLink* pCurrent = (HadLink*)links[m];
			auto currentDirect = pCurrent->direct;
			auto currentGroups = pCurrent->groups;
			for (int n = m + 1; n < links.size(); n++)
			{
				HadLink* pNext = (HadLink*)links[n];
				auto nextDirect = pNext->direct;
				auto nextGroups = pNext->groups;
				if (pCurrent->direct != 1 && pNext->direct != 1) {
					buildDirect23Topo(pCurrent, pNext);
				}
				else if(pCurrent->direct == 1 && pNext->direct == 1) {
					pCurrent->direct = 2;
					pNext->direct = 2;
					pCurrent->groups = getGroups(pCurrent, currentGroups, pCurrent->direct);
					pNext->groups = getGroups(pNext, nextGroups, pNext->direct);
					buildDirect23Topo(pCurrent, pNext);

					pCurrent->direct = 2;
					pNext->direct = 3;
					pCurrent->groups = getGroups(pCurrent, currentGroups, pCurrent->direct);
					pNext->groups = getGroups(pNext, nextGroups, pNext->direct);
					buildDirect23Topo(pCurrent, pNext);

					pCurrent->direct = 3;
					pNext->direct = 2;
					pCurrent->groups = getGroups(pCurrent, currentGroups, pCurrent->direct);
					pNext->groups = getGroups(pNext, nextGroups, pNext->direct);
					buildDirect23Topo(pCurrent, pNext);

					pCurrent->direct = 3;
					pNext->direct = 3;
					pCurrent->groups = getGroups(pCurrent, currentGroups, pCurrent->direct);
					pNext->groups = getGroups(pNext, nextGroups, pNext->direct);
					buildDirect23Topo(pCurrent, pNext);
				}
				else if (pCurrent->direct == 1 && (pNext->direct == 2 || pNext->direct == 3)) {
					pCurrent->direct = 2;
					pCurrent->groups = getGroups(pCurrent, currentGroups, pCurrent->direct);
					buildDirect23Topo(pCurrent, pNext);

					pCurrent->direct = 3;
					pCurrent->groups = getGroups(pCurrent, currentGroups, pCurrent->direct);
					buildDirect23Topo(pCurrent, pNext);
				}
				else if ((pCurrent->direct == 2 || pCurrent->direct == 3) && pNext->direct == 1) {
					pNext->direct = 2;
					pNext->groups = getGroups(pNext, nextGroups, pNext->direct);
					buildDirect23Topo(pCurrent, pNext);

					pNext->direct = 3;
					pNext->groups = getGroups(pNext, nextGroups, pNext->direct);
					buildDirect23Topo(pCurrent, pNext);
				}

				pNext->direct = nextDirect;
				pNext->groups = nextGroups;
				pCurrent->direct = currentDirect;
				pCurrent->groups = currentGroups;
			}
		}

		// RoadBoundary(解决人工画的线有一边没连上的情况)
		std::vector<HadElement*> laneGroups = center->query(ElementType::HAD_LANE_GROUP);
		for (auto& pElement : laneGroups)
		{
			HadLaneGroup* pCurrent = (HadLaneGroup*)pElement;
			if (pCurrent->next.size() == 1) {
				HadLaneGroup* pNext = (HadLaneGroup*)pCurrent->next[0];
				if (pNext->previous.size() == 1) {
					if (pCurrent->roadBoundaries.size() != 2 || pNext->roadBoundaries.size() != 2)
						continue;

					auto& leftBoundary = pCurrent->roadBoundaries[0];
					auto& rightBoundary = pCurrent->roadBoundaries[1];
					auto& nextLeftBoundary = pNext->roadBoundaries[0];
					auto& nextRightBoundary = pNext->roadBoundaries[1];
					if (std::find(leftBoundary->next.begin(), leftBoundary->next.end(), nextLeftBoundary) == leftBoundary->next.end()) {
						leftBoundary->next.push_back(nextLeftBoundary);
					}
					if (std::find(rightBoundary->next.begin(), rightBoundary->next.end(), nextRightBoundary) == rightBoundary->next.end()) {
						rightBoundary->next.push_back(nextRightBoundary);
					}
					if (std::find(nextLeftBoundary->previous.begin(), nextLeftBoundary->previous.end(), leftBoundary) == nextLeftBoundary->previous.end()) {
						nextLeftBoundary->previous.push_back(leftBoundary);
					}
					if (std::find(nextRightBoundary->previous.begin(), nextRightBoundary->previous.end(), rightBoundary) == nextRightBoundary->previous.end()) {
						nextRightBoundary->previous.push_back(rightBoundary);
					}
				}
			}
		}
	}

}