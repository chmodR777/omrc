#include "stdafx.h"
#include "MeshTopoBuilder.h"
#include <algorithm>

namespace OMDB
{	
	void MeshTopoBuilder::buildTopo(DbMesh* const pGrid)
	{
		// Link
		std::vector<DbRecord*> links = pGrid->query(RecordType::DB_HAD_LINK);

		// LaneGroup
		for (auto& pElement : links)
		{
			DbLink* pLink = (DbLink*)pElement;
			buildTopo(pLink, false);
		}

		auto buildDirect23Topo = [](DbLink* pCurrent, DbLink* pNext) {
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

		// Node
		std::vector<DbRecord*> nodes = pGrid->query(RecordType::DB_HAD_NODE);
		for (int i = 0; i < nodes.size(); i++)
		{
			DbNode* pNode = (DbNode*)nodes[i];
			for (int m = 0; m < pNode->links.size(); m++)
			{
				DbLink* pCurrent = (DbLink*)pNode->links[m];
				auto currentDirect = pCurrent->direct;
				auto currentGroups = pCurrent->lanePas;
				for (int n = m + 1; n < pNode->links.size(); n++)
				{
					DbLink* pNext = (DbLink*)pNode->links[n];
					auto nextDirect = pNext->direct;
					auto nextGroups = pNext->lanePas;
					if (pCurrent->direct != 1 && pNext->direct != 1) {
						buildDirect23Topo(pCurrent, pNext);
					}
					else if (pCurrent->direct == 1 && pNext->direct == 1) {
						pCurrent->direct = 2;
						pNext->direct = 2;
						pCurrent->lanePas = getGroups(pCurrent, currentGroups, pCurrent->direct);
						pNext->lanePas = getGroups(pNext, nextGroups, pNext->direct);
						buildDirect23Topo(pCurrent, pNext);

						pCurrent->direct = 2;
						pNext->direct = 3;
						pCurrent->lanePas = getGroups(pCurrent, currentGroups, pCurrent->direct);
						pNext->lanePas = getGroups(pNext, nextGroups, pNext->direct);
						buildDirect23Topo(pCurrent, pNext);

						pCurrent->direct = 3;
						pNext->direct = 2;
						pCurrent->lanePas = getGroups(pCurrent, currentGroups, pCurrent->direct);
						pNext->lanePas = getGroups(pNext, nextGroups, pNext->direct);
						buildDirect23Topo(pCurrent, pNext);

						pCurrent->direct = 3;
						pNext->direct = 3;
						pCurrent->lanePas = getGroups(pCurrent, currentGroups, pCurrent->direct);
						pNext->lanePas = getGroups(pNext, nextGroups, pNext->direct);
						buildDirect23Topo(pCurrent, pNext);
					}
					else if (pCurrent->direct == 1 && (pNext->direct == 2 || pNext->direct == 3)) {
						pCurrent->direct = 2;
						pCurrent->lanePas = getGroups(pCurrent, currentGroups, pCurrent->direct);
						buildDirect23Topo(pCurrent, pNext);

						pCurrent->direct = 3;
						pCurrent->lanePas = getGroups(pCurrent, currentGroups, pCurrent->direct);
						buildDirect23Topo(pCurrent, pNext);
					}
					else if ((pCurrent->direct == 2 || pCurrent->direct == 3) && pNext->direct == 1) {
						pNext->direct = 2;
						pNext->lanePas = getGroups(pNext, nextGroups, pNext->direct);
						buildDirect23Topo(pCurrent, pNext);

						pNext->direct = 3;
						pNext->lanePas = getGroups(pNext, nextGroups, pNext->direct);
						buildDirect23Topo(pCurrent, pNext);
					}

					pNext->direct = nextDirect;
					pNext->lanePas = nextGroups;
					pCurrent->direct = currentDirect;
					pCurrent->lanePas = currentGroups;

				}
			}
		}

	}

	void MeshTopoBuilder::buildTopo(DbLink* pLink, bool crossGridOnly)
	{
		if (pLink->lanePas.size() < 2)
			return;

		std::sort(pLink->lanePas.begin(), pLink->lanePas.end(),
			[&](const DbRdLinkLanePa* first, const DbRdLinkLanePa* second)->bool {
				auto relLinkFirst = getRelLinkPair(pLink, first);
				auto relLinkSecond = getRelLinkPair(pLink, second);
				return relLinkFirst.second.startOffset < relLinkSecond.second.startOffset;
			}
		);

		auto buildDirect2Topo = [](DbLink* pLink, std::vector<DbRdLinkLanePa*>& groups, bool crossGridOnly) {
			if (groups.size() < 2)
				return;

			for (int i = 0; i < groups.size() - 1; i++) {
				DbRdLinkLanePa* pCurrent = groups[i];
				DbRdLinkLanePa* pNext = groups[i + 1];
				bool crossGrid = pCurrent->owner != pNext->owner;
				if (crossGridOnly && !crossGrid) {
					continue;
				}

				auto rc = getRelLinkPair(pLink, pCurrent);
				auto rn = getRelLinkPair(pLink, pNext);
				if (std::abs(rc.second.endOffset - rn.second.startOffset) < 0.01)
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

		auto buildDirect3Topo = [](DbLink* pLink, std::vector<DbRdLinkLanePa*>& groups, bool crossGridOnly) {
			if (groups.size() < 2)
				return;

			for (int i = groups.size() - 1; i > 0; i--) {
				DbRdLinkLanePa* pCurrent = groups[i];
				DbRdLinkLanePa* pNext = groups[i - 1];
				bool crossGrid = pCurrent->owner != pNext->owner;
				if (crossGridOnly && !crossGrid) {
					continue;
				}

				auto rc = getRelLinkPair(pLink, pCurrent);
				auto rn = getRelLinkPair(pLink, pNext);
				if (std::abs(rn.second.endOffset - rc.second.startOffset) < 0.01)
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
			buildDirect2Topo(pLink, pLink->lanePas, crossGridOnly);
		}
		else if (pLink->direct == 3) {
			buildDirect3Topo(pLink, pLink->lanePas, crossGridOnly);
		}
		else if (pLink->direct == 1) {
			buildDirect2Topo(pLink, getGroups(pLink, pLink->lanePas, 2), crossGridOnly);
			buildDirect3Topo(pLink, getGroups(pLink, pLink->lanePas, 3), crossGridOnly);
		}
	}

    void MeshTopoBuilder::buildTopo(const std::vector<DbMesh*>& grids)
    {
		for (auto& pMesh : grids)
		{
			buildTopo(pMesh);
		}
    }

	void MeshTopoBuilder::buildTopo(DbLink* pCurrent, DbLink* pNext)
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
		if (pCurrent->lanePas.empty() || pNext->lanePas.empty())
			return;

		DbRdLinkLanePa* pCurrentLastGroup = nullptr;
		DbRdLinkLanePa* pNextFirstGroup = nullptr;
		if (pCurrent->direct == 2 && pNext->direct == 2) {
			pCurrentLastGroup = *pCurrent->lanePas.rbegin();
			pNextFirstGroup = pNext->lanePas[0];
		}
		else if (pCurrent->direct == 2 && pNext->direct == 3) {
			pCurrentLastGroup = *pCurrent->lanePas.rbegin();
			pNextFirstGroup = *pNext->lanePas.rbegin();
		}
		else if (pCurrent->direct == 3 && pNext->direct == 2) {
			pCurrentLastGroup = pCurrent->lanePas[0];
			pNextFirstGroup = pNext->lanePas[0];
		}
		else if (pCurrent->direct == 3 && pNext->direct == 3) {
			pCurrentLastGroup = pCurrent->lanePas[0];
			pNextFirstGroup = *pNext->lanePas.rbegin();
		}

		// 车道组构建关系
		buildTopo(pCurrentLastGroup, pNextFirstGroup);
		if (pCurrent->owner != pNext->owner &&
			pCurrent->crossGrid && pNext->crossGrid &&
			pCurrentLastGroup != nullptr && pNextFirstGroup != nullptr) {
			//pCurrentLastGroup->crossGrid = true;
			//pNextFirstGroup->crossGrid = true;
		}

	}

	void MeshTopoBuilder::buildTopo(DbRdLinkLanePa* pCurrent, DbRdLinkLanePa* pNext)
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
	}

	std::pair<int64, DbRdLinkLanePa::DbRelLink> MeshTopoBuilder::getRelLinkPair(const DbLink* pLink, const DbRdLinkLanePa* lanePa)
	{
		std::pair<int64, DbHadLinkLanePa::DbRelLink> relLinkPair;
		for (auto& pair : lanePa->relLinks) {
			if (pair.second.relLinkid == pLink->uuid) {
				relLinkPair = pair;
				break;
			}
		}
		return relLinkPair;
	}

	std::vector<DbRdLinkLanePa*> MeshTopoBuilder::getGroups(DbLink* pLink, std::vector<DbRdLinkLanePa*>& lanePas, int direct)
	{
		std::vector<DbRdLinkLanePa*> grps;
		for (auto lanePa : lanePas) {
			for (auto& relLinkPair : lanePa->relLinks) {
				if (relLinkPair.second.relLinkid == pLink->uuid) {
					if (relLinkPair.second.directType == direct) {
						if (std::find(grps.begin(), grps.end(), lanePa) == grps.end()) {
							grps.push_back(lanePa);
							break;
						}
					}
				}
			}
		}

		std::sort(grps.begin(), grps.end(),
			[&](const DbRdLinkLanePa* first, const DbRdLinkLanePa* second)->bool {
				auto relLinkFirst = getRelLinkPair(pLink, first);
				auto relLInkSecond = getRelLinkPair(pLink, second);
				return relLinkFirst.second.startOffset < relLInkSecond.second.startOffset;
			}
		);
		return grps;
	}

	void MeshTopoBuilder::buildTopoCrossGrid(DbMesh* center, const std::vector<DbMesh*>* nearby)
	{
		auto isCrossGrid = [](const DbLink* pLink)->bool {
			if (pLink->crossGrid)
				return true;
			//for (auto group : pLink->lanePas) {
			//	if (group->crossGrid)
			//		return true;
			//}
			return false;
		};

		// 处理跨网格情况
		std::vector<DbLink*> links;
		auto addCrossGridLink = [&](DbMesh* pGrid) {
			std::vector<DbRecord*>& tmp = pGrid->query(RecordType::DB_HAD_LINK);
			for_each(tmp.begin(), tmp.end(), [&](DbRecord* pElement)->void
				{
					DbLink* pLink = (DbLink*)pElement;
					bool crossGrid = isCrossGrid(pLink);
					if (!crossGrid) {
						for (auto p : pLink->previous) {
							if (isCrossGrid((DbLink*)p)) {
								crossGrid = true;
								break;
							}
						}
					}
					if (!crossGrid) {
						for (auto n : pLink->next) {
							if (isCrossGrid((DbLink*)n)) {
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
		for_each(nearby->begin(), nearby->end(), [&](DbMesh* pGrid)->void
			{
				addCrossGridLink(pGrid);
			}
		);

		auto buildDirect23Topo = [](DbLink* pCurrent, DbLink* pNext) {
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

		// Link
		for (int m = 0; m < links.size(); m++)
		{
			DbLink* pCurrent = (DbLink*)links[m];
			auto currentDirect = pCurrent->direct;
			auto currentGroups = pCurrent->lanePas;
			for (int n = m + 1; n < links.size(); n++)
			{
				DbLink* pNext = (DbLink*)links[n];
				auto nextDirect = pNext->direct;
				auto nextGroups = pNext->lanePas;
				if (pCurrent->direct != 1 && pNext->direct != 1) {
					buildDirect23Topo(pCurrent, pNext);
				}
				else if(pCurrent->direct == 1 && pNext->direct == 1) {
					pCurrent->direct = 2;
					pNext->direct = 2;
					pCurrent->lanePas = getGroups(pCurrent, currentGroups, pCurrent->direct);
					pNext->lanePas = getGroups(pNext, nextGroups, pNext->direct);
					buildDirect23Topo(pCurrent, pNext);

					pCurrent->direct = 2;
					pNext->direct = 3;
					pCurrent->lanePas = getGroups(pCurrent, currentGroups, pCurrent->direct);
					pNext->lanePas = getGroups(pNext, nextGroups, pNext->direct);
					buildDirect23Topo(pCurrent, pNext);

					pCurrent->direct = 3;
					pNext->direct = 2;
					pCurrent->lanePas = getGroups(pCurrent, currentGroups, pCurrent->direct);
					pNext->lanePas = getGroups(pNext, nextGroups, pNext->direct);
					buildDirect23Topo(pCurrent, pNext);

					pCurrent->direct = 3;
					pNext->direct = 3;
					pCurrent->lanePas = getGroups(pCurrent, currentGroups, pCurrent->direct);
					pNext->lanePas = getGroups(pNext, nextGroups, pNext->direct);
					buildDirect23Topo(pCurrent, pNext);
				}
				else if (pCurrent->direct == 1 && (pNext->direct == 2 || pNext->direct == 3)) {
					pCurrent->direct = 2;
					pCurrent->lanePas = getGroups(pCurrent, currentGroups, pCurrent->direct);
					buildDirect23Topo(pCurrent, pNext);

					pCurrent->direct = 3;
					pCurrent->lanePas = getGroups(pCurrent, currentGroups, pCurrent->direct);
					buildDirect23Topo(pCurrent, pNext);
				}
				else if ((pCurrent->direct == 2 || pCurrent->direct == 3) && pNext->direct == 1) {
					pNext->direct = 2;
					pNext->lanePas = getGroups(pNext, nextGroups, pNext->direct);
					buildDirect23Topo(pCurrent, pNext);

					pNext->direct = 3;
					pNext->lanePas = getGroups(pNext, nextGroups, pNext->direct);
					buildDirect23Topo(pCurrent, pNext);
				}

				pNext->direct = nextDirect;
				pNext->lanePas = nextGroups;
				pCurrent->direct = currentDirect;
				pCurrent->lanePas = currentGroups;
			}
		}

	}

}