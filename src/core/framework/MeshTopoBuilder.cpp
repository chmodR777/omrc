#include "stdafx.h"
#include "MeshTopoBuilder.h"
#include <algorithm>
#include <mutex>
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

		// Node
		std::vector<DbRecord*> nodes = pGrid->query(RecordType::DB_HAD_NODE);
		for (int i = 0; i < nodes.size(); i++)
		{
			DbNode* pNode = (DbNode*)nodes[i];
			buildTopo(pNode->links);
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
				if (relLinkFirst.second.startOffset != relLinkSecond.second.startOffset)
					return relLinkFirst.second.startOffset < relLinkSecond.second.startOffset;
				return relLinkFirst.second.endOffset < relLinkSecond.second.endOffset;
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
					// buildTopo(pCurrent, pNext);
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
					// buildTopo(pCurrent, pNext);
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

	void MeshTopoBuilder::buildTopo(std::vector<DbLink*>& links)
	{
		for (int m = 0; m < links.size(); m++)
		{
			DbLink* pCurrent = (DbLink*)links[m];
			for (int n = m + 1; n < links.size(); n++)
			{
				DbLink* pNext = (DbLink*)links[n];
				if (pCurrent->direct != 1 && pNext->direct != 1) {
					auto currentDirectLink = DbDirectLink::getDirectLink(pCurrent, pCurrent->direct);
					auto nextDirectLink = DbDirectLink::getDirectLink(pNext, pNext->direct);
					buildDirectTopo(&currentDirectLink, &nextDirectLink);
				}
				else if (pCurrent->direct == 1 && pNext->direct == 1) {
					auto currentDirectLink = DbDirectLink::getDirectLink(pCurrent, 2);
					auto nextDirectLink = DbDirectLink::getDirectLink(pNext, 2);
					buildDirectTopo(&currentDirectLink, &nextDirectLink);

					currentDirectLink = DbDirectLink::getDirectLink(pCurrent, 2);
					nextDirectLink = DbDirectLink::getDirectLink(pNext, 3);
					buildDirectTopo(&currentDirectLink, &nextDirectLink);

					currentDirectLink = DbDirectLink::getDirectLink(pCurrent, 3);
					nextDirectLink = DbDirectLink::getDirectLink(pNext, 2);
					buildDirectTopo(&currentDirectLink, &nextDirectLink);

					currentDirectLink = DbDirectLink::getDirectLink(pCurrent, 3);
					nextDirectLink = DbDirectLink::getDirectLink(pNext, 3);
					buildDirectTopo(&currentDirectLink, &nextDirectLink);
				}
				else if (pCurrent->direct == 1 && (pNext->direct == 2 || pNext->direct == 3)) {
					auto currentDirectLink = DbDirectLink::getDirectLink(pCurrent, 2);
					auto nextDirectLink = DbDirectLink::getDirectLink(pNext, pNext->direct);
					buildDirectTopo(&currentDirectLink, &nextDirectLink);

					currentDirectLink = DbDirectLink::getDirectLink(pCurrent, 3);
					buildDirectTopo(&currentDirectLink, &nextDirectLink);
				}
				else if ((pCurrent->direct == 2 || pCurrent->direct == 3) && pNext->direct == 1) {
					auto currentDirectLink = DbDirectLink::getDirectLink(pCurrent, pCurrent->direct);
					auto nextDirectLink = DbDirectLink::getDirectLink(pNext, 2);
					buildDirectTopo(&currentDirectLink, &nextDirectLink);

					nextDirectLink = DbDirectLink::getDirectLink(pNext, 3);
					buildDirectTopo(&currentDirectLink, &nextDirectLink);
				}

			}
		}
	}

	void MeshTopoBuilder::buildDirectTopo(DbDirectLink* pCurrent, DbDirectLink* pNext)
	{
		if (pCurrent->direct == 2 && pNext->direct == 2) {
			if (pCurrent->getStartNode() == pNext->getEndNode()) {
				buildLinkTopo(pNext, pCurrent);
			}
			else if (pCurrent->getEndNode() == pNext->getStartNode()) {
				buildLinkTopo(pCurrent, pNext);
			}
		}
		else if (pCurrent->direct == 2 && pNext->direct == 3) {
			if (pCurrent->getEndNode() == pNext->getEndNode()) {
				buildLinkTopo(pCurrent, pNext);
			}
			else if (pCurrent->getStartNode() == pNext->getStartNode()) {
				buildLinkTopo(pNext, pCurrent);
			}
		}
		else if (pCurrent->direct == 3 && pNext->direct == 2) {
			if (pCurrent->getEndNode() == pNext->getEndNode()) {
				buildLinkTopo(pNext, pCurrent);
			}
			else if (pCurrent->getStartNode() == pNext->getStartNode()) {
				buildLinkTopo(pCurrent, pNext);
			}
		}
		else if (pCurrent->direct == 3 && pNext->direct == 3) {
			if (pCurrent->getStartNode() == pNext->getEndNode()) {
				buildLinkTopo(pCurrent, pNext);
			}
			else if (pCurrent->getEndNode() == pNext->getStartNode()) {
				buildLinkTopo(pNext, pCurrent);
			}
		}
	}

	void MeshTopoBuilder::buildLinkTopo(DbDirectLink* pCurrent, DbDirectLink* pNext)
	{
		// 进来该函数的都是有向Link
		if (pCurrent->direct == 1 || pNext->direct == 1) {
			return;
		}

		auto pCurrentLink = pCurrent->link;
		auto pNextLink = pNext->link;
		{
			std::lock_guard<Spinlock> lock(pCurrentLink->nextLock);
			if (std::find(pCurrentLink->next.begin(), pCurrentLink->next.end(), pNextLink) == pCurrentLink->next.end())
			{
				pCurrentLink->next.push_back(pNextLink);
			}
		}

		{
			std::lock_guard<Spinlock> lock(pCurrentLink->previousLock);
			if (std::find(pNextLink->previous.begin(), pNextLink->previous.end(), pCurrentLink) == pNextLink->previous.end())
			{
				pNextLink->previous.push_back(pCurrentLink);
			}
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
				if (relLinkFirst.second.startOffset != relLInkSecond.second.startOffset)
					return relLinkFirst.second.startOffset < relLInkSecond.second.startOffset;
				return relLinkFirst.second.endOffset < relLInkSecond.second.endOffset;
			}
		);
		return grps;
	}

	void MeshTopoBuilder::buildTopoCrossGrid(DbMesh* center, const std::vector<DbMesh*>* nearby)
	{
		// 处理跨网格情况
		std::map<int64, DbNode*> nodes;
		auto addCrossGridLink = [&](DbMesh* pGrid) {
			std::vector<DbRecord*>& tmp = pGrid->query(RecordType::DB_HAD_NODE);
			for_each(tmp.begin(), tmp.end(), [&](DbRecord* pElement)->void
				{
					bool crossGridNode = false;
					DbNode* pNode = (DbNode*)pElement;
					for (auto idx = 0; idx < pNode->links.size(); idx++)
						if (pNode->links[idx]->crossGrid)
							crossGridNode = true; 
					if (crossGridNode) {
						nodes.emplace(pNode->uuid, pNode);
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

		// Link
		for (auto pair : nodes) {
			buildTopo(pair.second->links);
		}
	}

}