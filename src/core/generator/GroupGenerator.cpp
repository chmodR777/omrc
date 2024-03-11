#include "stdafx.h"
#include "GroupGenerator.h"
#include "algorithm/grap_point_algorithm.h"
#include <algorithm>
namespace OMDB
{
    void GroupGenerator::generate(DbMesh* const pMesh)
    {
		std::vector<DbRecord*>& links = pMesh->query(RecordType::DB_HAD_LINK);
        for (auto hl : links) {
            DbLink* phl = (DbLink*)hl;
			
			fixLanePaError(phl);
            if (phl->groups.empty()) {
				for (auto lanePa : phl->lanePas) {
					generateGroup(pMesh, phl, lanePa);
				}

				// 只有一组
				if (phl->lanePas.empty()) {

				}
            }
        }
    }

	void GroupGenerator::generateGroup(DbMesh* const pMesh, DbLink* pLink, DbRdLinkLanePa* lanePa)
	{
		int64 id = lanePa->uuid;
		int64 lgPaPid = lanePa->uuid;
		lanePa->generateHdData = true;
		DbLgLink* pLgLink = (DbLgLink*)pMesh->query(id, RecordType::DB_HAD_LG_LINK);
		if (pLgLink == nullptr) {
			pLgLink = (DbLgLink*)pMesh->alloc(RecordType::DB_HAD_LG_LINK);
			pLgLink->isGenerated = lanePa->isGenerated;
			pLgLink->uuid = id;
			pMesh->insert(pLgLink->uuid, pLgLink);
			pLink->groups.push_back(pLgLink);
		}

		for (auto pair : lanePa->relLinks) {
			pLgLink->relLinks.emplace(pair);
		}
	}

    void GroupGenerator::generateRelation(DbMesh* const pMesh, std::vector<DbMesh*>* nearby)
    {
		UNREFERENCED_PARAMETER(pMesh);
		UNREFERENCED_PARAMETER(nearby);
    }

	//20595884:84209477108308970数据错误(link方向为2,包含2和3方向的lanePa)
	//20599546:84207696039058426数据错误(link方向为2,只包含3方向的lanePa)
	void GroupGenerator::fixLanePaError(DbLink* const pLink)
	{
		auto direct = pLink->direct;
		if (direct == 2 || direct == 3)
		{
			for (auto iter = pLink->lanePas.begin(); iter != pLink->lanePas.end();)
			{
				auto relLinkPair = getRelLinkPair(pLink, *iter);
				if (relLinkPair.second.directType != direct)
				{
					iter = pLink->lanePas.erase(iter);
					continue;
				}
				iter++;
			}
		}
	}

}
