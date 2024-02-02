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

}
