#include "stdafx.h"
#include "RoadGenerator.h"
#include "algorithm/grap_point_algorithm.h"
#include <algorithm>
namespace OMDB
{
    void RoadGenerator::generate(DbMesh* const pMesh)
    {
        UNREFERENCED_PARAMETER(pMesh);
    }

    void RoadGenerator::generateRelation(DbMesh* const pMesh, std::vector<DbMesh*>* nearby)
    {
		m_mesh = pMesh;
		m_nearbyMesh = nearby;

        mergeLanePas();
    }

    void RoadGenerator::mergeLanePas()
    {
        auto lanePas = m_mesh->query(RecordType::DB_RD_LINK_LANEPA);
        for (auto lanePa : lanePas) 
        {
            DbRdLinkLanePa* pLanePa = (DbRdLinkLanePa*)lanePa;
			if (172649414767622659 == pLanePa->uuid)
				printf("");

			mergeLanePa(pLanePa);
        }
    }

    void RoadGenerator::mergeLanePa(DbRdLinkLanePa* pLanePa)
    {
        auto& generatedLanePaMap = pLanePa->generatedLanePas;
        for (auto generatedLanePaPair : generatedLanePaMap)
        {
            auto& generatedLanePas = generatedLanePaPair.second;
			if (generatedLanePas.size() <= 1)
				continue;

			// 按右转->直行->左转的方向排序,优先保留右转车道组
			std::sort(generatedLanePas.begin(), generatedLanePas.end(),
				[](DbRdLinkLanePa* first, DbRdLinkLanePa* second) {
					return first->lanePaAngle > second->lanePaAngle;
				});

			// 真正生成的车道组,需合并其他车道组(车道边界、车道线)
			auto generatedLanePa = generatedLanePas.front();
			for (auto iter = generatedLanePas.begin(); iter != generatedLanePas.end(); ) {
				auto otherLanePa = *iter;
				if (otherLanePa == generatedLanePa) {
					iter++;
					continue;
				}

				// TODO 合并车道边界,车道中心线等

				// 逻辑删除道路边界
				for (auto pBoundary : otherLanePa->roadBoundaries) {
					pBoundary->markDeleted = true;
				}

				// 逻辑删除车道组
				otherLanePa->markDeleted = true;
				for (auto& relLinkPair : otherLanePa->relLinks) {
					auto pLink = (DbLink*)m_mesh->query(relLinkPair.second.relLinkid, RecordType::DB_HAD_LINK);
					auto lanePaIter = std::find(pLink->lanePas.begin(), pLink->lanePas.end(), otherLanePa);
					if (lanePaIter != pLink->lanePas.end()) {
						pLink->lanePas.erase(lanePaIter);
					}
				}
				DbLgLink* pLgLink = (DbLgLink*)m_mesh->query(otherLanePa->uuid, RecordType::DB_HAD_LG_LINK);
				if (pLgLink != nullptr) {
					pLgLink->markDeleted = true;
				}

				// 最后从列表中删除该项
				iter = generatedLanePas.erase(iter);
			}
        }

    }

}
