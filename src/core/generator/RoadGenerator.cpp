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

			// ����ת->ֱ��->��ת�ķ�������,���ȱ�����ת������
			std::sort(generatedLanePas.begin(), generatedLanePas.end(),
				[](DbRdLinkLanePa* first, DbRdLinkLanePa* second) {
					return first->lanePaAngle > second->lanePaAngle;
				});

			// �������ɵĳ�����,��ϲ�����������(�����߽硢������)
			auto generatedLanePa = generatedLanePas.front();
			for (auto iter = generatedLanePas.begin(); iter != generatedLanePas.end(); ) {
				auto otherLanePa = *iter;
				if (otherLanePa == generatedLanePa) {
					iter++;
					continue;
				}

				// TODO �ϲ������߽�,���������ߵ�

				// �߼�ɾ����·�߽�
				for (auto pBoundary : otherLanePa->roadBoundaries) {
					pBoundary->markDeleted = true;
				}

				// �߼�ɾ��������
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

				// �����б���ɾ������
				iter = generatedLanePas.erase(iter);
			}
        }

    }

}
