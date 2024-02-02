#include "stdafx.h"
#include "LinkGenerator.h"
#include "framework/MeshTopoBuilder.h"
#include <algorithm>
namespace OMDB
{
    void LinkGenerator::generate(DbMesh* const pMesh)
    {
		generateLanePas(pMesh);
		generateGroups(pMesh);

		std::vector<DbRecord*>& links = pMesh->query(RecordType::DB_HAD_LINK);
		for (auto hl : links) {
			DbLink* phl = (DbLink*)hl;
			auto pLinkName = (DbLinkName*)pMesh->query(phl->uuid, RecordType::DB_HAD_LINK_NAME);
			if (pLinkName != nullptr) {
				if (std::find(phl->linkNames.begin(), phl->linkNames.end(), pLinkName) == phl->linkNames.end())
					phl->linkNames.push_back(pLinkName);
				for (auto& linkNamePa : pLinkName->linkNamePas) {
					auto roadNameGroup = (DbRoadName*)pMesh->query(linkNamePa.nameGroup, RecordType::DB_ROAD_NAME);
					if (roadNameGroup != nullptr) {
						if (std::find(linkNamePa.roadNames.begin(), linkNamePa.roadNames.end(), roadNameGroup) == linkNamePa.roadNames.end())
							linkNamePa.roadNames.push_back(roadNameGroup);
					}
				}
				// linkNamePa信息从0~1排序
				std::sort(pLinkName->linkNamePas.begin(), pLinkName->linkNamePas.end(),
					[](const auto& first, const auto& second)->bool {
						if (first.startOffset != second.startOffset)
							return first.startOffset < second.startOffset;
						return first.endOffset < second.endOffset;
					});
			}
			auto startNode = (DbNode*)pMesh->query(phl->startNode, RecordType::DB_HAD_NODE);
			auto endNode = (DbNode*)pMesh->query(phl->endNode, RecordType::DB_HAD_NODE);
			phl->crossGrid = startNode->meshIds.size() > 1 || endNode->meshIds.size() > 1;
			if (std::find(startNode->links.begin(), startNode->links.end(), phl) == startNode->links.end())
				startNode->links.push_back(phl);
			if (std::find(endNode->links.begin(), endNode->links.end(), phl) == endNode->links.end())
				endNode->links.push_back(phl);
			phl->startNodePtr = startNode;
			phl->endNodePtr = endNode;

			// 建立路网索引
			dbLinkInfo info;
			info._link = phl;
			auto& tmpLine = phl->geometry.vertexes;
			//coordinatesTransform.convert(tmpLine.data(), tmpLine.size());
			info._linkBox2T = BOX_2T(tmpLine);
			info._linkPoints2T = LINESTRING_2T(tmpLine);
			pMesh->insertLinkInfo(info);
		}

		// 建立topo关系
		MeshTopoBuilder::buildTopo(pMesh);
    }

    void LinkGenerator::generateRelation(DbMesh* const pMesh, std::vector<DbMesh*>* nearby)
    {
		// 建立跨网格topo关系
		MeshTopoBuilder::buildTopoCrossGrid(pMesh, nearby);
    }

    void LinkGenerator::generateLanePas(DbMesh* const pMesh)
    {
		// CLM LANEPA
		for (auto& lp : pMesh->query(RecordType::DB_RD_LINK_LANEPA)) {
			DbRdLinkLanePa* pLinkLanePa = (DbRdLinkLanePa*)lp;
			for (auto& pair : pLinkLanePa->relLinks) {
				DbLink* pLink = (DbLink*)pMesh->query(pair.second.relLinkid, RecordType::DB_HAD_LINK);
				if (pLink != nullptr) {
					if (std::find(pLink->lanePas.begin(), pLink->lanePas.end(), pLinkLanePa) == pLink->lanePas.end())
						pLink->lanePas.push_back(pLinkLanePa);
				}
			}
		}
    }

    void LinkGenerator::generateGroups(DbMesh* const pMesh)
    {
		// HAD LANEGROUP
		// 如下高精表数据删除了就是用CLM生成的
		if (CompileSetting::instance()->isGenerateHdData)
		{
			pMesh->queryTable(RecordType::DB_HAD_LG_LINK).clear();
			pMesh->queryTable(RecordType::DB_HAD_ROAD_BOUNDARY_LINK).clear();
			pMesh->queryTable(RecordType::DB_HAD_LANE_MARK_LINK).clear();
			pMesh->queryTable(RecordType::DB_HAD_OBJECT_BARRIER).clear();
			std::vector<DbRecord*>& links = pMesh->query(RecordType::DB_HAD_LINK);
			for (auto hl : links) {
				DbLink* phl = (DbLink*)hl;
				for (auto& vertex : phl->geometry.vertexes) {
					vertex.z = 0;
				}
			}
		}

		for (auto& lg : pMesh->query(RecordType::DB_HAD_LG_LINK)) {
			DbLgLink* plg = (DbLgLink*)lg;
			for (auto& pair : plg->relLinks) {
				DbLink* pLink = (DbLink*)pMesh->query(pair.second.relLinkid, RecordType::DB_HAD_LINK);
				if (pLink != nullptr) {
					if (std::find(pLink->groups.begin(), pLink->groups.end(), plg) == pLink->groups.end())
						pLink->groups.push_back(plg);
				}
			}
		}
    }

}
