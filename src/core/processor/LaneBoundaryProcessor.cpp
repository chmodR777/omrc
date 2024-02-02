#include "stdafx.h"
#include "LaneBoundaryProcessor.h"
#include <algorithm>
namespace OMDB
{
	void LaneBoundaryProcessor::process(DbMesh* const pMesh, HadGrid* pGrid)
	{
		std::vector<DbRecord*>& nodes = pMesh->query(RecordType::DB_HAD_LANE_MARK_NODE);
		for (auto nd : nodes)
		{
			DbLaneMarkNode* pnd = (DbLaneMarkNode*)nd;
			HadLaneBoundaryNode* pNode = (HadLaneBoundaryNode*)pGrid->alloc(ElementType::HAD_LANE_BOUNDARY_NODE);
			pNode->position = pnd->geometry;
			pNode->originId = pnd->uuid;
			pGrid->insert(pNode->originId, pNode);
		}
		std::vector<DbRecord*>& links = pMesh->query(RecordType::DB_HAD_LANE_MARK_LINK);
		for (auto& hl : links)
		{
			DbLaneMarkLink* phl = (DbLaneMarkLink*)hl;
			HadLaneBoundary* pBoundary = (HadLaneBoundary*)pGrid->alloc(ElementType::HAD_LANE_BOUNDARY);
			pBoundary->originId = phl->uuid;
			pBoundary->location = phl->geometry;
			// ���������phl->relLgs������
			// pBoundary->lgMarkDirect = (unsigned short)phl->lgMarkDirect;

			pBoundary->markings.resize(phl->markings.size());

			memcpy(pBoundary->markings.data(), phl->markings.data(), sizeof(HadLaneBoundary::HadLaneBoundaryMarking) * phl->markings.size());
			pBoundary->boundaryType = (BoundaryType)phl->boundaryType;
			pBoundary->startNode = (HadLaneBoundaryNode*)pGrid->query(phl->startLaneMarkNodeId, ElementType::HAD_LANE_BOUNDARY_NODE);
			pBoundary->endNode = (HadLaneBoundaryNode*)pGrid->query(phl->endLaneMarkNodeId, ElementType::HAD_LANE_BOUNDARY_NODE);
			// PA
			std::vector<DbRecord*>& pas = pMesh->query(RecordType::DB_HAD_LANE_MARK_PA);
			for (auto pa : pas)
			{
				DbLaneMarkPA* ppa = (DbLaneMarkPA*)pa;
				if (ppa->relLaneMarkLinkId == pBoundary->originId)
				{
					for (DbPAValue* paValue : ppa->paValues)
					{
						HadPartAttribute* pAttribute = (HadPartAttribute*)pGrid->alloc(ElementType::HAD_PART_ATTRIBUTE);
						pAttribute->originId = ppa->uuid;
						pAttribute->start = ppa->startOffset;
						pAttribute->end = ppa->endOffset;
						pAttribute->seqNum = paValue->seqNum;
						pAttribute->name = paValue->attributeType;
						pAttribute->value = paValue->attributeValue;
						pAttribute->points = ppa->geometry;
						pBoundary->attributes.push_back(pAttribute);
					}
				}
			}
			// �����߽�PA��Ϣ��0~1����
			std::sort(pBoundary->attributes.begin(), pBoundary->attributes.end(),
				[](const HadPartAttribute* first, const HadPartAttribute* second)->bool {
					if (first->start != second->start)
						return first->start < second->start;
					return first->end < second->end;
				});

			// relLglink,��һ������
			// �����߽���-->���������߹�ϵ
			for (auto& laneMarkRel : phl->laneMarkRels)
			{
				HadLane* pLane = (HadLane*)pGrid->query(laneMarkRel.relLaneLinkId, ElementType::HAD_LANE);
				if (pLane == nullptr)
					continue;

				//�����߽���-->���������߹�ϵ�У�sideֵ���߽���λ�ڳ������������Ϊ2���Ҳ�Ϊ1��
				if (laneMarkRel.side == 1)
					pLane->rightBoundary = pBoundary;
				else if (laneMarkRel.side == 2)
					pLane->leftBoundary = pBoundary;

                // ��ת��PA
                if (pLane->leftBoundary && pLane->laneTurnwaiting)
                    pLane->leftBoundary->turnwaitingPAs.emplace_back(pLane->laneTurnwaiting);
                if (pLane->rightBoundary && pLane->laneTurnwaiting)
                    pLane->rightBoundary->turnwaitingPAs.emplace_back(pLane->laneTurnwaiting);

				//ͨ�������߽���-->���������ߵĹ�ϵ�����㳵���߽�����š�
				pBoundary->seqNumberOnLG = pLane->seqNumber + laneMarkRel.side % 2;

				// TODO ������ʱ���ڳ����߽�û�ж�Ӧ�����������
				if (pLane->linkGroup != nullptr) {
					if (std::find(pLane->linkGroup->laneBoundaries.begin(), pLane->linkGroup->laneBoundaries.end(), pBoundary) == pLane->linkGroup->laneBoundaries.end()) {
						pLane->linkGroup->laneBoundaries.push_back(pBoundary);
					}
					pBoundary->linkGroups.emplace(pLane->linkGroup->originId, pLane->linkGroup);
					pBoundary->seqNumberOnLGs.emplace(pLane->linkGroup->originId, pBoundary->seqNumberOnLG);
				}
			}

			// һ�������߽�������ٽ����������鹲���,����lgMarkDirect��lgMarkSeqNum��ͬ
			for (auto& relLg : phl->relLgs) {
				HadLaneBoundary::HadLgMarkRel hadRelLg;
				hadRelLg.lgMarkDirect = relLg.second.lgMarkDirect;
				hadRelLg.lgMarkSeqNum = relLg.second.lgMarkSeqNum;
				pBoundary->relLgs.emplace(relLg.first, hadRelLg);

				HadLaneGroup* pGroup = (HadLaneGroup*)pGrid->query(relLg.first, ElementType::HAD_LANE_GROUP);
				if (pGroup != nullptr)
				{
					// ��һ��ͨ��pLane�����Ѿ������,�����Ҫȥ����
					if (std::find(pGroup->laneBoundaries.begin(), pGroup->laneBoundaries.end(), pBoundary) == pGroup->laneBoundaries.end()) {
						pGroup->laneBoundaries.push_back(pBoundary);
					}
					pBoundary->linkGroups.emplace(relLg.first, pGroup);
				}
			}

			pGrid->insert(pBoundary->originId, pBoundary);
		}
	}

	void LaneBoundaryProcessor::processRelation(DbMesh* const pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby)
	{
		std::vector<DbRecord*>& links = pMesh->query(RecordType::DB_HAD_LANE_MARK_LINK);
		for (auto& hl : links)
		{
			DbLaneMarkLink* phl = (DbLaneMarkLink*)hl;
			HadLaneBoundary* pBoundary = (HadLaneBoundary*)pGrid->query(phl->uuid, ElementType::HAD_LANE_BOUNDARY);
			if (pBoundary == nullptr) {
				continue;
			}

			// relLglink,��һ������
			// �����߽���-->���������߹�ϵ
			for (auto& laneMarkRel : phl->laneMarkRels)
			{
				HadLane* pLane = (HadLane*)queryNearby(pGrid, nearby, laneMarkRel.relLaneLinkId, ElementType::HAD_LANE);
				if (pLane == nullptr)
					continue;

				//�����߽���-->���������߹�ϵ�У�sideֵ���߽���λ�ڳ������������Ϊ2���Ҳ�Ϊ1��
				if (laneMarkRel.side == 1)
					pLane->rightBoundary = pBoundary;
				else if (laneMarkRel.side == 2)
					pLane->leftBoundary = pBoundary;

                // ��ת��PA
                if (pLane->leftBoundary&& pLane->laneTurnwaiting)
                    pLane->leftBoundary->turnwaitingPAs.emplace_back(pLane->laneTurnwaiting) ;
                if (pLane->rightBoundary&& pLane->laneTurnwaiting)
                    pLane->rightBoundary->turnwaitingPAs.emplace_back(pLane->laneTurnwaiting);
				
				//ͨ�������߽���-->���������ߵĹ�ϵ�����㳵���߽�����š�
				pBoundary->seqNumberOnLG = pLane->seqNumber + laneMarkRel.side % 2;

				// TODO ������ʱ���ڳ����߽�û�ж�Ӧ�����������
				if (pLane->linkGroup != nullptr) {
					if (std::find(pLane->linkGroup->laneBoundaries.begin(), pLane->linkGroup->laneBoundaries.end(), pBoundary) == pLane->linkGroup->laneBoundaries.end()) {
						pLane->linkGroup->laneBoundaries.push_back(pBoundary);
					}
					pBoundary->linkGroups.emplace(pLane->linkGroup->originId, pLane->linkGroup);
					pBoundary->seqNumberOnLGs.emplace(pLane->linkGroup->originId, pBoundary->seqNumberOnLG);
				}
			}

			for (auto& relLg : phl->relLgs) {
				HadLaneGroup* pGroup = (HadLaneGroup*)queryNearby(pGrid, nearby, relLg.first, ElementType::HAD_LANE_GROUP);
				if (pGroup != nullptr)
				{
					// ��һ��ͨ��pLane�����Ѿ������,�����Ҫȥ����
					if (std::find(pGroup->laneBoundaries.begin(), pGroup->laneBoundaries.end(), pBoundary) == pGroup->laneBoundaries.end()) {
						pGroup->laneBoundaries.push_back(pBoundary);
					}
					pBoundary->linkGroups.emplace(relLg.first, pGroup);
				}
			}

			// ��������ͬһ���߽�Ҳ���ܴ��ڹ�ϵ
			HadLaneBoundary* nearbyPrb = (HadLaneBoundary*)queryNearby(pGrid, nearby, phl->uuid, ElementType::HAD_LANE_BOUNDARY);
			if (nearbyPrb == nullptr) {
				continue;
			}

			for (auto& relLg : nearbyPrb->relLgs) {
				pBoundary->relLgs.emplace(relLg.first, relLg.second);
				HadLaneGroup* pGroup = (HadLaneGroup*)queryNearby(pGrid, nearby, relLg.first, ElementType::HAD_LANE_GROUP);
				if (pGroup != nullptr)
				{
					pBoundary->linkGroups.emplace(relLg.first, pGroup);
				}
			}
		}

		//�����������鰴�ڳ������ڴ����ҵ�˳������
		for (auto obj : pGrid->query(ElementType::HAD_LANE_GROUP))
		{
			HadLaneGroup* pGroup = (HadLaneGroup*)obj;
			int64 lgId = pGroup->originId;
			std::sort(pGroup->laneBoundaries.begin(), pGroup->laneBoundaries.end(),
				[=](const HadLaneBoundary* first, const HadLaneBoundary* second)->bool {
					return first->getSeqNumberOnLG(lgId) < second->getSeqNumberOnLG(lgId); });
		}

	}

}
