#include "stdafx.h"
#include "LaneProcessor.h"
#include <algorithm>
namespace OMDB
{
    void LaneProcessor::process(DbMesh* const pMesh, HadGrid* pGrid)
    {
		std::vector<DbRecord*>& nodes = pMesh->query(RecordType::DB_HAD_LANE_NODE);
		for (auto nd : nodes)
		{
			DbLaneNode* pnd = (DbLaneNode*)nd;
			HadLaneNode* pNode = (HadLaneNode*)pGrid->alloc(ElementType::HAD_LANE_NODE);
			pNode->position = pnd->geometry;
			pNode->originId = pnd->uuid;
			pGrid->insert(pNode->originId, pNode);
		}

		std::vector<DbRecord*>& links = pMesh->query(RecordType::DB_HAD_LANE_LINK);
		for (auto hl : links)
		{
			DbLaneLink* phl = (DbLaneLink*)hl;
			HadLane* pLane = (HadLane*)pGrid->alloc(ElementType::HAD_LANE);
			pLane->originId = phl->uuid;
			pLane->location = phl->geometry;
			pLane->laneType = phl->laneType;
			pLane->startNode = (HadLaneNode*)pGrid->query(phl->startLaneNodeId, ElementType::HAD_LANE_NODE);
			pLane->endNode = (HadLaneNode*)pGrid->query(phl->endLaneNodeId, ElementType::HAD_LANE_NODE);
			pLane->conditionType = phl->conditionType;
			pLane->width = phl->width;
			pLane->leftBoundary = nullptr;
			pLane->rightBoundary = nullptr;
			pLane->seqNumber = phl->seqNumber;
			// ������Ŵ�1��ʼ,-1Ϊ��ͨ��·��ս������,ʵ��Ϊ���󳵵�
			// �����ܴ���-2,��ʱ-2Ϊ���󳵵�,-1Ϊ�ڶ�����
			// https://jira.navinfo.com/browse/APPWL-46
			if (pLane->seqNumber <= -1) {
				pLane->seqNumber += 1;
			}

			// TEXT
			DbFixedSpeedLimit* fSpeedLimit = (DbFixedSpeedLimit*)pMesh->query(phl->uuid, RecordType::DB_HAD_LANE_FIXED_SPEEDLIMIT);
			if (fSpeedLimit != nullptr)
			{
				pLane->speedLimit.maxSpeedLimit = fSpeedLimit->maxSpeedLimit;
				pLane->speedLimit.minSpeedLimit = fSpeedLimit->minSpeedLimit;
				pLane->speedLimit.maxSpeedLimitSource = (FixedSpeedLimitSource)fSpeedLimit->maxSpeedLimitSource;
				pLane->speedLimit.minSpeedLimitSource = (FixedSpeedLimitSource)fSpeedLimit->minSpeedLimitSource;
			}

			// TurnWaiting

			DbLaneLinkTurnwaiting* flaneLinkTurnwaiting = (DbLaneLinkTurnwaiting*)pMesh->query(phl->uuid, RecordType::DB_HAD_LANE_LINK_TURNWAITING);
            if (flaneLinkTurnwaiting != nullptr)
            {
				HadLaneTurnwaiting* pLaneTurn = (HadLaneTurnwaiting*)pGrid->alloc(ElementType::HAD_LANE_TURNWAITING);
				pLaneTurn->startOffset = flaneLinkTurnwaiting->startOffset;
				pLaneTurn->endOffset = flaneLinkTurnwaiting->endOffset;
				pLaneTurn->positions = flaneLinkTurnwaiting->geometry;
				pLane->laneTurnwaiting = pLaneTurn;
            }

			HadLaneGroup* pGroup = (HadLaneGroup*)pGrid->query(phl->relLgId, ElementType::HAD_LANE_GROUP);
			if (pGroup != nullptr)
			{
				pGroup->width += pLane->width;
				pGroup->lanes.push_back(pLane);
				pLane->linkGroup = pGroup;
			}

			pGrid->insert(pLane->originId, pLane);
		}
    }

	void LaneProcessor::processRelation(DbMesh* const pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby)
	{
		std::vector<DbRecord*>& links = pMesh->query(RecordType::DB_HAD_LANE_LINK);
		for (auto hl : links)
		{
			DbLaneLink* phl = (DbLaneLink*)hl;
			HadLane* pLane = (HadLane*)pGrid->query(phl->uuid, ElementType::HAD_LANE);
			if (pLane == nullptr) {
				continue;
			}

			HadLaneGroup* pGroup = (HadLaneGroup*)queryNearby(pGrid, nearby, phl->relLgId, ElementType::HAD_LANE_GROUP);
			if (pGroup != nullptr)
			{
				pGroup->width += pLane->width;
				pGroup->lanes.push_back(pLane);
				pLane->linkGroup = pGroup;
			}

		}

		//�����߽��߼������鰴�ڳ������ڴ����ҵ�˳������
		for (auto obj : pGrid->query(ElementType::HAD_LANE_GROUP))
		{
			HadLaneGroup* pGroup = (HadLaneGroup*)obj;
			std::sort(pGroup->lanes.begin(), pGroup->lanes.end(),
				[](const HadLane* first, const HadLane* second)->bool {return first->seqNumber < second->seqNumber; });

		}

		//�����������С���١�
		for (auto obj : pGrid->query(ElementType::HAD_LANE_GROUP))
		{
			HadLaneGroup* pLG = (HadLaneGroup*)obj;

			if (pLG->lanes.size() == 0)
				continue;

			int maxSpeedLimit = INT_MIN;  // �ٶ����ޣ���λ��km/h
			int minSpeedLimit = INT_MAX;  // �ٶ����ޣ���λ��km/h
			for (auto pLane : pLG->lanes)
			{
				maxSpeedLimit = pLane->speedLimit.maxSpeedLimit > maxSpeedLimit ? pLane->speedLimit.maxSpeedLimit : maxSpeedLimit;
				minSpeedLimit = pLane->speedLimit.minSpeedLimit < minSpeedLimit ? pLane->speedLimit.minSpeedLimit : minSpeedLimit;
			}
			pLG->maxSpeedLimit = maxSpeedLimit;
			pLG->minSpeedLimit = minSpeedLimit;
		}
	}

}
