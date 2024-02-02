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
			// 车道序号从1开始,-1为普通道路左拐借道车道,实际为最左车道
			// 还可能存在-2,此时-2为最左车道,-1为第二车道
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

		//车道边界线几何数组按在车道组内从左到右的顺序排序。
		for (auto obj : pGrid->query(ElementType::HAD_LANE_GROUP))
		{
			HadLaneGroup* pGroup = (HadLaneGroup*)obj;
			std::sort(pGroup->lanes.begin(), pGroup->lanes.end(),
				[](const HadLane* first, const HadLane* second)->bool {return first->seqNumber < second->seqNumber; });

		}

		//车道组最大最小限速。
		for (auto obj : pGrid->query(ElementType::HAD_LANE_GROUP))
		{
			HadLaneGroup* pLG = (HadLaneGroup*)obj;

			if (pLG->lanes.size() == 0)
				continue;

			int maxSpeedLimit = INT_MIN;  // 速度上限，单位：km/h
			int minSpeedLimit = INT_MAX;  // 速度下限，单位：km/h
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
