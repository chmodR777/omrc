#include "stdafx.h"
#include "TollGateProcessor.h"
#include "../loader/DbTollGateLoader.h"
#include <algorithm>

namespace OMDB
{
    void TollGateProcessor::process(DbMesh* const pMesh, HadGrid* pGrid)
    {
		std::vector<DbRecord*>& vctRecord = pMesh->query(RecordType::DB_HAD_TOLLGATE);
		for (auto element : vctRecord)
		{
			DbTollGate* pDbTollGate = (DbTollGate*)element;
			HadTollGate* pHadTollGate = (HadTollGate*)pGrid->alloc(ElementType::HAD_TOLLGATE);

			pHadTollGate->originId = pDbTollGate->uuid;
			pHadTollGate->tollName = pDbTollGate->tollName;

			//DbLinkPA* pDbLinkPa = (DbLinkPA*)pMesh->query(pDbTollGate->paPid, RecordType::DB_HAD_LINK_PA);
			//pHadTollGate->position = pDbLinkPa->geometry.postions[0];
			pHadTollGate->position = pDbTollGate->geometry;

			/// 遍历收费站所有车道。当车道数据非空时，此收费站对应车道有效。
			for (auto tollLane : pDbTollGate->tollLanes)
			{
				// 收费站补丁数据,后续该段代码可删除
				if (DbTollGateLoader::linkCardTypes.count(tollLane->laneLinkPid))
				{
					tollLane->cardType = DbTollGateLoader::linkCardTypes[tollLane->laneLinkPid];
				}
				if (pGrid->query(tollLane->laneLinkPid, ElementType::HAD_LANE) != nullptr)
				{
					HadTollGate::PtrTollLane hadTollLane = std::make_shared<HadTollGate::TollLane>();
					hadTollLane->laneLinkPid = tollLane->laneLinkPid;
					hadTollLane->seqNum = tollLane->seqNum;
					//hadTollLane.cardType = tollLane->cardType;
					hadTollLane->payType = HadTollGate::LanePayType(tollLane->cardType);

					if (hadTollLane->payType == HadTollGate::LanePayType::eUnkonwn)
					{
						if (tollLane->payMethod & 0x01)	//PAY_METHOD 0bit:ETC 
						{
							hadTollLane->payType = HadTollGate::LanePayType::eETC;
							if (tollLane->payMethod & ~0x01) 
							{
								hadTollLane->payType = (HadTollGate::LanePayType)((int32)HadTollGate::LanePayType::eETC|(int32)HadTollGate::LanePayType::eManual);
							}
						}
						else if (tollLane->payMethod != 0)
							hadTollLane->payType = HadTollGate::LanePayType::eManual;
					}

					pHadTollGate->tollLanes.push_back(hadTollLane);
				}
			}

			//按tollLane->seqNum排序
			std::sort(pHadTollGate->tollLanes.begin(), pHadTollGate->tollLanes.end(),
				[](const HadTollGate::PtrTollLane& first, const HadTollGate::PtrTollLane& second)->bool {return first->seqNum < second->seqNum; });

			pGrid->insert(pHadTollGate->originId, pHadTollGate);
		}
    }

	void TollGateProcessor::processRelation(DbMesh* const pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby)
	{
		std::vector<DbRecord*>& vctRecord = pMesh->query(RecordType::DB_HAD_TOLLGATE);
		for (auto element : vctRecord)
		{
			DbTollGate* pDbTollGate = (DbTollGate*)element;
			HadTollGate* pHadTollGate = (HadTollGate*)pGrid->query(pDbTollGate->uuid, ElementType::HAD_TOLLGATE);
			if (pHadTollGate == nullptr) {
				continue;
			}

			/// 遍历收费站所有车道。当车道数据非空时，此收费站对应车道有效。
			for (auto tollLane : pDbTollGate->tollLanes)
			{
				// 收费站补丁数据,后续该段代码可删除
				if (DbTollGateLoader::linkCardTypes.count(tollLane->laneLinkPid))
				{
					tollLane->cardType = DbTollGateLoader::linkCardTypes[tollLane->laneLinkPid];
				}
				if (queryNearby(pGrid, nearby, tollLane->laneLinkPid, ElementType::HAD_LANE) != nullptr)
				{
					HadTollGate::PtrTollLane hadTollLane = std::make_shared<HadTollGate::TollLane>();
					hadTollLane->laneLinkPid = tollLane->laneLinkPid;
					hadTollLane->seqNum = tollLane->seqNum;
					//hadTollLane.cardType = tollLane->cardType;
					hadTollLane->payType = HadTollGate::LanePayType(tollLane->cardType);

					if (hadTollLane->payType == HadTollGate::LanePayType::eUnkonwn)
					{
						if (tollLane->payMethod & 0x01)	//PAY_METHOD 0bit:ETC
							hadTollLane->payType = HadTollGate::LanePayType::eETC;
						else if (tollLane->payMethod != 0)
							hadTollLane->payType = HadTollGate::LanePayType::eManual;
					}

					pHadTollGate->tollLanes.push_back(hadTollLane);
				}
			}

			//按tollLane->seqNum排序
			std::sort(pHadTollGate->tollLanes.begin(), pHadTollGate->tollLanes.end(),
				[](const HadTollGate::PtrTollLane& first, const HadTollGate::PtrTollLane& second)->bool {return first->seqNum < second->seqNum; });
		}
	}

}
