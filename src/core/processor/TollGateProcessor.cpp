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

			/// �����շ�վ���г��������������ݷǿ�ʱ�����շ�վ��Ӧ������Ч��
			for (auto tollLane : pDbTollGate->tollLanes)
			{
				// �շ�վ��������,�����öδ����ɾ��
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

			//��tollLane->seqNum����
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

			/// �����շ�վ���г��������������ݷǿ�ʱ�����շ�վ��Ӧ������Ч��
			for (auto tollLane : pDbTollGate->tollLanes)
			{
				// �շ�վ��������,�����öδ����ɾ��
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

			//��tollLane->seqNum����
			std::sort(pHadTollGate->tollLanes.begin(), pHadTollGate->tollLanes.end(),
				[](const HadTollGate::PtrTollLane& first, const HadTollGate::PtrTollLane& second)->bool {return first->seqNum < second->seqNum; });
		}
	}

}
