#include "stdafx.h"

#include <algorithm>

#include "DbLinkLoader.h"
#include "DbRecordIterator.h"

namespace OMDB
{
    void DbLinkLoader::load()
    {
        recordIterator = DbRecordIterator(m_pSqlite3);
        loadLink();

        loadLinkPA();
        loadLinkPAValue();

		loadLinkDataLevel();
		loadLinkSeparation();
        loadLinkMedian();
        loadLinkOverheadObstruction();
        loadLinkTollArea();
		loadLinkForm();

		loadLinkName();
		loadRoadName();

        loadLinkLanePa();
        loadLgLink();
        loadLgAssociation();

        loadNode();
        loadNodeForm();
        loadNodeMesh();

        loadZLevel();
        loadLaneInfo();
        loadLaneInfoLink();

        loadRdLinkLanePa();
        loadRdLaneLinkCLM();
        loadRdLaneLinkCLMAcess();
        loadRdLaneLinkCLMCondition();
        loadRdLaneLinkCLMSpeedLimit();

		loadRdLaneLanePa();
		loadRdLaneTopoDetail();
		loadRdLaneTopoCond();
		loadRdLaneTopoVia();

        loadLinkSpeedLimit();
    }
    void DbLinkLoader::loadLink()
    {
        // TODO SEPARATION_LEFT,SEPARATION_RIGHT,MEDIAN_LEFT,MEDIAN_RIGHT,OVERHEAD_OBSTRUCTION
        std::string sql = "SELECT LINK_PID,S_NODE_PID,E_NODE_PID,KIND,MESH,MULTI_DIGITIZED,DIRECT,GEOMETRY FROM HAD_LINK";
        recordIterator.resetWithSqlStr(sql);
        while (recordIterator.hasNextRecord())
        {
            sqlite3_stmt* stmt = recordIterator.record();
            DbLink* pLink = (DbLink*)m_pDatabase->alloc(RecordType::DB_HAD_LINK);
            pLink->uuid = sqlite3_column_int64(stmt, 0);
            pLink->startNode = sqlite3_column_int64(stmt, 1);
            pLink->endNode = sqlite3_column_int64(stmt, 2);
            pLink->kind = sqlite3_column_int(stmt, 3);

			pLink->multi_digitized = sqlite3_column_int(stmt, 5);
			pLink->direct = sqlite3_column_int(stmt, 6);
			//pLink->separation_left = sqlite3_column_int(stmt, 7);
            //pLink->separation_right = sqlite3_column_int(stmt, 8);
            //pLink->median_left = sqlite3_column_int(stmt, 9);
            //pLink->median_right = sqlite3_column_int(stmt, 10);
            //pLink->overhead_obstruction = sqlite3_column_int(stmt, 11);

            const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, 7);
            int size = sqlite3_column_bytes(stmt, 7);
            praseLineString3d(geometry,size, pLink->geometry);
            m_pDatabase->insert(pLink->uuid, pLink);
        }
    }

//     void DbLinkLoader::loadNoneMesh()
//     {
//         std::string sql = "SELECT NODE_PID,MESH FROM HAD_NODE_MESH";
//         recordIterator.resetWithSqlStr(sql);
//         while (recordIterator.hasNextRecord())
//         {
//             sqlite3_stmt* stmt = recordIterator.record();
//             int64 id = sqlite3_column_int64(stmt, 0);
//             DbNode* pNode = (DbNode*)m_pDatabase->query(id, RecordType::DB_HAD_NODE);
//             if (pNode != nullptr)
//             {
//                 pNode->meshId = sqlite3_column_int(stmt, 1);
//             }
//         }
//     }

    void DbLinkLoader::loadLinkPA()
    {
        std::string sql = "SELECT PA_PID,FEATURE_PID,PA_TYPE,S_OFFSET,E_OFFSET,GEOMETRY FROM HAD_LINK_PA";
        recordIterator.resetWithSqlStr(sql);
        while (recordIterator.hasNextRecord())
        {
            sqlite3_stmt* stmt = recordIterator.record();

            int i = 0;
            DbLinkPA* pLinkPA = (DbLinkPA*)m_pDatabase->alloc(RecordType::DB_HAD_LINK_PA);
            pLinkPA->uuid = sqlite3_column_int64(stmt, i++);
            pLinkPA->relLinkId = sqlite3_column_int64(stmt, i++);
            pLinkPA->featureType = sqlite3_column_int(stmt, i++);
            pLinkPA->startOffset = sqlite3_column_double(stmt, i++);
            pLinkPA->endOffset = sqlite3_column_double(stmt, i++);
            const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, i);
            int size = sqlite3_column_bytes(stmt, i);
            praseMultiPoint3d(geometry,size, pLinkPA->geometry);
            m_pDatabase->insert(pLinkPA->uuid, pLinkPA);
        }
    }

    void DbLinkLoader::loadLinkPAValue()
    {
        std::string sql = "SELECT PA_PID,ATTRIBUTE_TYPE,ATTRIBUTE_VALUE FROM HAD_LINK_PA_VALUE";
        recordIterator.resetWithSqlStr(sql);
        while (recordIterator.hasNextRecord())
        {
            sqlite3_stmt* stmt = recordIterator.record();
            int64 id = sqlite3_column_int64(stmt, 0);

            DbLinkPA* pLinkPA = (DbLinkPA*)m_pDatabase->query(id, RecordType::DB_HAD_LINK_PA);
            if (pLinkPA != nullptr)
            {
                DbPAValue* pPAValue = (DbPAValue*)m_pDatabase->alloc(RecordType::DB_HAD_PA_VALUE);
                pPAValue->attributeType = sqlite3_column_int(stmt, 1);
                pPAValue->attributeValue = sqlite3_column_int(stmt, 2);
                pLinkPA->paValues.push_back(pPAValue);
            }
        }
    }

	void DbLinkLoader::loadLinkDataLevel()
	{
		std::string sql = "SELECT LINK_PID,FEATURE_TYPE,S_OFFSET,E_OFFSET,DATA_LEVEL,GEOMETRY FROM HAD_LINK_DATALEVEL";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();

			int i = 0;
			DbLinkDataLevel dataLevel;
			dataLevel.uuid = sqlite3_column_int64(stmt, i++);
			dataLevel.featureType = sqlite3_column_int64(stmt, i++);
			dataLevel.startOffset = sqlite3_column_double(stmt, i++);
			dataLevel.endOffset = sqlite3_column_double(stmt, i++);
			dataLevel.dataLevel = sqlite3_column_int(stmt, i++);
			const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, i);
			int size = sqlite3_column_bytes(stmt, i);
			praseMultiPoint3d(geometry, size, dataLevel.geometry);
			DbLink* pLink = (DbLink*)m_pDatabase->query(dataLevel.uuid, RecordType::DB_HAD_LINK);
			if (pLink != nullptr)
			{
				pLink->dataLevels.push_back(dataLevel);
			}
		}
	}

	void DbLinkLoader::loadLinkSeparation()
	{
		std::string sql = "SELECT LINK_PID,FEATURE_TYPE,S_OFFSET,E_OFFSET,SIDE,SEPARATION,GEOMETRY FROM HAD_LINK_SEPARATION";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();

			int i = 0;
            DbLinkSeparation separation;
			separation.uuid = sqlite3_column_int64(stmt, i++);
			separation.featureType = sqlite3_column_int64(stmt, i++);
			separation.startOffset = sqlite3_column_double(stmt, i++);
			separation.endOffset = sqlite3_column_double(stmt, i++);
            separation.side = sqlite3_column_int(stmt, i++);
			separation.separation = sqlite3_column_int(stmt, i++);
			const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, i);
			int size = sqlite3_column_bytes(stmt, i);
			praseMultiPoint3d(geometry, size, separation.geometry);
			DbLink* pLink = (DbLink*)m_pDatabase->query(separation.uuid, RecordType::DB_HAD_LINK);
			if (pLink != nullptr)
			{
                pLink->separations.push_back(separation);
			}
		}
	}

	void DbLinkLoader::loadLinkMedian()
	{
		std::string sql = "SELECT LINK_PID,FEATURE_TYPE,S_OFFSET,E_OFFSET,SIDE,MEDIAN,GEOMETRY FROM HAD_LINK_MEDIAN";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();

			int i = 0;
            DbLinkMedian median;
			median.uuid = sqlite3_column_int64(stmt, i++);
			median.featureType = sqlite3_column_int64(stmt, i++);
			median.startOffset = sqlite3_column_double(stmt, i++);
			median.endOffset = sqlite3_column_double(stmt, i++);
			median.side = sqlite3_column_int(stmt, i++);
			median.median = sqlite3_column_int(stmt, i++);
			const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, i);
			int size = sqlite3_column_bytes(stmt, i);
			praseMultiPoint3d(geometry, size, median.geometry);
			DbLink* pLink = (DbLink*)m_pDatabase->query(median.uuid, RecordType::DB_HAD_LINK);
			if (pLink != nullptr)
			{
				pLink->medians.push_back(median);
			}
		}
	}

    void DbLinkLoader::loadLinkOverheadObstruction()
    {
		std::string sql = "SELECT LINK_PID,FEATURE_TYPE,S_OFFSET,E_OFFSET,GEOMETRY FROM HAD_LINK_OVER_OBST";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();

			int i = 0;
            DbLinkOverheadObstruction overheadObstruction;
			overheadObstruction.uuid = sqlite3_column_int64(stmt, i++);
			overheadObstruction.featureType = sqlite3_column_int64(stmt, i++);
			overheadObstruction.startOffset = sqlite3_column_double(stmt, i++);
			overheadObstruction.endOffset = sqlite3_column_double(stmt, i++);
			const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, i);
			int size = sqlite3_column_bytes(stmt, i);
			praseMultiPoint3d(geometry, size, overheadObstruction.geometry);
			DbLink* pLink = (DbLink*)m_pDatabase->query(overheadObstruction.uuid, RecordType::DB_HAD_LINK);
			if (pLink != nullptr)
			{
				pLink->overheadObstructions.push_back(overheadObstruction);
			}
		}
    }

    void DbLinkLoader::loadLinkTollArea()
    {
		std::string sql = "SELECT LINK_PID,FEATURE_TYPE,S_OFFSET,E_OFFSET,GEOMETRY FROM HAD_LINK_TOLLAREA";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();

			int i = 0;
            DbLinkTollArea tollArea;
			tollArea.uuid = sqlite3_column_int64(stmt, i++);
			tollArea.featureType = sqlite3_column_int64(stmt, i++);
			tollArea.startOffset = sqlite3_column_double(stmt, i++);
			tollArea.endOffset = sqlite3_column_double(stmt, i++);
			const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, i);
			int size = sqlite3_column_bytes(stmt, i);
			praseMultiPoint3d(geometry, size, tollArea.geometry);
			DbLink* pLink = (DbLink*)m_pDatabase->query(tollArea.uuid, RecordType::DB_HAD_LINK);
			if (pLink != nullptr)
			{
				pLink->tollAreas.push_back(tollArea);
			}
		}
    }

	void DbLinkLoader::loadLinkForm()
	{
		std::string sql = "SELECT LINK_PID,FORM_OF_WAY FROM HAD_LINK_FORM";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			int64 id = sqlite3_column_int64(stmt, 0);

			DbLink* pLink = (DbLink*)m_pDatabase->query(id, RecordType::DB_HAD_LINK);
			if (pLink != nullptr)
			{
				pLink->wayTypes.push_back(sqlite3_column_int(stmt, 1));
			}
		}
	}

	void DbLinkLoader::loadLinkName()
	{
		std::string sql = "SELECT LINK_PID,FEATURE_TYPE,S_OFFSET,E_OFFSET,NAME_GROUPID,SEQ_NUM,NAME_CLASS,NAME_TYPE,GEOMETRY FROM HAD_LINK_NAME";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();

			int i = 0;
			int64 id = sqlite3_column_int64(stmt, i++);
			DbLinkName* pLinkName = (DbLinkName*)m_pDatabase->query(id, RecordType::DB_HAD_LINK_NAME);
			if (pLinkName == nullptr)
			{
				pLinkName = (DbLinkName*)m_pDatabase->alloc(RecordType::DB_HAD_LINK_NAME);
				pLinkName->uuid = id;
				m_pDatabase->insert(pLinkName->uuid, pLinkName);
			}

			DbLinkName::DbLinkNamePa linkNamePa;
			linkNamePa.featureType = sqlite3_column_int(stmt, i++);
			linkNamePa.startOffset = sqlite3_column_double(stmt, i++);
			linkNamePa.endOffset = sqlite3_column_double(stmt, i++);
			linkNamePa.nameGroup = sqlite3_column_int(stmt, i++);
			linkNamePa.seqNum = sqlite3_column_int(stmt, i++);
			linkNamePa.nameClass = sqlite3_column_int(stmt, i++);
			linkNamePa.nameType = sqlite3_column_int(stmt, i++);
			const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, i);
			int size = sqlite3_column_bytes(stmt, i);
			praseMultiPoint3d(geometry, size, linkNamePa.geometry);
			pLinkName->linkNamePas.push_back(linkNamePa);
		}
	}

	void DbLinkLoader::loadRoadName()
	{
		std::string sql = "SELECT NAME_GROUPID,LANG_CODE,NAME,TYPE,BASE,PREFIX,SUFFIX,CODE_TYPE FROM ROAD_NAME";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();

			int64 id = sqlite3_column_int64(stmt, 0);
			DbRoadName* pRoadName = (DbRoadName*)m_pDatabase->query(id, RecordType::DB_ROAD_NAME);
			if (pRoadName == nullptr)
			{
				pRoadName = (DbRoadName*)m_pDatabase->alloc(RecordType::DB_ROAD_NAME);
				pRoadName->uuid = id;
				m_pDatabase->insert(pRoadName->uuid, pRoadName);
			}

			DbRoadName::DbRoadNameGroup roadNameGroup;
			roadNameGroup.langCode = std::wstring((const wchar_t*)sqlite3_column_text16(stmt, 1));
			roadNameGroup.name = std::wstring((const wchar_t*)sqlite3_column_text16(stmt, 2));
			roadNameGroup.type = std::wstring((const wchar_t*)sqlite3_column_text16(stmt, 3));
			roadNameGroup.base = std::wstring((const wchar_t*)sqlite3_column_text16(stmt, 4));
			roadNameGroup.prefix = std::wstring((const wchar_t*)sqlite3_column_text16(stmt, 5));
			roadNameGroup.suffix = std::wstring((const wchar_t*)sqlite3_column_text16(stmt, 6));
			roadNameGroup.codeType = sqlite3_column_int(stmt, 7);
			pRoadName->nameGroups.push_back(roadNameGroup);
		}
	}

	void DbLinkLoader::loadZLevel()
	{
		std::string sql = "SELECT ZLEVEL_ID,FEATURE_PID,SHP_SEQ_NUM,START_END,ZLEVEL,GEOMETRY FROM HAD_ZLEVEL WHERE FEATURE_TYPE=1";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			int64 id = sqlite3_column_int64(stmt, 0);
			DbZLevel* pZLevel = (DbZLevel*)m_pDatabase->query(id, RecordType::DB_HAD_ZLEVEL);
			if (pZLevel == nullptr)
			{
				pZLevel = (DbZLevel*)m_pDatabase->alloc(RecordType::DB_HAD_ZLEVEL);
				pZLevel->uuid = id;
				m_pDatabase->insert(pZLevel->uuid, pZLevel);
			}

			DbZLevel::DbRelLink relLink;
			relLink.relLinkid = sqlite3_column_int64(stmt, 1);
			relLink.shpSeqNum = sqlite3_column_int(stmt, 2);
			relLink.startEnd = sqlite3_column_int(stmt, 3);
			relLink.zLevel = sqlite3_column_int(stmt, 4);

			const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, 5);
			int size = sqlite3_column_bytes(stmt, 5);

			prasePoint3d(geometry, size, relLink.geometry);
			pZLevel->relLinks.push_back(relLink);
		}

		// 相同Z Level ID的记录按照z level从小到大排序
		for (auto& zl : m_pDatabase->query(RecordType::DB_HAD_ZLEVEL)) {
			DbZLevel* pZLevel = (DbZLevel*)zl;
			std::sort(pZLevel->relLinks.begin(), pZLevel->relLinks.end(),
				[](const auto& first, const auto& second)->bool {
					return first.zLevel < second.zLevel;
				});
		}
	}

	void DbLinkLoader::loadLaneInfo()
    {
		std::string sql = "SELECT LANEINFO_ID,IN_LANE_INFO,REACH_DIR,EXTEND_LANE_INFO,BUS_LANE_INFO FROM HAD_LANEINFO";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
            DbLaneInfo* pLaneInfo = (DbLaneInfo*)m_pDatabase->alloc(RecordType::DB_HAD_LANEINFO);
            pLaneInfo->uuid = sqlite3_column_int64(stmt, 0);
            pLaneInfo->inLaneInfo = std::wstring((const wchar_t*)sqlite3_column_text16(stmt, 1));
            pLaneInfo->reachDir = sqlite3_column_int(stmt, 2);
            pLaneInfo->extendLaneInfo = std::wstring((const wchar_t*)sqlite3_column_text16(stmt, 3));
            pLaneInfo->busLaneInfo = std::wstring((const wchar_t*)sqlite3_column_text16(stmt, 4));

            m_pDatabase->insert(pLaneInfo->uuid, pLaneInfo);
		}
    }

    void DbLinkLoader::loadLaneInfoLink()
    {
		std::string sql = "SELECT LANEINFO_ID,SEQ_NUM,LINK_PID,DIRECT FROM HAD_LANEINFO_LINK";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			int64 laneInfoId = sqlite3_column_int64(stmt, 0);
            int seqNum = sqlite3_column_int(stmt, 1);
            int64 linkId = sqlite3_column_int64(stmt, 2);
            int direct = sqlite3_column_int(stmt, 3);

            DbLink::DbRelLaneInfo relLaneInfo;
            relLaneInfo.seqNum = seqNum;
            relLaneInfo.direct = direct;
			DbLink* pLink = (DbLink*)m_pDatabase->query(linkId, RecordType::DB_HAD_LINK);
			if (pLink != nullptr)
			{
                pLink->relLaneInfos.emplace(laneInfoId, relLaneInfo);
			}
		}
    }

    void DbLinkLoader::loadRdLinkLanePa()
    {
		std::string sql = "SELECT LINK_LGPA_PID,LINK_PID,S_OFFSET,E_OFFSET,DIRECT FROM RD_LINK_LANEPA";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();

			int64 id = sqlite3_column_int64(stmt, 0);
			DbRdLinkLanePa* pLinkLanePa = (DbRdLinkLanePa*)m_pDatabase->query(id, RecordType::DB_RD_LINK_LANEPA);
			if (pLinkLanePa == nullptr)
			{
				pLinkLanePa = (DbRdLinkLanePa*)m_pDatabase->alloc(RecordType::DB_RD_LINK_LANEPA);
				pLinkLanePa->uuid = id;
				m_pDatabase->insert(pLinkLanePa->uuid, pLinkLanePa);
			}

            DbRdLinkLanePa::DbRelLink relLink;
			relLink.relLinkid = sqlite3_column_int64(stmt, 1);
			relLink.startOffset = sqlite3_column_double(stmt, 2);
			relLink.endOffset = sqlite3_column_double(stmt, 3);
			relLink.directType = sqlite3_column_int(stmt, 4);
			pLinkLanePa->relLinks.emplace(id, relLink);
		}
    }

	void DbLinkLoader::loadRdLaneLinkCLM()
	{
		std::string sql = "SELECT LANE_LINK_PID,LANE_TYPE,TRANTYPE,ARROW_DIR,RES_WIDTH,RES_HIGH FROM RD_LANE_LINK_CLM";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			DbRdLaneLinkCLM* pLaneLinkCLM = (DbRdLaneLinkCLM*)m_pDatabase->alloc(RecordType::DB_RD_LANE_LINK_CLM);
			sqlite3_stmt* stmt = recordIterator.record();
			pLaneLinkCLM->uuid = sqlite3_column_int64(stmt, 0);
			pLaneLinkCLM->laneType = sqlite3_column_int(stmt, 1);
			pLaneLinkCLM->tranType = sqlite3_column_int(stmt, 2);
			pLaneLinkCLM->arrowDir = std::wstring((const wchar_t*)sqlite3_column_text16(stmt, 3));
			m_pDatabase->insert(pLaneLinkCLM->uuid, pLaneLinkCLM);
		}
	}

    void DbLinkLoader::loadRdLaneLinkCLMAcess()
    {
		std::string sql = "SELECT LANE_LINK_PID,ACCESS_CHARACTERISTIC,VALID_PERIOD FROM RD_LANE_LINK_ACCESS_CLM";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			int64 id = sqlite3_column_int64(stmt, 0);
            DbRdLaneLinkCLM* pLaneLinkCLM = (DbRdLaneLinkCLM*)m_pDatabase->query(id, RecordType::DB_RD_LANE_LINK_CLM);
			if (pLaneLinkCLM != nullptr)
			{
				DbRdLaneLinkCLM::DbRdLaneLinkAccessCLM access;
				access.characteristic = sqlite3_column_int(stmt, 1);
				access.validPeriod = std::wstring((const wchar_t*)sqlite3_column_text16(stmt, 2));
				pLaneLinkCLM->accesses.push_back(access);
			}
		}
    }

    void DbLinkLoader::loadRdLaneLinkCLMCondition()
    {
		std::string sql = "SELECT LANE_LINK_PID,TYPE,DIRECT,VALID_PERIOD FROM RD_LANE_LINK_CONDITION_CLM";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			int64 id = sqlite3_column_int64(stmt, 0);
			DbRdLaneLinkCLM* pLaneLinkCLM = (DbRdLaneLinkCLM*)m_pDatabase->query(id, RecordType::DB_RD_LANE_LINK_CLM);
			if (pLaneLinkCLM != nullptr)
			{
				DbRdLaneLinkCLM::DbRdLaneLinkConditionCLM condition;
				condition.type = sqlite3_column_int(stmt, 1);
				condition.direct = sqlite3_column_int(stmt, 2);
				condition.validPeriod = std::wstring((const wchar_t*)sqlite3_column_text16(stmt, 3));
				pLaneLinkCLM->conditions.push_back(condition);
			}
		}
    }

    void DbLinkLoader::loadRdLaneLinkCLMSpeedLimit()
    {
		std::string sql = "SELECT LANE_LINK_PID,MAX_SPEED,MIN_SPEED FROM RD_LANE_LINK_SPEEDLIMIT_CLM";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			int64 id = sqlite3_column_int64(stmt, 0);
			DbRdLaneLinkCLM* pLaneLinkCLM = (DbRdLaneLinkCLM*)m_pDatabase->query(id, RecordType::DB_RD_LANE_LINK_CLM);
			if (pLaneLinkCLM != nullptr)
			{
				DbRdLaneLinkCLM::DbRdLaneLinkSpeedLimitCLM speedLimit;
				speedLimit.maxSpeed = sqlite3_column_int(stmt, 1);
				speedLimit.minSpeed = sqlite3_column_int(stmt, 2);
				pLaneLinkCLM->speedLimits.push_back(speedLimit);
			}
		}
    }

	void DbLinkLoader::loadRdLaneLanePa()
	{
		std::string sql = "SELECT LANE_LINK_PID,LINK_LGPA_PID,LANE_SEQ_NUM FROM RD_LANE_LANEPA";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			int64 laneLinkPid = sqlite3_column_int64(stmt, 0);
			int64 linkLgpaPid = sqlite3_column_int64(stmt, 1);
			int seqNum = sqlite3_column_int(stmt, 2);

			DbRdLinkLanePa::DbRelRdLinkLane relLinkLane;
			relLinkLane.relLinkLaneId = laneLinkPid;
			relLinkLane.seqNum = seqNum;
			DbRdLinkLanePa* pLinkLanePa = (DbRdLinkLanePa*)m_pDatabase->query(linkLgpaPid, RecordType::DB_RD_LINK_LANEPA);
			if (pLinkLanePa != nullptr)
			{
				pLinkLanePa->relLinkLanes.push_back(relLinkLane);
			}
		}
	}

	void DbLinkLoader::loadRdLaneTopoDetail()
	{
		std::string sql = "SELECT TOPO_PID,IN_LANE_LINK_PID,OUT_LANE_LINK_PID,THROUGH_TURN,LANE_CHANGE FROM RD_LANE_TOPO_DETAIL";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			DbRdLaneTopoDetail* pLaneTopoDetail = (DbRdLaneTopoDetail*)m_pDatabase->alloc(RecordType::DB_RD_LANE_TOPO_DETAIL);
			sqlite3_stmt* stmt = recordIterator.record();
			pLaneTopoDetail->uuid = sqlite3_column_int64(stmt, 0);
			pLaneTopoDetail->inLaneLinkPid = sqlite3_column_int64(stmt, 1);
			pLaneTopoDetail->outLaneLinkPid = sqlite3_column_int64(stmt, 2);
			pLaneTopoDetail->throughTurn = sqlite3_column_int(stmt, 3);
			pLaneTopoDetail->laneChange = sqlite3_column_int(stmt, 4);
			m_pDatabase->insert(pLaneTopoDetail->uuid, pLaneTopoDetail);
		}
	}

	void DbLinkLoader::loadRdLaneTopoCond()
	{
		std::string sql = "SELECT TOPO_PID,VEHICLE_TYPE,VALID_PERIOD FROM RD_LANE_TOPO_COND";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			int64 id = sqlite3_column_int64(stmt, 0);
			DbRdLaneTopoDetail* pLaneTopoDetail = (DbRdLaneTopoDetail*)m_pDatabase->query(id, RecordType::DB_RD_LANE_TOPO_DETAIL);
			if (pLaneTopoDetail != nullptr)
			{
				DbRdLaneTopoDetail::DbRdLaneTopoCond cond;
				cond.vehicleType = sqlite3_column_int(stmt, 1);
				cond.validPeriod = std::wstring((const wchar_t*)sqlite3_column_text16(stmt, 2));
				pLaneTopoDetail->conditions.push_back(cond);
			}
		}
	}

	void DbLinkLoader::loadRdLaneTopoVia()
	{
		std::string sql = "SELECT TOPO_PID,LANE_LINK_PID,SEQ_NUM FROM RD_LANE_TOPO_VIA";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			int64 id = sqlite3_column_int64(stmt, 0);
			DbRdLaneTopoDetail* pLaneTopoDetail = (DbRdLaneTopoDetail*)m_pDatabase->query(id, RecordType::DB_RD_LANE_TOPO_DETAIL);
			if (pLaneTopoDetail != nullptr)
			{
				DbRdLaneTopoDetail::DbRdLaneTopoVia via;
				via.relLinkLaneId = sqlite3_column_int64(stmt, 1);
				via.seqNum = sqlite3_column_int(stmt, 2);
				pLaneTopoDetail->viaLinkLanes.push_back(via);
			}
		}
	}

    void DbLinkLoader::loadLinkSpeedLimit()
    {
        std::string sql = "SELECT SPEED_ID, LINK_PID, FEATURE_TYPE, S_OFFSET, E_OFFSET, GEOMETRY, DIRECTION, MAX_SPEED, MIN_SPEED, MAX_SPEED_SOURCE, MIN_SPEED_SOURCE, IS_LANE_DEPENDENT\
            FROM HAD_LINK_SPEEDLIMIT";
        recordIterator.resetWithSqlStr(sql);
        while (recordIterator.hasNextRecord())
        {
            sqlite3_stmt* stmt = recordIterator.record();

            DbLinkSpeedLimit* pDbFSL = (DbLinkSpeedLimit*)m_pDatabase->alloc(RecordType::DB_HAD_LINK_FIXED_SPEEDLIMIT);
            pDbFSL->uuid = sqlite3_column_int64(stmt, 0);
            pDbFSL->relLinkId = sqlite3_column_int64(stmt, 1);
            pDbFSL->featureType = sqlite3_column_int(stmt, 2);
            if (pDbFSL->featureType == 2)
            {
                pDbFSL->startOffset = sqlite3_column_double(stmt, 3);
                pDbFSL->endOffset = sqlite3_column_double(stmt, 4);
				const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, 5);
				int size = sqlite3_column_bytes(stmt, 5);
				praseMultiPoint3d(geometry, size, pDbFSL->geometry);
            }
            pDbFSL->direction = sqlite3_column_int(stmt, 6);
            pDbFSL->maxSpeedLimit = sqlite3_column_int(stmt, 7);
            pDbFSL->minSpeedLimit = sqlite3_column_int(stmt, 8);
            pDbFSL->maxSpeedLimitSource = (DbSpeedLimitSource)sqlite3_column_int(stmt, 9);
            pDbFSL->minSpeedLimitSource = (DbSpeedLimitSource)sqlite3_column_int(stmt, 10);
            // pDbFSL->maxSpeedLimitClass = (MaxSpeedLimitClass)sqlite3_column_int(stmt, 7);
            pDbFSL->isLaneDependent = sqlite3_column_int(stmt, 11);

            m_pDatabase->insert(pDbFSL->uuid, pDbFSL);
        }
    }

    void DbLinkLoader::loadNodeForm()
    {
        std::string sql = "SELECT NODE_PID,FORM_OF_WAY FROM HAD_NODE_FORM";
        recordIterator.resetWithSqlStr(sql);
        while (recordIterator.hasNextRecord())
        {
            sqlite3_stmt* stmt = recordIterator.record();
            int64 id = sqlite3_column_int64(stmt, 0);
            DbNode* pNode = (DbNode*)m_pDatabase->query(id, RecordType::DB_HAD_NODE);
            if (pNode != nullptr)
            {
                pNode->wayType = sqlite3_column_int(stmt, 1);
            }
        }
    }

	void DbLinkLoader::loadNodeMesh()
	{
		std::string sql = "SELECT NODE_PID,MESH FROM HAD_NODE_MESH";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			int64 id = sqlite3_column_int64(stmt, 0);
			DbNode* pNode = (DbNode*)m_pDatabase->query(id, RecordType::DB_HAD_NODE);
			if (pNode != nullptr)
			{
				pNode->meshIds.push_back(sqlite3_column_int(stmt, 1));
			}
		}
	}

	void DbLinkLoader::loadNode()
    {
        std::string sql = "SELECT NODE_PID,INTERSECTION_PID,GEOMETRY FROM HAD_NODE";
        recordIterator.resetWithSqlStr(sql);
        while (recordIterator.hasNextRecord())
        {
            sqlite3_stmt* stmt = recordIterator.record();
            DbNode* pNode = (DbNode*)m_pDatabase->alloc(RecordType::DB_HAD_NODE);
            pNode->uuid = sqlite3_column_int64(stmt, 0);
            pNode->intersectionId = sqlite3_column_int64(stmt, 1);

            const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, 2);
            int size = sqlite3_column_bytes(stmt, 2);

            prasePoint3d(geometry, size, pNode->geometry);
            m_pDatabase->insert(pNode->uuid, pNode);
        }
    }

    void DbLinkLoader::loadLinkLanePa()
    {
		std::string sql = "SELECT LINK_LGPA_PID,LINK_PID,S_OFFSET,E_OFFSET,DIRECT FROM HAD_LINK_LANEPA";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();

			int64 id = sqlite3_column_int64(stmt, 0);
			DbHadLinkLanePa* pLinkLanePa = (DbHadLinkLanePa*)m_pDatabase->query(id, RecordType::DB_HAD_LINK_LANEPA);
			if (pLinkLanePa == nullptr)
			{
				pLinkLanePa = (DbHadLinkLanePa*)m_pDatabase->alloc(RecordType::DB_HAD_LINK_LANEPA);
				pLinkLanePa->uuid = id;
				m_pDatabase->insert(pLinkLanePa->uuid, pLinkLanePa);
			}

            DbHadLinkLanePa::DbRelLink relLink;
			relLink.relLinkid = sqlite3_column_int64(stmt, 1);
			relLink.startOffset = sqlite3_column_double(stmt, 2);
			relLink.endOffset = sqlite3_column_double(stmt, 3);
			relLink.directType = sqlite3_column_int(stmt, 4);
			pLinkLanePa->relLinks.emplace(id, relLink);
		}
    }

    void DbLinkLoader::loadLgLink()
    {
        std::string sql = "SELECT LG_ID,LINK_LGPA_PID,TYPE FROM HAD_LG_LINK";
        recordIterator.resetWithSqlStr(sql);
        while (recordIterator.hasNextRecord())
        {
            sqlite3_stmt* stmt = recordIterator.record();

            int64 id = sqlite3_column_int64(stmt, 0);
            int64 lgPaPid = sqlite3_column_int64(stmt, 1);
            DbLgLink* pLgLink = (DbLgLink*)m_pDatabase->query(id, RecordType::DB_HAD_LG_LINK);
            if (pLgLink == nullptr)
            {
				pLgLink = (DbLgLink*)m_pDatabase->alloc(RecordType::DB_HAD_LG_LINK);
                pLgLink->uuid = id;
                m_pDatabase->insert(pLgLink->uuid, pLgLink);
            }

            DbHadLinkLanePa* pLinkLanePa = (DbHadLinkLanePa*)m_pDatabase->query(lgPaPid, RecordType::DB_HAD_LINK_LANEPA);
            if (pLinkLanePa != nullptr)
            {
                DbHadLinkLanePa::DbRelLink relLink = pLinkLanePa->relLinks[lgPaPid];
				relLink.type = sqlite3_column_int(stmt, 2);
				pLgLink->relLinks.emplace(relLink.relLinkid, relLink);
            }
        }
    }

    void DbLinkLoader::loadLgAssociation()
    {
        std::string sql = "SELECT GROUP_ID,DIRECT,LG_ID,LG_ID_PAIR FROM HAD_LG_ASSOCIATION";
        recordIterator.resetWithSqlStr(sql);
        while (recordIterator.hasNextRecord())
        {
            sqlite3_stmt* stmt = recordIterator.record();

            int64 id = sqlite3_column_int64(stmt, 0);
            DbLgAssociation* pLgAssociation = (DbLgAssociation*)m_pDatabase->query(id, RecordType::DB_HAD_LG_ASSOCIATION);
            if (pLgAssociation == nullptr) {
                pLgAssociation = (DbLgAssociation*)m_pDatabase->alloc(RecordType::DB_HAD_LG_ASSOCIATION);
                pLgAssociation->uuid = id;
                m_pDatabase->insert(pLgAssociation->uuid, pLgAssociation);
            }
            
            DbLgAssociation::DbRelLgAssociation relLgAssociation;
            relLgAssociation.directType = sqlite3_column_int(stmt, 1);
            relLgAssociation.firstLgLinkId = sqlite3_column_int64(stmt, 2);
            relLgAssociation.secondLgLinkId = sqlite3_column_int64(stmt, 3);
            pLgAssociation->relLgAssociations.push_back(relLgAssociation);
        }
    }



}
