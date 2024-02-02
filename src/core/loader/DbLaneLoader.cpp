#include "stdafx.h"

#include "DbLaneLoader.h"
#include "DbRecordIterator.h"
#include "spatialite.h"
namespace OMDB
{
	void DbLaneLoader::load()
	{
		recordIterator = DbRecordIterator(m_pSqlite3);
		loadLaneLink();
		loadLaneLinkGeo();
		loadLaneLinkLg();
		loadLaneLinkCondition();
		loadLaneNode();
		loadLaneSpeedLimit();
		loadLaneLinkTurnwaiting();
	}

	void DbLaneLoader::loadLaneLink()
	{
		std::string sql = "SELECT LANE_LINK_PID,LANE_TYPE,IS_INTERSECTION,WIDTH FROM HAD_LANE_LINK";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			DbLaneLink* pLaneLink = (DbLaneLink*)m_pDatabase->alloc(RecordType::DB_HAD_LANE_LINK);
			pLaneLink->uuid = sqlite3_column_int64(stmt, 0);
			pLaneLink->laneType = sqlite3_column_int(stmt, 1);
			pLaneLink->isIntersection = sqlite3_column_int(stmt, 2);
			// pLaneLink->arrowDir = sqlite3_column_int(stmt, 3);
			pLaneLink->width = sqlite3_column_int(stmt, 3);
			m_pDatabase->insert(pLaneLink->uuid, pLaneLink);
		}
	}
	void DbLaneLoader::loadLaneLinkGeo()
	{
		std::string sql = "SELECT LANE_LINK_PID,S_LANE_NODE_PID,E_LANE_NODE_PID,GEOMETRY FROM HAD_LANE_LINK_GEO";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			int i = 0;
			int64 id = sqlite3_column_int64(stmt, i++);
			DbLaneLink* pLaneLink = (DbLaneLink*)m_pDatabase->query(id, RecordType::DB_HAD_LANE_LINK);
			if (pLaneLink != nullptr)
			{
				pLaneLink->startLaneNodeId = sqlite3_column_int64(stmt, i++);
				pLaneLink->endLaneNodeId = sqlite3_column_int64(stmt, i++);
				const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, i);
				int size = sqlite3_column_bytes(stmt, i);
				praseLineString3d(geometry, size, pLaneLink->geometry);
			}
		}
	}
	void DbLaneLoader::loadLaneLinkLg()
	{
		std::string sql = "SELECT LANE_LINK_PID,LG_ID,LANE_NUM,SEQ_NUM FROM HAD_LANE_LINK_LG";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
            int i = 0;
            int64 id = sqlite3_column_int64(stmt, i++);
            DbLaneLink* pLaneLink = (DbLaneLink*)m_pDatabase->query(id, RecordType::DB_HAD_LANE_LINK);
            if (pLaneLink != nullptr)
            {
                pLaneLink->relLgId = sqlite3_column_int64(stmt, i++);
                pLaneLink->laneNum = sqlite3_column_int(stmt, i++);
                pLaneLink->seqNumber = sqlite3_column_int(stmt, i++);
                // pLaneLink->laneDir = sqlite3_column_int(stmt, i++);
            }
		}
	}

	void DbLaneLoader::loadLaneLinkCondition()
	{
		std::string sql = "SELECT LANE_LINK_PID,TYPE FROM HAD_LANE_LINK_CONDITION";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			int64 id = sqlite3_column_int64(stmt, 0);
			DbLaneLink* pLaneLink = (DbLaneLink*)m_pDatabase->query(id, RecordType::DB_HAD_LANE_LINK);
			if (pLaneLink != nullptr)
			{
				pLaneLink->conditionType = sqlite3_column_int64(stmt, 1);
			}
		}
	}

	void DbLaneLoader::loadLaneNode()
	{
		std::string sql = "SELECT LANE_NODE_PID,GEOMETRY FROM HAD_LANE_NODE";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
            DbLaneNode* pLaneNode = (DbLaneNode*)m_pDatabase->alloc(RecordType::DB_HAD_LANE_NODE);
            pLaneNode->uuid = sqlite3_column_int64(stmt, 0);

            const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, 1);
            int size = sqlite3_column_bytes(stmt, 1);
            prasePoint3d(geometry,size, pLaneNode->geometry);
            m_pDatabase->insert(pLaneNode->uuid, pLaneNode);
		}
	}

	void DbLaneLoader::loadLaneSpeedLimit()
	{
		std::string sql = "SELECT LANE_LINK_PID, MAX_SPEED, MIN_SPEED, MAX_SPEED_SOURCE, MIN_SPEED_SOURCE FROM HAD_LANE_LINK_SPEEDLIMIT";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();

			DbFixedSpeedLimit* pDbFSL = (DbFixedSpeedLimit*)m_pDatabase->alloc(RecordType::DB_HAD_LANE_FIXED_SPEEDLIMIT);
			pDbFSL->uuid = sqlite3_column_int64(stmt, 0);
			pDbFSL->maxSpeedLimit = sqlite3_column_int(stmt, 1);
			pDbFSL->minSpeedLimit = sqlite3_column_int(stmt, 2);
			pDbFSL->maxSpeedLimitSource = (DbSpeedLimitSource)sqlite3_column_int(stmt, 3);
			pDbFSL->minSpeedLimitSource = (DbSpeedLimitSource)sqlite3_column_int(stmt, 4);

			m_pDatabase->insert(pDbFSL->uuid, pDbFSL);
		}
	}
    void DbLaneLoader::loadLaneLinkTurnwaiting()
    {
        std::string sql = "SELECT LANE_LINK_PID, FEATURE_TYPE, S_OFFSET, E_OFFSET,GEOMETRY FROM HAD_LANE_LINK_TURNWAITING";
        recordIterator.resetWithSqlStr(sql);
        while (recordIterator.hasNextRecord())
        {
            sqlite3_stmt* stmt = recordIterator.record();
            int i = 0;
			DbLaneLinkTurnwaiting* pLaneLinkTurnwaiting = (DbLaneLinkTurnwaiting*)m_pDatabase->alloc(RecordType::DB_HAD_LANE_LINK_TURNWAITING);
            if (pLaneLinkTurnwaiting != nullptr)
            {
				pLaneLinkTurnwaiting->uuid = sqlite3_column_int64(stmt, i++);
				pLaneLinkTurnwaiting->featureType = sqlite3_column_int(stmt, i++);
				pLaneLinkTurnwaiting->startOffset = sqlite3_column_double(stmt, i++);
				pLaneLinkTurnwaiting->endOffset = sqlite3_column_double(stmt, i++);
                const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, i);
                int size = sqlite3_column_bytes(stmt, i);
				praseMultiPoint3d(geometry, size, pLaneLinkTurnwaiting->geometry);

				m_pDatabase->insert(pLaneLinkTurnwaiting->uuid, pLaneLinkTurnwaiting);
            }
        }
    }
}
