#include "stdafx.h"

#include "DbRoadBoundaryLoader.h"
#include "DbRecordIterator.h"
namespace OMDB
{
    void DbRoadBoundaryLoader::load()
    {
        recordIterator = DbRecordIterator(m_pSqlite3);
        loadRoadBoundLink();
        loadRoadBoundNode();
        loadRoadBoundBoundaryType();
        loadLgRoadBoundRel();
    }
    void DbRoadBoundaryLoader::loadRoadBoundLink()
    {
        // BOUNDARY_TYPE
        std::string sql = "SELECT ROAD_BOUND_LINK_PID,S_ROAD_BOUND_NODE_PID,E_ROAD_BOUND_NODE_PID,GEOMETRY FROM HAD_RDBOUND_LINK";
        recordIterator.resetWithSqlStr(sql);
        while (recordIterator.hasNextRecord())
        {
            sqlite3_stmt* stmt = recordIterator.record();
            DbRoadBoundLink* pLink = (DbRoadBoundLink*)m_pDatabase->alloc(RecordType::DB_HAD_ROAD_BOUNDARY_LINK);
            pLink->uuid = sqlite3_column_int64(stmt, 0);
            pLink->starRoadBoundNodeId = sqlite3_column_int64(stmt, 1);
            pLink->endRoadBoundNodeId = sqlite3_column_int64(stmt, 2);
            // pLink->boundaryType = sqlite3_column_int(stmt, 3);
            const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, 3);
            int size = sqlite3_column_bytes(stmt, 3);
            praseLineString3d(geometry,size, pLink->geometry);
            m_pDatabase->insert(pLink->uuid, pLink);
        }
    }

    void DbRoadBoundaryLoader::loadRoadBoundBoundaryType()
    {
        const int BOUNDARY_TYPE = 7;
		int64 rdBoundPASeed = 100000000000000000;
		std::string sql = "SELECT ROAD_BOUND_LINK_PID,FEATURE_TYPE,S_OFFSET,E_OFFSET,BOUNDARY_TYPE,GEOMETRY FROM HAD_RDBOUND_BOUNDARYTYPE";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();

			int i = 0;
			int64 id = sqlite3_column_int64(stmt, i++);
			int64 featureType = sqlite3_column_int(stmt, i++);
			if (featureType == 1)
			{
                DbRoadBoundLink* pLink = (DbRoadBoundLink*)m_pDatabase->query(id, RecordType::DB_HAD_ROAD_BOUNDARY_LINK);
				if (pLink != nullptr)
				{
                    pLink->boundaryType = sqlite3_column_int(stmt, 4);
				}
			}
			else if (featureType == 2)
			{
				DbRoadBoundLink* pLink = (DbRoadBoundLink*)m_pDatabase->query(id, RecordType::DB_HAD_ROAD_BOUNDARY_LINK);
				if (pLink != nullptr && pLink->boundaryType == -1)
				{
					pLink->boundaryType = DB_HAD_APPLY_PA_REFERENCE;
				}
                DbRoadBoundPA* pLinkPA = (DbRoadBoundPA*)m_pDatabase->alloc(RecordType::DB_HAD_ROAD_BOUNDARY_PA);
                pLinkPA->relRoadBoundLinkId = id;
                pLinkPA->uuid = id + rdBoundPASeed;
                pLinkPA->startOffset = sqlite3_column_double(stmt, i++);
                pLinkPA->endOffset = sqlite3_column_double(stmt, i++);

				DbPAValue* pPAValue = (DbPAValue*)m_pDatabase->alloc(RecordType::DB_HAD_PA_VALUE);
				pPAValue->attributeType = BOUNDARY_TYPE;
				pPAValue->attributeValue = sqlite3_column_int(stmt, i++);
                pLinkPA->paValues.push_back(pPAValue);

				const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, i);
				int size = sqlite3_column_bytes(stmt, i);

				praseMultiPoint3d(geometry, size, pLinkPA->geometry);
				m_pDatabase->insert(pLinkPA->uuid, pLinkPA);
                rdBoundPASeed++;
			}
		}
    }

    void DbRoadBoundaryLoader::loadRoadBoundNode()
    {
        std::string sql = "SELECT ROAD_BOUND_NODE_PID,GEOMETRY FROM HAD_RDBOUND_NODE";
        recordIterator.resetWithSqlStr(sql);
        while (recordIterator.hasNextRecord())
        {
            sqlite3_stmt* stmt = recordIterator.record();

            DbRoadBoundNode* pNode = (DbRoadBoundNode*)m_pDatabase->alloc(RecordType::DB_HAD_ROAD_BOUNDARY_NODE);
            pNode->uuid = sqlite3_column_int64(stmt, 0);

            const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, 1);
            int size = sqlite3_column_bytes(stmt, 1);

            prasePoint3d(geometry, size, pNode->geometry);
            m_pDatabase->insert(pNode->uuid, pNode);
        }
    }

    void DbRoadBoundaryLoader::loadLgRoadBoundRel()
    {
        std::string sql = "SELECT LG_ID,ROAD_BOUND_LINK_PID,SIDE,DIRECT FROM HAD_LG_RDBOUND_REL";
        recordIterator.resetWithSqlStr(sql);
        while (recordIterator.hasNextRecord())
        {
            sqlite3_stmt* stmt = recordIterator.record();
            int64 id = sqlite3_column_int64(stmt, 1);

            DbRoadBoundLink* pLinkPA = (DbRoadBoundLink*)m_pDatabase->query(id, RecordType::DB_HAD_ROAD_BOUNDARY_LINK);
            if (pLinkPA != nullptr)
            {
                DbRoadBoundLink::DbLgRoadBoundREL relLg;
                int64 relLgLinkId = sqlite3_column_int64(stmt, 0);
                relLg.side = sqlite3_column_int(stmt, 2);
                relLg.direction = sqlite3_column_int(stmt, 3);
                pLinkPA->relLgs.emplace(relLgLinkId, relLg);
            }

        }
    }
}
