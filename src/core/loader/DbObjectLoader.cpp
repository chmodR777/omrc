#include "stdafx.h"
#include "DbObjectLoader.h"
#include "DbRecordIterator.h"
namespace OMDB
{

    void DbObjectLoader::load()
    {
        recordIterator = DbRecordIterator(m_pSqlite3);
        loadBarrier();
        loadLgRel("HAD_OBJECT_BARRIER_LG", DbObjectType::BARRIER);

        loadWall();
        loadLgRel("HAD_OBJECT_WALL_LG", DbObjectType::WALL);

        loadText();
        loadLaneLinkRel("HAD_OBJECT_TEXT_LANE", DbObjectType::TEXT);

        loadArrow();
        loadLaneLinkRel("HAD_OBJECT_ARROW_LANE", DbObjectType::ARROW);

        loadCrossWalk();
        loadLgRel("HAD_OBJECT_CROSS_WALK_LG", DbObjectType::CROSS_WALK);

        loadTrafficSign();
        loadLgRel("HAD_OBJECT_TRAFFIC_SIGN_LG", DbObjectType::TRAFFIC_SIGN);

        loadStopLocation();
        loadLgRel("HAD_OBJECT_STOPLOCATION_LG", DbObjectType::STOPLOCATION);
        loadLaneLinkRel("HAD_OBJECT_STOPLOCATION_LANE", DbObjectType::STOPLOCATION);

        loadFillArea();
        loadLgRel("HAD_OBJECT_FILL_AREA_LG", DbObjectType::FILL_AREA);   

        loadTrafficLights();
        loadLgRel("HAD_OBJECT_TRAFFIC_LIGHTS_LG", DbObjectType::TRAFFIC_LIGHTS);
        loadLaneLinkRel("HAD_OBJECT_TRAFFIC_LIGHTS_LANE", DbObjectType::TRAFFIC_LIGHTS);

        loadPole();
    }

    void DbObjectLoader::loadBarrier()
    {
        std::string sql = "SELECT OBJECT_PID,BARRIER_TYPE,GEOMETRY FROM HAD_OBJECT_BARRIER";
        recordIterator.resetWithSqlStr(sql);
        while (recordIterator.hasNextRecord())
        {
            sqlite3_stmt* stmt = recordIterator.record();
            DbBarrier* pBarrier = (DbBarrier*)m_pDatabase->alloc(RecordType::DB_HAD_OBJECT_BARRIER);
            pBarrier->uuid = sqlite3_column_int64(stmt, 0);
            pBarrier->barrierType = sqlite3_column_int(stmt, 1);
            const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, 2);
            int size = sqlite3_column_bytes(stmt, 2);
            praseMultiLineString3d(geometry, size, pBarrier->geometry);
            m_pDatabase->insert(pBarrier->uuid, pBarrier);
        }
    }

    void DbObjectLoader::loadWall()
    {
        std::string sql = "SELECT OBJECT_PID,TYPE,GEOMETRY FROM HAD_OBJECT_WALL";
        recordIterator.resetWithSqlStr(sql);
        while (recordIterator.hasNextRecord())
        {
            sqlite3_stmt* stmt = recordIterator.record();
            DbWall* pWall = (DbWall*)m_pDatabase->alloc(RecordType::DB_HAD_OBJECT_WALL);
            pWall->uuid = sqlite3_column_int64(stmt, 0);
            pWall->wallType = sqlite3_column_int(stmt, 1);
            const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, 2);
            int size = sqlite3_column_bytes(stmt, 2);
            praseMultiLineString3d(geometry, size, pWall->geometry);
            m_pDatabase->insert(pWall->uuid, pWall);
        }
    }

    void DbObjectLoader::loadText()
    {
        std::string sql = "SELECT OBJECT_PID, CENTRE_X, CENTRE_Y, CENTRE_Z, LENGTH, WIDTH, COLOR, GEOMETRY, TEXT_STRING FROM HAD_OBJECT_TEXT";
        recordIterator.resetWithSqlStr(sql);
        while (recordIterator.hasNextRecord())
        {
            sqlite3_stmt* stmt = recordIterator.record();
            DbText* pText = (DbText*)m_pDatabase->alloc(RecordType::DB_HAD_OBJECT_TEXT);
            if (nullptr == pText)
            {
                continue;
            }
            pText->uuid = sqlite3_column_int64(stmt, 0);
            double centerX = sqlite3_column_double(stmt, 1);
            double centerY = sqlite3_column_double(stmt, 2);
            double centerZ = sqlite3_column_double(stmt, 3);
            pText->center = ToPoint3d(centerX, centerY, centerZ);
            pText->length = sqlite3_column_int(stmt, 4);
            pText->width = sqlite3_column_int(stmt, 5);
            pText->color = sqlite3_column_int(stmt, 6);
            
            const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, 7);
            int size = sqlite3_column_bytes(stmt, 7);
            prasePolygon3d(geometry, size, pText->geometry);

            // 20230411 按V1.5添加
   //         wchar_t* wText = (wchar_t*)sqlite3_column_text16(stmt, 8);
			//DWORD dwNum = WideCharToMultiByte(CP_OEMCP, NULL, wText, -1, NULL, 0, NULL, FALSE);//WideCharToMultiByte的运用 
			//char* psText = new char[dwNum];
			//WideCharToMultiByte(CP_OEMCP, NULL, wText, -1, psText, dwNum, NULL, FALSE);// WideCharToMultiByte的再次运用
			//pText->textContent = psText;
			//delete[]psText;//psText的清除
            pText->textContent = std::wstring((const wchar_t*)sqlite3_column_text16(stmt, 8));

            m_pDatabase->insert(pText->uuid, pText);
        }
    }

    void DbObjectLoader::loadArrow()
    {
        std::string sql = "SELECT OBJECT_PID,CENTRE_X,CENTRE_Y,CENTRE_Z,LENGTH,WIDTH,COLOR,ARROW_CLASS,GEOMETRY FROM HAD_OBJECT_ARROW";
        recordIterator.resetWithSqlStr(sql);
        while (recordIterator.hasNextRecord())
        {
            sqlite3_stmt* stmt = recordIterator.record();
            DbArrow* pArrow = (DbArrow*)m_pDatabase->alloc(RecordType::DB_HAD_OBJECT_ARROW);
            pArrow->uuid = sqlite3_column_int64(stmt, 0);
            double centerX = sqlite3_column_double(stmt, 1);
            double centerY = sqlite3_column_double(stmt, 2);
            double centerZ = sqlite3_column_double(stmt, 3);
            pArrow->center = ToPoint3d(centerX, centerY, centerZ);

            pArrow->length = sqlite3_column_int(stmt, 4);
            pArrow->width = sqlite3_column_int(stmt, 5);
            pArrow->color = sqlite3_column_int(stmt, 6);
            pArrow->arrowClass = sqlite3_column_int(stmt, 7);
            const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, 8);
            int size = sqlite3_column_bytes(stmt, 8);
            prasePolygon3d(geometry, size, pArrow->geometry);
            m_pDatabase->insert(pArrow->uuid, pArrow);
        }
    }

    void DbObjectLoader::loadCrossWalk()
    {
        std::string sql = "SELECT OBJECT_PID,COLOR,GEOMETRY FROM HAD_OBJECT_CROSS_WALK";
        recordIterator.resetWithSqlStr(sql);
        while (recordIterator.hasNextRecord())
        {
            sqlite3_stmt* stmt = recordIterator.record();
            DbCrossWalk* pCrossWalk = (DbCrossWalk*)m_pDatabase->alloc(RecordType::DB_HAD_OBJECT_CROSS_WALK);
            pCrossWalk->uuid = sqlite3_column_int64(stmt, 0);
            pCrossWalk->color = sqlite3_column_int(stmt, 1);
            const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, 2);
            int size = sqlite3_column_bytes(stmt, 2);
            prasePolygon3d(geometry, size, pCrossWalk->geometry);
            m_pDatabase->insert(pCrossWalk->uuid, pCrossWalk);
        }
    }

    void DbObjectLoader::loadFillArea()
    {
        std::string sql = "SELECT OBJECT_PID,GEOMETRY FROM HAD_OBJECT_FILL_AREA";
        recordIterator.resetWithSqlStr(sql);
        while (recordIterator.hasNextRecord())
        {
            sqlite3_stmt* stmt = recordIterator.record();
            DbFillArea* pFillArea = (DbFillArea*)m_pDatabase->alloc(RecordType::DB_HAD_OBJECT_FILL_AREA);
            pFillArea->uuid = sqlite3_column_int64(stmt, 0);
            const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, 1);
            int size = sqlite3_column_bytes(stmt, 1);
            prasePolygon3d(geometry, size, pFillArea->geometry);
            m_pDatabase->insert(pFillArea->uuid, pFillArea);
        }
    }

    void DbObjectLoader::loadTrafficSign()
    {
        std::string sql = "SELECT OBJECT_PID,TRAFSIGN_SHAPE,SIGN_TYPE,COLOR,HEADING,GEOMETRY FROM HAD_OBJECT_TRAFFIC_SIGN";
        recordIterator.resetWithSqlStr(sql);
        while (recordIterator.hasNextRecord())
        {
            sqlite3_stmt* stmt = recordIterator.record();
            DbTrafficSign* pSign = (DbTrafficSign*)m_pDatabase->alloc(RecordType::DB_HAD_OBJECT_TRAFFIC_SIGN);
            pSign->uuid = sqlite3_column_int64(stmt, 0);
            pSign->trafSignShape = sqlite3_column_int(stmt, 1);
            pSign->signType = std::atoi ((const char*)sqlite3_column_text(stmt, 2));
            pSign->color = sqlite3_column_int(stmt, 3);
            pSign->heading = sqlite3_column_double(stmt, 4);
            const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, 5);
            int size = sqlite3_column_bytes(stmt, 5);
            prasePolygon3d(geometry, size, pSign->geometry);
            m_pDatabase->insert(pSign->uuid, pSign);
        }
    }

    void DbObjectLoader::loadStopLocation()
    {
        std::string sql = "SELECT OBJECT_PID,WIDTH,COLOR,LOCATION_TYPE,GEOMETRY FROM HAD_OBJECT_STOPLOCATION";
        recordIterator.resetWithSqlStr(sql);
        while (recordIterator.hasNextRecord())
        {
            sqlite3_stmt* stmt = recordIterator.record();
            DbStopLocation* pStopLocation = (DbStopLocation*)m_pDatabase->alloc(RecordType::DB_HAD_OBJECT_STOPLOCATION);
            pStopLocation->uuid = sqlite3_column_int64(stmt, 0);
            pStopLocation->width = sqlite3_column_int(stmt, 1);
            pStopLocation->color = sqlite3_column_int(stmt, 2);
            pStopLocation->locationType = sqlite3_column_int(stmt, 3);
            const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, 4);
            int size = sqlite3_column_bytes(stmt, 4);
            praseLineString3d(geometry, size, pStopLocation->geometry);
            m_pDatabase->insert(pStopLocation->uuid, pStopLocation);
        }
    }
    
	void DbObjectLoader::loadTrafficLights()
	{
		std::string sql = "SELECT OBJECT_PID,TRAFFIC_LIGHT_TYPE,ORIENTATION,ROW_NUMBER,COLUMN_NUMBER,HEADING,GEOMETRY FROM HAD_OBJECT_TRAFFIC_LIGHTS";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
            DbTrafficLights* pTrafficLights = (DbTrafficLights*)m_pDatabase->alloc(RecordType::DB_HAD_OBJECT_TRAFFIC_LIGHTS);
			pTrafficLights->uuid = sqlite3_column_int64(stmt, 0);
			pTrafficLights->type = sqlite3_column_int(stmt, 1);
            pTrafficLights->orientation = sqlite3_column_int(stmt, 2);
			pTrafficLights->rowNumber = sqlite3_column_int(stmt, 3);
			pTrafficLights->columnNumber = sqlite3_column_int(stmt, 4);
            pTrafficLights->heading = sqlite3_column_double(stmt, 5);
			const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, 6);
			int size = sqlite3_column_bytes(stmt, 6);
            prasePolygon3d(geometry, size, pTrafficLights->geometry);
			m_pDatabase->insert(pTrafficLights->uuid, pTrafficLights);
		}
	}


	void DbObjectLoader::loadPole()
	{
		std::string sql = "SELECT OBJECT_PID,POLE_TYPE,DIAMETER_TOP,DIAMETER_BOTTOM,GEOMETRY FROM HAD_OBJECT_POLE";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			DbPole* pPole = (DbPole*)m_pDatabase->alloc(RecordType::DB_HAD_OBJECT_POLE);
			pPole->uuid = sqlite3_column_int64(stmt, 0);
			pPole->type = sqlite3_column_int(stmt, 1);
			pPole->diameterTop = sqlite3_column_int(stmt, 2);
			pPole->diameterBottom = sqlite3_column_int(stmt, 3);
			const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, 4);
			int size = sqlite3_column_bytes(stmt, 4);
			praseLineString3d(geometry, size, pPole->geometry);
			m_pDatabase->insert(pPole->uuid, pPole);
		}
	}


    void DbObjectLoader::loadLgRel(const char* tableName, DbObjectType objectType)
    {
        std::string sql = "SELECT OBJECT_PID,LG_ID FROM ";
        sql.append(tableName);
        recordIterator.resetWithSqlStr(sql);
        while (recordIterator.hasNextRecord())
        {
            sqlite3_stmt* stmt = recordIterator.record();
            int64 id = sqlite3_column_int64(stmt, 0);
            DbLgRel* pLgRel = (DbLgRel*)m_pDatabase->query(id, RecordType::DB_HAD_OBJECT_LG_REL);
            if (pLgRel == nullptr) {
                pLgRel = (DbLgRel*)m_pDatabase->alloc(RecordType::DB_HAD_OBJECT_LG_REL);
                pLgRel->uuid = id;
                m_pDatabase->insert(pLgRel->uuid, pLgRel);
            }

            DbLgRel::DbRelLg relLg;
            relLg.objectType = (int)objectType;
            relLg.relLgId = sqlite3_column_int64(stmt, 1);
            pLgRel->relLgs.emplace(relLg.relLgId, relLg);
        }
    }

    void DbObjectLoader::loadLaneLinkRel(const char* tableName, DbObjectType objectType)
    {
        std::string sql = "SELECT OBJECT_PID,LANE_LINK_PID FROM ";
        sql.append(tableName);
        recordIterator.resetWithSqlStr(sql);
        while (recordIterator.hasNextRecord())
        {
            sqlite3_stmt* stmt = recordIterator.record();
            int64 uuid = sqlite3_column_int64(stmt, 0);

            DbLaneLinkRel* pLaneLinkRel = (DbLaneLinkRel*)m_pDatabase->query(uuid, RecordType::DB_HAD_OBJECT_LANE_LINK_REL);
			if (pLaneLinkRel == nullptr)
            {
                pLaneLinkRel = (DbLaneLinkRel*)m_pDatabase->alloc(RecordType::DB_HAD_OBJECT_LANE_LINK_REL);
                pLaneLinkRel->uuid = uuid;
				m_pDatabase->insert(uuid, pLaneLinkRel);
			}

            DbLaneLinkRel::DbRelLaneLInk relLaneLink;
            relLaneLink.objectType = (int)objectType;
            relLaneLink.relLaneLinkId = sqlite3_column_int64(stmt, 1);
            pLaneLinkRel->relLaneLinkIds.emplace(relLaneLink.relLaneLinkId, relLaneLink);
        }
    }

    void DbObjectLoader::loadObjectRel()
    {

    }
}
