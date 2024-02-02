#include "stdafx.h"
#include "DbTollGateLoader.h"
#include "DbRecordIterator.h"
namespace OMDB
{
    // 全局的收费站补丁数据
    std::map<int64, int> DbTollGateLoader::linkCardTypes;
    void DbTollGateLoader::LoadLinkCardTypes(sqlite3* pDb)
    {
        m_pSqlite3 = pDb;
        recordIterator = DbRecordIterator(m_pSqlite3);
		std::string sql = "SELECT LINK_PID,CARD_TYPE FROM HAD_LANE_LINK_TOLL";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			int64 id = sqlite3_column_int64(stmt, 0);
            int cardType = sqlite3_column_int(stmt, 1);
            linkCardTypes.emplace(id, cardType);
		}
    }

    void DbTollGateLoader::load()
    {
        recordIterator = DbRecordIterator(m_pSqlite3);
        loadTollGate();
        loadTollGateType();
        loadTollGatePassage();
        loadTollGatePayMethod();
		loadTollGatePayCardType();
		loadTollGateName();
    }

    void DbTollGateLoader::loadTollGate()
    {
        std::string sql = "SELECT TOLLGATE_PID, LINK_PID, OFFSET, GEOMETRY FROM HAD_TOLLGATE";
        recordIterator.resetWithSqlStr(sql);
        while (recordIterator.hasNextRecord())
        {
            sqlite3_stmt* stmt = recordIterator.record();
            DbTollGate* pTollGate = (DbTollGate*)m_pDatabase->alloc(RecordType::DB_HAD_TOLLGATE);
            if (nullptr == pTollGate)
            {
                continue;
            }

            int i = 0;
            pTollGate->uuid = sqlite3_column_int64(stmt, i++);
            pTollGate->linkPid = sqlite3_column_int64(stmt, i++);
            pTollGate->offset = sqlite3_column_double(stmt, i++);
			const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, i);
			int size = sqlite3_column_bytes(stmt, i);
			prasePoint3d(geometry, size, pTollGate->geometry);

            m_pDatabase->insert(pTollGate->uuid, pTollGate);
        }
    }

    void DbTollGateLoader::loadTollGateType()
    {
		std::string sql = "SELECT TOLLGATE_PID, TOLL_TYPE FROM HAD_TOLLGATE_TYPE";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			int64 id = sqlite3_column_int64(stmt, 0);

			DbTollGate* pTollGate = (DbTollGate*)m_pDatabase->query(id, RecordType::DB_HAD_TOLLGATE);
			if (pTollGate != nullptr)
			{
                pTollGate->tollType = sqlite3_column_int(stmt, 1);
			}
		}
    }

    void DbTollGateLoader::loadTollGatePassage()
    {
		std::string sql = "SELECT PID, TOLLGATE_PID, SEQ_NUM, LANE_LINK_PID FROM HAD_TOLLGATE_PASSAGE";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
            int64 pid = sqlite3_column_int64(stmt, 0);
			int64 tollgatePid = sqlite3_column_int64(stmt, 1);

			DbTollGate* pTollGate = (DbTollGate*)m_pDatabase->query(tollgatePid, RecordType::DB_HAD_TOLLGATE);
			if (pTollGate != nullptr)
			{
				DbTollLane* pTollLane = (DbTollLane*)m_pDatabase->alloc(RecordType::DB_HAD_TOLL_LANE);
				if (nullptr == pTollLane)
					continue;

				pTollLane->seqNum = sqlite3_column_int(stmt, 2);
				pTollLane->laneLinkPid = sqlite3_column_int64(stmt, 3);
				if (linkCardTypes.count(pTollLane->laneLinkPid))
				{
					pTollLane->cardType = linkCardTypes[pTollLane->laneLinkPid];
				}
				pTollGate->tollLanes.push_back(pTollLane);
                
                m_pDatabase->insert(pid, pTollLane);
			}
		}
    }

    void DbTollGateLoader::loadTollGatePayMethod()
    {
		std::string sql = "SELECT PID, PAY_METHOD FROM HAD_TOLLGATE_PAYMETHOD";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			int64 id = sqlite3_column_int64(stmt, 0);

			DbTollLane* pTollLane = (DbTollLane*)m_pDatabase->query(id, RecordType::DB_HAD_TOLL_LANE);
			if (nullptr == pTollLane)
				continue;

			pTollLane->payMethod = sqlite3_column_int(stmt, 1);
		}
    }

	void DbTollGateLoader::loadTollGatePayCardType()
	{
		std::string sql = "SELECT PID, CARD_TYPE FROM HAD_TOLLGATE_CARDTYPE";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			int64 id = sqlite3_column_int64(stmt, 0);

			DbTollLane* pTollLane = (DbTollLane*)m_pDatabase->query(id, RecordType::DB_HAD_TOLL_LANE);
			if (nullptr == pTollLane)
				continue;

			pTollLane->cardType = sqlite3_column_int(stmt, 1);
		}
	}

	void DbTollGateLoader::loadTollGateName()
	{
		std::string sql = "SELECT NAME_ID, TOLLGATE_PID, LANG_CODE, NAME FROM HAD_TOLLGATE_NAME";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			int64 nameId = sqlite3_column_int64(stmt, 0);
			int64 tollId = sqlite3_column_int64(stmt, 1);
			std::wstring langCode = std::wstring((const wchar_t*)sqlite3_column_text16(stmt, 2));
			std::wstring tollName = std::wstring((const wchar_t*)sqlite3_column_text16(stmt, 3));
			if (langCode == L"CHI")
			{
				DbTollGate* pTollGate = (DbTollGate*)m_pDatabase->query(tollId, RecordType::DB_HAD_TOLLGATE);
				if (pTollGate != nullptr)
				{
					pTollGate->tollName = tollName;
				}
			}
		}
	}

}
