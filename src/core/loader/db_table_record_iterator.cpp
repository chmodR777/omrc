#include "stdafx.h"
#include "db_table_record_iterator.h"

namespace hadc
{
	DbTableRecordIterator* DbTableRecordIterator::allocWithDb(sqlite3* db)
	{
		CQ_ASSERT(db != nullptr);
		return NcNew(DbTableRecordIterator, db);
	}

	DbTableRecordIterator::~DbTableRecordIterator()
	{
		release(m_sqlStr);
		if (m_stmt != nullptr)
			sqlite3_finalize(m_stmt);
	}

	bool DbTableRecordIterator::resetWithSqlStr(const char* sqlStr)
	{
		if (m_stmt != nullptr)
			sqlite3_finalize(m_stmt);

		int sqlStrLen = (int)cq_strlen(sqlStr);

		release(m_sqlStr);
		m_sqlStr = NcString::allocWithAnsiCharacters(sqlStr, sqlStrLen);

		if (SQLITE_OK != sqlite3_prepare_v2(m_db, sqlStr, sqlStrLen, &m_stmt, NULL))
		{
			printError("Create sqlite3_stmt with <%s> failed: %s", sqlStr, sqlite3_errmsg(m_db));
			m_stmt = nullptr;
			return false;
		}

		return true;
	}

	bool DbTableRecordIterator::hasNextRecord()
	{
		if (m_stmt == nullptr)
			return false;

		int stepRet = sqlite3_step(m_stmt);
		if (stepRet == SQLITE_ROW)
			return true;

		if (stepRet != SQLITE_DONE)
			printError("Step SQL statement[%S] failed: ", m_sqlStr->cstr(), sqlite3_errmsg(m_db));

		sqlite3_finalize(m_stmt);
		m_stmt = nullptr;

		return false;
	}
}
