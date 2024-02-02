#pragma once

#include "nc_cpp.h"
#include "sqlite3/sqlite3.h"

namespace hadc
{
	/**
		@brief sqlite3 数据表记录迭代器
	*/
	class DbTableRecordIterator : public NcObjectCpp
	{
	public:
		static DbTableRecordIterator* allocWithDb(sqlite3* db);
		static DbTableRecordIterator* instanceWithDb(sqlite3* db) { return autorelease(allocWithDb(db)); }

		bool resetWithSqlStr(const char* sqlStr);
		bool hasNextRecord();
		sqlite3_stmt* record() { return m_stmt; }

	protected:
		DbTableRecordIterator(sqlite3* db) : m_db(db), m_stmt(nullptr), m_sqlStr(nullptr) {}
		~DbTableRecordIterator();

	private:
		NcString* m_sqlStr;
		sqlite3* m_db;
		sqlite3_stmt* m_stmt;
	};

	class SqliteDbGuard
	{
	public:
		SqliteDbGuard(sqlite3* db) : m_db(db) {}
		~SqliteDbGuard()
		{
			if (SQLITE_OK != sqlite3_close(m_db))
				printError("Close Database failed: %s", sqlite3_errmsg(m_db));
		}

	private:
		sqlite3* m_db;
	};

	class SqliteStmtGuard
	{
	public:
		SqliteStmtGuard(sqlite3_stmt* stmt) : m_stmt(stmt) {}
		~SqliteStmtGuard()
		{
			sqlite3_finalize(m_stmt);
		}

	private:
		sqlite3_stmt* m_stmt;
	};
}
