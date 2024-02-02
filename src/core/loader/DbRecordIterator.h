#pragma once

#include "sqlite3/sqlite3.h"
#include <string>
namespace OMDB
{

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


	class DbRecordIterator
	{
	public:
        DbRecordIterator();
        DbRecordIterator(sqlite3* db) : m_pDb(db), m_pStmt(nullptr) {}
        ~DbRecordIterator();

		bool resetWithSqlStr(const std::string& sqlStr);
		bool hasNextRecord();
		sqlite3_stmt* record() { return m_pStmt; }

	private:
		sqlite3* m_pDb;
		sqlite3_stmt* m_pStmt;
	};
}
