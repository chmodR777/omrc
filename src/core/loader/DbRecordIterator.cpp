#include "stdafx.h"

#include "DbRecordIterator.h"

namespace OMDB
{

	DbRecordIterator::DbRecordIterator():
		m_pDb(nullptr),
		m_pStmt(nullptr)
	{
	}

	DbRecordIterator::~DbRecordIterator()
	{
		if (m_pStmt != nullptr)
			sqlite3_finalize(m_pStmt);
	}

	bool DbRecordIterator::resetWithSqlStr(const std::string& sqlStr)
	{
		if (m_pStmt != nullptr)
			sqlite3_finalize(m_pStmt);
		if (SQLITE_OK != sqlite3_prepare_v2(m_pDb, sqlStr.c_str(), sqlStr.length(), &m_pStmt, NULL))
		{
			m_pStmt = nullptr;
			return false;
		}

		return true;
	}

	bool DbRecordIterator::hasNextRecord()
	{
		if (m_pStmt == nullptr)
			return false;

		int stepRet = sqlite3_step(m_pStmt);
		if (stepRet == SQLITE_ROW)
			return true;
		sqlite3_finalize(m_pStmt);
		m_pStmt = nullptr;

		return false;
	}
}
