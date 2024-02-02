#include "stdafx.h"
#include "DbIntersectionLoader.h"
namespace OMDB
{

	void DbIntersectionLoader::load()
	{
		recordIterator = DbRecordIterator(m_pSqlite3);
		loadIntersection();
		loadIntersectionType();
		loadIntersectionLG();
		loadIntersectionLanePA();
		loadIntersectionMesh();
		loadIntersectionLink();
		loadIntersectionNode();
		loadIntersectionPoint();
	}

	void DbIntersectionLoader::loadIntersection()
	{
		// TYPE
		std::string sql = "SELECT INTERSECTION_PID FROM HAD_INTERSECTION";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			DbIntersection* pIntersection = (DbIntersection*)m_pDatabase->alloc(RecordType::DB_HAD_INTERSECTION);
			if (nullptr == pIntersection)
			{
				continue;
			}
			pIntersection->uuid = sqlite3_column_int64(stmt, 0);
			// pIntersection->intersectionType = sqlite3_column_int(stmt, 1);  // v1.5
			m_pDatabase->insert(pIntersection->uuid, pIntersection);
		}
	}

	void DbIntersectionLoader::loadIntersectionType()
	{
		std::string sql = "SELECT INTERSECTION_PID, TYPE FROM HAD_INTERSECTION_TYPE";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			int64 uuid = sqlite3_column_int64(stmt, 0);

			DbIntersection *pIntersection = (DbIntersection*)m_pDatabase->query(uuid, RecordType::DB_HAD_INTERSECTION);
			if (pIntersection != nullptr)
			{
				pIntersection->intersectionType = sqlite3_column_int(stmt, 1);
			}
		}
	}

	void DbIntersectionLoader::loadIntersectionLG()
	{
		std::string sql = "SELECT INTERSECTION_PID, LG_ID FROM HAD_INTERSECTION_LG";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			int64 uuid = sqlite3_column_int64(stmt, 0);

			DbIntersection* pIntersection = (DbIntersection*)m_pDatabase->query(uuid, RecordType::DB_HAD_INTERSECTION);
			if (pIntersection != nullptr)
			{
				pIntersection->refLaneGroups.push_back(sqlite3_column_int64(stmt, 1));
			}
		}
	}

	void DbIntersectionLoader::loadIntersectionLanePA()
	{
		std::string sql = "SELECT INTERSECTION_PID, LINK_LGPA_PID FROM HAD_INTERSECTION_LANEPA";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			int64 uuid = sqlite3_column_int64(stmt, 0);

			DbIntersection* pIntersection = (DbIntersection*)m_pDatabase->query(uuid, RecordType::DB_HAD_INTERSECTION);
			if (pIntersection != nullptr)
			{
				pIntersection->refLanePAs.push_back(sqlite3_column_int64(stmt, 1));
			}
		}
	}

	void DbIntersectionLoader::loadIntersectionMesh()
	{
		std::string sql = "SELECT INTERSECTION_PID, MESH FROM HAD_INTERSECTION_MESH";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			int64 uuid = sqlite3_column_int64(stmt, 0);

			DbIntersection* pIntersection = (DbIntersection*)m_pDatabase->query(uuid, RecordType::DB_HAD_INTERSECTION);
			if (pIntersection != nullptr)
			{
				pIntersection->refMeshs.push_back(sqlite3_column_int(stmt, 1));
			}
		}
	}

	void DbIntersectionLoader::loadIntersectionLink()
	{
		std::string sql = "SELECT INTERSECTION_PID, LINK_PID, FEATURE_TYPE, S_OFFSET, E_OFFSET, GEOMETRY, DIRECT, CENTER_LINK FROM HAD_INTERSECTION_LINK";
		// sql = "SELECT INTERSECTION_PID, FEATURE_PID, FEATURE_TYPE, DIRECT, CENTER_LINK FROM HAD_INTERSECTION_LINK"; // #v1.5
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			int64 uuid = sqlite3_column_int64(stmt, 0);

			DbIntersection* pIntersection = (DbIntersection*)m_pDatabase->query(uuid, RecordType::DB_HAD_INTERSECTION);
			if (pIntersection != nullptr)
			{
				DbIntersection::OutLink outlink;
				outlink.linkId = sqlite3_column_int64(stmt, 1);
				outlink.featureType = sqlite3_column_int(stmt, 2);
				outlink.startOffset = sqlite3_column_double(stmt, 3);
				outlink.endOffset = sqlite3_column_double(stmt, 4);
				const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, 5);
				int size = sqlite3_column_bytes(stmt, 5);
				praseMultiPoint3d(geometry, size, outlink.geometry);
				outlink.direction = sqlite3_column_int(stmt, 6);
				outlink.isCenterLink = sqlite3_column_int(stmt, 7);
				//outlink.direction = sqlite3_column_int(stmt, 3);
				//outlink.isCenterLink = sqlite3_column_int(stmt, 4);

				pIntersection->outLinks.push_back(outlink);
			}
		}
	}

	void DbIntersectionLoader::loadIntersectionNode()
	{
		std::string sql = "SELECT INTERSECTION_PID, NODE_PID FROM HAD_INTERSECTION_NODE";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			int64 uuid = sqlite3_column_int64(stmt, 0);

			DbIntersection* pIntersection = (DbIntersection*)m_pDatabase->query(uuid, RecordType::DB_HAD_INTERSECTION);
			if (pIntersection != nullptr)
			{
				pIntersection->refNodes.push_back(sqlite3_column_int64(stmt, 1));
			}
		}
	}

	void DbIntersectionLoader::loadIntersectionPoint()
	{
		std::string sql = "SELECT INTERSECTION_PID, LINK_PID, OFFSET, GEOMETRY FROM HAD_INTERSECTION_POINT";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			int64 uuid = sqlite3_column_int64(stmt, 0);

			DbIntersection* pIntersection = (DbIntersection*)m_pDatabase->query(uuid, RecordType::DB_HAD_INTERSECTION);
			if (pIntersection != nullptr)
			{
				DbIntersection::OutlinePA outline;
				outline.linkId = sqlite3_column_int64(stmt, 1);
				outline.offset = sqlite3_column_double(stmt, 2);
				const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, 3);
				int size = sqlite3_column_bytes(stmt, 3);
				prasePoint3d(geometry, size, outline.position);

				pIntersection->outLines.push_back(outline);
			}
		}
	}

}