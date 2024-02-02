#include "stdafx.h"

#include "DbLaneMarkLoader.h"
#include "DbRecordIterator.h"
namespace OMDB
{

	void DbLaneMarkLoader::load()
	{
		recordIterator = DbRecordIterator(m_pSqlite3);
		loadLaneMarkLink();
		loadLaneMarkMarkingGeo();

		loadLaneMarkPA();
		loadLaneMarkPAValue();

		// 以下两个字段依赖PA先建立
		loadLaneMarkBoundaryType();
		loadLaneMarkTraversal();

		loadLaneMarkRel();
		loadLgMarkRel();
		loadLaneMarkNode();
	}
	void DbLaneMarkLoader::loadLaneMarkLink()
	{
		// BOUNDARY_TYPE,MARKING_COUNT
		std::string sql = "SELECT LANE_MARK_LINK_PID,S_LANE_MARK_NODE_PID,E_LANE_MARK_NODE_PID,GEOMETRY FROM HAD_LANE_MARK_LINK";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
            int i = 0;
			DbLaneMarkLink* pLaneMarkLink = (DbLaneMarkLink*)m_pDatabase->alloc(RecordType::DB_HAD_LANE_MARK_LINK);
            pLaneMarkLink->uuid = sqlite3_column_int64(stmt, i++);
            pLaneMarkLink->startLaneMarkNodeId = sqlite3_column_int64(stmt, i++);
            pLaneMarkLink->endLaneMarkNodeId = sqlite3_column_int64(stmt, i++);
            //pLaneMarkLink->boundaryType = sqlite3_column_int(stmt, i++);
			//pLaneMarkLink->markingCount = sqlite3_column_int(stmt, i++);
            const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, i);
            int size = sqlite3_column_bytes(stmt, i);
            praseLineString3d(geometry,size, pLaneMarkLink->geometry);
            m_pDatabase->insert(pLaneMarkLink->uuid, pLaneMarkLink);
		}
	}

	void DbLaneMarkLoader::loadLaneMarkMarkingGeo()
	{
		std::string sql = "SELECT LANE_MARK_LINK_PID,MARKING_COUNT,GEOMETRY FROM HAD_LANE_MARK_MARKING_GEO";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			int i = 0;
			int64 id = sqlite3_column_int64(stmt, i++);
			DbLaneMarkLink* pMarkLink = (DbLaneMarkLink*)m_pDatabase->query(id, RecordType::DB_HAD_LANE_MARK_LINK);
			if (pMarkLink != nullptr && pMarkLink->geometry.vertexes.empty())
			{
				pMarkLink->markingCount = sqlite3_column_int64(stmt, i++);
				const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, i);
				int size = sqlite3_column_bytes(stmt, i);
				praseLineString3d(geometry, size, pMarkLink->geometry);
			}
		}
	}

	void DbLaneMarkLoader::loadLaneMarkBoundaryType()
	{
		std::string sql = "SELECT FEATURE_PID,FEATURE_CLASS,BOUNDARY_TYPE FROM HAD_LANE_MARK_BOUNDARYTYPE";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();

			int i = 0;
			int64 id = sqlite3_column_int64(stmt, i++);
			int64 featureClass = sqlite3_column_int(stmt, i++);
			if (featureClass == 1)
			{
				DbLaneMarkLink* pMarkLink = (DbLaneMarkLink*)m_pDatabase->query(id, RecordType::DB_HAD_LANE_MARK_LINK);
				if (pMarkLink != nullptr)
				{
					pMarkLink->boundaryType = sqlite3_column_int(stmt, 1);
				}
			}
			else if (featureClass == 2)
			{
				DbLaneMarkPA* pLaneMarkPA = (DbLaneMarkPA*)m_pDatabase->query(id, RecordType::DB_HAD_LANE_MARK_PA);
				if (pLaneMarkPA != nullptr)
				{
					DbPAValue* pPAValue = (DbPAValue*)m_pDatabase->alloc(RecordType::DB_HAD_PA_VALUE);
					pPAValue->attributeType = (int)DbLaneBoundaryPAType::BOUNDARY_TYPE;
					pPAValue->attributeValue = sqlite3_column_int(stmt, i++);
					pLaneMarkPA->paValues.push_back(pPAValue);
				}
			}
		}
	}

	void DbLaneMarkLoader::loadLaneMarkTraversal()
	{
		std::string sql = "SELECT LANE_MARK_LINK_PID,FEATURE_TYPE,S_OFFSET,E_OFFSET,TRAVERSAL,GEOMETRY FROM HAD_LANE_MARK_TRAVERSAL";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();

			int i = 0;
			int64 id = sqlite3_column_int64(stmt, i++);
			int64 featureType = sqlite3_column_int(stmt, i++);
			double startOffset = sqlite3_column_double(stmt, i++);
			double endOffset = sqlite3_column_double(stmt, i++);
			if (featureType == 1)
			{
				DbLaneMarkLink* pMarkLink = (DbLaneMarkLink*)m_pDatabase->query(id, RecordType::DB_HAD_LANE_MARK_LINK);
				if (pMarkLink != nullptr)
				{
					pMarkLink->traversal = sqlite3_column_int(stmt, 1);
				}
			}
			else if (featureType == 2)
			{
				// PA
				std::vector<DbRecord*>& pas = m_pDatabase->query(RecordType::DB_HAD_LANE_MARK_PA);
				for (auto pa : pas)
				{
					DbLaneMarkPA* ppa = (DbLaneMarkPA*)pa;
					if (ppa->relLaneMarkLinkId == id && startOffset == ppa->startOffset && endOffset == ppa->endOffset)
					{
						DbPAValue* pPAValue = (DbPAValue*)m_pDatabase->alloc(RecordType::DB_HAD_PA_VALUE);
						pPAValue->attributeType = (int)DbLaneBoundaryPAType::TRAVERSAL;
						pPAValue->attributeValue = sqlite3_column_int(stmt, i++);

						DbLaneMarkPA* pLaneMarkPA = ppa;
						pLaneMarkPA->paValues.push_back(pPAValue);
						break;
					}
				}
			}
		}
	}

	void DbLaneMarkLoader::loadLaneMarkPA()
	{
		std::string sql = "SELECT PART_PID,LANE_MARK_LINK_PID,S_OFFSET,E_OFFSET,GEOMETRY FROM HAD_LANE_MARK_PART";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();

			int i = 0;
			DbLaneMarkPA* pLaneMarkPa = (DbLaneMarkPA*)m_pDatabase->alloc(RecordType::DB_HAD_LANE_MARK_PA);
			pLaneMarkPa->uuid = sqlite3_column_int64(stmt, i++);
			pLaneMarkPa->relLaneMarkLinkId = sqlite3_column_int64(stmt, i++);
			// pLaneMarkPa->featureType = sqlite3_column_int(stmt, i++);
			pLaneMarkPa->startOffset = sqlite3_column_double(stmt, i++);
			pLaneMarkPa->endOffset = sqlite3_column_double(stmt, i++);

			const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, i);
			int size = sqlite3_column_bytes(stmt, i);

			praseMultiPoint3d(geometry, size, pLaneMarkPa->geometry);
			m_pDatabase->insert(pLaneMarkPa->uuid, pLaneMarkPa);
		}
	}

	void DbLaneMarkLoader::loadLaneMarkPAValue()
	{
		std::string sql = "SELECT FEATURE_PID,FEATURE_CLASS,MARKING_COUNT,MARK_SEQ_NUM,MARK_TYPE,MARK_COLOR,LATERAL_OFFSET FROM HAD_LANE_MARK_MARKING_ATTR";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			
			int i = 0;
			int64 id = sqlite3_column_int64(stmt, i++);
			int64 featureClass = sqlite3_column_int(stmt, i++);
			if (featureClass == 1)
			{
				DbLaneMarkLink* pMarkLink = (DbLaneMarkLink*)m_pDatabase->query(id, RecordType::DB_HAD_LANE_MARK_LINK);
				if (pMarkLink != nullptr)
				{
					pMarkLink->markingCount = sqlite3_column_int(stmt, i++);
					DbLaneMarkLink::DbLaneMarkLinkMarking marking;
					marking.markSeqNum = sqlite3_column_int(stmt, i++);
					marking.markType = sqlite3_column_int(stmt, i++);
					marking.markColor = sqlite3_column_int(stmt, i++);
					marking.lateralOffset = sqlite3_column_int(stmt, i++);
					pMarkLink->markings.push_back(marking);
				}
			}
			else if (featureClass == 2)
			{
				DbLaneMarkPA* pLaneMarkPA = (DbLaneMarkPA*)m_pDatabase->query(id, RecordType::DB_HAD_LANE_MARK_PA);
				if (pLaneMarkPA != nullptr)
				{
					// HAD_LANE_MARK_LINK_MARKING表删除,导致原本PA数据存在的markings没有了,这里需要补上
					DbLaneMarkLink* pMarkLink = (DbLaneMarkLink*)m_pDatabase->query(pLaneMarkPA->relLaneMarkLinkId, RecordType::DB_HAD_LANE_MARK_LINK);
					if (pMarkLink != nullptr && pMarkLink->markings.empty())
					{
						DbLaneMarkLink::DbLaneMarkLinkMarking marking;
						marking.markSeqNum = DB_HAD_APPLY_PA_REFERENCE;
						marking.markType = DB_HAD_APPLY_PA_REFERENCE;
						marking.markColor = DB_HAD_APPLY_PA_REFERENCE;
						marking.lateralOffset = DB_HAD_APPLY_PA_REFERENCE;
						pMarkLink->markings.push_back(marking);
					}

					int markSeqNum = -1;
					std::vector<DbPAValue*> paValues;
					// 如果将来枚举遍历值不是连续的这里需要跟着改动
					for (auto paType = DbLaneBoundaryPAType::MARK_COUNT; paType <= DbLaneBoundaryPAType::LATERAL_OFFSET; paType = (DbLaneBoundaryPAType)(int(paType) + 1))
					{
						if (paType == DbLaneBoundaryPAType::MARK_COUNT || paType == DbLaneBoundaryPAType::MARK_SEQ_NUM || paType == DbLaneBoundaryPAType::MARK_TYPE ||
							paType == DbLaneBoundaryPAType::MARK_COLOR || paType == DbLaneBoundaryPAType::LATERAL_OFFSET) {
							DbPAValue* pPAValue = (DbPAValue*)m_pDatabase->alloc(RecordType::DB_HAD_PA_VALUE);
							pPAValue->attributeType = (int)paType;
							pPAValue->attributeValue = sqlite3_column_int(stmt, i++);
							if (paType == DbLaneBoundaryPAType::MARK_SEQ_NUM) {
								markSeqNum = pPAValue->attributeValue;
							}
							paValues.push_back(pPAValue);
						}
					}
					for (auto pPAValue : paValues) {
						pPAValue->seqNum = markSeqNum;
						pLaneMarkPA->paValues.push_back(pPAValue);
					}
				}
			}
		}
	}

	void DbLaneMarkLoader::loadLaneMarkNode()
	{
		std::string sql = "SELECT LANE_MARK_NODE_PID,GEOMETRY FROM HAD_LANE_MARK_NODE";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();

            DbLaneMarkNode* pLaneMarkNode = (DbLaneMarkNode*)m_pDatabase->alloc(RecordType::DB_HAD_LANE_MARK_NODE);
            pLaneMarkNode->uuid = sqlite3_column_int64(stmt, 0);
            const unsigned char* geometry = (const unsigned char*)sqlite3_column_blob(stmt, 1);
            int size = sqlite3_column_bytes(stmt, 1);
            prasePoint3d(geometry, size, pLaneMarkNode->geometry);
            m_pDatabase->insert(pLaneMarkNode->uuid, pLaneMarkNode);
		}
	}
	void DbLaneMarkLoader::loadLaneMarkRel()
	{
		std::string sql = "SELECT LANE_LINK_PID,LANE_MARK_LINK_PID,SIDE,DIRECT FROM HAD_LANE_MARK_REL";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();

			int64 id = sqlite3_column_int64(stmt, 1);
			DbLaneMarkLink* pMarkLink = (DbLaneMarkLink*)m_pDatabase->query(id, RecordType::DB_HAD_LANE_MARK_LINK);
			if (pMarkLink != nullptr)
			{
				//pMarkLink->relLaneLinkId = sqlite3_column_int64(stmt, 0);
				//pMarkLink->side = sqlite3_column_int(stmt, 2);
				//pMarkLink->direct = sqlite3_column_int(stmt, 3);

				DbLaneMarkLink::DbLaneMarkRel laneMarkRel;
				laneMarkRel.relLaneLinkId = sqlite3_column_int64(stmt, 0);
				laneMarkRel.side = sqlite3_column_int(stmt, 2);
				laneMarkRel.direct = sqlite3_column_int(stmt, 3);
				pMarkLink->laneMarkRels.push_back(laneMarkRel);
			}
		}
	}

	void DbLaneMarkLoader::loadLgMarkRel()
	{
		std::string sql = "SELECT LG_ID,LANE_MARK_LINK_PID,SEQ_NUM,DIRECT FROM HAD_LG_MARK_REL";
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			int64 id = sqlite3_column_int64(stmt, 1);

			DbLaneMarkLink* pMarkLink = (DbLaneMarkLink*)m_pDatabase->query(id,RecordType::DB_HAD_LANE_MARK_LINK);
			if (pMarkLink != nullptr)
			{
				DbLaneMarkLink::DbLgMarkRel relLg;
				int64 relLgLinkId = sqlite3_column_int64(stmt, 0);
				relLg.lgMarkSeqNum = sqlite3_column_int(stmt, 2);
				relLg.lgMarkDirect = sqlite3_column_int(stmt, 3);
				pMarkLink->relLgs.emplace(relLgLinkId, relLg);
			}
		}
	}
}
