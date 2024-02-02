#include "stdafx.h"
#include "DbMesh.h"
#include <mutex>
namespace OMDB
{
	std::vector<DbRecord*> DbMesh::INVALID;
	FastTable<int64, DbRecord*> INVALID_TABLE;

	DbMesh::DbMesh()
	{
		m_batchedAllocator.initWithBlockSize(4096);
	}

#define  FREE_RECORD(objectType,className) \
		case objectType:\
		{\
				((className*)pRecord)->~className(); \
		}\
		break;
	DbMesh::~DbMesh()
	{
		for (auto& pRecord : m_data)
		{
			switch (pRecord->recordType)
			{
					 FREE_RECORD(RecordType::DB_HAD_LINK, DbLink)
					 FREE_RECORD(RecordType::DB_HAD_LINK_PA, DbLinkPA)
					 FREE_RECORD(RecordType::DB_HAD_LINK_NAME, DbLinkName)
					 FREE_RECORD(RecordType::DB_ROAD_NAME, DbRoadName)
					 FREE_RECORD(RecordType::DB_HAD_NODE, DbNode)
					 FREE_RECORD(RecordType::DB_HAD_PA_VALUE, DbPAValue)
					 FREE_RECORD(RecordType::DB_HAD_LINK_FIXED_SPEEDLIMIT, DbLinkSpeedLimit)
					 FREE_RECORD(RecordType::DB_HAD_ROAD_BOUNDARY_LINK, DbRoadBoundLink)
					 FREE_RECORD(RecordType::DB_HAD_ROAD_BOUNDARY_NODE, DbRoadBoundNode)
					 FREE_RECORD(RecordType::DB_HAD_ROAD_BOUNDARY_PA, DbRoadBoundPA)
					 FREE_RECORD(RecordType::DB_HAD_LINK_LANEPA, DbHadLinkLanePa)
					 FREE_RECORD(RecordType::DB_HAD_LG_LINK, DbLgLink)
					 FREE_RECORD(RecordType::DB_HAD_LG_ASSOCIATION, DbLgAssociation)
					 FREE_RECORD(RecordType::DB_HAD_LANE_LINK, DbLaneLink)
					 FREE_RECORD(RecordType::DB_HAD_LANE_NODE, DbLaneNode)
					 FREE_RECORD(RecordType::DB_HAD_LANE_MARK_LINK, DbLaneMarkLink)
					 FREE_RECORD(RecordType::DB_HAD_LANE_MARK_NODE, DbLaneMarkNode)
					 FREE_RECORD(RecordType::DB_HAD_LANE_MARK_PA, DbLaneMarkPA)
					 FREE_RECORD(RecordType::DB_HAD_LANE_FIXED_SPEEDLIMIT, DbFixedSpeedLimit)
					 FREE_RECORD(RecordType::DB_HAD_OBJECT_CROSS_WALK, DbCrossWalk)
					 FREE_RECORD(RecordType::DB_HAD_OBJECT_STOPLOCATION, DbStopLocation)
					 FREE_RECORD(RecordType::DB_HAD_OBJECT_FILL_AREA, DbFillArea)
					 FREE_RECORD(RecordType::DB_HAD_OBJECT_TEXT, DbText)
					 FREE_RECORD(RecordType::DB_HAD_OBJECT_ARROW, DbArrow)
					 FREE_RECORD(RecordType::DB_HAD_OBJECT_WALL, DbWall)
					 FREE_RECORD(RecordType::DB_HAD_OBJECT_TRAFFIC_SIGN, DbTrafficSign)
					 FREE_RECORD(RecordType::DB_HAD_OBJECT_BARRIER, DbBarrier)
					 FREE_RECORD(RecordType::DB_HAD_OBJECT_LG_REL, DbLgRel)
					 FREE_RECORD(RecordType::DB_HAD_OBJECT_LANE_LINK_REL, DbLaneLinkRel)
					 FREE_RECORD(RecordType::DB_HAD_TOLLGATE, DbTollGate)
					 FREE_RECORD(RecordType::DB_HAD_TOLL_LANE, DbTollLane)
					 FREE_RECORD(RecordType::DB_HAD_INTERSECTION, DbIntersection)
					 FREE_RECORD(RecordType::DB_HAD_OBJECT_TRAFFIC_LIGHTS, DbTrafficLights)
					 FREE_RECORD(RecordType::DB_HAD_OBJECT_POLE, DbPole)
					 FREE_RECORD(RecordType::DB_HAD_LANE_LINK_TURNWAITING, DbLaneLinkTurnwaiting)

					 FREE_RECORD(RecordType::DB_HAD_ZLEVEL, DbZLevel)
					 FREE_RECORD(RecordType::DB_HAD_LANEINFO, DbLaneInfo)
					 FREE_RECORD(RecordType::DB_RD_LINK_LANEPA, DbRdLinkLanePa)
					 FREE_RECORD(RecordType::DB_RD_LANE_LINK_CLM, DbRdLaneLinkCLM)
					 FREE_RECORD(RecordType::DB_RD_LANE_TOPO_DETAIL, DbRdLaneTopoDetail)
			}
		}
		m_batchedAllocator.freeAll();
	}
#define  ALLOC_RECORD(elementType,className) \
		case elementType:\
		{\
					void* pData = m_batchedAllocator.allocMemory(sizeof(className)); \
					pRecord = new (pData) className; \
					pRecord->recordType = elementType;\
					m_data.push_back(pRecord);\
		}\
		break;

// #define  ALLOC_RECORD(elementType,className) \
// 		case elementType:\
// 		{\
// 					if(m_mapBatchedAllocator.find(elementType) == m_mapBatchedAllocator.end()) \
// 					{\
// 						m_mapBatchedAllocator[elementType] = BatchedAllocator();\
// 						m_mapBatchedAllocator[elementType].initWithBlockSize(4096);\
// 					}\
// 					void* pData = m_mapBatchedAllocator[elementType].allocMemory(sizeof(className)); \
// 					pRecord = new (pData) className; \
// 					pRecord->recordType = elementType;\
// 					m_data.push_back(pRecord);\
// 		}\
// 		break;

	OMDB::DbRecord* DbMesh::alloc(RecordType objectType)
	{
		DbRecord* pRecord = nullptr;

		switch (objectType)
		{
				ALLOC_RECORD(RecordType::DB_HAD_LINK, DbLink)
				ALLOC_RECORD(RecordType::DB_HAD_LINK_PA, DbLinkPA)
				ALLOC_RECORD(RecordType::DB_HAD_LINK_NAME, DbLinkName)
				ALLOC_RECORD(RecordType::DB_ROAD_NAME, DbRoadName)
				ALLOC_RECORD(RecordType::DB_HAD_NODE, DbNode)
				ALLOC_RECORD(RecordType::DB_HAD_PA_VALUE, DbPAValue)
				ALLOC_RECORD(RecordType::DB_HAD_LINK_FIXED_SPEEDLIMIT, DbLinkSpeedLimit)
				ALLOC_RECORD(RecordType::DB_HAD_ROAD_BOUNDARY_LINK, DbRoadBoundLink)
				ALLOC_RECORD(RecordType::DB_HAD_ROAD_BOUNDARY_NODE, DbRoadBoundNode)
				ALLOC_RECORD(RecordType::DB_HAD_ROAD_BOUNDARY_PA, DbRoadBoundPA)
				ALLOC_RECORD(RecordType::DB_HAD_LINK_LANEPA, DbHadLinkLanePa)
				ALLOC_RECORD(RecordType::DB_HAD_LG_LINK, DbLgLink)
				ALLOC_RECORD(RecordType::DB_HAD_LG_ASSOCIATION, DbLgAssociation)
				ALLOC_RECORD(RecordType::DB_HAD_LANE_LINK, DbLaneLink)
				ALLOC_RECORD(RecordType::DB_HAD_LANE_NODE, DbLaneNode)
				ALLOC_RECORD(RecordType::DB_HAD_LANE_MARK_LINK, DbLaneMarkLink)
				ALLOC_RECORD(RecordType::DB_HAD_LANE_MARK_NODE, DbLaneMarkNode)
				ALLOC_RECORD(RecordType::DB_HAD_LANE_MARK_PA, DbLaneMarkPA)
				ALLOC_RECORD(RecordType::DB_HAD_LANE_FIXED_SPEEDLIMIT, DbFixedSpeedLimit)
				ALLOC_RECORD(RecordType::DB_HAD_OBJECT_CROSS_WALK, DbCrossWalk)
				ALLOC_RECORD(RecordType::DB_HAD_OBJECT_STOPLOCATION, DbStopLocation)
				ALLOC_RECORD(RecordType::DB_HAD_OBJECT_FILL_AREA, DbFillArea)
				ALLOC_RECORD(RecordType::DB_HAD_OBJECT_TEXT, DbText)
				ALLOC_RECORD(RecordType::DB_HAD_OBJECT_ARROW, DbArrow)
				ALLOC_RECORD(RecordType::DB_HAD_OBJECT_WALL, DbWall)
				ALLOC_RECORD(RecordType::DB_HAD_OBJECT_TRAFFIC_SIGN, DbTrafficSign)
				ALLOC_RECORD(RecordType::DB_HAD_OBJECT_BARRIER, DbBarrier)
				ALLOC_RECORD(RecordType::DB_HAD_OBJECT_LG_REL, DbLgRel)
				ALLOC_RECORD(RecordType::DB_HAD_OBJECT_LANE_LINK_REL, DbLaneLinkRel)
				ALLOC_RECORD(RecordType::DB_HAD_TOLLGATE, DbTollGate)
				ALLOC_RECORD(RecordType::DB_HAD_TOLL_LANE, DbTollLane)
				ALLOC_RECORD(RecordType::DB_HAD_INTERSECTION, DbIntersection)
				ALLOC_RECORD(RecordType::DB_HAD_OBJECT_TRAFFIC_LIGHTS, DbTrafficLights)
				ALLOC_RECORD(RecordType::DB_HAD_OBJECT_POLE, DbPole)
                ALLOC_RECORD(RecordType::DB_HAD_LANE_LINK_TURNWAITING, DbLaneLinkTurnwaiting)

				ALLOC_RECORD(RecordType::DB_HAD_ZLEVEL, DbZLevel)
				ALLOC_RECORD(RecordType::DB_HAD_LANEINFO, DbLaneInfo)
				ALLOC_RECORD(RecordType::DB_RD_LINK_LANEPA, DbRdLinkLanePa)
				ALLOC_RECORD(RecordType::DB_RD_LANE_LINK_CLM, DbRdLaneLinkCLM)
				ALLOC_RECORD(RecordType::DB_RD_LANE_TOPO_DETAIL, DbRdLaneTopoDetail)
		}
		return pRecord;
	}

	void DbMesh::insert(int64 id, DbRecord* record)
	{
		if (438130976007165448 == id)
			printf("");
		record->owner = this;
		auto iter = m_tbTable.find(record->recordType);
		if (iter == m_tbTable.end())
		{
			FastTable<int64, DbRecord*> v;
			v.insert(id, record);
			m_tbTable.emplace(record->recordType, v);
		}
		else
		{
			m_tbTable[record->recordType].insert(id, record);
		}
	}

	DbRecord* DbMesh::query(int64 id, RecordType recordType)
	{
		DbRecord* pRecord = nullptr;
        auto iter = m_tbTable.find(recordType);
        if (iter != m_tbTable.end())
        {
			if (iter->second.exist(id))
			{
				pRecord = iter->second[id];
			}
        }
		return pRecord;
	}

	std::vector<DbRecord*>& DbMesh::query(RecordType recordType)
	{
		auto iter = m_tbTable.find(recordType);
		if (iter == m_tbTable.end())
			return INVALID;

		return iter->second.values();
	}

	bool DbMesh::erase(int64 id, RecordType recordType)
	{
		auto iter = m_tbTable.find(recordType);
		if (iter == m_tbTable.end())
			return false;

		return iter->second.erase(id);
	}

	FastTable<int64, DbRecord*>& DbMesh::queryTable(RecordType recordType)
	{
		auto iter = m_tbTable.find(recordType);
		if (iter == m_tbTable.end())
			return INVALID_TABLE;

		return iter->second;
	}

	void DbMesh::setId(int32 id)
	{
		m_meshId = id;
	}

	int32 DbMesh::getId()
	{
		return m_meshId;
	}

	void DbMesh::insertLinkInfo(dbLinkInfo& info)
	{
		m_links.push_back(info);
		m_linkBoxes.push_back(info._linkBox2T);
	}
	std::vector<dbLinkInfo>& DbMesh::getLinkInfos()
	{
		return m_links;
	}
	std::shared_ptr<rtree_type_2box> DbMesh::getLinkRtree()
	{
		auto& tmp = m_linkRtree;
		if (tmp || m_linkBoxes.empty())
			return tmp;

		parameters param;
		index_getter_2box originIndBox(m_linkBoxes);
		m_linkRtree = std::make_shared<rtree_type_2box>(boost::irange<std::size_t>(0lu, m_linkBoxes.size()), param, originIndBox);
		return m_linkRtree;
	}
}
