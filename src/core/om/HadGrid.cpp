#include "stdafx.h"
#include "HadGrid.h"
#include "cq_math.h"
namespace OMDB
{

	std::vector<HadElement*> HadGrid::INVALID;

	OMDB::HadElement* HadGrid::query(int64 id, ElementType objectType)
	{
        HadElement* pElement = nullptr;
        auto iter = m_tbTable.find(objectType);
        if (iter != m_tbTable.end())
        {
            if (iter->second.exist(id))
            {
                pElement = iter->second[id];
            }
        }

        return pElement;
	}

	std::vector<HadElement*>& HadGrid::query(ElementType objectType)
	{
        auto iter = m_tbTable.find(objectType);
        if (iter == m_tbTable.end())
            return INVALID;

        return iter->second.values();

	}

	void HadGrid::setId(int32 id)
	{
		m_meshId = id;
		uint32 ndsId = MeshId_toNdsGridId(id);
		Rect rect;
		NdsGridId_getRect(ndsId, &rect);

		Point min02;
		Point min84{ rect.left,rect.top };
		Math_wgsToMars(&min84, &min02);

		Point max02;
		Point max84{ rect.right,rect.bottom };
		Math_wgsToMars(&max84, &max02);

		m_extent.min.lat = (int64)min02.y * 1000;
		m_extent.min.lon = (int64)min02.x * 1000;
		m_extent.max.lat = (int64)max02.y * 1000;
		m_extent.max.lon = (int64)max02.x * 1000;
	}

	int32 HadGrid::getId() const
	{
		return m_meshId;
	}

	HadGrid::HadGrid()
	{
		m_batchedAllocator.initWithBlockSize(4096);
	}
#define  FREE_ELEMENT(elementType,className) \
		case elementType:\
		{\
			((className*)pElement)->~className(); \
		}\
		break;

	HadGrid::~HadGrid()
	{
		for (auto& pElement : m_data)
		{
			switch (pElement->objectType)
			{
				FREE_ELEMENT(ElementType::HAD_LINK, HadLink)
				FREE_ELEMENT(ElementType::HAD_NODE, HadNode)
				FREE_ELEMENT(ElementType::HAD_ROAD_BOUNDARY, HadRoadBoundary)
				FREE_ELEMENT(ElementType::HAD_ROAD_BOUNDARY_NODE, HadRoadBoundaryNode)
				FREE_ELEMENT(ElementType::HAD_LANE, HadLane)
				FREE_ELEMENT(ElementType::HAD_LANE_NODE, HadLaneNode)
				FREE_ELEMENT(ElementType::HAD_LANE_BOUNDARY, HadLaneBoundary)
				FREE_ELEMENT(ElementType::HAD_LANE_BOUNDARY_NODE, HadLaneBoundaryNode)
				FREE_ELEMENT(ElementType::HAD_LANE_GROUP, HadLaneGroup)
				FREE_ELEMENT(ElementType::HAD_LG_ASSOCIATIOIN, HadLaneGroupAssociation)
				FREE_ELEMENT(ElementType::HAD_PART_ATTRIBUTE, HadPartAttribute)
				FREE_ELEMENT(ElementType::HAD_OBJECT_CROSS_WALK, HadCrossWalk)
				FREE_ELEMENT(ElementType::HAD_OBJECT_STOPLOCATION, HadStopLocation)
				FREE_ELEMENT(ElementType::HAD_OBJECT_FILL_AREA, HadFillArea)
				FREE_ELEMENT(ElementType::HAD_OBJECT_TEXT, HadText)
				FREE_ELEMENT(ElementType::HAD_OBJECT_ARROW, HadArrow)
				FREE_ELEMENT(ElementType::HAD_OBJECT_WALL, HadWall)
				FREE_ELEMENT(ElementType::HAD_OBJECT_TRAFFIC_SIGN, HadTrafficSign)
				FREE_ELEMENT(ElementType::HAD_OBJECT_BARRIER, HadBarrier)
				FREE_ELEMENT(ElementType::HAD_INTERSECTION, HadIntersection)
				FREE_ELEMENT(ElementType::HAD_TOLLGATE, HadTollGate)
				FREE_ELEMENT(ElementType::HAD_OBJECT_TRAFFIC_LIGHTS, HadTrafficLights)
				FREE_ELEMENT(ElementType::HAD_OBJECT_POLE, HadPole)
				FREE_ELEMENT(ElementType::HAD_LANE_TURNWAITING, HadLaneTurnwaiting)
				FREE_ELEMENT(ElementType::HAD_OBJECT_SPEED_BUMP, HadSpeedBump)
			}
		}
		m_data.clear();
		m_batchedAllocator.freeAll();
	}

#define  ALLOC_ELEMENT(elementType,className) \
		case elementType:\
		{\
					void* pData = m_batchedAllocator.allocMemory(sizeof(className)); \
					pElement = new (pData) className; \
					pElement->objectType = elementType;\
					m_data.push_back(pElement);\
		}\
		break;

	OMDB::HadElement* HadGrid::alloc(ElementType objectType)
	{
		HadElement* pElement = nullptr;
		switch (objectType)
		{
			ALLOC_ELEMENT(ElementType::HAD_LINK,HadLink)
			ALLOC_ELEMENT(ElementType::HAD_NODE, HadNode)
			ALLOC_ELEMENT(ElementType::HAD_ROAD_BOUNDARY, HadRoadBoundary)
			ALLOC_ELEMENT(ElementType::HAD_ROAD_BOUNDARY_NODE, HadRoadBoundaryNode)
			ALLOC_ELEMENT(ElementType::HAD_LANE, HadLane)
			ALLOC_ELEMENT(ElementType::HAD_LANE_NODE, HadLaneNode)
			ALLOC_ELEMENT(ElementType::HAD_LANE_BOUNDARY, HadLaneBoundary)
			ALLOC_ELEMENT(ElementType::HAD_LANE_BOUNDARY_NODE, HadLaneBoundaryNode)
			ALLOC_ELEMENT(ElementType::HAD_LANE_GROUP, HadLaneGroup)
			ALLOC_ELEMENT(ElementType::HAD_LG_ASSOCIATIOIN, HadLaneGroupAssociation)
			ALLOC_ELEMENT(ElementType::HAD_PART_ATTRIBUTE, HadPartAttribute)
			ALLOC_ELEMENT(ElementType::HAD_OBJECT_CROSS_WALK, HadCrossWalk)
			ALLOC_ELEMENT(ElementType::HAD_OBJECT_STOPLOCATION, HadStopLocation)
			ALLOC_ELEMENT(ElementType::HAD_OBJECT_FILL_AREA, HadFillArea)
			ALLOC_ELEMENT(ElementType::HAD_OBJECT_TEXT, HadText)
			ALLOC_ELEMENT(ElementType::HAD_OBJECT_ARROW, HadArrow)
			ALLOC_ELEMENT(ElementType::HAD_OBJECT_WALL, HadWall)
			ALLOC_ELEMENT(ElementType::HAD_OBJECT_TRAFFIC_SIGN, HadTrafficSign)
			ALLOC_ELEMENT(ElementType::HAD_OBJECT_BARRIER, HadBarrier)
			ALLOC_ELEMENT(ElementType::HAD_INTERSECTION, HadIntersection)
			ALLOC_ELEMENT(ElementType::HAD_TOLLGATE, HadTollGate)
			ALLOC_ELEMENT(ElementType::HAD_OBJECT_TRAFFIC_LIGHTS, HadTrafficLights)
			ALLOC_ELEMENT(ElementType::HAD_OBJECT_POLE, HadPole)
			ALLOC_ELEMENT(ElementType::HAD_LANE_TURNWAITING, HadLaneTurnwaiting)
			ALLOC_ELEMENT(ElementType::HAD_OBJECT_SPEED_BUMP, HadSpeedBump)
		}

		return pElement;
	}

	void HadGrid::insert(int64 id, HadElement* pObject)
	{
        pObject->owner = this;
        auto iter = m_tbTable.find(pObject->objectType);
        if (iter == m_tbTable.end())
        {
            FastTable < int64, HadElement* > v;
            v.insert(id, pObject);
            m_tbTable.emplace(pObject->objectType, v);
        }
        else
        {
            m_tbTable[pObject->objectType].insert(id, pObject);
        }

	}

}
