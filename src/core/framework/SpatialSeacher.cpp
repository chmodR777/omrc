#include "stdafx.h"
#include "SpatialSeacher.h"
#include <algorithm>
#include "cq_math.h"
#include "geometry/map_point3d64.h"
#include "../om/HadElement.h"
#define HAD_GRID_NDS_LEVEL	13

namespace OMDB
{
    std::vector<HadLaneGroup*> SpatialSeacher::seachNearby2d(HadGrid* pGrid, HadLaneGroup* pGroup, int32 tolerance /*= 0*/)
    {
        std::vector<HadLaneGroup*> v;
        BoundingBox2d input = pGroup->extent.boundingbox2d();
        for (auto& pElement : pGrid->query(ElementType::HAD_LANE_GROUP))
        {
            HadLaneGroup* pLink = (HadLaneGroup*)pElement;
            if (pLink == pGroup)
                continue;

            if (input.overlap(pLink->extent.boundingbox2d(), tolerance) || 
                input.contain(pLink->extent.boundingbox2d(), tolerance) || 
                pLink->extent.boundingbox2d().contain(input, tolerance)
                )
            {
                v.push_back(pLink);
            }
        }

        return v;
    }

	std::vector<HadLaneGroup*> SpatialSeacher::seachNearby2d(const std::vector<HadGrid*>& grids, HadLaneGroup* pGroup, int32 tolerance /*= 0*/)
	{
		std::vector<HadLaneGroup*> v;
		if (grids.empty())
			return v;
		BoundingBox2d extent = pGroup->owner->getBoundingbox2d();

		NdsRect ndsRect = BoundingBox2d::toNdsRect(extent);
		NdsGridIdIterator gridIdIterator;
		NdsGridIdIterator_constructWithNdsRect(&gridIdIterator, ndsRect, HAD_GRID_NDS_LEVEL);
		NdsGridId gridId;
		while ((gridId = NdsGridIdIterator_next(&gridIdIterator)) != invalidNdsGridId)
		{
			int32 meshId = NdsGridId_toMeshId(gridId);
			auto iter = std::find_if(grids.begin(), grids.end(), [&](HadGrid* pGrid)->bool {
				if (pGrid->getId() == meshId)
				{
					return true;
				}
				else
				{
					return false;
				}
				});
			if (iter == grids.end())
				continue;

			HadGrid* grid = *iter;

			if (grid->getBoundingbox2d().overlap(extent, tolerance) ||
				grid->getBoundingbox2d().contain(extent, tolerance)
				)
			{
				std::vector<HadElement*>& groups = grid->query(ElementType::HAD_LANE_GROUP);
				for (auto& e : groups)
				{
					HadLaneGroup* gp = (HadLaneGroup*)e;
					if (gp == pGroup)
						continue;

					if (gp->extent.boundingbox2d().overlap(pGroup->extent.boundingbox2d(), tolerance) ||
						gp->extent.boundingbox2d().contain(pGroup->extent.boundingbox2d(), tolerance) ||
						pGroup->extent.boundingbox2d().contain(gp->extent.boundingbox2d(), tolerance))
					{
						v.push_back(gp);
					}
				}

			}
		}
		return v;
	}
	
	std::vector<HadLaneGroup*> SpatialSeacher::seachNearby(const std::vector<HadGrid*>& grids, const BoundingBox3d& input, int32 tolerance /*= 0*/)
	{
		std::vector<HadLaneGroup*> v;
		if (grids.empty())
			return v;
		BoundingBox2d extent = input.boundingbox2d();
		NdsRect ndsRect = BoundingBox2d::toNdsRect(extent);

		NdsGridIdIterator gridIdIterator;
		NdsGridIdIterator_constructWithNdsRect(&gridIdIterator, ndsRect, HAD_GRID_NDS_LEVEL);
		NdsGridId gridId;
		while ((gridId = NdsGridIdIterator_next(&gridIdIterator)) != invalidNdsGridId)
		{
			int32 meshId = NdsGridId_toMeshId(gridId);
            
            auto iter = std::find_if(grids.begin(), grids.end(), [&](HadGrid* pGrid)->bool {
                if (pGrid->getId() == meshId)
                {
                    return true;
                }
                else
                {
                    return false;
                }
                });
            if (iter == grids.end())
                continue;

            HadGrid* grid = *iter;

			if (grid->getBoundingbox2d().overlap(extent, tolerance) ||
				grid->getBoundingbox2d().contain(extent, tolerance)
				)
			{
				std::vector<HadElement*>& groups = grid->query(ElementType::HAD_LANE_GROUP);
				for (auto& e : groups)
				{
					HadLaneGroup* gp = (HadLaneGroup*)e;
					if (gp->extent.overlap(input, tolerance) ||
						gp->extent.contain(input, tolerance) ||
						input.contain(gp->extent, tolerance))
					{
						v.push_back(gp);
					}
				}

			}
		}
		return v;
	}
	
	std::vector<HadLaneGroup*> SpatialSeacher::seachNearby(const std::vector<HadGrid*>& grids, const BoundingBox2d& input, int32 tolerance /*= 0*/)
	{
		std::vector<HadLaneGroup*> v;
		if (grids.empty())
			return v;
		
		NdsRect ndsRect = BoundingBox2d::toNdsRect(input);

		NdsGridIdIterator gridIdIterator;
		NdsGridIdIterator_constructWithNdsRect(&gridIdIterator, ndsRect, HAD_GRID_NDS_LEVEL);
		NdsGridId gridId;
		while ((gridId = NdsGridIdIterator_next(&gridIdIterator)) != invalidNdsGridId)
		{
			int32 meshId = NdsGridId_toMeshId(gridId);
			auto iter = std::find_if(grids.begin(), grids.end(), [&](HadGrid* pGrid)->bool {
				if (pGrid->getId() == meshId)
				{
					return true;
				}
				else
				{
					return false;
				}
				});
			if (iter == grids.end())
				continue;

			HadGrid* grid = *iter;

			if (grid->getBoundingbox2d().overlap(input, tolerance) ||
				grid->getBoundingbox2d().contain(input, tolerance)
				)
			{
				std::vector<HadElement*>& groups = grid->query(ElementType::HAD_LANE_GROUP);
				for (auto& e : groups)
				{
					HadLaneGroup* gp = (HadLaneGroup*)e;
					if (gp->extent.boundingbox2d().overlap(input, tolerance) ||
						gp->extent.boundingbox2d().contain(input, tolerance) ||
						input.contain(gp->extent.boundingbox2d(), tolerance))
					{
						v.push_back(gp);
					}
				}
			}
		}
		return v;

	}
	
	std::vector<HadLaneGroup*> SpatialSeacher::seachNearby3d(const std::vector<HadGrid*>& grids, HadLaneGroup* pGroup, int32 tolerance /*= 0*/)
	{
		std::vector<HadLaneGroup*> v;
		if (grids.empty())
			return v;
		BoundingBox2d extent = pGroup->owner->getBoundingbox2d();
		NdsRect ndsRect = BoundingBox2d::toNdsRect(extent);

		NdsGridIdIterator gridIdIterator;
		NdsGridIdIterator_constructWithNdsRect(&gridIdIterator, ndsRect, HAD_GRID_NDS_LEVEL);
		NdsGridId gridId;
		while ((gridId = NdsGridIdIterator_next(&gridIdIterator)) != invalidNdsGridId)
		{
			int32 meshId = NdsGridId_toMeshId(gridId);
			auto iter = std::find_if(grids.begin(), grids.end(), [&](HadGrid* pGrid)->bool {
				if (pGrid->getId() == meshId)
				{
					return true;
				}
				else
				{
					return false;
				}
				});
			if (iter == grids.end())
				continue;

			HadGrid* grid = *iter;

			if (grid->getBoundingbox2d().overlap(extent, tolerance) ||
				grid->getBoundingbox2d().contain(extent, tolerance))
			{
				std::vector<HadElement*>& groups = grid->query(ElementType::HAD_LANE_GROUP);
				for (auto& e : groups)
				{
					HadLaneGroup* gp = (HadLaneGroup*)e;
					if (gp == pGroup)
						continue;

					if (gp->extent.overlap(pGroup->extent, tolerance) || gp->extent.contain(pGroup->extent, tolerance) || pGroup->extent.contain(gp->extent, tolerance))
					{
						v.push_back(gp);
					}
				}

			}
		}
		return v;
	}
    
	std::vector<HadLaneGroup*> SpatialSeacher::seachNearby3d(HadGrid* pGrid, HadLaneGroup* pGroup, int32 tolerance /*= 0*/)
    {
        std::vector<HadLaneGroup*> v;
        BoundingBox3d input = pGroup->extent;
        for (auto& pElement : pGrid->query(ElementType::HAD_LANE_GROUP))
        {
            HadLaneGroup* pLink = (HadLaneGroup*)pElement;
            if (pLink == pGroup)
                continue;
            
            if (input.overlap(pLink->extent, tolerance) || 
                input.contain(pLink->extent, tolerance) || 
                pLink->extent.contain(input, tolerance))
            {
                v.push_back(pLink);
            }
        }
        return v;
    }
	
	std::vector<HadLaneGroup*> SpatialSeacher::seachNearby(HadGrid* pGrid, const BoundingBox3d& input, int32 tolerance /*= 0*/)
    {
        std::vector<HadLaneGroup*> v;
        for (auto& pElement : pGrid->query(ElementType::HAD_LANE_GROUP))
        {
            HadLaneGroup* pLink = (HadLaneGroup*)pElement;

            if (input.overlap(pLink->extent,tolerance) || 
                input.contain(pLink->extent,tolerance) || 
                pLink->extent.contain(input,tolerance))
            {
                v.push_back(pLink);
            }
        }

        return v;
    }
    
	std::vector<HadLaneGroup*> SpatialSeacher::seachNearby(HadGrid* pGrid, const BoundingBox2d& input, int32 tolerance /*= 0*/)
    {
        std::vector<HadLaneGroup*> v;
        for (auto& pElement : pGrid->query(ElementType::HAD_LANE_GROUP))
        {
            HadLaneGroup* pLink = (HadLaneGroup*)pElement;

            if (input.overlap(pLink->extent.boundingbox2d(), tolerance) || 
                input.contain(pLink->extent.boundingbox2d(), tolerance) || 
                pLink->extent.boundingbox2d().contain(input, tolerance))
            {
                v.push_back(pLink);
            }
        }
        return v;
    }

}


