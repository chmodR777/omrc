#include "stdafx.h"
#include "ObjectProcessor.h"
#include "algorithm/grap_point_algorithm.h"
namespace OMDB
{

    void ObjectProcessor::process(DbMesh* const pMesh, HadGrid* pGrid)
    {
		// arrow
		processArrow(pMesh, pGrid);

		// fillArea
		processFillArea(pMesh, pGrid);

		// crossWalk
		processCrossWalk(pMesh, pGrid);

		// trafficSign
		processTrafficSign(pMesh, pGrid);

		// stopLocation
		processStopLocation(pMesh, pGrid);

		// text
		processText(pMesh, pGrid);

		// barrier
		processBarrier(pMesh, pGrid);

		// wall
		processWall(pMesh, pGrid);

		// trafficLights
		processTrafficLights(pMesh, pGrid);

		// pole
		processPole(pMesh, pGrid);

		//speed bump
		processSpeedBump(pMesh, pGrid);
    }

	void ObjectProcessor::processRelation(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby)
	{
		// arrow
		processArrow(pMesh, pGrid, nearby);

		// fillArea
		processFillArea(pMesh, pGrid, nearby);

		// crossWalk
		processCrossWalk(pMesh, pGrid, nearby);

		// trafficSign
		processTrafficSign(pMesh, pGrid, nearby);

		// stopLocation
		processStopLocation(pMesh, pGrid, nearby);

		// text
		processText(pMesh, pGrid, nearby);

		// barrier
		processBarrier(pMesh, pGrid, nearby);

		// wall
		processWall(pMesh, pGrid, nearby);

		// trafficLights
		processTrafficLights(pMesh, pGrid, nearby);

		// pole
		processPole(pMesh, pGrid, nearby);

		// speed bump
		processSpeedBump(pMesh, pGrid, nearby);
	}


	void ObjectProcessor::processArrow(DbMesh* pMesh, HadGrid* pGrid)
	{
		std::vector<DbRecord*>& arrows = pMesh->query(RecordType::DB_HAD_OBJECT_ARROW);
		for (auto ar : arrows)
		{
			DbArrow* par = (DbArrow*)ar;
			HadArrow* pArrow = (HadArrow*)pGrid->alloc(ElementType::HAD_OBJECT_ARROW);
			pArrow->postion = par->center;
			pArrow->originId = par->uuid;
			pArrow->length = par->length;
			pArrow->width = par->width;
			pArrow->color = (Color)par->color;
			pArrow->polygon = par->geometry;
			pArrow->arrowClass = (HadArrow::ArrowType)par->arrowClass;
			if (pArrow->arrowClass == HadArrow::ArrowType::UNKNOWN) {
				pArrow->arrowClass = HadArrow::ArrowType::STRAIGHT;
			}
			pArrow->direction = -1; // TODO unknow

			// arrow -> lane -> linkgroup
			DbLaneLinkRel* pLaneLinkRel = (DbLaneLinkRel*)pMesh->query(par->uuid, RecordType::DB_HAD_OBJECT_LANE_LINK_REL);
			if (pLaneLinkRel != nullptr) {
				for (auto& relLaneLink : pLaneLinkRel->relLaneLinkIds)
				{
					HadLane* refLane = (HadLane*)pGrid->query(relLaneLink.first, ElementType::HAD_LANE);
					if (refLane == nullptr)
						continue;

					pArrow->refLanes.push_back(refLane);
					refLane->objects.push_back(pArrow);
					if (refLane->linkGroup != nullptr)
					{
						refLane->linkGroup->objects.push_back(pArrow);
						pArrow->laneGroups.push_back(refLane->linkGroup);
					}
				}
			}

			pGrid->insert(pArrow->originId, pArrow);
		}
	}

	void ObjectProcessor::processArrow(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby)
    {
		std::vector<DbRecord*>& arrows = pMesh->query(RecordType::DB_HAD_OBJECT_ARROW);
		for (auto ar : arrows)
		{
			DbArrow* par = (DbArrow*)ar;
			HadArrow* pArrow = (HadArrow*)pGrid->query(par->uuid, ElementType::HAD_OBJECT_ARROW);
			if (pArrow == nullptr) {
				continue;
			}

			// arrow -> lane -> linkgroup
			DbLaneLinkRel* pLaneLinkRel = (DbLaneLinkRel*)pMesh->query(par->uuid, RecordType::DB_HAD_OBJECT_LANE_LINK_REL);
			if (pLaneLinkRel != nullptr) {
                for (auto& relLaneLink : pLaneLinkRel->relLaneLinkIds)
                {
                    HadLane* refLane = (HadLane*)queryNearby(pGrid, nearby, relLaneLink.first, ElementType::HAD_LANE);
                    if (refLane == nullptr)
                        continue;

                    pArrow->refLanes.push_back(refLane);
					refLane->objects.push_back(pArrow);
                    if (refLane->linkGroup != nullptr)
                    {
                        refLane->linkGroup->objects.push_back(pArrow);
                        pArrow->laneGroups.push_back(refLane->linkGroup);
                    }
                }
			}
		}
    }


	void ObjectProcessor::processFillArea(DbMesh* pMesh, HadGrid* pGrid)
	{
		std::vector<DbRecord*>& fillAreas = pMesh->query(RecordType::DB_HAD_OBJECT_FILL_AREA);
		for (auto fa : fillAreas)
		{
			DbFillArea* pfa = (DbFillArea*)fa;
			HadFillArea* pFillArea = (HadFillArea*)pGrid->alloc(ElementType::HAD_OBJECT_FILL_AREA);
			pFillArea->originId = pfa->uuid;
			pFillArea->polygon = pfa->geometry;

			// fillArea -> linkgroup
			DbLgRel* pLgRel = (DbLgRel*)pMesh->query(pfa->uuid, RecordType::DB_HAD_OBJECT_LG_REL);
			if (pLgRel != nullptr) {
				for (auto& relLg : pLgRel->relLgs) {
					HadLaneGroup* pGroup = (HadLaneGroup*)pGrid->query(relLg.first, ElementType::HAD_LANE_GROUP);
					if (pGroup != nullptr)
					{
						pGroup->objects.push_back(pFillArea);
						pFillArea->laneGroups.push_back(pGroup);
					}
				}
			}

			pGrid->insert(pFillArea->originId, pFillArea);
		}
	}

	void ObjectProcessor::processFillArea(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby)
    {
		std::vector<DbRecord*>& fillAreas = pMesh->query(RecordType::DB_HAD_OBJECT_FILL_AREA);
		for (auto fa : fillAreas)
		{
			DbFillArea* pfa = (DbFillArea*)fa;
			HadFillArea* pFillArea = (HadFillArea*)pGrid->query(pfa->uuid, ElementType::HAD_OBJECT_FILL_AREA);
			if (pFillArea == nullptr) {
				continue;
			}

			// fillArea -> linkgroup
			DbLgRel* pLgRel = (DbLgRel*)pMesh->query(pfa->uuid, RecordType::DB_HAD_OBJECT_LG_REL);
			if (pLgRel != nullptr) {
				for (auto& relLg : pLgRel->relLgs) {
					HadLaneGroup* pGroup = (HadLaneGroup*)queryNearby(pGrid, nearby, relLg.first, ElementType::HAD_LANE_GROUP);
					if (pGroup != nullptr)
					{
						pGroup->objects.push_back(pFillArea);
						pFillArea->laneGroups.push_back(pGroup);
					}
				}
			}

		}
    }

	void ObjectProcessor::processCrossWalk(DbMesh* pMesh, HadGrid* pGrid)
	{
		std::vector<DbRecord*>& crossWalks = pMesh->query(RecordType::DB_HAD_OBJECT_CROSS_WALK);
		for (auto cw : crossWalks)
		{
			DbCrossWalk* pcw = (DbCrossWalk*)cw;
			HadCrossWalk* pCrossWalk = (HadCrossWalk*)pGrid->alloc(ElementType::HAD_OBJECT_CROSS_WALK);
			pCrossWalk->originId = pcw->uuid;
			pCrossWalk->color = (Color)pcw->color;
			pCrossWalk->polygon = pcw->geometry;

			// crossWalk -> linkgroup
			DbLgRel* pLgRel = (DbLgRel*)pMesh->query(pcw->uuid, RecordType::DB_HAD_OBJECT_LG_REL);
			if (pLgRel != nullptr) {
				for (auto& relLg : pLgRel->relLgs) {
					HadLaneGroup* pGroup = (HadLaneGroup*)pGrid->query(relLg.first, ElementType::HAD_LANE_GROUP);
					if (pGroup != nullptr)
					{
						pGroup->objects.push_back(pCrossWalk);
						pCrossWalk->laneGroups.push_back(pGroup);
					}
				}
			}

			pGrid->insert(pCrossWalk->originId, pCrossWalk);
		}
	}

	void ObjectProcessor::processCrossWalk(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby)
    {
        std::vector<DbRecord*>& crossWalks = pMesh->query(RecordType::DB_HAD_OBJECT_CROSS_WALK);
        for (auto cw : crossWalks)
        {
            DbCrossWalk* pcw = (DbCrossWalk*)cw;
            HadCrossWalk* pCrossWalk = (HadCrossWalk*)pGrid->query(pcw->uuid, ElementType::HAD_OBJECT_CROSS_WALK);
			if (pCrossWalk == nullptr) {
				continue;
			}

            // crossWalk -> linkgroup
            DbLgRel* pLgRel = (DbLgRel*)pMesh->query(pcw->uuid, RecordType::DB_HAD_OBJECT_LG_REL);
            if (pLgRel != nullptr) {
                for (auto& relLg : pLgRel->relLgs) {
                    HadLaneGroup* pGroup = (HadLaneGroup*)queryNearby(pGrid, nearby, relLg.first, ElementType::HAD_LANE_GROUP);
                    if (pGroup != nullptr)
                    {
                        pGroup->objects.push_back(pCrossWalk);
                        pCrossWalk->laneGroups.push_back(pGroup);
                    }
                }
            }

        }
    }

	void ObjectProcessor::processTrafficSign(DbMesh* pMesh, HadGrid* pGrid)
	{
		std::vector<DbRecord*>& signs = pMesh->query(RecordType::DB_HAD_OBJECT_TRAFFIC_SIGN);
		for (auto si : signs)
		{
			DbTrafficSign* psi = (DbTrafficSign*)si;
			HadTrafficSign* pSign = (HadTrafficSign*)pGrid->alloc(ElementType::HAD_OBJECT_TRAFFIC_SIGN);
			pSign->originId = psi->uuid;
			pSign->shapeType = (HadTrafficSign::ShapeType)psi->trafSignShape;

			int signType = psi->signType;
			int kind1 = signType / 100;
			if (kind1 < 2 || kind1 > 4)
				continue;

			int kind2 = signType % 100;
			if (kind2 <= 1)
				continue;
			pSign->speed = (kind2-1) * 5;	//每个小类内容5公里递增
			//pSign->content = std::to_string(speed);

			pSign->signType = HadTrafficSign::SignType(kind1);
			pSign->color = (Color)psi->color;
			pSign->polygon = psi->geometry;

			pSign->centerPt = { 0 };
			pSign->grapPt = { 0 };

			// sign -> linkgroup
			DbLgRel* pLgRel = (DbLgRel*)pMesh->query(psi->uuid, RecordType::DB_HAD_OBJECT_LG_REL);
			if (pLgRel != nullptr) {
				for (auto& relLg : pLgRel->relLgs) {
					HadLaneGroup* pGroup = (HadLaneGroup*)pGrid->query(relLg.first, ElementType::HAD_LANE_GROUP);
					if (pGroup != nullptr)
					{
						pGroup->objects.push_back(pSign);
						pSign->laneGroups.push_back(pGroup);
					}
				}
			}

			pGrid->insert(pSign->originId, pSign);
		}
	}

	void ObjectProcessor::processTrafficSign(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby)
    {
        std::vector<DbRecord*>& signs = pMesh->query(RecordType::DB_HAD_OBJECT_TRAFFIC_SIGN);
        for (auto si : signs)
        {
            DbTrafficSign* psi = (DbTrafficSign*)si;
            HadTrafficSign* pSign = (HadTrafficSign*)pGrid->query(psi->uuid, ElementType::HAD_OBJECT_TRAFFIC_SIGN);
            if (pSign == nullptr) {
                continue;
            }

			int signType = psi->signType;
			int kind1 = signType / 100;
			if (kind1 < 2 || kind1 > 4)
			    continue;

			int kind2 = signType % 100;
			if (kind2 <= 1)
				continue;
			pSign->speed = (kind2-1) * 5;	//每个小类内容5公里递增
            //pSign->content = std::to_string(speed);

			pSign->signType = HadTrafficSign::SignType(kind1);
            pSign->color = (Color)psi->color;
            pSign->polygon = psi->geometry;

			pSign->centerPt = { 0 };
			pSign->grapPt = { 0 };

            // sign -> linkgroup
            DbLgRel* pLgRel = (DbLgRel*)pMesh->query(psi->uuid, RecordType::DB_HAD_OBJECT_LG_REL);
            if (pLgRel != nullptr) {
                for (auto& relLg : pLgRel->relLgs) {
                    HadLaneGroup* pGroup = (HadLaneGroup*)queryNearby(pGrid, nearby, relLg.first, ElementType::HAD_LANE_GROUP);
                    if (pGroup != nullptr)
                    {
                        pGroup->objects.push_back(pSign);
                        pSign->laneGroups.push_back(pGroup);
                    }
                }
            }

        }
    }

	void ObjectProcessor::processStopLocation(DbMesh* pMesh, HadGrid* pGrid)
	{
		std::vector<DbRecord*>& stopLocations = pMesh->query(RecordType::DB_HAD_OBJECT_STOPLOCATION);
		for (auto sl : stopLocations)
		{
			DbStopLocation* psl = (DbStopLocation*)sl;
			HadStopLocation* pStopLocation = (HadStopLocation*)pGrid->alloc(ElementType::HAD_OBJECT_STOPLOCATION);
			pStopLocation->originId = psl->uuid;
			pStopLocation->width = psl->width;
			pStopLocation->color = psl->color;
			pStopLocation->locationType = psl->locationType;
			pStopLocation->location = psl->geometry;

			// stopLocation -> linkgroup
			DbLgRel* pLgRel = (DbLgRel*)pMesh->query(psl->uuid, RecordType::DB_HAD_OBJECT_LG_REL);
			if (pLgRel != nullptr) {
				for (auto& relLg : pLgRel->relLgs) {
					HadLaneGroup* pGroup = (HadLaneGroup*)pGrid->query(relLg.first, ElementType::HAD_LANE_GROUP);
					if (pGroup != nullptr)
					{
						pGroup->objects.push_back(pStopLocation);
						pStopLocation->laneGroups.push_back(pGroup);
					}
				}
			} else {
				// stopLocation -> lane -> linkgroup
				DbLaneLinkRel* pLaneLinkRel = (DbLaneLinkRel*)pMesh->query(psl->uuid, RecordType::DB_HAD_OBJECT_LANE_LINK_REL);
				if (pLaneLinkRel != nullptr) {
					for (auto& relLaneLink : pLaneLinkRel->relLaneLinkIds)
					{
						HadLane* refLane = (HadLane*)pGrid->query(relLaneLink.first, ElementType::HAD_LANE);
						if (refLane == nullptr)
							continue;

						pStopLocation->refLanes.push_back(refLane);
						refLane->objects.push_back(pStopLocation);
						if (refLane->linkGroup != nullptr)
						{
							refLane->linkGroup->objects.push_back(pStopLocation);
							pStopLocation->laneGroups.push_back(refLane->linkGroup);
						}
					}
				}
			}

			pGrid->insert(pStopLocation->originId, pStopLocation);
		}
	}

	void ObjectProcessor::processStopLocation(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby)
    {
        std::vector<DbRecord*>& stopLocations = pMesh->query(RecordType::DB_HAD_OBJECT_STOPLOCATION);
        for (auto sl : stopLocations)
        {
            DbStopLocation* psl = (DbStopLocation*)sl;
            HadStopLocation* pStopLocation = (HadStopLocation*)pGrid->query(psl->uuid, ElementType::HAD_OBJECT_STOPLOCATION);
			if (pStopLocation == nullptr) {
				continue;
			}

            // stopLocation -> linkgroup
            DbLgRel* pLgRel = (DbLgRel*)pMesh->query(psl->uuid, RecordType::DB_HAD_OBJECT_LG_REL);
            if (pLgRel != nullptr) {
                for (auto& relLg : pLgRel->relLgs) {
                    HadLaneGroup* pGroup = (HadLaneGroup*)queryNearby(pGrid, nearby, relLg.first, ElementType::HAD_LANE_GROUP);
                    if (pGroup != nullptr)
                    {
                        pGroup->objects.push_back(pStopLocation);
                        pStopLocation->laneGroups.push_back(pGroup);
                    }
                }
			} else {
				// stopLocation -> lane -> linkgroup
				DbLaneLinkRel* pLaneLinkRel = (DbLaneLinkRel*)pMesh->query(psl->uuid, RecordType::DB_HAD_OBJECT_LANE_LINK_REL);
				if (pLaneLinkRel != nullptr) {
					for (auto& relLaneLink : pLaneLinkRel->relLaneLinkIds)
					{
						HadLane* refLane = (HadLane*)queryNearby(pGrid, nearby, relLaneLink.first, ElementType::HAD_LANE);
						if (refLane == nullptr)
							continue;

						pStopLocation->refLanes.push_back(refLane);
						refLane->objects.push_back(pStopLocation);
						if (refLane->linkGroup != nullptr)
						{
							refLane->linkGroup->objects.push_back(pStopLocation);
							pStopLocation->laneGroups.push_back(refLane->linkGroup);
						}
					}
				}
			}

        }
    }

	void ObjectProcessor::processText(DbMesh* pMesh, HadGrid* pGrid)
	{
		std::vector<DbRecord*>& texts = pMesh->query(RecordType::DB_HAD_OBJECT_TEXT);
		for (auto text : texts)
		{
			DbText* pText = (DbText*)text;
			HadText* hadText = (HadText*)pGrid->alloc(ElementType::HAD_OBJECT_TEXT);
			hadText->postion = pText->center;
			hadText->originId = pText->uuid;
			hadText->length = pText->length;
			hadText->width = pText->width;
			hadText->color = (Color)pText->color;
			hadText->polygon = pText->geometry;
			hadText->content = pText->textContent; // #TODO         
			hadText->speedLimitRangeText = nullptr;
			hadText->nextText = nullptr;
			hadText->previousText = nullptr;

			// text -> lane -> linkgroup
			DbLaneLinkRel* pLaneLinkRel = (DbLaneLinkRel*)pMesh->query(pText->uuid, RecordType::DB_HAD_OBJECT_LANE_LINK_REL);
			if (pLaneLinkRel != nullptr)
			{
				for (auto& relLaneLink : pLaneLinkRel->relLaneLinkIds)
				{
					HadLane* refLane = (HadLane*)pGrid->query(relLaneLink.first, ElementType::HAD_LANE);
					if (refLane == nullptr)
						continue;

					hadText->refLanes.push_back(refLane);
					refLane->objects.push_back(hadText);
					if (refLane->linkGroup != nullptr)
					{
						refLane->linkGroup->objects.push_back(hadText);
						hadText->laneGroups.push_back(refLane->linkGroup);
					}
				}
			}

			pGrid->insert(hadText->originId, hadText);
		}
	}

	void ObjectProcessor::processText(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby)
    {
		std::vector<DbRecord*>& texts = pMesh->query(RecordType::DB_HAD_OBJECT_TEXT);
		for (auto text : texts)
		{
			DbText* pText = (DbText*)text;
			HadText* hadText = (HadText*)pGrid->query(pText->uuid, ElementType::HAD_OBJECT_TEXT);
			if (hadText == nullptr) {
				continue;
			}

			// text -> lane -> linkgroup
			DbLaneLinkRel* pLaneLinkRel = (DbLaneLinkRel*)pMesh->query(pText->uuid, RecordType::DB_HAD_OBJECT_LANE_LINK_REL);
			if (pLaneLinkRel != nullptr)
			{
				for (auto& relLaneLink : pLaneLinkRel->relLaneLinkIds)
				{
					HadLane* refLane = (HadLane*)queryNearby(pGrid, nearby, relLaneLink.first, ElementType::HAD_LANE);
				    if (refLane == nullptr)
					    continue;

					hadText->refLanes.push_back(refLane);
					refLane->objects.push_back(hadText);
					if (refLane->linkGroup != nullptr)
					{
						refLane->linkGroup->objects.push_back(hadText);
						hadText->laneGroups.push_back(refLane->linkGroup);
					}
				}
			}

		}
    }


	void ObjectProcessor::processBarrier(DbMesh* pMesh, HadGrid* pGrid)
	{
		std::vector<DbRecord*>& barriers = pMesh->query(RecordType::DB_HAD_OBJECT_BARRIER);
		for (auto barrier : barriers)
		{
			DbBarrier* dbBarrier = (DbBarrier*)barrier;
			HadBarrier* hadBarrier = (HadBarrier*)pGrid->alloc(ElementType::HAD_OBJECT_BARRIER);
			hadBarrier->originId = dbBarrier->uuid;
			hadBarrier->barrierType = HadBarrier::BarrierType(dbBarrier->barrierType);
			hadBarrier->location = dbBarrier->geometry;

			// barrier -> linkgroup
			DbLgRel* pLgRel = (DbLgRel*)pMesh->query(dbBarrier->uuid, RecordType::DB_HAD_OBJECT_LG_REL);
			if (pLgRel != nullptr)
			{
				for (auto& relLg : pLgRel->relLgs)
				{
					HadLaneGroup* pGroup = (HadLaneGroup*)pGrid->query(relLg.first, ElementType::HAD_LANE_GROUP);
					if (pGroup != nullptr)
					{
						pGroup->objects.push_back(hadBarrier);
						hadBarrier->laneGroups.push_back(pGroup);
					}
				}
			}
			pGrid->insert(hadBarrier->originId, hadBarrier);
		}
	}

	void ObjectProcessor::processBarrier(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby)
    {
		std::vector<DbRecord*>& barriers = pMesh->query(RecordType::DB_HAD_OBJECT_BARRIER);
		for (auto barrier : barriers)
		{
			DbBarrier* dbBarrier = (DbBarrier*)barrier;
			HadBarrier* hadBarrier = (HadBarrier*)pGrid->query(dbBarrier->uuid, ElementType::HAD_OBJECT_BARRIER);
			if (hadBarrier == nullptr) {
				continue;
			}

			// barrier -> linkgroup
			DbLgRel* pLgRel = (DbLgRel*)pMesh->query(dbBarrier->uuid, RecordType::DB_HAD_OBJECT_LG_REL);
			if (pLgRel != nullptr)
			{
				for (auto& relLg : pLgRel->relLgs)
				{
					HadLaneGroup* pGroup = (HadLaneGroup*)queryNearby(pGrid, nearby, relLg.first, ElementType::HAD_LANE_GROUP);
					if (pGroup != nullptr)
					{
						pGroup->objects.push_back(hadBarrier);
						hadBarrier->laneGroups.push_back(pGroup);
					}
				}
			}
		}
    }

	void ObjectProcessor::processWall(DbMesh* pMesh, HadGrid* pGrid)
	{
		std::vector<DbRecord*>& walls = pMesh->query(RecordType::DB_HAD_OBJECT_WALL);
		for (auto wall : walls)
		{
			DbWall* dbWall = (DbWall*)wall;
			HadWall* hadWall = (HadWall*)pGrid->alloc(ElementType::HAD_OBJECT_WALL);
			hadWall->originId = dbWall->uuid;
			hadWall->wallType = HadWall::WallType(dbWall->wallType);
			hadWall->location = dbWall->geometry;

			// wall -> linkgroup
			DbLgRel* pLgRel = (DbLgRel*)pMesh->query(dbWall->uuid, RecordType::DB_HAD_OBJECT_LG_REL);
			if (pLgRel != nullptr)
			{
				for (auto& relLg : pLgRel->relLgs)
				{
					HadLaneGroup* pGroup = (HadLaneGroup*)pGrid->query(relLg.first, ElementType::HAD_LANE_GROUP);
					if (pGroup != nullptr)
					{
						pGroup->objects.push_back(hadWall);
						hadWall->laneGroups.push_back(pGroup);
					}
				}
			}
			pGrid->insert(hadWall->originId, hadWall);
		}
	}

	void ObjectProcessor::processWall(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby)
    {
		std::vector<DbRecord*>& walls = pMesh->query(RecordType::DB_HAD_OBJECT_WALL);
		for (auto wall : walls)
		{
			DbWall* dbWall = (DbWall*)wall;
			HadWall* hadWall = (HadWall*)pGrid->query(dbWall->uuid, ElementType::HAD_OBJECT_WALL);
			if (hadWall == nullptr) {
				continue;
			}

			// wall -> linkgroup
			DbLgRel* pLgRel = (DbLgRel*)pMesh->query(dbWall->uuid, RecordType::DB_HAD_OBJECT_LG_REL);
			if (pLgRel != nullptr)
			{
				for (auto& relLg : pLgRel->relLgs)
				{
					HadLaneGroup* pGroup = (HadLaneGroup*)queryNearby(pGrid, nearby, relLg.first, ElementType::HAD_LANE_GROUP);
					if (pGroup != nullptr)
					{
						pGroup->objects.push_back(hadWall);
						hadWall->laneGroups.push_back(pGroup);
					}
				}
			}
		}
    }


	void ObjectProcessor::processTrafficLights(DbMesh* pMesh, HadGrid* pGrid)
	{
		std::vector<DbRecord*>& trafficLights = pMesh->query(RecordType::DB_HAD_OBJECT_TRAFFIC_LIGHTS);
		for (auto tl : trafficLights)
		{
			DbTrafficLights* ptl = (DbTrafficLights*)tl;
			HadTrafficLights* pTrafficLights = (HadTrafficLights*)pGrid->alloc(ElementType::HAD_OBJECT_TRAFFIC_LIGHTS);
			pTrafficLights->originId = ptl->uuid;
			pTrafficLights->type = ptl->type;
			pTrafficLights->orientation = ptl->orientation;
			pTrafficLights->rowNumber = ptl->rowNumber;
			pTrafficLights->columnNumber = ptl->columnNumber;
			pTrafficLights->heading = ptl->heading;
			pTrafficLights->location = ptl->geometry;

			// trafficLights -> linkgroup
			DbLgRel* pLgRel = (DbLgRel*)pMesh->query(ptl->uuid, RecordType::DB_HAD_OBJECT_LG_REL);
			if (pLgRel != nullptr) {
				for (auto& relLg : pLgRel->relLgs) {
					HadLaneGroup* pGroup = (HadLaneGroup*)pGrid->query(relLg.first, ElementType::HAD_LANE_GROUP);
					if (pGroup != nullptr)
					{
						pGroup->objects.push_back(pTrafficLights);
						pTrafficLights->laneGroups.push_back(pGroup);
					}
				}
			}
			else {
				// trafficLights -> lane -> linkgroup
				DbLaneLinkRel* pLaneLinkRel = (DbLaneLinkRel*)pMesh->query(ptl->uuid, RecordType::DB_HAD_OBJECT_LANE_LINK_REL);
				if (pLaneLinkRel != nullptr) {
					for (auto& relLaneLink : pLaneLinkRel->relLaneLinkIds)
					{
						HadLane* refLane = (HadLane*)pGrid->query(relLaneLink.first, ElementType::HAD_LANE);
						if (refLane == nullptr)
							continue;

						pTrafficLights->refLanes.push_back(refLane);
						refLane->objects.push_back(pTrafficLights);
						if (refLane->linkGroup != nullptr)
						{
							refLane->linkGroup->objects.push_back(pTrafficLights);
							pTrafficLights->laneGroups.push_back(refLane->linkGroup);
						}
					}
				}
			}

			pGrid->insert(pTrafficLights->originId, pTrafficLights);
		}
	}

	void ObjectProcessor::processTrafficLights(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby)
	{
		std::vector<DbRecord*>& trafficLights = pMesh->query(RecordType::DB_HAD_OBJECT_TRAFFIC_LIGHTS);
		for (auto tl : trafficLights)
		{
			DbTrafficLights* psl = (DbTrafficLights*)tl;
			HadTrafficLights* ptrafficLights = (HadTrafficLights*)pGrid->query(psl->uuid, ElementType::HAD_OBJECT_TRAFFIC_LIGHTS);
			if (ptrafficLights == nullptr) {
				continue;
			}

			// trafficLights -> linkgroup
			DbLgRel* pLgRel = (DbLgRel*)pMesh->query(psl->uuid, RecordType::DB_HAD_OBJECT_LG_REL);
			if (pLgRel != nullptr) {
				for (auto& relLg : pLgRel->relLgs) {
					HadLaneGroup* pGroup = (HadLaneGroup*)queryNearby(pGrid, nearby, relLg.first, ElementType::HAD_LANE_GROUP);
					if (pGroup != nullptr)
					{
						pGroup->objects.push_back(ptrafficLights);
						ptrafficLights->laneGroups.push_back(pGroup);
					}
				}
			}
			else {
				// trafficLights -> lane -> linkgroup
				DbLaneLinkRel* pLaneLinkRel = (DbLaneLinkRel*)pMesh->query(psl->uuid, RecordType::DB_HAD_OBJECT_LANE_LINK_REL);
				if (pLaneLinkRel != nullptr) {
					for (auto& relLaneLink : pLaneLinkRel->relLaneLinkIds)
					{
						HadLane* refLane = (HadLane*)queryNearby(pGrid, nearby, relLaneLink.first, ElementType::HAD_LANE);
						if (refLane == nullptr)
							continue;

						ptrafficLights->refLanes.push_back(refLane);
						refLane->objects.push_back(ptrafficLights);
						if (refLane->linkGroup != nullptr)
						{
							refLane->linkGroup->objects.push_back(ptrafficLights);
							ptrafficLights->laneGroups.push_back(refLane->linkGroup);
						}
					}
				}
			}

		}
	}


	void ObjectProcessor::processPole(DbMesh* pMesh, HadGrid* pGrid)
	{
		std::vector<DbRecord*>& trafficLights = pMesh->query(RecordType::DB_HAD_OBJECT_POLE);
		for (auto tl : trafficLights)
		{
			DbPole* pPole = (DbPole*)tl;
			HadPole* pHadPole = (HadPole*)pGrid->alloc(ElementType::HAD_OBJECT_POLE);
			pHadPole->originId = pPole->uuid;
			pHadPole->type = pPole->type;
			pHadPole->location = pPole->geometry;

			// pole -> linkgroup
			DbLgRel* pLgRel = (DbLgRel*)pMesh->query(pPole->uuid, RecordType::DB_HAD_OBJECT_LG_REL);
			if (pLgRel != nullptr) {
				for (auto& relLg : pLgRel->relLgs) {
					HadLaneGroup* pGroup = (HadLaneGroup*)pGrid->query(relLg.first, ElementType::HAD_LANE_GROUP);
					if (pGroup != nullptr)
					{
						pGroup->objects.push_back(pHadPole);
						pHadPole->laneGroups.push_back(pGroup);
					}
				}
			}
			else {
				// pole -> lane -> linkgroup
				DbLaneLinkRel* pLaneLinkRel = (DbLaneLinkRel*)pMesh->query(pPole->uuid, RecordType::DB_HAD_OBJECT_LANE_LINK_REL);
				if (pLaneLinkRel != nullptr) {
					for (auto& relLaneLink : pLaneLinkRel->relLaneLinkIds)
					{
						HadLane* refLane = (HadLane*)pGrid->query(relLaneLink.first, ElementType::HAD_LANE);
						if (refLane == nullptr)
							continue;

						pHadPole->refLanes.push_back(refLane);
						refLane->objects.push_back(pHadPole);
						if (refLane->linkGroup != nullptr)
						{
							refLane->linkGroup->objects.push_back(pHadPole);
							pHadPole->laneGroups.push_back(refLane->linkGroup);
						}
					}
				}
			}

			pGrid->insert(pHadPole->originId, pHadPole);
		}
	}

	void ObjectProcessor::processPole(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby)
	{
		std::vector<DbRecord*>& trafficLights = pMesh->query(RecordType::DB_HAD_OBJECT_POLE);
		for (auto tl : trafficLights)
		{
			DbPole* pPole = (DbPole*)tl;
			HadPole* pHadPole = (HadPole*)pGrid->query(pPole->uuid, ElementType::HAD_OBJECT_POLE);
			if (pHadPole == nullptr) {
				continue;
			}

			// pole -> linkgroup
			DbLgRel* pLgRel = (DbLgRel*)pMesh->query(pPole->uuid, RecordType::DB_HAD_OBJECT_LG_REL);
			if (pLgRel != nullptr) {
				for (auto& relLg : pLgRel->relLgs) {
					HadLaneGroup* pGroup = (HadLaneGroup*)queryNearby(pGrid, nearby, relLg.first, ElementType::HAD_LANE_GROUP);
					if (pGroup != nullptr)
					{
						pGroup->objects.push_back(pHadPole);
						pHadPole->laneGroups.push_back(pGroup);
					}
				}
			}
			else {
				// pole -> lane -> linkgroup
				DbLaneLinkRel* pLaneLinkRel = (DbLaneLinkRel*)pMesh->query(pPole->uuid, RecordType::DB_HAD_OBJECT_LANE_LINK_REL);
				if (pLaneLinkRel != nullptr) {
					for (auto& relLaneLink : pLaneLinkRel->relLaneLinkIds)
					{
						HadLane* refLane = (HadLane*)queryNearby(pGrid, nearby, relLaneLink.first, ElementType::HAD_LANE);
						if (refLane == nullptr)
							continue;

						pHadPole->refLanes.push_back(refLane);
						refLane->objects.push_back(pHadPole);
						if (refLane->linkGroup != nullptr)
						{
							refLane->linkGroup->objects.push_back(pHadPole);
							pHadPole->laneGroups.push_back(refLane->linkGroup);
						}
					}
				}
			}

		}
	}

	void ObjectProcessor::processSpeedBump(DbMesh* pMesh, HadGrid* pGrid)
	{
		std::vector<DbRecord*>& speedBumps = pMesh->query(RecordType::DB_HAD_OBJECT_SPEED_BUMP);
		for (auto item : speedBumps)
		{
			DbSpeedBump* speedBumpDB = (DbSpeedBump*)item;
			HadSpeedBump* speedBumpHad = (HadSpeedBump*)pGrid->alloc(ElementType::HAD_OBJECT_SPEED_BUMP);
			speedBumpHad->originId = speedBumpDB->uuid;
			speedBumpHad->heading = speedBumpDB->heading;
			speedBumpHad->polygon = speedBumpDB->geometry;

			// speed bump -> linkgroup
			DbLgRel* pLgRel = (DbLgRel*)pMesh->query(speedBumpDB->uuid, RecordType::DB_HAD_OBJECT_LG_REL);
			if (pLgRel != nullptr) {
				for (auto& relLg : pLgRel->relLgs) {
					HadLaneGroup* pGroup = (HadLaneGroup*)pGrid->query(relLg.first, ElementType::HAD_LANE_GROUP);
					if (pGroup != nullptr)
					{
						pGroup->objects.push_back(speedBumpHad);
						speedBumpHad->laneGroups.push_back(pGroup);
					}
				}
			}
			pGrid->insert(speedBumpHad->originId, speedBumpHad);
		}
	}

	void ObjectProcessor::processSpeedBump(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby)
	{
		std::vector<DbRecord*>& speedBumps = pMesh->query(RecordType::DB_HAD_OBJECT_SPEED_BUMP);
		for (auto cw : speedBumps)
		{
			DbSpeedBump* pcw = (DbSpeedBump*)cw;
			HadSpeedBump* pSpeedBump = (HadSpeedBump*)pGrid->query(pcw->uuid, ElementType::HAD_OBJECT_SPEED_BUMP);
			if (pSpeedBump == nullptr) {
				continue;
			}

			// speed bump -> linkgroup
			DbLgRel* pLgRel = (DbLgRel*)pMesh->query(pcw->uuid, RecordType::DB_HAD_OBJECT_LG_REL);
			if (pLgRel != nullptr) {
				for (auto& relLg : pLgRel->relLgs) {
					HadLaneGroup* pGroup = (HadLaneGroup*)queryNearby(pGrid, nearby, relLg.first, ElementType::HAD_LANE_GROUP);
					if (pGroup != nullptr)
					{
						pGroup->objects.push_back(pSpeedBump);
						pSpeedBump->laneGroups.push_back(pGroup);
					}
				}
			}

		}
	}




}
