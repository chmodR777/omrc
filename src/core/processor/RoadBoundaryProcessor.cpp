#include "stdafx.h"
#include "RoadBoundaryProcessor.h"
#include "algorithm/grap_point_algorithm.h"
#include <algorithm>
#include "CompileSetting.h"

namespace OMDB
{
    void RoadBoundaryProcessor::process(DbMesh* const pMesh, HadGrid* pGrid)
    {
        std::vector<DbRecord*>& nodes = pMesh->query(RecordType::DB_HAD_ROAD_BOUNDARY_NODE);
        for (auto nd : nodes)
        {
            DbRoadBoundNode* pnd = (DbRoadBoundNode*)nd;
            HadRoadBoundaryNode* pNode = (HadRoadBoundaryNode*)pGrid->alloc(ElementType::HAD_ROAD_BOUNDARY_NODE);
            pNode->position = pnd->geometry;
            pNode->originId = pnd->uuid;
            pGrid->insert(pNode->originId, pNode);
        }
        std::vector<DbRecord*>& links = pMesh->query(RecordType::DB_HAD_ROAD_BOUNDARY_LINK);
        for (auto hl : links)
        {
            DbRoadBoundLink* phl = (DbRoadBoundLink*)hl;
            if (phl->markDeleted) { // 删除了,不生成HD数据
                continue;
            }

            HadRoadBoundary* prb = (HadRoadBoundary*)pGrid->alloc(ElementType::HAD_ROAD_BOUNDARY);
            prb->originId = phl->uuid;
            prb->location = phl->geometry;
            prb->startNode = (HadRoadBoundaryNode*)pGrid->query(phl->starRoadBoundNodeId, ElementType::HAD_ROAD_BOUNDARY_NODE);
            prb->endNode = (HadRoadBoundaryNode*)pGrid->query(phl->endRoadBoundNodeId, ElementType::HAD_ROAD_BOUNDARY_NODE);
            prb->boundaryType = (BoundaryType)phl->boundaryType;

            // PA
            std::vector<DbRecord*>& pas = pMesh->query(RecordType::DB_HAD_ROAD_BOUNDARY_PA);
            for (auto pa : pas)
            {
                DbRoadBoundPA* ppa = (DbRoadBoundPA*)pa;
                if (ppa->relRoadBoundLinkId == prb->originId)
                {
                    for (DbPAValue* paValue : ppa->paValues)
                    {
                        HadPartAttribute* pAttribute = (HadPartAttribute*)pGrid->alloc(ElementType::HAD_PART_ATTRIBUTE);
                        pAttribute->originId = ppa->uuid;
                        pAttribute->start = ppa->startOffset;
                        pAttribute->end = ppa->endOffset;
                        pAttribute->seqNum = paValue->seqNum;
                        pAttribute->name = paValue->attributeType;
                        pAttribute->value = paValue->attributeValue;
                        pAttribute->points = ppa->geometry;
                        for (auto& pt : pAttribute->points.postions) {
                            int ZERO_ZVALUE = 0;

                            if (CompileSetting::instance()->isDaimlerShangHai)
                            {
                                ZERO_ZVALUE -= 1200;
                            }

                            if (pt.z == ZERO_ZVALUE)
                            {
								size_t si, ei;
								MapPoint3D64 grappedPt = {};
								GrapPointAlgorithm::grapOrMatchNearestPoint(pt, phl->geometry.vertexes, grappedPt, si, ei);
								if (grappedPt.pos.lon != 0 && grappedPt.pos.lat != 0 && grappedPt.z != 0) {
									pt.z = grappedPt.z;
								}
                            }
                        }
                        prb->attributes.push_back(pAttribute);
                    }
                }
            }
            // 道路边界PA信息从0~1排序
            std::sort(prb->attributes.begin(), prb->attributes.end(),
                [](const HadPartAttribute* first, const HadPartAttribute* second)->bool {
                    if (first->start != second->start)
                        return first->start < second->start;
                    return first->end < second->end;
                });

            // 一个车道边界可能是周边两个车道组共享的,但是direction和side不同
            for (auto& relLg : phl->relLgs) {
                HadRoadBoundary::HADLgRoadBoundREL hadRelLg;
                hadRelLg.direction = relLg.second.direction;
                hadRelLg.side = relLg.second.side;
                prb->relLgs.emplace(relLg.first, hadRelLg);
                HadLaneGroup* pGroup = (HadLaneGroup*)pGrid->query(relLg.first, ElementType::HAD_LANE_GROUP);
                if (pGroup != nullptr)
                {
                    pGroup->roadBoundaries.push_back(prb);
                    prb->linkGroups.emplace(relLg.first, pGroup);
                }
            }

            pGrid->insert(prb->originId, prb);
        }
    }

    void RoadBoundaryProcessor::processRelation(DbMesh* const pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby)
    {
		std::vector<DbRecord*>& links = pMesh->query(RecordType::DB_HAD_ROAD_BOUNDARY_LINK);
		for (auto hl : links)
		{
			DbRoadBoundLink* phl = (DbRoadBoundLink*)hl;
			HadRoadBoundary* prb = (HadRoadBoundary*)pGrid->query(phl->uuid, ElementType::HAD_ROAD_BOUNDARY);
			if (prb == nullptr) {
				continue;
			}

            // 兼容generator
            if (prb->startNode == nullptr) {
                prb->startNode = (HadRoadBoundaryNode*)queryNearby(pGrid, nearby, phl->starRoadBoundNodeId, ElementType::HAD_ROAD_BOUNDARY_NODE);
            }
            if (prb->endNode == nullptr) {
                prb->endNode = (HadRoadBoundaryNode*)queryNearby(pGrid, nearby, phl->endRoadBoundNodeId, ElementType::HAD_ROAD_BOUNDARY_NODE);
            }

			// 一个车道边界可能是周边两个车道组共享的,但是direction和side不同
			for (auto& relLg : phl->relLgs) {
				HadLaneGroup* pGroup = (HadLaneGroup*)queryNearby(pGrid, nearby, relLg.first, ElementType::HAD_LANE_GROUP);
				if (pGroup != nullptr)
				{
					pGroup->roadBoundaries.push_back(prb);
					prb->linkGroups.emplace(relLg.first, pGroup);
				}
			}

            // 隔壁网格同一条边界也可能存在关系
			HadRoadBoundary* nearbyPrb = (HadRoadBoundary*)queryNearby(pGrid, nearby, phl->uuid, ElementType::HAD_ROAD_BOUNDARY);
			if (nearbyPrb == nullptr) {
				continue;
			}

			for (auto& relLg : nearbyPrb->relLgs) {
                prb->relLgs.emplace(relLg.first, relLg.second);
				HadLaneGroup* pGroup = (HadLaneGroup*)queryNearby(pGrid, nearby, relLg.first, ElementType::HAD_LANE_GROUP);
				if (pGroup != nullptr)
				{
					prb->linkGroups.emplace(relLg.first, pGroup);
				}
			}
		}
    }
}
