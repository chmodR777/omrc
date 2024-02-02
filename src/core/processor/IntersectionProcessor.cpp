#include "stdafx.h"
#include "IntersectionProcessor.h"

namespace OMDB
{
	void IntersectionProcessor::process(DbMesh* const pMesh, HadGrid* pGrid)
	{
		//
		std::vector<DbRecord*>& intersections = pMesh->query(RecordType::DB_HAD_INTERSECTION);
		for (auto intersct : intersections)
		{
			DbIntersection* pIntersct = (DbIntersection*)intersct;
			HadIntersection* hadIntersct = (HadIntersection*)pGrid->alloc(ElementType::HAD_INTERSECTION);
			if (hadIntersct == nullptr)
			{
				assert(0);
				continue;
			}
			hadIntersct->originId = pIntersct->uuid;
			hadIntersct->intersectionType = pIntersct->intersectionType;
			hadIntersct->refLanePAs = pIntersct->refLanePAs;

			for (auto iter = pIntersct->refMeshs.begin(); iter != pIntersct->refMeshs.end(); ++iter)
			{
				if ((*iter) == pGrid->getId())
				{
					hadIntersct->refMeshs.push_back(pGrid);
				}
			}

			for (auto iter = pIntersct->refNodes.begin(); iter != pIntersct->refNodes.end(); ++iter)
			{
				HadNode* hNode = (HadNode*)pGrid->query(*iter, ElementType::HAD_NODE);
				if (hNode != nullptr)
				{
					hadIntersct->refNodes.push_back(hNode);
				}
			}

			for (auto iter = pIntersct->refLaneGroups.begin(); iter != pIntersct->refLaneGroups.end(); ++iter)
			{
				HadLaneGroup* pGroup = (HadLaneGroup*)pGrid->query(*iter, ElementType::HAD_LANE_GROUP);
				if (pGroup != nullptr)
				{
					hadIntersct->refLaneGroups.push_back(pGroup);
					pGroup->inIntersection = hadIntersct->originId;
					if (isUTurn(pGroup))
						hadIntersct->isUTurn = true;
					if (isRoundabout(pGroup))
						hadIntersct->isRoundabout = true;
				}
			}
			std::vector<HadElement*> allLinks = pGrid->query(ElementType::HAD_LINK);
			for (auto iter = pIntersct->outLinks.begin(); iter != pIntersct->outLinks.end(); ++iter)
			{
				//v1.5
				if ((*iter).featureType == 1)
				{
					HadLink* pLink = (HadLink*)pGrid->query((*iter).linkId, ElementType::HAD_LINK);
					if (pLink != nullptr)
					{
						hadIntersct->refOutLinks.push_back(pLink);
					}
				}
				else if ((*iter).featureType == 2)
				{
					for (size_t i = 0; i < allLinks.size(); i++)
					{
						HadLink* pCurLink = (HadLink*)allLinks[i];
						if (pCurLink != nullptr)
						{
							for (auto iterpa = pCurLink->attributes.begin(); iterpa != pCurLink->attributes.end(); ++iterpa)
							{
								if ((*iterpa) != nullptr && (*iterpa)->originId == (*iter).linkId)
								{
									hadIntersct->refLinkPAs.push_back(*iterpa);
								}
							}
						}
					}
				}
			}

			// 
			for (auto iter = pIntersct->refPoints.begin(); iter != pIntersct->refPoints.end(); ++iter)
			{
				int64 linkPointPAId = *iter;
				for (size_t i = 0; i < allLinks.size(); i++)
				{
					HadLink* pCurLink = (HadLink*)allLinks[i];
					if (pCurLink != nullptr)
					{
						for (auto iterpa = pCurLink->attributes.begin(); iterpa != pCurLink->attributes.end(); ++iterpa)
						{
							if ((*iterpa) != nullptr && (*iterpa)->originId == linkPointPAId)
							{
								hadIntersct->refPointPAs.push_back(*iterpa);
							}
						}
					}
				}
			}

			/*for (auto iter = pIntersct->outLines.begin(); iter != pIntersct->outLines.end(); ++iter)
			{
				HadIntersection::OutlinePA outline;
				outlinePACopy(*iter, outline);
				hadIntersct->outLines.push_back(outline);
			}*/

			pGrid->insert(hadIntersct->originId, hadIntersct);
		}
	}

	void IntersectionProcessor::processRelation(DbMesh* const pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby)
	{
		//
		std::vector<DbRecord*>& intersections = pMesh->query(RecordType::DB_HAD_INTERSECTION);
		for (auto intersct : intersections)
		{
			DbIntersection* pIntersct = (DbIntersection*)intersct;
			HadIntersection* hadIntersct = (HadIntersection*)pGrid->query(pIntersct->uuid, ElementType::HAD_INTERSECTION);
			if (hadIntersct == nullptr)
			{
				assert(0);
				continue;
			}

			for (auto iter = pIntersct->refNodes.begin(); iter != pIntersct->refNodes.end(); ++iter)
			{
				HadNode* hNode = (HadNode*)queryNearby(pGrid, nearby, *iter, ElementType::HAD_NODE);
				if (hNode != nullptr)
				{
					hadIntersct->refNodes.push_back(hNode);
				}
			}

			for (auto iter = pIntersct->refLaneGroups.begin(); iter != pIntersct->refLaneGroups.end(); ++iter)
			{
				HadLaneGroup* pGroup = (HadLaneGroup*)queryNearby(pGrid, nearby, *iter, ElementType::HAD_LANE_GROUP);
				if (pGroup != nullptr)
				{
					hadIntersct->refLaneGroups.push_back(pGroup);
				}
			}
			std::vector<HadElement*> allLinks = pGrid->query(ElementType::HAD_LINK);
			for (auto iter = pIntersct->outLinks.begin(); iter != pIntersct->outLinks.end(); ++iter)
			{
				//v1.5
				if ((*iter).featureType == 1)
				{
					HadLink* pLink = (HadLink*)queryNearby(pGrid, nearby, (*iter).linkId, ElementType::HAD_LINK);
					if (pLink != nullptr)
					{
						hadIntersct->refOutLinks.push_back(pLink);
					}
				}
			}

		}
	}

	void IntersectionProcessor::outlinkCopy(const DbIntersection::OutLink& src, HadIntersection::OutLink& dst)
	{
		dst.linkId = src.linkId;
		dst.linkType = src.featureType;
		dst.direction = src.direction;
		dst.eOffset = src.startOffset;
		dst.sOffset = src.endOffset;
		dst.geometry = src.geometry;
		dst.isCenterLink = src.isCenterLink;
	}
	void IntersectionProcessor::outlinePACopy(const DbIntersection::OutlinePA& src, HadIntersection::OutlinePA& dst)
	{
		dst.linkId = src.linkId;
		dst.offset = src.offset;
		dst.position = src.position;
	}
}