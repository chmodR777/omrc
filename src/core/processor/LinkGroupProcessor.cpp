#include "stdafx.h"
#include "LinkGroupProcessor.h"
#define HAD_GRID_NDS_LEVEL	13
namespace OMDB
{
	void LinkGroupProcessor::process(DbMesh* const pMesh, HadGrid* pGrid)
	{
		for (auto& lg : pMesh->query(RecordType::DB_HAD_LG_LINK))
		{
			DbLgLink* plg = (DbLgLink*)lg;
			HadLaneGroup* pGroup = (HadLaneGroup*)pGrid->alloc(ElementType::HAD_LANE_GROUP);
			pGroup->originId = plg->uuid;
			for (auto& pair : plg->relLinks)
			{
				HadLink* pLink = (HadLink*)pGrid->query(pair.second.relLinkid, ElementType::HAD_LINK);
				if (pLink != nullptr) {
					HadLaneGroup::HadRelLink relLink;
					relLink.link = pLink;
					relLink.start = pair.second.startOffset;
					relLink.end = pair.second.endOffset;
					relLink.direct = pair.second.directType;
					relLink.link->groups.push_back(pGroup);
					pGroup->relLinks.emplace(relLink.link, relLink);
				}
			}

			pGrid->insert(pGroup->originId, pGroup);
		}
	}

	void LinkGroupProcessor::processRelation(DbMesh* const pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby)
	{
		for (auto& lg : pMesh->query(RecordType::DB_HAD_LG_LINK))
		{
			DbLgLink* plg = (DbLgLink*)lg;
			HadLaneGroup* pGroup = (HadLaneGroup*)pGrid->query(plg->uuid, ElementType::HAD_LANE_GROUP);
			if (pGroup == nullptr) {
				continue;
			}

			for (auto& pair : plg->relLinks)
			{
				HadLink* pLink = (HadLink*)queryNearby(pGrid, nearby, pair.second.relLinkid, ElementType::HAD_LINK);
				if (pLink != nullptr) {
					HadLaneGroup::HadRelLink relLink;
					relLink.link = pLink;
					relLink.start = pair.second.startOffset;
					relLink.end = pair.second.endOffset;
					relLink.link->groups.push_back(pGroup);
					pGroup->relLinks.emplace(relLink.link, relLink);
				}
			}
		}
	}

}
