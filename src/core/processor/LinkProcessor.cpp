#include "stdafx.h"
#include "LinkProcessor.h"
#include <algorithm>
namespace OMDB
{
	int LinkProcessor::LINK_DIRECT_PA_NAME = 52;
	void LinkProcessor::process(DbMesh* const pMesh, HadGrid* pGrid)
	{
		std::vector<DbRecord*>& nodes = pMesh->query(RecordType::DB_HAD_NODE);
		for (auto nd : nodes)
		{
			DbNode* pNd = (DbNode*)nd;
			HadNode* pNode = (HadNode*)pGrid->alloc(ElementType::HAD_NODE);
			pNode->originId = pNd->uuid;
			pNode->position = pNd->geometry;
			pNode->meshIds = pNd->meshIds;
			pNode->intersectionId = pNd->intersectionId;
			pNode->wayType = pNd->wayType;
			pGrid->insert(pNode->originId, pNode);
		}

		std::vector<DbRecord*>& links = pMesh->query(RecordType::DB_HAD_LINK);
		for (auto hl : links)
		{
			DbLink* phl = (DbLink*)hl;
			HadLink* pLink = (HadLink*)pGrid->alloc(ElementType::HAD_LINK);
			pLink->originId = phl->uuid;
			pLink->location = phl->geometry;
			pLink->startNode = (HadNode*)pGrid->query(phl->startNode, ElementType::HAD_NODE);
			pLink->endNode = (HadNode*)pGrid->query(phl->endNode, ElementType::HAD_NODE);
			pLink->crossGrid = pLink->startNode->meshIds.size() > 1 || pLink->endNode->meshIds.size() > 1;
			if (std::find(pLink->startNode->links.begin(), pLink->startNode->links.end(), pLink) == pLink->startNode->links.end())
				pLink->startNode->links.push_back(pLink);
			if (std::find(pLink->endNode->links.begin(), pLink->endNode->links.end(), pLink) == pLink->endNode->links.end())
				pLink->endNode->links.push_back(pLink);

			pLink->multi_digitized = phl->multi_digitized;
			pLink->direct = phl->direct;
			pLink->separation_left = phl->separation_left;
			pLink->separation_right = phl->separation_right;
			pLink->median_left = phl->median_left;
			pLink->median_right = phl->median_right;
			pLink->overhead_obstruction = phl->overhead_obstruction;
			pLink->wayTypes = phl->wayTypes;

			// Fixed SpeedLimit
			DbLinkSpeedLimit* fSpeedLimit = (DbLinkSpeedLimit*)(pMesh->query(phl->uuid, RecordType::DB_HAD_LINK_FIXED_SPEEDLIMIT));
			if (fSpeedLimit != nullptr)
			{
				pLink->speedLimit.maxSpeedLimit = fSpeedLimit->maxSpeedLimit;
				pLink->speedLimit.maxSpeedLimitSource = (FixedSpeedLimitSource)fSpeedLimit->maxSpeedLimitSource;
				pLink->speedLimit.minSpeedLimit = fSpeedLimit->minSpeedLimit;
				pLink->speedLimit.minSpeedLimitSource = (FixedSpeedLimitSource)fSpeedLimit->minSpeedLimitSource;
			}

			// PA
			std::vector<DbRecord*>& pas = pMesh->query(RecordType::DB_HAD_LINK_PA);
			std::set<int> directs;
			for (auto pa : pas)
			{
				DbLinkPA* ppa = (DbLinkPA*)pa;
				if (ppa->relLinkId == pLink->originId)
				{
					for (DbPAValue* paValue : ppa->paValues)
					{
						HadPartAttribute* pAttribute = (HadPartAttribute*)pGrid->alloc(ElementType::HAD_PART_ATTRIBUTE);
						pAttribute->start = ppa->startOffset;
						pAttribute->end = ppa->endOffset;
						pAttribute->seqNum = paValue->seqNum;
						pAttribute->name = paValue->attributeType;
						pAttribute->value = paValue->attributeValue;
						pAttribute->originId = ppa->uuid;
						pAttribute->points = ppa->geometry;
						if (pAttribute->name == LINK_DIRECT_PA_NAME) 
						{
							directs.emplace(pAttribute->value);
						}

						// Fixed SpeedLimit
						DbLinkSpeedLimit* fSpeedLimit = (DbLinkSpeedLimit*)(pMesh->query(pAttribute->originId, RecordType::DB_HAD_LINK_FIXED_SPEEDLIMIT));
						if (fSpeedLimit != nullptr)
						{
							pAttribute->speedLimit.maxSpeedLimit = fSpeedLimit->maxSpeedLimit;
							pAttribute->speedLimit.maxSpeedLimitSource = (FixedSpeedLimitSource)fSpeedLimit->maxSpeedLimitSource;
							pAttribute->speedLimit.minSpeedLimit = fSpeedLimit->minSpeedLimit;
							pAttribute->speedLimit.minSpeedLimitSource = (FixedSpeedLimitSource)fSpeedLimit->minSpeedLimitSource;
						}

						pLink->attributes.push_back(pAttribute);
					}
				}
			}

			processDataLevel(phl, pLink);
			processSeparation(phl, pLink, pGrid);
			processMedian(phl, pLink, pGrid);
			processOverheadObstruction(phl, pLink, pGrid);
			processTollArea(phl, pLink, pGrid);
			if (directs.size() == 1) {
				pLink->direct = *directs.begin();
			}
			if (pLink->direct == DB_HAD_APPLY_PA_REFERENCE) {
				pLink->direct = 2;
			}

			// 道路PA信息从0~1排序
			std::sort(pLink->attributes.begin(), pLink->attributes.end(),
				[](const HadPartAttribute* first, const HadPartAttribute* second)->bool {
					if (first->start != second->start)
						return first->start < second->start;
					return first->end < second->end;
				});


			pGrid->insert(pLink->originId, pLink);

		}
	}

	void LinkProcessor::processDataLevel(DbLink* const dbLink, HadLink* pLink)
	{
		for (auto& dataLevel : dbLink->dataLevels)
		{
			HadLink::HadLinkDataLevel tmp;
			tmp.linkId = dataLevel.uuid;
			tmp.featureType = dataLevel.featureType;
			tmp.startOffset = dataLevel.startOffset;
			tmp.endOffset = dataLevel.endOffset;
			tmp.dataLevel = dataLevel.dataLevel;
			tmp.geometry = dataLevel.geometry;
			pLink->dataLevels.push_back(tmp);
		}
	}

	void LinkProcessor::processSeparation(DbLink* const dbLink, HadLink* pLink, HadGrid* pGrid)
	{
		const int LINK_SEPARATION_LEFT_PA_NAME = 18;
		const int LINK_SEPARATION_RIGHT_PA_NAME = 19;
		for (auto& separation : dbLink->separations)
		{
			for (auto& dataLevel : dbLink->dataLevels)
			{
				// dataLevel等于1才有效
				if (dataLevel.dataLevel == 1)
				{
					// SEPARATION指示LINK
					if (separation.featureType == 1) // LINK
					{
						if (separation.side == 0) // 左侧
						{
							pLink->separation_left = separation.separation;
						}
						else if (separation.side == 1) // 右侧
						{
							pLink->separation_right = separation.separation;
						}
					}
					// DATALEVEL指示LINK
					if (dataLevel.featureType == 1)
					{
						if (separation.featureType == 2) // LINK PA
						{
							HadPartAttribute* pAttribute = allocPartAttribute(pGrid, separation);
							pAttribute->value = separation.separation;
							pAttribute->originId = separation.uuid;
							if (separation.side == 0) // 左侧
							{
								pAttribute->name = LINK_SEPARATION_LEFT_PA_NAME;
								if (!pLink->separation_left) {
									pLink->separation_left = DB_HAD_APPLY_PA_REFERENCE;
								}
							}
							else if (separation.side == 1) // 右侧
							{
								pAttribute->name = LINK_SEPARATION_RIGHT_PA_NAME;
								if (!pLink->separation_right) {
									pLink->separation_right = DB_HAD_APPLY_PA_REFERENCE;
								}
							}
							pLink->attributes.push_back(pAttribute);
						}
					}
					// DATALEVEL指示LINK PA
					else if (dataLevel.featureType == 2)
					{
						if (separation.featureType == 2) // LINK PA
						{
							if (dataLevel.startOffset <= separation.startOffset && separation.endOffset <= dataLevel.endOffset)
							{
								HadPartAttribute* pAttribute = allocPartAttribute(pGrid, separation);
								pAttribute->value = separation.separation;
								pAttribute->originId = separation.uuid;
								if (separation.side == 0) // 左侧
								{
									pAttribute->name = LINK_SEPARATION_LEFT_PA_NAME;
									if (!pLink->separation_left) {
										pLink->separation_left = DB_HAD_APPLY_PA_REFERENCE;
									}
								}
								else if (separation.side == 1) // 右侧
								{
									pAttribute->name = LINK_SEPARATION_RIGHT_PA_NAME;
									if (!pLink->separation_right) {
										pLink->separation_right = DB_HAD_APPLY_PA_REFERENCE;
									}
								}
								pLink->attributes.push_back(pAttribute);
							}
						}
					}
				}
			}
		}
	}

	void LinkProcessor::processMedian(DbLink* const dbLink, HadLink* pLink, HadGrid* pGrid)
	{
		const int LINK_MEDIAN_LEFT_PA_NAME = 20;
		const int LINK_MEDIAN_RIGHT_PA_NAME = 21;
		for (auto& median : dbLink->medians)
		{
			for (auto& dataLevel : dbLink->dataLevels)
			{
				// dataLevel等于1才有效
				if (dataLevel.dataLevel == 1)
				{
					// MEDIAN指示LINK
					if (median.featureType == 1) // LINK
					{
						if (median.side == 0) // 左侧
						{
							pLink->median_left = median.median;
						}
						else if (median.side == 1) // 右侧
						{
							pLink->median_right = median.median;
						}
					}
					// DATALEVEL指示LINK
					if (dataLevel.featureType == 1)
					{
						if (median.featureType == 2) // LINK PA
						{
							HadPartAttribute* pAttribute = allocPartAttribute(pGrid, median);
							pAttribute->value = median.median;
							pAttribute->originId = median.uuid;
							if (median.side == 0) // 左侧
							{
								pAttribute->name = LINK_MEDIAN_LEFT_PA_NAME;
								if (!pLink->median_left) {
									pLink->median_left = DB_HAD_APPLY_PA_REFERENCE;
								}
							}
							else if (median.side == 1) // 右侧
							{
								pAttribute->name = LINK_MEDIAN_RIGHT_PA_NAME;
								if (!pLink->median_right) {
									pLink->median_right = DB_HAD_APPLY_PA_REFERENCE;
								}
							}
							pLink->attributes.push_back(pAttribute);
						}
					}
					// DATALEVEL指示LINK PA
					else if (dataLevel.featureType == 2)
					{
						if (median.featureType == 2) // LINK PA
						{
							if (dataLevel.startOffset <= median.startOffset && median.endOffset <= dataLevel.endOffset)
							{
								HadPartAttribute* pAttribute = allocPartAttribute(pGrid, median);
								pAttribute->value = median.median;
								pAttribute->originId = median.uuid;
								if (median.side == 0) // 左侧
								{
									pAttribute->name = LINK_MEDIAN_LEFT_PA_NAME;
									if (!pLink->median_left) {
										pLink->median_left = DB_HAD_APPLY_PA_REFERENCE;
									}
								}
								else if (median.side == 1) // 右侧
								{
									pAttribute->name = LINK_MEDIAN_RIGHT_PA_NAME;
									if (!pLink->median_right) {
										pLink->median_right = DB_HAD_APPLY_PA_REFERENCE;
									}
								}
								pLink->attributes.push_back(pAttribute);
							}
						}
					}
				}
			}
		}
	}

	void LinkProcessor::processOverheadObstruction(DbLink* const dbLink, HadLink* pLink, HadGrid* pGrid)
	{
		const int LINK_OVERHEAD_OBSTRUCTION_PA_NAME = 17;
		for (auto& overheadObstruction : dbLink->overheadObstructions)
		{
			for (auto& dataLevel : dbLink->dataLevels)
			{
				// dataLevel等于1才有效
				if (dataLevel.dataLevel == 1)
				{
					// OVERHEAD_OBSTRUCTION指示LINK
					if (overheadObstruction.featureType == 1) // LINK
					{
						pLink->overhead_obstruction = 1;
					}
					// DATALEVEL指示LINK
					if (dataLevel.featureType == 1)
					{
						if (overheadObstruction.featureType == 2) // LINK PA
						{
							HadPartAttribute* pAttribute = allocPartAttribute(pGrid, overheadObstruction);
							pAttribute->name = LINK_OVERHEAD_OBSTRUCTION_PA_NAME;
							pAttribute->originId = overheadObstruction.uuid;
							pAttribute->value = 1;
							pLink->attributes.push_back(pAttribute);
							if (!pLink->overhead_obstruction) {
								pLink->overhead_obstruction = DB_HAD_APPLY_PA_REFERENCE;
							}
						}
					}
					// DATALEVEL指示LINK PA
					else if (dataLevel.featureType == 2)
					{
						if (overheadObstruction.featureType == 2) // LINK PA
						{
							if (dataLevel.startOffset <= overheadObstruction.startOffset && overheadObstruction.endOffset <= dataLevel.endOffset)
							{
								HadPartAttribute* pAttribute = allocPartAttribute(pGrid, overheadObstruction);
								pAttribute->name = LINK_OVERHEAD_OBSTRUCTION_PA_NAME;
								pAttribute->originId = overheadObstruction.uuid;
								pAttribute->value = 1;
								pLink->attributes.push_back(pAttribute);
								if (!pLink->overhead_obstruction) {
									pLink->overhead_obstruction = DB_HAD_APPLY_PA_REFERENCE;
								}
							}
						}
					}
				}
			}
		}
	}

	void LinkProcessor::processTollArea(DbLink* const dbLink, HadLink* pLink, HadGrid* pGrid)
	{
		for (auto& tollArea : dbLink->tollAreas)
		{
			for (auto& dataLevel : dbLink->dataLevels)
			{
				// dataLevel等于1才有效
				if (dataLevel.dataLevel == 1)
				{
					// TOLL_AREA指示LINK
					if (tollArea.featureType == 1) // LINK
					{
						pLink->wayTypes.push_back(LINK_IS_IN_TOLL_AREA);
					}
					// DATALEVEL指示LINK
					if (dataLevel.featureType == 1)
					{
						if (tollArea.featureType == 2) // LINK PA
						{
							HadPartAttribute* pAttribute = allocPartAttribute(pGrid, tollArea);
							pAttribute->name = LINK_WAY_TYPE_PA_NAME;
							pAttribute->originId = tollArea.uuid;
							pAttribute->value = LINK_IS_IN_TOLL_AREA;
							pLink->attributes.push_back(pAttribute);
							pLink->wayTypes.push_back(DB_HAD_APPLY_PA_REFERENCE);
						}
					}
					// DATALEVEL指示LINK PA
					else if (dataLevel.featureType == 2)
					{
						if (tollArea.featureType == 2) // LINK PA
						{
							if (dataLevel.startOffset <= tollArea.startOffset && tollArea.endOffset <= dataLevel.endOffset)
							{
								HadPartAttribute* pAttribute = allocPartAttribute(pGrid, tollArea);
								pAttribute->name = LINK_WAY_TYPE_PA_NAME;
								pAttribute->originId = tollArea.uuid;
								pAttribute->value = LINK_IS_IN_TOLL_AREA;
								pLink->attributes.push_back(pAttribute);
								pLink->wayTypes.push_back(DB_HAD_APPLY_PA_REFERENCE);
							}
						}
					}
				}
			}
		}
	}

	void LinkProcessor::processRelation(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby)
	{
		UNREFERENCED_PARAMETER(pMesh);
		UNREFERENCED_PARAMETER(pGrid);
		UNREFERENCED_PARAMETER(nearby);
	}
}
