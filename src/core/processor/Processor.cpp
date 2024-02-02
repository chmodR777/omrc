#include "stdafx.h"
#include "Processor.h"

namespace OMDB
{

	void Processor::Process(DbMesh* const pMesh, HadGrid* pGrid)
	{
		return process(pMesh, pGrid);
	}

	void Processor::ProcessRelation(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby)
	{
		return processRelation(pMesh, pGrid, nearby);
	}

	HadElement* Processor::queryNearby(HadGrid* pGrid, std::vector<HadGrid*>* nearby, int64 id, ElementType objectType)
	{
		UNREFERENCED_PARAMETER(pGrid);
		HadElement* pElement = nullptr;
		//HadElement* pElement = pGrid->query(id, objectType);
		if (pElement == nullptr && nearby != nullptr) {
			for (auto grid : *nearby) {
				pElement = grid->query(id, objectType);
				if (pElement != nullptr) {
					return pElement;
				}
			}
		}

		return nullptr;
	}

	HadPartAttribute* Processor::allocPartAttribute(HadGrid* pGrid, DbPAField& paField)
	{
		HadPartAttribute* pAttribute = (HadPartAttribute*)pGrid->alloc(ElementType::HAD_PART_ATTRIBUTE);
		pAttribute->start = paField.startOffset;
		pAttribute->end = paField.endOffset;
		pAttribute->points = paField.geometry;
		return pAttribute;
	}

	bool Processor::isUTurn(HadLaneGroup* pLinkGroup)
	{
		for (auto& relLink : pLinkGroup->relLinks) {
			for (auto& wayType : relLink.first->wayTypes) {
				if (wayType == LINK_IS_IN_UTURN) {
					return true;
				}

				if (wayType == DB_HAD_APPLY_PA_REFERENCE) {
					std::vector<RangeF> intervals;
					for (auto paValue : relLink.first->attributes) {
						if (paValue->name == LINK_WAY_TYPE_PA_NAME && paValue->value == LINK_IS_IN_UTURN) {

							intervals.emplace_back(RangeF_make(paValue->start, paValue->end));
						}
					}
					if (!intervals.empty())
					{
						// 排序区间
						std::sort(intervals.begin(), intervals.end(), [](RangeF& s1, RangeF& s2) {
							if (s1.lower != s2.lower)
								return s1.lower < s2.lower;

							else
								return s1.upper < s2.upper;

							});
						// 合并重叠的区间
						std::vector<RangeF> merged;
						RangeF curRange = merged.front();
						for (int i = 1; i < intervals.size(); i++)
						{
							if (curRange.upper + 0.01 >= intervals[i].lower)
							{
								curRange.upper = max(curRange.upper + 0.01, intervals[i].upper);

							}
							else
							{
								merged.push_back(curRange);
								curRange = intervals[i];
							}
						}
						merged.push_back(curRange);

						for (auto& range : merged)
						{
							if (range.lower <= relLink.second.start + 0.01 && relLink.second.end <= range.upper + 0.01) {
								return true;
							}
						}

					}

				}


			}
		}
		return false;
	}

	bool Processor::isRoundabout(HadLaneGroup* pLinkGroup)
	{
		for (auto& relLink : pLinkGroup->relLinks) {
			for (auto& wayType : relLink.first->wayTypes) {
				if (wayType == LINK_IS_IN_ROUNDABOUT) {
					return true;
				}

				if (wayType == DB_HAD_APPLY_PA_REFERENCE) {
					std::vector<RangeF> intervals;
					for (auto paValue : relLink.first->attributes) {
						if (paValue->name == LINK_WAY_TYPE_PA_NAME && paValue->value == LINK_IS_IN_ROUNDABOUT) {

							intervals.emplace_back(RangeF_make(paValue->start, paValue->end));
						}
					}
					if (!intervals.empty())
					{
						// 排序区间
						std::sort(intervals.begin(), intervals.end(), [](RangeF& s1, RangeF& s2) {
							if (s1.lower != s2.lower)
								return s1.lower < s2.lower;

							else
								return s1.upper < s2.upper;

							});
						// 合并重叠的区间
						std::vector<RangeF> merged;
						RangeF curRange = merged.front();
						for (int i = 1; i < intervals.size(); i++)
						{
							if (curRange.upper + 0.01 >= intervals[i].lower)
							{
								curRange.upper = max(curRange.upper + 0.01, intervals[i].upper);

							}
							else
							{
								merged.push_back(curRange);
								curRange = intervals[i];
							}
						}
						merged.push_back(curRange);

						for (auto& range : merged)
						{
							if (range.lower <= relLink.second.start + 0.01 && relLink.second.end <= range.upper + 0.01) {
								return true;
							}
						}

					}

				}


			}
		}
		return false;
	}


}
