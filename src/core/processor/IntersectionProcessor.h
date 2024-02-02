#pragma once
#include "Processor.h"
namespace OMDB
{
	class IntersectionProcessor : public Processor
	{
	protected:
		virtual void process(DbMesh* const pMesh, HadGrid* pGrid) override;
		virtual void processRelation(DbMesh* const pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby) override;

	public:
		void outlinkCopy(const DbIntersection::OutLink& src, HadIntersection::OutLink& dst);
		void outlinePACopy(const DbIntersection::OutlinePA& src, HadIntersection::OutlinePA& dst);
	};
}