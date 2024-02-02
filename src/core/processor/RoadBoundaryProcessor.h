#pragma once
#include "Processor.h"
namespace OMDB
{
	class RoadBoundaryProcessor : public Processor
	{
	protected:
		virtual void process(DbMesh* const pMesh, HadGrid* pGrid) override;
		virtual void processRelation(DbMesh* const pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby) override;

	};
}

