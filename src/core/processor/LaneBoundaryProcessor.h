#pragma once
#include "Processor.h"
namespace OMDB
{
	class LaneBoundaryProcessor : public Processor
	{
	protected:
		virtual void process(DbMesh* const pMesh, HadGrid* pGrid);
		virtual void processRelation(DbMesh* const pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby) override;

	};
}

