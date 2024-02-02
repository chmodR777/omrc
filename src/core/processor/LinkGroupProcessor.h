#pragma once
#include "Processor.h"
namespace OMDB
{
	class LinkGroupProcessor : public Processor
	{
	protected:
		virtual void process(DbMesh* const pMesh, HadGrid* pGrid) override;
		virtual void processRelation(DbMesh* const pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby) override;
	};
}

