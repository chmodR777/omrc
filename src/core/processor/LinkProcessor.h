#pragma once
#include "Processor.h"
namespace OMDB
{
	class LinkProcessor : public Processor
	{
	protected:
		virtual void process(DbMesh* const pMesh, HadGrid* pGrid) override;
		virtual void processRelation(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby) override;
	private:
		void processDataLevel(DbLink* const dbLink, HadLink* pLink);
		void processSeparation(DbLink* const dbLink, HadLink* pLink, HadGrid* pGrid);
		void processMedian(DbLink* const dbLink, HadLink* pLink, HadGrid* pGrid);
		void processOverheadObstruction(DbLink* const dbLink, HadLink* pLink, HadGrid* pGrid);
		void processTollArea(DbLink* const dbLink, HadLink* pLink, HadGrid* pGrid);

		static int LINK_DIRECT_PA_NAME;
	};
}

