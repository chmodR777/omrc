#pragma once
#include "Processor.h"
namespace OMDB
{
	class ObjectProcessor : public Processor
	{
	protected:
		virtual void process(DbMesh* const pMesh, HadGrid* pGrid) override;
		virtual void processRelation(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby) override;

		void processArrow(DbMesh* pMesh, HadGrid* pGrid);
		void processArrow(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby);

		void processFillArea(DbMesh* pMesh, HadGrid* pGrid);
		void processFillArea(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby);

		void processCrossWalk(DbMesh* pMesh, HadGrid* pGrid);
		void processCrossWalk(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby);

		void processTrafficSign(DbMesh* pMesh, HadGrid* pGrid);
		void processTrafficSign(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby);

		void processStopLocation(DbMesh* pMesh, HadGrid* pGrid);
		void processStopLocation(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby);

		void processText(DbMesh* pMesh, HadGrid* pGrid);
		void processText(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby);

		void processBarrier(DbMesh* pMesh, HadGrid* pGrid);
		void processBarrier(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby);

		void processWall(DbMesh* pMesh, HadGrid* pGrid);
		void processWall(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby);


		void processTrafficLights(DbMesh* pMesh, HadGrid* pGrid);
		void processTrafficLights(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby);

		void processPole(DbMesh* pMesh, HadGrid* pGrid);
		void processPole(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby);
	};
}

