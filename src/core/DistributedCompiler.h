#pragma once
#include "om/HadElement.h"
#include "navi_rds/Entity.h"
#include "navi_rds/Routing.h"
#include "record/DbMesh.h"
#include "compiler/Compiler.h"
namespace OMDB
{
	class DistributedCompiler
	{
	public:
		void execute();
	protected:
		void compile(HadGrid* pGrid, std::vector<HadGrid*>& nearby, 
			RDS::RdsTile* pTile, RDS::RoutingTile* pRoutingTile);

		void process(DbMesh* pMesh, HadGrid* pGrid);
		void processRelation(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby);
		void writeBinaryHeader(const char* path, const std::vector<uint32>& ids);
		void writeBinaryMesh(const char* path, const std::map<uint32, Blob>& data);
	};

}

