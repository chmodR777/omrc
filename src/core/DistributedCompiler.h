#pragma once
#include "om/HadElement.h"
#include "navi_rds/Entity.h"
#include "record/DbMesh.h"
namespace OMDB
{
	class DistributedCompiler
	{
		struct Blob
		{
			void* data;
			size_t size;
		};
	public:
		void execute();
	protected:
		void compile(HadGrid* pGrid, std::vector<HadGrid*>& nearby, RDS::RdsTile* pTile);

		void generate(DbMesh* pMesh, std::vector<DbMesh*>* nearby);
		void process(DbMesh* pMesh, HadGrid* pGrid);
		void processRelation(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby);
		void writeBinaryHeader(const char* path, const std::vector<uint32>& ids);
		void writeBinaryMesh(const char* path, const std::map<uint32, Blob>& data);
	};

}

