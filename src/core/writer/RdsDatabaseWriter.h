#pragma once
#include "navi_rds/Entity.h"
namespace OMDB
{
	class RdsDatabaseWriter
	{
	public:
		void writeBinary(const char* path, RDS::RdsDatabase* pDatabase);
		void writeSpatialite(const std::string& directory,RDS::RdsDatabase* pDatabase);
		void writeSpatialite(const std::string& directory, std::vector<RDS::RdsTile*>& tiles);
		void writeSpatialite(const std::string& directory, RDS::RdsTile* pTile);
		static std::string base64EncodedGridList(std::vector<uint32>& gridIdList);
			
	protected:
		void writeBinaryHeader(sqlite3* pDb, RDS::RdsDatabase* pDatabase);
		void writeBinaryMesh(sqlite3* pDb, RDS::RdsDatabase* pDatabase);
	};
}

