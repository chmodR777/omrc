#pragma once
#include <queue>
#include <map>
#include <unordered_map>
#include "record/DbMesh.h"
#include "core/om/HadGrid.h"
namespace OMDB
{
	class MeshCache
	{
	private:
		std::map<uint32, DbMesh*> m_data;
		std::map<uint32, std::vector<uint32>> m_mapNearby;
		void load(uint32 meshId,DbMesh* pMesh);
		void searchNearBy(uint32 id, std::vector<uint32>& nearby);
		void loadOmdb(const std::vector<uint32>& ids);
		void loadOmrp(const std::vector<uint32>& ids);
		void generateZ(DbMesh* pMesh);
		void generateWithZRelation();
		void generateRelation();
	public:
		~MeshCache();
		void query(uint32 meshId, DbMesh*& center);
		void query(uint32 meshId, DbMesh*& center, std::vector<DbMesh*>& nearby);
		void reset(const std::vector<uint32>& ids);
	};
}


