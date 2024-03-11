#pragma once
#include <vector>
#include "../record/DbRecord.h"
#include "../record/DbMesh.h"
namespace OMDB
{
	/// <summary>
	/// 拓扑关系生成器
	/// </summary>
	class MeshTopoBuilder
	{
	public:

		static void buildTopo(std::vector<DbLink*>& links);
		static void buildDirectTopo(DbDirectLink* pCurrent, DbDirectLink* pNext);
		static void buildLinkTopo(DbDirectLink* pCurrent, DbDirectLink* pNext);

		static void buildTopoCrossGrid(DbMesh* center,const std::vector<DbMesh*>* nearby);

		static void buildTopo(DbMesh* const grids);
		static void buildTopo(DbLink* pLink, bool crossGridOnly);
	private:
		static std::pair<int64, DbRdLinkLanePa::DbRelLink> getRelLinkPair(const DbLink* pLink, const DbRdLinkLanePa* lanePa);
		static std::vector<DbRdLinkLanePa*> getGroups(DbLink* pLink, std::vector<DbRdLinkLanePa*>& groups, int direct);
	};
}

