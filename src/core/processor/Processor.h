#pragma once
#include "../record/DbMesh.h"
#include "../om/HadGrid.h"

#define DB_HAD_APPLY_PA_REFERENCE	 -99
#define DB_HAD_NOT_APPLY_PA_REFERENCE	 999999
#define LINK_IS_IN_TOLL_AREA (94)
#define LINK_WAY_TYPE_PA_NAME (68)
#define LINK_MULTI_DIGITIZED_PA_NAME (51)

// had_link_form属性
#define LINK_IS_IN_TUNNEL  (31)        //隧道
#define LINK_IS_IN_ROUNDABOUT (33)     //环岛
#define LINK_IS_IN_UTURN (35)          //掉头

namespace OMDB
{
	class Processor
	{
	public:
		void Process(DbMesh* const pMesh, HadGrid* pGrid);
		void ProcessRelation(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby);

	protected:
		virtual void process(DbMesh* const pMesh, HadGrid* pGrid) = 0;
		virtual void processRelation(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby) = 0;

		// 查询周边网格
		// 只有当前网格不存在数据,且周边网格存在数据的情况下返回结果
		HadElement* queryNearby(HadGrid* pGrid, std::vector<HadGrid*>* nearby, int64 id, ElementType objectType);

		// 申请HadPartAttribute
		HadPartAttribute* allocPartAttribute(HadGrid* pGrid, DbPAField& paField);

	protected:
		static bool isUTurn(HadLaneGroup* pLinkGroup);
		static bool isRoundabout(HadLaneGroup* pLinkGroup);

	};
}

