#pragma once
#include <vector>
#include "../om/HadElement.h"
#include "../om/HadGrid.h"
namespace OMDB
{
	/// <summary>
	/// 空间搜索类,当前只支持AABB包围盒重叠判断
	/// </summary>
	class SpatialSeacher
	{
	public:
		static std::vector<HadLaneGroup*> seachNearby3d(const std::vector<HadGrid*>& grids, HadLaneGroup* pGroup, int32 tolerance = 0);
		static std::vector<HadLaneGroup*> seachNearby2d(const std::vector<HadGrid*>& grids, HadLaneGroup* pGroup, int32 tolerance = 0);
		static std::vector<HadLaneGroup*> seachNearby(const std::vector<HadGrid*>& grids, const BoundingBox3d& input, int32 tolerance = 0);
		static std::vector<HadLaneGroup*> seachNearby(const std::vector<HadGrid*>& grids, const BoundingBox2d& input, int32 tolerance = 0);

        static std::vector<HadLaneGroup*> seachNearby3d(HadGrid* pGrid, HadLaneGroup* pGroup, int32 tolerance = 0);
		static std::vector<HadLaneGroup*> seachNearby2d(HadGrid* pGrid, HadLaneGroup* pGroup, int32 tolerance = 0);
		static std::vector<HadLaneGroup*> seachNearby(HadGrid* pGrid, const BoundingBox3d& input, int32 tolerance = 0);
		static std::vector<HadLaneGroup*> seachNearby(HadGrid* pGrid, const BoundingBox2d& input, int32 tolerance = 0);
	};
}

