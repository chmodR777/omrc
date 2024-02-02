#pragma once
#include <vector>
#include "../om/HadElement.h"
#include "../om/HadGrid.h"
namespace OMDB
{
	/// <summary>
	/// �ռ�������,��ǰֻ֧��AABB��Χ���ص��ж�
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

