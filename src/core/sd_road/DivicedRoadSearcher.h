#pragma once

#include "core/record/DbMesh.h"
#include "DirLinkPath.h"

namespace OMDB
{
namespace sd
{

	struct DividedRoad
	{
		// 上下行道路的左侧道路
		DirLinkPath leftPath;
		// 上下行道路的右侧道路
		DirLinkPath rightPath;
	};

	/**
	 * @brief 负责上下线搜索
	*/
	class DivicedRoadSearcher
	{
		// 当前网格mesh数据
		DbMesh* m_mesh;

		/**
		 * @brief 判断Link是否是上下行道路
		 * @param link 
		 * @return 
		*/
		static bool isDividedRoadZGen(DbLink& link);

		/**
		 * @brief 近邻搜索得到的其它link
		*/
		struct NeighborLink
		{
			DbLink* link;
			double direction; // link方向
			double directionDiff; // link与搜索输入的期望方向平行程度。
			MapPoint64 nearestPointOnLink; // link上与搜索输入的点最近的点
		};

		/**
		 * @brief 从Link上的指定点向一定距离范围内搜索其它link，并且搜索的结果按照与direction方向的平行程度从大到小排序，返回结果中越前面的link越平行。
		 * @param link 当前侧道路
		 * @param point 当前侧道路
		 * @param maxFindDist 最大搜索距离。单位经纬度。
		 * @param direction
		 * @return Array of [link, link方向，方向定义与direction相同]
		*/
		std::vector<NeighborLink> searchParallelNeighborLinks(DbLink& link, const MapPoint3D64& point, const double maxFindDist, const double direction);
		/**
		 * @brief ZGenerator中使用的抓路逻辑。抓取与有向路段相同的另一个有向路段
		 * @param grabRoadIsLeft 抓取到的对侧道路是否是沿着道路方向的左侧道路
		 * @return 如果没抓到返回无效DirLink
		*/
		DirLink grabAnotherRoadZGen(DirLink &link, bool& grabRoadIsLeft);

		/**
		 * @brief 沿DirLink顺方向和逆方向延伸搜索道路
		 * @param beginLinkOnLeft 是否是上下行的左侧或者右侧道路
		 * @return 
		*/
		DirLinkPath extendRoad(DirLink &beginLink, bool beginLinkOnLeft, std::set<int64> &visitedLinks);


	public:
		DivicedRoadSearcher(DbMesh* mesh);

		/**
		 * @brief ZGenerator中的上下行搜索逻辑
		 * @return 
		*/
		std::vector<DividedRoad> searchZGen();


	};

}
}
