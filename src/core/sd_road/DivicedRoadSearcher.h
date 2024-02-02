#pragma once

#include "core/record/DbMesh.h"
#include "DirLinkPath.h"

namespace OMDB
{
namespace sd
{

	struct DividedRoad
	{
		// �����е�·������·
		DirLinkPath leftPath;
		// �����е�·���Ҳ��·
		DirLinkPath rightPath;
	};

	/**
	 * @brief ��������������
	*/
	class DivicedRoadSearcher
	{
		// ��ǰ����mesh����
		DbMesh* m_mesh;

		/**
		 * @brief �ж�Link�Ƿ��������е�·
		 * @param link 
		 * @return 
		*/
		static bool isDividedRoadZGen(DbLink& link);

		/**
		 * @brief ���������õ�������link
		*/
		struct NeighborLink
		{
			DbLink* link;
			double direction; // link����
			double directionDiff; // link�������������������ƽ�г̶ȡ�
			MapPoint64 nearestPointOnLink; // link������������ĵ�����ĵ�
		};

		/**
		 * @brief ��Link�ϵ�ָ������һ�����뷶Χ����������link�����������Ľ��������direction�����ƽ�г̶ȴӴ�С���򣬷��ؽ����Խǰ���linkԽƽ�С�
		 * @param link ��ǰ���·
		 * @param point ��ǰ���·
		 * @param maxFindDist ����������롣��λ��γ�ȡ�
		 * @param direction
		 * @return Array of [link, link���򣬷�������direction��ͬ]
		*/
		std::vector<NeighborLink> searchParallelNeighborLinks(DbLink& link, const MapPoint3D64& point, const double maxFindDist, const double direction);
		/**
		 * @brief ZGenerator��ʹ�õ�ץ·�߼���ץȡ������·����ͬ����һ������·��
		 * @param grabRoadIsLeft ץȡ���ĶԲ��·�Ƿ������ŵ�·���������·
		 * @return ���ûץ��������ЧDirLink
		*/
		DirLink grabAnotherRoadZGen(DirLink &link, bool& grabRoadIsLeft);

		/**
		 * @brief ��DirLink˳������淽������������·
		 * @param beginLinkOnLeft �Ƿ��������е��������Ҳ��·
		 * @return 
		*/
		DirLinkPath extendRoad(DirLink &beginLink, bool beginLinkOnLeft, std::set<int64> &visitedLinks);


	public:
		DivicedRoadSearcher(DbMesh* mesh);

		/**
		 * @brief ZGenerator�е������������߼�
		 * @return 
		*/
		std::vector<DividedRoad> searchZGen();


	};

}
}
