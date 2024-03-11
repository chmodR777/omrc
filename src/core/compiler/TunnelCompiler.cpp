#include "stdafx.h"
#include <functional>
#include "TunnelCompiler.h"
#include "grap_point_algorithm.h"
#include <queue>
#include "polyline_simplifier.h"
#include "../framework/SpatialSeacher.h"
#include "clipper.hpp"
#include "algorithm/geography_algorithm.h"
namespace OMDB
{

	forceinline bool tunnelMapPoint3D64_compare(MapPoint3D64 a, MapPoint3D64 b)
	{
		auto pa = point_t(a.pos.lon, a.pos.lat, static_cast<int64>(a.z));
		auto pb = point_t(b.pos.lon, b.pos.lat, static_cast<int64>(b.z));
		if (bg::distance(pa, pb) < 50) {
			return true;
		}
		else {
			return false;
		}
	}

	template <typename className>
	className *TunnelCompiler::ALLOC_ELEMENT()
	{
		void *pData = m_batchedAllocator.allocMemory(sizeof(className));
		className *pElement = new (pData) className;
		return pElement;
	}

	RdsTunnel::TunnelType ToRsdTunnelType(tunnel_type type)
	{
		RdsTunnel::TunnelType t = RdsTunnel::TunnelType::UNKNOWN;
		switch (type)
		{
		case OMDB::tunnel_type_N:
			t = RdsTunnel::TunnelType::N;
			break;
		case OMDB::tunnel_type_Y:
			t = RdsTunnel::TunnelType::Y;
			break;
		case OMDB::tunnel_type_N_S:
			t = RdsTunnel::TunnelType::N_S;
			break;
		case OMDB::tunnel_type_N_E:
			t = RdsTunnel::TunnelType::N_E;
			break;
		case OMDB::tunnel_type_N_B:
			t = RdsTunnel::TunnelType::N_B;
			break;
		default:
			break;
		}
		return t;
	}

	void TunnelCompiler::compile(HadGrid *const pGrid, const std::vector<HadGrid *> &nearby, RdsTile *pTile)
	{
		if (CompileSetting::instance()->isDaimlerShangHai)
			return;
	
		m_pGrid = pGrid;
		m_pTile = pTile;
		m_batchedAllocator.initWithBlockSize(4096);

		grabTunnel(pGrid, nearby);
		makeTopology();
		calcLineAndBox();
		divideToGroup();
		createTunnel();
		splitYTunnel();
		splitToGrid();
		calcThicknessAndHeight(nearby);
		storeToGrid();

		m_batchedAllocator.freeAll();
	}

	void TunnelCompiler::grabTunnel(HadGrid *const pGrid, const std::vector<HadGrid *> &nearby)
	{
//#define  USE_FILTER
#ifdef USE_FILTER
		FastTable<int64, int> filterID;
		filterID.insert(157974269251754242, 157974269251754242);
		filterID.insert(127893310993273345, 127893310993273345);
		
#endif USE_FILTER
	
		auto add2Tunnelink = [&](HadLaneGroup *laneGroup)
		{
#ifdef USE_FILTER
			if (!filterID.exist(laneGroup->originId))
			{
				return;
			}
#endif USE_FILTER
			if (!m_laneGroup2Tunnel.exist(laneGroup))
			{
				TunnelLink *pElement = ALLOC_ELEMENT<TunnelLink>();
				pElement->initTunnelLink(laneGroup);
				m_tunnelLinkArray.emplace_back(pElement);
				m_laneGroup2Tunnel.insert(laneGroup, pElement);
			}
		};

		for (auto &pElement : pGrid->query(ElementType::HAD_LANE_GROUP))
		{
			HadLaneGroup *pLaneGroup = (HadLaneGroup *)pElement;

			//判断是否为普通路
			if (CompileSetting::instance()->isNotCompileUrbanData)
			{
				if (!isProDataLevel(pLaneGroup))
					continue;
			}

			if (isTunnelArea(pLaneGroup))
			{
				add2Tunnelink(pLaneGroup);
			}
		}

		for (auto nearGrid : nearby)
		{
			for (auto &pElement : nearGrid->query(ElementType::HAD_LANE_GROUP))
			{
				HadLaneGroup *pLaneGroup = (HadLaneGroup *)pElement;

				//判断是否为普通路
				if (CompileSetting::instance()->isNotCompileUrbanData)
				{
					if (!isProDataLevel(pLaneGroup))
						continue;
				}

				if (isTunnelArea(pLaneGroup))
				{
					add2Tunnelink(pLaneGroup);
				}
			}
		}
	}

	void TunnelCompiler::calcLineAndBox()
	{
		for (auto tunnelLink : m_tunnelLinkArray)
		{
			HadLaneGroup *laneGroup = tunnelLink->_laneGroup;

			MultiLineString3d lines;
			HadLane *hadLane = nullptr;
			HadLaneBoundary *hadLaneBoundary = nullptr;
			getLaneGroupLines(laneGroup, lines, hadLane, hadLaneBoundary);
			tunnelLink->_middleLane = hadLane;
			tunnelLink->_middleLaneBoundary = hadLaneBoundary;

			if (lines.lines.size() == 3)
			{
				tunnelLink->_leftLine = lines.lines[0];
				tunnelLink->_middleLine = lines.lines[1];
				tunnelLink->_rightLine = lines.lines[2];
			}

			for (int32 i = 0; i < tunnelLink->_leftLine.vertexes.size(); ++i)
			{
				tunnelLink->box.combinePoint(tunnelLink->_leftLine.vertexes[i].pos);
			}

			for (int32 i = 0; i < tunnelLink->_rightLine.vertexes.size(); ++i)
			{
				tunnelLink->box.combinePoint(tunnelLink->_rightLine.vertexes[i].pos);
			}

			if (tunnelLink->box.isValid())
			{
				tunnelLink->boxEx = tunnelLink->box;
				tunnelLink->boxEx.expand(49 * 9);
			}
		}
	}

	void TunnelCompiler::calcThicknessAndHeight(const std::vector<HadGrid*>& nearByGrid)
	{
		
		std::set<TunnelLink *> calcTunnelink;
		for (int32 i = 0; i < m_NTunnelArray.size(); i++)
		{
			NTunnel *nTunnel = m_NTunnelArray[i];
			
			if (m_tunnelGroups.exist(nTunnel->groupId)
				&& m_tunnelGroups[nTunnel->groupId]->hasInGrid(m_pGrid->getId()))
			{
				for (auto tunnel : m_tunnelGroups[nTunnel->groupId]->tunnnels)
				{
					calcTunnelink.insert(tunnel);
				}
			}

			
			
		}
		for (int32 i = 0; i < m_YTunnelArray.size(); i++)
		{
			YTunnel *yTunnel = m_YTunnelArray[i];
			if (m_tunnelGroups.exist(yTunnel->groupId) &&
				m_tunnelGroups[yTunnel->groupId]->hasInGrid(m_pGrid->getId()))
			{
				for (auto tunnel : m_tunnelGroups[yTunnel->groupId]->tunnnels)
				{
					calcTunnelink.insert(tunnel);
				}
			}
			
	
		}

		for (auto tunnel : calcTunnelink)
		{
			std::vector<TunnelLinkGlandHeightInfo> heightInfos;
			MapPoint3D64 *pCurPolygon = NULL;
			size_t nCurPolygonPtNum = 0;
			HadGrid *pGrid = tunnel->_laneGroup->owner;
			if (!pGrid)
			{
				continue;
			}
			// 空间搜索。仅通过地物对象的BoundingBox2d空间关系搜索。
			std::vector<HadGrid*> searchGrid = nearByGrid;
			searchGrid.emplace_back(m_pGrid);
			std::vector<HadLaneGroup*> pNearbyLinkGroups = SpatialSeacher::seachNearby2d(searchGrid, tunnel->_laneGroup);

			if (pNearbyLinkGroups.size() > 0)
				nCurPolygonPtNum = makeLaneGroupPolygon(tunnel->_laneGroup, pCurPolygon); // 创建当前车道组面闭合多边形。

			if (!nCurPolygonPtNum)
			{
				continue;
			}

			for (HadLaneGroup *pNearbyLinkGroup : pNearbyLinkGroups)
			{
				

				
				// 相连的
				if (isTouchWithHadLaneGroup(pNearbyLinkGroup, tunnel->_laneGroup))
				{
					continue;
				}
				// LA
				HadRelLaneGroupAssociation laneGroupAsso;
				if (isLALaneGroup(pNearbyLinkGroup, tunnel->_laneGroup, laneGroupAsso))
				{
					continue;
				}


				int32 height = TUNNEL_HEIGHT;
				if (hasOverLap(tunnel, pCurPolygon, nCurPolygonPtNum, pNearbyLinkGroup, height, heightInfos))
				{
					tunnel->height = min(height, tunnel->height);
					continue;
				}

				MapPoint3D64* pNearByPolygon = NULL;
				size_t nNearByPolygonPtNum = 0;
				MapPoint3D64 pt1Closest, pt2Closest;

				nNearByPolygonPtNum = makeLaneGroupPolygon(pNearbyLinkGroup, pNearByPolygon);

				if (!nNearByPolygonPtNum)
				{
					continue;
				}
			

				//是否平行.有上下层,但不压盖
				if ( getClosePoint(pCurPolygon, nCurPolygonPtNum, pNearByPolygon, nNearByPolygonPtNum, pt1Closest, pt2Closest))
				{
					if (abs(pt2Closest.z - pt1Closest.z) > 100)
					{
						continue;
					}
				}

				if(abs(height) < 1000 || height == TUNNEL_HEIGHT)
				{
					// 平行判断
	
					if (hasIntersectExtend(pCurPolygon, nCurPolygonPtNum, pNearByPolygon, nNearByPolygonPtNum))
					{
						int32 thickNess = TUNNEL_THICKNESS;
						float distance = getDistance(pCurPolygon, nCurPolygonPtNum, pNearByPolygon, nNearByPolygonPtNum);

						thickNess = distance / 2;
						if (thickNess <=0)
						{
							thickNess = 0;
						}
						
						thickNess = min(thickNess, TUNNEL_THICKNESS);
						tunnel->thickness = min(tunnel->thickness, thickNess);
						if (tunnel->isYBranchInterface)
						{
							tunnel->thickness = max(tunnel->thickness, 50);
						}
					}
				}
			}

			mergeHighInfo(heightInfos);
			tunnel->linkGlandHeightInfos = heightInfos;
		}
	}
	

	void TunnelCompiler::makeTopology()
	{
		for (auto tunnel : m_tunnelLinkArray)
		{
			for (auto laneGroup : tunnel->_laneGroup->previous)
			{
				if (m_laneGroup2Tunnel.exist((HadLaneGroup *)laneGroup))
				{
					tunnel->_previous.emplace_back(m_laneGroup2Tunnel[(HadLaneGroup *)laneGroup]);
				}
			}
			for (auto laneGroup : tunnel->_laneGroup->next)
			{
				if (m_laneGroup2Tunnel.exist((HadLaneGroup *)laneGroup))
				{
					tunnel->_next.emplace_back(m_laneGroup2Tunnel[(HadLaneGroup *)laneGroup]);
				}
			}
		}
	}

	void TunnelCompiler::divideToGroup()
	{
		auto bfs_link = [&](TunnelLink *startlink, std::set<TunnelLink *> &linksOut)
		{
			linksOut.clear();

			std::queue<TunnelLink *> queueLink;

			queueLink.push(startlink);

			int depthMax = 0;
			while (!queueLink.empty())
			{
				TunnelLink *link = queueLink.front();
				queueLink.pop();

				linksOut.emplace(link);

				// 插入队列尾部
				for (auto link : link->_next)
				{
					if (linksOut.find(link) == linksOut.end())
					{
						queueLink.push(link);
					}
				}
				for (auto link : link->_previous)
				{
					if (linksOut.find(link) == linksOut.end())
					{
						queueLink.push(link);
					}
				}
			}
		};
		std::set<TunnelLink *> groupedLinks;

		// 编号
		int32 groupId = 1;
		for (auto tunnelLink : m_tunnelLinkArray)
		{

			if (groupedLinks.find(tunnelLink) == groupedLinks.end())
			{
				std::set<TunnelLink *> bfs_linksOut;
				// 广度遍历拓扑
				bfs_link(tunnelLink, bfs_linksOut);

				if (!bfs_linksOut.empty())
				{
					TunnelGroup *tunnelLinkGroup = ALLOC_ELEMENT<TunnelGroup>();
					for (auto tunnel : bfs_linksOut)
					{
						groupedLinks.emplace(tunnel);

						tunnel->groupId = groupId;
						tunnelLinkGroup->tunnnels.emplace(tunnel);
					}
					tunnelLinkGroup->groupId = groupId;
					m_tunnelGroups.insert(groupId, tunnelLinkGroup);
					groupId++;
				}
			}
		}

		// 统计分叉数
		for (auto tunnelGroup : m_tunnelGroups.values())
		{
			bool hasFork = false;
			for each (auto &tunnelLink in tunnelGroup->tunnnels)
			{

				if (tunnelLink->_previous.empty())
				{
					tunnelGroup->headlinks.insert(tunnelLink);
				}
				if (tunnelLink->_previous.size() > 1 || tunnelLink->_next.size() > 1)
				{
					hasFork = true;
				}
				// 只有1边是隧道
				if ((tunnelLink->_previous.size() && tunnelLink->_previous.size() < tunnelLink->_laneGroup->previous.size()) ||
					(tunnelLink->_next.size() && tunnelLink->_next.size() < tunnelLink->_laneGroup->next.size()))
				{
					hasFork = true;
				}
			}
			tunnelGroup->hasFork = hasFork;
		}
	}

	void TunnelCompiler::createTunnel()
	{
		for (auto tunnelGroup : m_tunnelGroups.values())
		{
			
			if (tunnelGroup->hasFork)
			{
				// Y
				createYLinkGroup(tunnelGroup);
			}
			else
			{ // N
				createNLinkGroup(tunnelGroup);
			}
		}
	}

	void TunnelCompiler::splitYTunnel()
	{

		int meshId = m_pGrid->getId();
		for (auto yTunnel : m_YTunnelArray)
		{

			if (!yTunnel->left.empty())
			{
				NTunnel *newNTunnel = ALLOC_ELEMENT<NTunnel>();
				newNTunnel->tunnnels = yTunnel->left;
				newNTunnel->groupId = yTunnel->groupId;
				// 改为顺序
				if (yTunnel->BranchType() == enum_tunnel_merge)
				{
					std::reverse(newNTunnel->tunnnels.begin(), newNTunnel->tunnnels.end());
					newNTunnel->setYBranchFromEnd();
					if (newNTunnel->hasPreviousYTunnelFork())
						newNTunnel->setYBranchFromStart();
				}
				else if (yTunnel->BranchType() == enum_tunnel_fork)
				{
					newNTunnel->setYBranchFromStart();
					if (newNTunnel->hasNextYTunnelMerge())
						newNTunnel->setYBranchFromEnd();
				}

				newNTunnel->setBelongYTunnel();
				m_NTunnelArray.emplace_back(newNTunnel);
				yTunnel->left.clear();
			}
			if (!yTunnel->right.empty())
			{
				NTunnel *newNTunnel = ALLOC_ELEMENT<NTunnel>();
				newNTunnel->tunnnels = yTunnel->right;
				newNTunnel->groupId = yTunnel->groupId;
				// 改为顺序
				if (yTunnel->BranchType() == enum_tunnel_merge)
				{
					std::reverse(newNTunnel->tunnnels.begin(), newNTunnel->tunnnels.end());
					newNTunnel->setYBranchFromEnd();
					if (newNTunnel->hasPreviousYTunnelFork())
						newNTunnel->setYBranchFromStart();
				}
				else if (yTunnel->BranchType() == enum_tunnel_fork)
				{
					newNTunnel->setYBranchFromStart();
					if (newNTunnel->hasNextYTunnelMerge())
						newNTunnel->setYBranchFromEnd();
				}

				newNTunnel->setBelongYTunnel();

				m_NTunnelArray.emplace_back(newNTunnel);
				yTunnel->right.clear();
			}

			if (yTunnel->main->main.size() > 1)
			{
				std::vector<TunnelLink *> splitTunnles;
				if (!yTunnel->main->leftExpand.empty() || !yTunnel->main->rightExpand.empty())
				{
					splitTunnles.assign(yTunnel->main->main.begin(), yTunnel->main->main.end());
					yTunnel->main->main.clear();
				}
				else
				{
					splitTunnles.assign(yTunnel->main->main.begin() + 1, yTunnel->main->main.end());
					yTunnel->main->main.resize(1);
				}

				NTunnel *newNTunnel = ALLOC_ELEMENT<NTunnel>();
				newNTunnel->tunnnels = splitTunnles;
				newNTunnel->groupId = yTunnel->groupId;
				// 改为顺序
				if (yTunnel->BranchType() == enum_tunnel_fork)
				{
					std::reverse(newNTunnel->tunnnels.begin(), newNTunnel->tunnnels.end());
				}

				newNTunnel->setBelongYTunnel();

				m_NTunnelArray.emplace_back(newNTunnel);
			}
		}
	}

	void TunnelCompiler::splitToGrid()
	{
		auto spliteNTunnel = [&](std::vector<TunnelLink *> &links) -> std::vector<std::vector<TunnelLink *>>
		{
			std::vector<std::vector<TunnelLink *>> splitArray(0);
			int gridId = links.at(0)->gridId();
			std::vector<TunnelLink *> tunnnels;
			for (auto tunnel : links)
			{
				if (gridId == tunnel->gridId())
				{
					tunnnels.emplace_back(tunnel);
				}
				else
				{
					splitArray.emplace_back(tunnnels);
					tunnnels.clear();
					tunnnels.emplace_back(tunnel);
					gridId = tunnel->gridId();
				}
			}

			if (!tunnnels.empty())
			{
				splitArray.emplace_back(tunnnels);
			}

			return splitArray;
		};

		int meshId = m_pGrid->getId();
		std::vector<NTunnel *> tunnelArray;
		for (int32 i = 0; i < m_NTunnelArray.size(); i++)
		{
			NTunnel *nTunnel = m_NTunnelArray[i];

			if (nTunnel->InSameGrid())
			{
				tunnelArray.emplace_back(nTunnel);
				continue;
			}

			std::vector<std::vector<TunnelLink *>> splitArray = spliteNTunnel(nTunnel->tunnnels);

			int index = 0;
			for (auto links : splitArray)
			{

				NTunnel *newNTunnel = ALLOC_ELEMENT<NTunnel>();

				newNTunnel->tunnnels = links;
				newNTunnel->groupId = nTunnel->groupId;
				if (nTunnel->isBelongYTunnel())
				{
					newNTunnel->setBelongYTunnel();
				}
				if (index == 0 && nTunnel->startisYBranch())
				{

					newNTunnel->setYBranchFromStart();
				}
				if (index == splitArray.size() - 1 && nTunnel->endIsYbranch())
				{

					newNTunnel->setYBranchFromEnd();
				}
				tunnelArray.emplace_back(newNTunnel);
				index++;
			}
		}

		m_NTunnelArray = tunnelArray;
	}

	void TunnelCompiler::storeToGrid()
	{

		std::vector<RdsTunnel *> rdsTunnelVector;

		for (int32 i = 0; i < m_NTunnelArray.size(); i++)
		{
			
			NTunnel *nTunnel = m_NTunnelArray[i];


			if (!nTunnel->hasInGrid(m_pGrid->getId()))
			{
				continue;
			}
		
			// 低于6.5米,厚度低于30cm不再输出
			if (nTunnel->needBeDisable(nullptr))
			{
				// addDisabledTunnel(nTunnel);
				continue;
			}

			if (createRdsTunnel(nTunnel))
			{
				nTunnel->foreachTunnel([&](TunnelLink* tunnel)
					{ 
						if (tunnel->_laneGroup->owner->getId() == m_pGrid->getId())
						{
							RdsGroup* pRdsGroup = queryGroup(tunnel->_laneGroup->originId, m_pTile);
							if (pRdsGroup)
								pRdsGroup->isCreatedTunnel = true;		
						}
					
					});
				
			}
			
		}
		for (int32 i = 0; i < m_YTunnelArray.size(); i++)
		{
		
		
			YTunnel *yTunnel = m_YTunnelArray[i];
		

			if (!yTunnel->hasInGrid(m_pGrid->getId()))
			{
				continue;
			}
		
			// 低于6.5米,厚度低于30cm不再输出
			if (yTunnel->needBeDisable(nullptr))
			{
				// addDisabledTunnel(nTunnel);
				continue;
			}

			if (createRdsTunnel(yTunnel))
			{
				yTunnel->foreachTunnel([&](TunnelLink* tunnel)
					{
						if (tunnel->_laneGroup->owner->getId() == m_pGrid->getId())
						{
							RdsGroup* pRdsGroup = queryGroup(tunnel->_laneGroup->originId, m_pTile);
							if (pRdsGroup)
								pRdsGroup->isCreatedTunnel = true;		
						}

					});
			}
			;
		}
	}

	// 父类的makeLaneGroupPolygon使用的车道边界,因车道边界点虚和车道同行方向可能相反,无法鉴别
	// 子类只使用道路边界来组成道路面.
	size_t TunnelCompiler::makeLaneGroupPolygon(HadLaneGroup* pGroup, MapPoint3D64*& polygon)
	{
		if (pGroup->roadBoundaries.size() >= 2)
		{

		size_t roadBoundaryNum = pGroup->roadBoundaries.size();
		size_t nPolyline1PtNum = pGroup->roadBoundaries[0]->location.vertexes.size();
		size_t nPolyline2PtNum = pGroup->roadBoundaries[roadBoundaryNum - 1]->location.vertexes.size();
	
		//添加路面左边线几何点。
		std::vector<MapPoint3D64> Line1Reverse(nPolyline1PtNum);
		if (directionEqual(pGroup->roadBoundaries[0], pGroup, 3))
			std::reverse_copy(pGroup->roadBoundaries[0]->location.vertexes.begin(), pGroup->roadBoundaries[0]->location.vertexes.end(), Line1Reverse.begin());
		else
		{
			std::copy(pGroup->roadBoundaries[0]->location.vertexes.begin(), pGroup->roadBoundaries[0]->location.vertexes.end(), Line1Reverse.begin());

		}	
		//添加路面右边线几何点。
		std::vector<MapPoint3D64> Line2Reverse(nPolyline2PtNum);
		if (directionEqual(pGroup->roadBoundaries[roadBoundaryNum - 1], pGroup, 2))
			std::reverse_copy(pGroup->roadBoundaries[roadBoundaryNum - 1]->location.vertexes.begin(), pGroup->roadBoundaries[roadBoundaryNum - 1]->location.vertexes.end(), Line2Reverse.begin());
		else
		{
			std::copy(pGroup->roadBoundaries[roadBoundaryNum - 1]->location.vertexes.begin(), pGroup->roadBoundaries[roadBoundaryNum - 1]->location.vertexes.end(), Line2Reverse.begin());

		}


		size_t nPtNum = Line1Reverse.size() + Line2Reverse.size() +1;
		polygon = new MapPoint3D64[nPtNum];
		
		memcpy(polygon, Line1Reverse.data(), Line1Reverse.size() * sizeof(MapPoint3D64));
		memcpy(polygon+ Line1Reverse.size(), Line2Reverse.data(), Line2Reverse.size() * sizeof(MapPoint3D64));
		polygon[nPtNum - 1] = Line1Reverse.front();

		return nPtNum;
	

	}
	if (pGroup->laneBoundaries.size() >= 2)
		{

			size_t laneBoundaryNum = pGroup->laneBoundaries.size();
			size_t nPolyline1PtNum = pGroup->laneBoundaries[0]->location.vertexes.size();
			size_t nPolyline2PtNum = pGroup->laneBoundaries[laneBoundaryNum - 1]->location.vertexes.size();

			//添加路面左边线几何点。
			std::vector<MapPoint3D64> Line1Reverse(nPolyline1PtNum);
			if (directionEqual(pGroup->laneBoundaries[0], pGroup, 3))
				std::reverse_copy(pGroup->laneBoundaries[0]->location.vertexes.begin(), pGroup->laneBoundaries[0]->location.vertexes.end(), Line1Reverse.begin());
			else
			{
				std::copy(pGroup->laneBoundaries[0]->location.vertexes.begin(), pGroup->laneBoundaries[0]->location.vertexes.end(), Line1Reverse.begin());

			}
			//添加路面右边线几何点。
			std::vector<MapPoint3D64> Line2Reverse(nPolyline2PtNum);
			if (directionEqual(pGroup->laneBoundaries[laneBoundaryNum - 1], pGroup, 2))
				std::reverse_copy(pGroup->laneBoundaries[laneBoundaryNum - 1]->location.vertexes.begin(), pGroup->laneBoundaries[laneBoundaryNum - 1]->location.vertexes.end(), Line2Reverse.begin());
			else
			{
				std::copy(pGroup->laneBoundaries[laneBoundaryNum - 1]->location.vertexes.begin(), pGroup->laneBoundaries[laneBoundaryNum - 1]->location.vertexes.end(), Line2Reverse.begin());

			}


			size_t nPtNum = Line1Reverse.size() + Line2Reverse.size() + 1;
			polygon = new MapPoint3D64[nPtNum];

			memcpy(polygon, Line1Reverse.data(), Line1Reverse.size() * sizeof(MapPoint3D64));
			memcpy(polygon + Line1Reverse.size(), Line2Reverse.data(), Line2Reverse.size() * sizeof(MapPoint3D64));
			polygon[nPtNum - 1] = Line1Reverse.front();

			return nPtNum;
		}
		return 0;
	}

	bool TunnelCompiler::createRdsTunnel(NTunnel* nTunnel)
	{

		if (nTunnel == nullptr)
		{
			return false;
		}

		if (nTunnel->tunnnels.empty())
		{
			return false;
		}

		MultiLineString3d lineVector;
		MultiLineString3d lineShape;
		LineString3d middleLine;
		getTunnelsShape(nTunnel->tunnnels, false, lineShape);
		getTunnelsMiddleLines(nTunnel->tunnnels, false, middleLine);

	
		if (lineShape.lines.size() != 2 || middleLine.vertexes.empty())
		{
			return false;
		}
		//抽希
		//DilutionTunnel(lineShape.lines[0].vertexes);
		//DilutionTunnel(lineShape.lines[1].vertexes);

		lineVector.lines.emplace_back(lineShape.lines[0]);
		lineVector.lines.emplace_back(middleLine);
		lineVector.lines.emplace_back(lineShape.lines[1]);
		for (auto& line : lineVector.lines)
		{
			auto& linePoints = line.vertexes;
			auto ip_line = std::unique(linePoints.begin(), linePoints.end(), tunnelMapPoint3D64_compare);
			linePoints.resize(std::distance(linePoints.begin(), ip_line));
		}

		std::vector<RdsTunnel::HeightInfo> heightInfo;
		getCourHeightInfo(nTunnel->tunnnels, lineVector.lines[1], heightInfo);
	
		//和前方隧道对象保持统一高度
		double NearByHeight;
		if (nTunnel->tunnelType() != tunnel_type_N_E)
		{
			getNerbyHeightInfo(nTunnel->tunnnels.back(), false, true, NearByHeight);
			if (NearByHeight != TUNNEL_HEIGHT)
			{
				if (heightInfo.empty() ||  heightInfo.back().endIndex+1 != lineVector.lines[1].vertexes.size())
				{
					heightInfo.emplace_back(RdsTunnel::HeightInfo{ (uint16)(lineVector.lines[1].vertexes.size() - 2),(uint16)(lineVector.lines[1].vertexes.size() - 1),(uint32)NearByHeight });
				}
				
			}
		}
		if (nTunnel->tunnelType() != tunnel_type_N_S)
		{
			getNerbyHeightInfo(nTunnel->tunnnels.front(), false, false, NearByHeight);
			if (NearByHeight != TUNNEL_HEIGHT)
			{
				if (heightInfo.empty() ||  heightInfo.front().startIndex != 0)
				{
					//heightInfo.insert(heightInfo.begin(), RdsTunnel::HeightInfo{ 0,1,(uint32)NearByHeight });heightInfo.front().height = min(heightInfo.front().height, NearByHeight);
					heightInfo.insert(heightInfo.begin(), RdsTunnel::HeightInfo{ 0,1,(uint32)NearByHeight });
				}
				
			}
		}

		RdsTunnel* rdsTunnel = (RdsTunnel*)createObject(m_pTile, EntityType::RDS_TUNNEL);
		// 创建N型隧道补面
		// createTunnelPatch(nTunnel, lineVector);
		convert(lineVector, rdsTunnel->contour);

		if (!heightInfo.empty())
		{
			rdsTunnel->heightInfos.emplace_back(heightInfo);
		}

		//end

		uint32 thickness = nTunnel->mixThickNess();
		uint32 height = nTunnel->mixHeight();
		TunnelGroup *tunnelGroup = nullptr;
		if (m_tunnelGroups.exist(nTunnel->groupId))
		{
			thickness = m_tunnelGroups[nTunnel->groupId]->mixthickness();
		}

		rdsTunnel->thickness = thickness;
		rdsTunnel->height = height;
		rdsTunnel->tunnelType = ToRsdTunnelType(nTunnel->tunnelType());

		return true;
	}

	bool TunnelCompiler::createRdsTunnel(YTunnel *yTunnel)
	{

		if (yTunnel == nullptr)
		{
			return false;
		}

		// 设置主要线
		setYTunnelMixExtendTunnelGeoLine(yTunnel->main);

		if (yTunnel->main->_leftLine.vertexes.empty() || yTunnel->main->_rightLine.vertexes.empty() || yTunnel->main->_middleLine.vertexes.empty())
		{
			return false;
		}
		std::vector<RdsTunnel::HeightInfo> heightInfoMain;
		std::vector<RdsTunnel::HeightInfo> heightInfoLeft;
		std::vector<RdsTunnel::HeightInfo> heightInfoRight;

		std::vector<TunnelLink *> tunnelLinksMain;
		yTunnel->main->foreachTunnel([&](TunnelLink *tunnel)
									 { tunnelLinksMain.emplace_back(tunnel); });
		// 隧道的主干的左右两边点抽稀
		// DilutionTunnel(yTunnel->main->_leftLine->points, yTunnel->main->_leftLine->pointCount);
		// DilutionTunnel(yTunnel->main->_rightLine->points, yTunnel->main->_rightLine->pointCount);
		MultiLineString3d lineVectors;
		lineVectors.lines.emplace_back(yTunnel->main->_leftLine);
		lineVectors.lines.emplace_back(yTunnel->main->_middleLine);
		lineVectors.lines.emplace_back(yTunnel->main->_rightLine);

		getCourHeightInfo(tunnelLinksMain, lineVectors.lines[1], heightInfoMain,false);

		double NearByHeight;
		getNerbyHeightInfo(yTunnel->main->mixYtunnel->main, yTunnel->main->main.empty(), yTunnel->BranchType() == enum_tunnel_merge, NearByHeight);
		if (NearByHeight != TUNNEL_HEIGHT)
		{
			if (!heightInfoMain.empty())
			{
				if (heightInfoMain.front().startIndex == 0 )
				{
					heightInfoMain.front().height = min(heightInfoMain.front().height, NearByHeight);
				}
				else {
					heightInfoMain.insert(heightInfoMain.begin(), RdsTunnel::HeightInfo{ 0,1,(uint32)NearByHeight });
				}
			}
		}

		MultiLineString3d leftlineVector, rightlineVector;
		if (yTunnel->BranchType() == enum_tunnel_merge)
		{

			if (!yTunnel->left.empty())
			{
				MultiLineString3d lineVector;
				LineString3d middleLine;

				std::vector<TunnelLink *> tunnnels = yTunnel->left;
				// 两边
				std::reverse(tunnnels.begin(), tunnnels.end());
				getTunnelsShape(tunnnels, true, lineVector);
				// 中心线
				getTunnelsMiddleLines(tunnnels, true, middleLine);

				if (lineVector.lines.size() != 2 || middleLine.vertexes.empty())
				{
					return false;
				}
				leftlineVector.lines.emplace_back(lineVector.lines[0]);
				leftlineVector.lines.emplace_back(middleLine);
				leftlineVector.lines.emplace_back(lineVector.lines[1]);

				getCourHeightInfo(tunnnels, middleLine, heightInfoLeft);

			}
			if (!yTunnel->right.empty())
			{
				MultiLineString3d lineVector;
				LineString3d middleLine;
				std::vector<TunnelLink *> tunnnels = yTunnel->right;
				std::reverse(tunnnels.begin(), tunnnels.end());
				getTunnelsShape(tunnnels, true, lineVector);

				getTunnelsMiddleLines(tunnnels, true, middleLine);
				std::reverse(middleLine.vertexes.begin(), middleLine.vertexes.end());

				if (lineVector.lines.size() != 2 || middleLine.vertexes.empty())
				{
					return false;
				}

				rightlineVector.lines.emplace_back(lineVector.lines[0]);
				rightlineVector.lines.emplace_back(middleLine);
				rightlineVector.lines.emplace_back(lineVector.lines[1]);

				getCourHeightInfo(tunnnels, middleLine, heightInfoRight);
			}
		}
		if (yTunnel->BranchType() == enum_tunnel_fork)
		{

			if (!yTunnel->left.empty())
			{
				MultiLineString3d lineVector;
				LineString3d middleLine;
				std::vector<TunnelLink *> tunnnels = yTunnel->left;
				getTunnelsShape(tunnnels, false, lineVector);
				getTunnelsMiddleLines(tunnnels, false, middleLine);
				if (lineVector.lines.size() != 2 || middleLine.vertexes.empty())
				{
					return false;
				}

				leftlineVector.lines.emplace_back(lineVector.lines[0]);
				leftlineVector.lines.emplace_back(middleLine);
				leftlineVector.lines.emplace_back(lineVector.lines[1]);

				getCourHeightInfo(tunnnels, leftlineVector.lines[1], heightInfoLeft);
			}
			if (!yTunnel->right.empty())
			{
				MultiLineString3d lineVector;
				LineString3d middleLine;
				std::vector<TunnelLink *> tunnnels = yTunnel->right;
				getTunnelsShape(tunnnels, false, rightlineVector);
				getTunnelsMiddleLines(tunnnels, false, middleLine);
				if (lineVector.lines.size() != 2 || middleLine.vertexes.empty())
				{
					return false;
				}

				rightlineVector.lines.emplace_back(lineVector.lines[0]);
				rightlineVector.lines.emplace_back(middleLine);
				rightlineVector.lines.emplace_back(lineVector.lines[1]);

				getCourHeightInfo(tunnnels, middleLine, heightInfoRight);
			}
		}

		// 两个出口有一个是隧道
		bool bLeftTunnle = yTunnel->_mainleftOutLaneGroup ? (m_laneGroup2Tunnel.exist(yTunnel->_mainleftOutLaneGroup)) : false;
		bool bRightTunnle = yTunnel->_mainrightOutLaneGroup ? (m_laneGroup2Tunnel.exist(yTunnel->_mainrightOutLaneGroup)) : false;

		if (bLeftTunnle || bRightTunnle)
		{
			if (leftlineVector.lines.empty())
			{
				std::vector<LineString3d> lines;
				if (yTunnel->_mainleftOutLaneGroup)
				{
					MultiLineString3d laneGroupLine;
					getLaneGroupLines(yTunnel->_mainleftOutLaneGroup, laneGroupLine);

					if (yTunnel->BranchType() == TunnelBranchingType::enum_tunnel_merge)
					{
						for (auto &l : laneGroupLine.lines)
						{
							std::reverse(l.vertexes.begin(), l.vertexes.end());
						}
						std::reverse(laneGroupLine.lines.begin(), laneGroupLine.lines.end());
					}
					for (auto &l : laneGroupLine.lines)
					{
						l.vertexes.resize(1);
					}
					leftlineVector.lines.insert(leftlineVector.lines.end(), laneGroupLine.lines.begin(), laneGroupLine.lines.end());

					if (m_laneGroup2Tunnel.exist(yTunnel->_mainleftOutLaneGroup))
					{
						TunnelLink *link = m_laneGroup2Tunnel[yTunnel->_mainleftOutLaneGroup];
						double NearByHeight;
						getNerbyHeightInfo(link, true, yTunnel->BranchType() == enum_tunnel_fork, NearByHeight);

						if (NearByHeight != TUNNEL_HEIGHT)
						{
							heightInfoLeft.emplace_back(RdsTunnel::HeightInfo{ 0,1,(uint32)NearByHeight });

							//std::vector<RdsTunnel::HeightInfo> mergedHeighInfos;
							//if (link->_middleLine.vertexes.size() >= 2)
							//{
							//	heightInfoLeft.emplace_back(RdsTunnel::HeightInfo{ (uint16)(link->_middleLine.vertexes.size() - 2),(uint16)(link->_middleLine.vertexes.size() - 1),(uint32)NearByHeight });

							//}
							//else
							//{
							//	heightInfoLeft.emplace_back(RdsTunnel::HeightInfo{ 0,1,(uint32)NearByHeight });

							//}

						}


					}
				}
			}

			if (rightlineVector.lines.empty())
			{
				std::vector<LineString3d> lines;
				if (yTunnel->_mainrightOutLaneGroup)
				{
					MultiLineString3d laneGroupLine;
					getLaneGroupLines(yTunnel->_mainrightOutLaneGroup, laneGroupLine);

					if (yTunnel->BranchType() == TunnelBranchingType::enum_tunnel_merge)
					{
						for (auto &l : laneGroupLine.lines)
						{
							std::reverse(l.vertexes.begin(), l.vertexes.end());
						}
						std::reverse(laneGroupLine.lines.begin(), laneGroupLine.lines.end());
					}
					for (auto &l : laneGroupLine.lines)
					{
						l.vertexes.resize(1);
					}
					rightlineVector.lines.insert(rightlineVector.lines.end(), laneGroupLine.lines.begin(), laneGroupLine.lines.end());

					if (m_laneGroup2Tunnel.exist(yTunnel->_mainrightOutLaneGroup))
					{
						TunnelLink *link = m_laneGroup2Tunnel[yTunnel->_mainrightOutLaneGroup];
					//	TunnelLink* link = m_laneGroup2Tunnel[yTunnel->_mainleftOutLaneGroup];
						double NearByHeight;
						getNerbyHeightInfo(link, true, yTunnel->BranchType() == enum_tunnel_fork, NearByHeight);
						if (NearByHeight != TUNNEL_HEIGHT)
						{
							heightInfoRight.emplace_back(RdsTunnel::HeightInfo{ 0,1,(uint32)NearByHeight });


						}

					}
				}
			}

			if (lineVectors.lines.size() == 3 && (!leftlineVector.lines.empty() || !rightlineVector.lines.empty()))
			{
				if (leftlineVector.lines.empty())
				{
					MapPoint3D64 pt1, pt2, pt3;
					pt1 = lineVectors.lines[0].vertexes.back();
					pt3 = rightlineVector.lines[0].vertexes.front();
					pt2 = MapPoint3D64{MapPoint64{(pt1.pos.lon + pt3.pos.lon) / 2, (pt1.pos.lat + pt3.pos.lat) / 2}, (pt1.z + pt3.z) / 2};
					leftlineVector.lines.emplace_back(LineString3d{std::vector<MapPoint3D64>{pt1}});
					leftlineVector.lines.emplace_back(LineString3d{std::vector<MapPoint3D64>{pt2}});
					leftlineVector.lines.emplace_back(LineString3d{std::vector<MapPoint3D64>{pt3}});
				}
				if (rightlineVector.lines.empty())
				{
					MapPoint3D64 pt1, pt2, pt3;
					pt1 = leftlineVector.lines[2].vertexes.front();
					pt3 = lineVectors.lines[2].vertexes.back();
					pt2 = MapPoint3D64{MapPoint64{(pt1.pos.lon + pt3.pos.lon) / 2, (pt1.pos.lat + pt3.pos.lat) / 2}, (pt1.z + pt3.z) / 2};
					rightlineVector.lines.emplace_back(LineString3d{std::vector<MapPoint3D64>{pt1}});
					rightlineVector.lines.emplace_back(LineString3d{std::vector<MapPoint3D64>{pt2}});
					rightlineVector.lines.emplace_back(LineString3d{std::vector<MapPoint3D64>{pt3}});
				}
				if (!leftlineVector.lines.empty() && !rightlineVector.lines.empty())
				{
					lineVectors.lines.insert(lineVectors.lines.end(), leftlineVector.lines.begin(), leftlineVector.lines.end());
					lineVectors.lines.insert(lineVectors.lines.end(), rightlineVector.lines.begin(), rightlineVector.lines.end());
				}
			}
		}

		RdsTunnel *rdsTunnel = nullptr;
		for (auto& line : lineVectors.lines)
		{
			auto& linePoints = line.vertexes;
			auto ip_line = std::unique(linePoints.begin(), linePoints.end(), tunnelMapPoint3D64_compare);
			linePoints.resize(std::distance(linePoints.begin(), ip_line));
		}

		// 创建Y型隧道补面
		// createTunnelPatch(yTunnel, lineVector, left_tunnels, right_tunnels);

		if (lineVectors.lines.size() == 3)
		{

			rdsTunnel = (RdsTunnel*)createObject(m_pTile, EntityType::RDS_TUNNEL);
			rdsTunnel->tunnelType = ToRsdTunnelType(tunnel_type_N);
			if (!heightInfoMain.empty())
			{
				rdsTunnel->heightInfos.emplace_back(heightInfoMain);
			}
		}

		if (lineVectors.lines.size() == 9)
		{

			rdsTunnel = (RdsTunnel*)createObject(m_pTile, EntityType::RDS_TUNNEL);
			rdsTunnel->tunnelType = ToRsdTunnelType(tunnel_type_Y);

			if (!heightInfoMain.empty() || !heightInfoLeft.empty() || !heightInfoRight.empty())
			{
					rdsTunnel->heightInfos.emplace_back(heightInfoMain);
					rdsTunnel->heightInfos.emplace_back(heightInfoLeft);
					rdsTunnel->heightInfos.emplace_back(heightInfoRight);
			}
		}
		if (rdsTunnel)
		{
			convert(lineVectors, rdsTunnel->contour);

			uint32 thickness = yTunnel->mixThickNess();
			uint32 height = yTunnel->mixHeight();
			TunnelGroup *tunnelGroup = nullptr;
			if (m_tunnelGroups.exist(yTunnel->groupId))
			{
				thickness = m_tunnelGroups[yTunnel->groupId]->mixthickness();
			}
			rdsTunnel->thickness = thickness;
			rdsTunnel->height = height;
			return true;
		}
		else
		{
			return false;
		}
	}

	void TunnelCompiler::setYTunnelMixExtendTunnelGeoLine(YTunnelMixExtend *yTunnelMixExtend)
	{
		std::vector<MapPoint3D64> leftPoints;
		std::vector<MapPoint3D64> middlePoints;
		std::vector<MapPoint3D64> rightPoints;

		MultiLineString3d lineVector;
		std::vector<TunnelLink *> tunnnels;
		std::vector<TunnelLink *> middleTunnels;
		LineString3d middleLine;

		if (yTunnelMixExtend->BranchType() == enum_tunnel_merge)
		{
			tunnnels = yTunnelMixExtend->main;
			middleTunnels.insert(middleTunnels.end(), tunnnels.rbegin(), tunnnels.rend());
			getTunnelsShape(tunnnels, true, lineVector);

			if (lineVector.lines.size() == 2)
			{
				leftPoints.insert(leftPoints.end(), lineVector.lines[0].vertexes.begin(), lineVector.lines[0].vertexes.end());
				rightPoints.insert(rightPoints.end(), lineVector.lines[1].vertexes.begin(), lineVector.lines[1].vertexes.end());
			}

			tunnnels = yTunnelMixExtend->leftExpand;
			middleTunnels.insert(middleTunnels.end(), tunnnels.begin(), tunnnels.end());
			lineVector.lines.clear();
			std::reverse(tunnnels.begin(), tunnnels.end());
			getTunnelsShape(tunnnels, true, lineVector);

			if (lineVector.lines.size() == 2)
			{
				leftPoints.insert(leftPoints.end(), lineVector.lines[0].vertexes.begin(), lineVector.lines[0].vertexes.end());
			}

			tunnnels = yTunnelMixExtend->rightExpand;
			lineVector.lines.clear();
			std::reverse(tunnnels.begin(), tunnnels.end());
			getTunnelsShape(tunnnels, true, lineVector);

			if (lineVector.lines.size() == 2)
			{
				rightPoints.insert(rightPoints.end(), lineVector.lines[1].vertexes.begin(), lineVector.lines[1].vertexes.end());
			}
			// 中心点
			std::reverse(middleTunnels.begin(), middleTunnels.end());
			getTunnelsMiddleLines(middleTunnels, false, middleLine);
			std::reverse(middleLine.vertexes.begin(), middleLine.vertexes.end());
			middlePoints = middleLine.vertexes;
		}
		else if (yTunnelMixExtend->BranchType() == enum_tunnel_fork)
		{
			tunnnels = yTunnelMixExtend->main;
			std::reverse(tunnnels.begin(), tunnnels.end());
			getTunnelsShape(tunnnels, false, lineVector);
			middleTunnels.insert(middleTunnels.end(), tunnnels.begin(), tunnnels.end());

			if (lineVector.lines.size() == 2)
			{
				leftPoints.insert(leftPoints.end(), lineVector.lines[0].vertexes.begin(), lineVector.lines[0].vertexes.end());

				rightPoints.insert(rightPoints.end(), lineVector.lines[1].vertexes.begin(), lineVector.lines[1].vertexes.end());
			}

			tunnnels = yTunnelMixExtend->leftExpand;
			lineVector.lines.clear();
			getTunnelsShape(tunnnels, false, lineVector);
			middleTunnels.insert(middleTunnels.end(), tunnnels.begin(), tunnnels.end());
			if (lineVector.lines.size() == 2)
			{
				leftPoints.insert(leftPoints.end(), lineVector.lines[0].vertexes.begin(), lineVector.lines[0].vertexes.end());
			}

			tunnnels = yTunnelMixExtend->rightExpand;
			lineVector.lines.clear();
			getTunnelsShape(tunnnels, false, lineVector);

			if (lineVector.lines.size() == 2)
			{
				rightPoints.insert(rightPoints.end(), lineVector.lines[1].vertexes.begin(), lineVector.lines[1].vertexes.end());
			}

			// 中心点
			std::reverse(middleTunnels.begin(), middleTunnels.end());
			getTunnelsMiddleLines(middleTunnels, true, middleLine);

			middlePoints = middleLine.vertexes;
		}

		// 去重
		auto ip_left = std::unique(leftPoints.begin(), leftPoints.end(), mapPoint3D64_compare);
		leftPoints.resize(std::distance(leftPoints.begin(), ip_left));

		auto ip_middle = std::unique(middlePoints.begin(), middlePoints.end(), mapPoint3D64_compare);
		middlePoints.resize(std::distance(middlePoints.begin(), ip_middle));

		auto ip_right = std::unique(rightPoints.begin(), rightPoints.end(), mapPoint3D64_compare);
		rightPoints.resize(std::distance(rightPoints.begin(), ip_right));

		yTunnelMixExtend->_leftLine.vertexes = leftPoints;
		yTunnelMixExtend->_middleLine.vertexes = middlePoints;
		yTunnelMixExtend->_rightLine.vertexes = rightPoints;
	}

	void TunnelCompiler::getTunnelsShape(std::vector<TunnelLink *> tunnnels, bool isReverse, MultiLineString3d &lineString3DVector)
	{

		LineString3d left_points, right_points;

		for (auto tunnel : tunnnels)
		{
			left_points.vertexes.insert(left_points.vertexes.end(), tunnel->_leftLine.vertexes.begin(), tunnel->_leftLine.vertexes.end());
			right_points.vertexes.insert(right_points.vertexes.end(), tunnel->_rightLine.vertexes.begin(), tunnel->_rightLine.vertexes.end());
		}

		if (isReverse)
		{
			std::reverse(left_points.vertexes.begin(), left_points.vertexes.end());
			std::reverse(right_points.vertexes.begin(), right_points.vertexes.end());
		}

		// 去重
		auto ip_left = std::unique(left_points.vertexes.begin(), left_points.vertexes.end(), mapPoint3D64_compare);
		left_points.vertexes.resize(std::distance(left_points.vertexes.begin(), ip_left));

		auto ip_right = std::unique(right_points.vertexes.begin(), right_points.vertexes.end(), mapPoint3D64_compare);
		right_points.vertexes.resize(std::distance(right_points.vertexes.begin(), ip_right));

		
		if (left_points.vertexes.empty() || right_points.vertexes.empty())
		{
			return;
		}
		if (isReverse)
		{
			lineString3DVector.lines.emplace_back(right_points);

			lineString3DVector.lines.emplace_back(left_points);
		}
		else
		{
			lineString3DVector.lines.emplace_back(left_points);

			lineString3DVector.lines.emplace_back(right_points);
		}
	}
	
	void TunnelCompiler::getTunnelsMiddleLines(std::vector<TunnelLink *> tunnnels, bool isReverse, LineString3d &lineString3DVector)
	{
		lineString3DVector.vertexes.clear();
		if (tunnnels.empty())
		{
			return;
		}

		TunnelLink *first = tunnnels[0];
		std::vector<HadSkeleton *> hadSkeletons;
		if (first->_middleLane)
		{
			HadLane *firstLane = first->_middleLane;
			hadSkeletons.emplace_back(firstLane);
			for (size_t i = 1; i < tunnnels.size(); i++)
			{

				for (auto lane : tunnnels[i]->_laneGroup->lanes)
				{
					if (firstLane->endNode->originId == lane->startNode->originId || firstLane->startNode->originId == lane->endNode->originId)
					{
						firstLane = lane;
						hadSkeletons.emplace_back(firstLane);
					}
				}
			}
		}
		else if (first->_middleLaneBoundary)
		{
			HadLaneBoundary *firstLaneBoundary = first->_middleLaneBoundary;
			hadSkeletons.emplace_back(firstLaneBoundary);
			for (size_t i = 1; i < tunnnels.size(); i++)
			{

				for (auto laneBoundary : tunnnels[i]->_laneGroup->laneBoundaries)
				{
					if (firstLaneBoundary->endNode->originId == laneBoundary->startNode->originId || firstLaneBoundary->startNode->originId == laneBoundary->endNode->originId)
					{
						firstLaneBoundary = laneBoundary;
						hadSkeletons.emplace_back(firstLaneBoundary);
					}
				}
			}
		}
		if (isReverse)
		{
			std::reverse(hadSkeletons.begin(), hadSkeletons.end());
		}
		for (auto hadSkeleton : hadSkeletons)
		{
			lineString3DVector.vertexes.insert(lineString3DVector.vertexes.end(), hadSkeleton->location.vertexes.begin(), hadSkeleton->location.vertexes.end());
		}
		// 去重
		auto ip_left = std::unique(lineString3DVector.vertexes.begin(), lineString3DVector.vertexes.end(), mapPoint3D64_compare);
		lineString3DVector.vertexes.resize(std::distance(lineString3DVector.vertexes.begin(), ip_left));
	}

	void TunnelCompiler::createNLinkGroup(TunnelGroup *tunnelGroup)
	{
		if (tunnelGroup->headlinks.size() != 1)
		{
			return;
		}

		NTunnel *nTunnel = ALLOC_ELEMENT<NTunnel>();
		TunnelLink *head = *tunnelGroup->headlinks.begin();
		nTunnel->tunnnels.emplace_back(head);
		while (head->nextisSingleBranch() && head->_next.size() == 1)
		{
			head = head->_next[0];
			nTunnel->tunnnels.emplace_back(head);
		}

		nTunnel->groupId = tunnelGroup->groupId;
		m_NTunnelArray.push_back(nTunnel);
	}

	void TunnelCompiler::createYLinkGroup(TunnelGroup *tunnelGroup)
	{

		std::vector<YTunnelMix *> yTunnelMixArray;
		std::vector<YTunnelMixExtend *> yTunnelMixExtendArray;

		FastTable<TunnelLink *, TunnelLink *> laTable;
		for (auto &tunnelLink : tunnelGroup->tunnnels)
		{

			YTunnelMix *yTunnelMix = createYTunnelMix(tunnelLink, laTable);

			if (yTunnelMix)
			{
				yTunnelMixArray.emplace_back(yTunnelMix);
			}
		}
		for (auto yTunnelMix : yTunnelMixArray)
		{
			YTunnelMixExtend *yTunnelMixExtend = createYTunnelMixExtend(yTunnelMix, laTable);

			if (yTunnelMixExtend)
			{
				yTunnelMixExtendArray.emplace_back(yTunnelMixExtend);
			}
		}

		for each (auto yTunnelMixExtend in yTunnelMixExtendArray)
		{
			YTunnel *yTunnel = createYTunnel(yTunnelMixExtend, laTable);
			if (yTunnel)
			{
				yTunnel->groupId = tunnelGroup->groupId;
				m_YTunnelArray.emplace_back(yTunnel);
			}
		}
	}
	
	YTunnelMix *TunnelCompiler::createYTunnelMix(TunnelLink *tunnelLink, FastTable<TunnelLink *, TunnelLink *> &laTable)
	{
		if (laTable.exist(tunnelLink))
		{
			return nullptr;
		}

		YTunnelMix *yTunnelMix = nullptr;
		if (tunnelLink == nullptr)
		{
			return nullptr;
		}

		HadLaneGroup *leftLaneGroup, *rightLaneGroup;
		TunnelLink *leftTunnel, *rightTunnel;
		leftLaneGroup = rightLaneGroup = nullptr;
		leftTunnel = rightTunnel = nullptr;
		TunnelBranchingType branchType = enum_tunnel_unknow;
		if (tunnelLink->_next.size() == 2)
		{
			branchType = enum_tunnel_fork;
			leftLaneGroup = tunnelLink->_next[0]->_laneGroup;
			rightLaneGroup = tunnelLink->_next[1]->_laneGroup;
		}
		if (tunnelLink->_previous.size() == 2)
		{
			branchType = enum_tunnel_merge;
			leftLaneGroup = tunnelLink->_previous[0]->_laneGroup;
			rightLaneGroup = tunnelLink->_previous[1]->_laneGroup;
		}
		if (tunnelLink->_next.size() == 1 && tunnelLink->_laneGroup->next.size() == 2)
		{
			branchType = enum_tunnel_fork;
			leftLaneGroup = (HadLaneGroup *)tunnelLink->_laneGroup->next[0];
			rightLaneGroup = (HadLaneGroup *)tunnelLink->_laneGroup->next[1];
		}
		if (tunnelLink->_previous.size() == 1 && tunnelLink->_laneGroup->previous.size() == 2)
		{
			branchType = enum_tunnel_merge;
			leftLaneGroup = (HadLaneGroup *)tunnelLink->_laneGroup->previous[0];
			rightLaneGroup = (HadLaneGroup *)tunnelLink->_laneGroup->previous[1];
		}
		if (!leftLaneGroup || !rightLaneGroup || branchType == enum_tunnel_unknow)
		{
			return nullptr;
		}

		if (!isLaneGroupOnLeft(leftLaneGroup, rightLaneGroup))
		{
			cq::swap(leftLaneGroup, rightLaneGroup);
		}
		leftTunnel = getTunnelLinkByLaneGroup(leftLaneGroup);
		rightTunnel = getTunnelLinkByLaneGroup(rightLaneGroup);

		if ((!leftTunnel && !rightTunnel) || laTable.exist(leftTunnel) || laTable.exist(rightTunnel))
		{
			return nullptr;
		}

		yTunnelMix = ALLOC_ELEMENT<YTunnelMix>();
		yTunnelMix->init();
		yTunnelMix->branchType = branchType;
		yTunnelMix->main = tunnelLink;
		yTunnelMix->main_laneGroup = tunnelLink->_laneGroup;
		if (branchType == enum_tunnel_fork)
		{
			yTunnelMix->left = leftTunnel;
			yTunnelMix->right = rightTunnel;
			yTunnelMix->left_laneGroup = leftLaneGroup;
			yTunnelMix->right_laneGroup = rightLaneGroup;
		}
		if (branchType == enum_tunnel_merge)
		{
			yTunnelMix->left = rightTunnel;
			yTunnelMix->right = leftTunnel;
			yTunnelMix->left_laneGroup = rightLaneGroup;
			yTunnelMix->right_laneGroup = leftLaneGroup;
		}

		laTable.insert(tunnelLink, tunnelLink);
		if (leftTunnel)
		{
			laTable.insert(leftTunnel, leftTunnel);
			auto& branchTypes = leftTunnel->branchTypes;
			if (enum_tunnel_unknow != branchType && 
				std::find(branchTypes.begin(), branchTypes.end(), branchType) == branchTypes.end())
			{
				branchTypes.push_back(branchType);
			}
		}
		if (rightTunnel)
		{
			laTable.insert(rightTunnel, rightTunnel);
			auto& branchTypes = rightTunnel->branchTypes;
			if (enum_tunnel_unknow != branchType &&
				std::find(branchTypes.begin(), branchTypes.end(), branchType) == branchTypes.end())
			{
				branchTypes.push_back(branchType);
			}
		}

		return yTunnelMix;
	}

	YTunnelMixExtend *TunnelCompiler::createYTunnelMixExtend(YTunnelMix *yTunnelMix, FastTable<TunnelLink *, TunnelLink *> &table)
	{
		if (yTunnelMix->branchType == enum_tunnel_unknow)
		{
			return nullptr;
		}

		YTunnelMixExtend *yTunnelMixExtend = ALLOC_ELEMENT<YTunnelMixExtend>();
		yTunnelMixExtend->init();
		yTunnelMixExtend->mixYtunnel = yTunnelMix;

		TunnelLink *left = yTunnelMix->left;
		TunnelLink *right = yTunnelMix->right;

		// 往前推送LA结束位置
		if (yTunnelMix->branchType == enum_tunnel_merge)
		{
			getYTunnelMixExtendLeftRight(left, right, false, yTunnelMixExtend->leftExpand, yTunnelMixExtend->rightExpand, table);
		}
		else if (yTunnelMix->branchType == enum_tunnel_fork)
		{
			getYTunnelMixExtendLeftRight(left, right, true, yTunnelMixExtend->leftExpand, yTunnelMixExtend->rightExpand, table);
		}
		if (yTunnelMix->main)
		{
			yTunnelMixExtend->main.emplace_back(yTunnelMix->main);
		}

		return yTunnelMixExtend;
	}

	YTunnel *TunnelCompiler::createYTunnel(YTunnelMixExtend *yTunnelMixExtend, FastTable<TunnelLink *, TunnelLink *> &table)
	{

		YTunnel *yTunnel = ALLOC_ELEMENT<YTunnel>();
		yTunnel->init();
		yTunnel->main = yTunnelMixExtend;

		bool bForward = true;
		if (yTunnelMixExtend->BranchType() == enum_tunnel_merge)
		{
			findNext(yTunnelMixExtend->mainEnd(), true, yTunnelMixExtend->main, false, table);
			findNext(yTunnelMixExtend->leftEnd(), false, yTunnel->left, yTunnelMixExtend->leftExpand.empty(), table);
			findNext(yTunnelMixExtend->rightEnd(), false, yTunnel->right, yTunnelMixExtend->rightExpand.empty(), table);
			bForward = false;
		}
		if (yTunnelMixExtend->BranchType() == enum_tunnel_fork)
		{
			findNext(yTunnelMixExtend->mainEnd(), false, yTunnelMixExtend->main, false, table);
			findNext(yTunnelMixExtend->leftEnd(), true, yTunnel->left, yTunnelMixExtend->leftExpand.empty(), table);
			findNext(yTunnelMixExtend->rightEnd(), true, yTunnel->right, yTunnelMixExtend->rightExpand.empty(), table);
			bForward = true;
		}

		if (!yTunnel->left.empty())
		{
			yTunnel->_mainleftOutLaneGroup = (*yTunnel->left.begin())->_laneGroup;
		}
		else
		{
			if (!yTunnelMixExtend->leftExpand.empty())
			{
				HadLaneGroup *endLaneSGroup = (*yTunnelMixExtend->leftExpand.rbegin())->_laneGroup;
				if (bForward && endLaneSGroup->next.size() == 1)
				{
					yTunnel->_mainleftOutLaneGroup = (HadLaneGroup *)endLaneSGroup->next[0];
				}
				else if (!bForward && endLaneSGroup->previous.size() == 1)
				{
					yTunnel->_mainleftOutLaneGroup = (HadLaneGroup *)endLaneSGroup->previous[0];
				}
			}
			else
			{
				yTunnel->_mainleftOutLaneGroup = yTunnel->main->mixYtunnel->left_laneGroup;
			}
		}

		if (!yTunnel->right.empty())
		{
			yTunnel->_mainrightOutLaneGroup = (*yTunnel->right.begin())->_laneGroup;
		}
		else
		{
			if (!yTunnelMixExtend->rightExpand.empty())
			{
				HadLaneGroup *endLaneSGroup = (*yTunnelMixExtend->rightExpand.rbegin())->_laneGroup;
				if (bForward && endLaneSGroup->next.size() == 1)
				{
					yTunnel->_mainrightOutLaneGroup = (HadLaneGroup *)endLaneSGroup->next[0];
				}
				else if (!bForward && endLaneSGroup->previous.size() == 1)
				{
					yTunnel->_mainrightOutLaneGroup = (HadLaneGroup *)endLaneSGroup->previous[0];
				}
			}
			else
			{
				yTunnel->_mainrightOutLaneGroup = yTunnel->main->mixYtunnel->right_laneGroup;
			}
		}
		return yTunnel;
	}

	void TunnelCompiler::findNext(TunnelLink *tunnelLink, bool bForward, std::vector<TunnelLink *> &outLink, bool includeFirst, FastTable<TunnelLink *, TunnelLink *> &table)
	{
		if (!tunnelLink)
		{
			return;
		}
		TunnelLink *link = tunnelLink;
		if (includeFirst)
		{
			table.insert(link, link);
			outLink.emplace_back(link);
		}
		while (link->bNPSingleBranch(bForward))
		{
			link = link->NPNo1TunnelLink(bForward);

			if (!link || table.exist(link))
			{
				return;
			}

			outLink.emplace_back(link);
			table.insert(link, link);
		}
	}

	void TunnelCompiler::getYTunnelMixExtendLeftRight(TunnelLink *left, TunnelLink *right, bool isForward,
													  std::vector<TunnelLink *> &leftOuts, std::vector<TunnelLink *> &rightOut, FastTable<TunnelLink *, TunnelLink *> &table)
	{
		if (!left || !right)
		{
			return;
		}

		HadRelLaneGroupAssociation laneGroupAsso;
		if (!isLALaneGroup(left->_laneGroup, right->_laneGroup, laneGroupAsso))
		{
			return;
		}
		leftOuts.emplace_back(left);
		rightOut.emplace_back(right);

		while (left && right && left->bNPSingleBranch(isForward) && right->bNPSingleBranch(isForward))
		{
			left = left->NPNo1TunnelLink(isForward);
			right = right->NPNo1TunnelLink(isForward);
			if (!left || !right)
			{
				break;
			}
			if (table.exist(left) || table.exist(right))
			{
				break;
			}

			
			HadRelLaneGroupAssociation laneGroupAsso;
			if (!isLALaneGroup(left->_laneGroup, right->_laneGroup, laneGroupAsso))
			{
				// 间距小于50也需要往后退
				/*	MapPoint3D64 *pLeftByPolygon = nullptr, *pRightByPolygon = nullptr;
					size_t nLeftByPolygonPtNum = 0, rRightByPolygonPtNum = 0;
					nLeftByPolygonPtNum = makeLaneGroupPolygon(left->_laneGroup, pLeftByPolygon);
					rRightByPolygonPtNum = makeLaneGroupPolygon(right->_laneGroup, pRightByPolygon);

					float distance = getDistance(pLeftByPolygon, nLeftByPolygonPtNum, pRightByPolygon, rRightByPolygonPtNum);
					int32 thickNess = distance / 2;
					thickNess = max(thickNess, 50);
					left->thickness = thickNess;
					right->thickness = thickNess;*/

				break;
			}

			leftOuts.emplace_back(left);
			rightOut.emplace_back(right);
			table.insert(left, left);
			table.insert(right, right);
		}
	}

	TunnelLink *TunnelCompiler::getTunnelLinkByLaneGroup(HadLaneGroup *hadLaneGroup)
	{
		if (m_laneGroup2Tunnel.exist(hadLaneGroup))
		{
			return m_laneGroup2Tunnel[hadLaneGroup];
		}
		return nullptr;
	}

	bool TunnelCompiler::isLALaneGroup(HadLaneGroup *l1, HadLaneGroup *l2, HadRelLaneGroupAssociation &laneGroupAsso)
	{
		if (!l1 || !l2)
		{
			return false;
		}
		for (auto asso : l1->associations)
		{
			for (auto assoRel : asso->relLaneGroupAssociations)
			{
				if ((assoRel.first == l1 && assoRel.second == l2) || (assoRel.first == l2 && assoRel.second == l1))
				{
					laneGroupAsso = assoRel;
					return true;
				}
			}
		}

		return false;
	}

	bool TunnelCompiler::isLaneGroupOnLeft(HadLaneGroup *leftLaneGroup, HadLaneGroup *rightLaneGroup)
	{
		if (!leftLaneGroup->laneBoundaries.empty() && !rightLaneGroup->laneBoundaries.empty()) {
			return GrapPointAlgorithm::isOnLeft((*leftLaneGroup->laneBoundaries.begin())->location.vertexes,
				(*rightLaneGroup->laneBoundaries.rbegin())->location.vertexes);
		}
		return GrapPointAlgorithm::isOnLeft((*leftLaneGroup->roadBoundaries.begin())->location.vertexes,
			(*rightLaneGroup->roadBoundaries.rbegin())->location.vertexes);
	}

	TunnelBranchingType TunnelCompiler::laBranchType(HadRelLaneGroupAssociation &la)
	{
		double disStart = GrapPointAlgorithm::geoLengthD((*la.firstLaneBoundary->location.vertexes.begin()).pos.toNdsPoint(),
														 (*la.secondLaneBoundary->location.vertexes.begin()).pos.toNdsPoint());
		double disEnd = GrapPointAlgorithm::geoLengthD((*la.firstLaneBoundary->location.vertexes.rbegin()).pos.toNdsPoint(),
													   (*la.secondLaneBoundary->location.vertexes.rbegin()).pos.toNdsPoint());

		return disStart < disEnd ? enum_tunnel_fork : enum_tunnel_merge;
	}
	void TunnelCompiler::getLaneGroupLines(HadLaneGroup *laneGroup, MultiLineString3d &lines)
	{
		HadLane *hadLane = nullptr;
		HadLaneBoundary *hadLaneBoundary = nullptr;
		getLaneGroupLines(laneGroup, lines, hadLane, hadLaneBoundary);
	}
	void TunnelCompiler::getLaneGroupLines(HadLaneGroup *laneGroup, MultiLineString3d &lines, HadLane *&middleLane, HadLaneBoundary *&middleLaneBoudary)
	{

		if (!laneGroup)
		{
			return;
		}
		lines.lines.clear();
		lines.lines.resize(3);
		LineString3d middelLine;
		if (laneGroup->roadBoundaries.size() >= 2)
		{

			lines.lines[0] = laneGroup->roadBoundaries.at(0)->location;
			lines.lines[2] = laneGroup->roadBoundaries.at(laneGroup->roadBoundaries.size() - 1)->location;
		}
		else if (laneGroup->laneBoundaries.size() >= 2)
		{
			lines.lines[0] = laneGroup->laneBoundaries.at(0)->location;
			lines.lines[2] = laneGroup->laneBoundaries.at(laneGroup->laneBoundaries.size() - 1)->location;
		}
		if (laneGroup->laneBoundaries.size() >= 2)
		{
			int middleLine = laneGroup->laneBoundaries.size() / 2;
			lines.lines[1] = laneGroup->laneBoundaries.at(middleLine)->location;
			middleLaneBoudary = laneGroup->laneBoundaries.at(middleLine);
		}

		double dis = UINT32_MAX;
		for (size_t i = 0; i < laneGroup->lanes.size(); i++)
		{
			double disA = GrapPointAlgorithm::geoLengthD(laneGroup->lanes[i]->location.vertexes[0].pos.toNdsPoint(),
														 lines.lines[0].vertexes[0].pos.toNdsPoint());
			double disB = GrapPointAlgorithm::geoLengthD(laneGroup->lanes[i]->location.vertexes[0].pos.toNdsPoint(),
														 lines.lines[2].vertexes[0].pos.toNdsPoint());

			if (cq_abs(disA - disB) < dis)
			{
				dis = cq_abs(disA - disB);
				middelLine = laneGroup->lanes[i]->location;
				middleLane = laneGroup->lanes[i];
				middleLaneBoudary = nullptr;
			}
		}
		for (size_t i = 0; i < laneGroup->laneBoundaries.size(); i++)
		{
			continue;
			double disA = GrapPointAlgorithm::geoLengthD(laneGroup->laneBoundaries[i]->location.vertexes[0].pos.toNdsPoint(),
														 lines.lines[0].vertexes[0].pos.toNdsPoint());
			double disB = GrapPointAlgorithm::geoLengthD(laneGroup->laneBoundaries[i]->location.vertexes[0].pos.toNdsPoint(),
														 lines.lines[2].vertexes[0].pos.toNdsPoint());

			if (cq_abs(disA - disB) < dis)
			{
				dis = cq_abs(disA - disB);
				middelLine = laneGroup->laneBoundaries[i]->location;
				middleLaneBoudary = laneGroup->laneBoundaries[i];
				middleLane = nullptr;
			}
		}

		if (!middelLine.vertexes.empty())
		{
			lines.lines[1] = middelLine;
		}
	}

	bool TunnelCompiler::hasOverLap(TunnelLink *tunnelLink,
									MapPoint3D64 *pCurPolygon, size_t nCurPolygonPtNum, HadLaneGroup *pNearbyLg,
									int32 &overlapHeight, std::vector<TunnelLinkGlandHeightInfo> &heightInfos)
	{

	
		overlapHeight = TUNNEL_HEIGHT;
		bool bOverlap = false;
		// 计算几何中心点函数。
		auto getCenterPt = [](const ClipperLib::Path &path)
		{
			MapPoint3D64 ptCenter = {0};
			size_t size = path.size();

			int64 nTotalX = 0, nTotalY = 0;
			for (int i = 0; i < size; i++)
			{
				nTotalX += path[i].X;
				nTotalY += path[i].Y;
			}
			ptCenter.pos.lon = nTotalX / size;
			ptCenter.pos.lat = nTotalY / size;
			return ptCenter;
		};

		HadLaneGroup *pCurLg = tunnelLink->_laneGroup;
		// 两个车道面相交判定
		MapPoint3D64 *pNearbyPolygon = NULL;
		size_t nNearbyPolygonPtNum = makeLaneGroupPolygon(pNearbyLg, pNearbyPolygon); // 创建附近的车道组面闭合多边形。
		if (nNearbyPolygonPtNum == 0)
			return false;

		ClipperLib::PolyTree polyTree;
		bool isIntersect = _intersect(pCurPolygon, nCurPolygonPtNum, pNearbyPolygon, nNearbyPolygonPtNum, polyTree);
		delete[] pNearbyPolygon;

		if (!isIntersect)
			return false;

		
			// 整理相交部分
#ifdef __DEBUG_GRADE_SEPARATION__
		printInfo("-->车道组%I64d与车道组%I64d相交，共%d个相交区域，tree子节点数%d ",
				  pCurLg->originId, pNearbyLg->originId, polyTree.Total(), polyTree.ChildCount());
#endif //__DEBUG_GRADE_SEPARATION__

		ClipperLib::Paths paths;
		ClipperLib::ClosedPathsFromPolyTree(polyTree, paths); // 得到闭合轮廓
#ifdef __DEBUG_GRADE_SEPARATION__
		printInfo("\t闭合轮廓数%d", paths.size());
#endif									  //__DEBUG_GRADE_SEPARATION__
		ClipperLib::CleanPolygons(paths); // 清理对后续处理无用的几何

		// 遍历所有相交的轮廓，计算车道面空间关系

		//// 计算中心点
		if (pCurLg->lanes.empty() || pNearbyLg->lanes.empty()) {
			return false;
		}

		//TODO 此处可以考虑用别的要素么?
		linestring_t curLg_line = LINESTRING_T(pCurLg->lanes[0]->location.vertexes);
		linestring_t nearlg_line = LINESTRING_T(pNearbyLg->lanes[0]->location.vertexes);
		linestring_t middle_line = LINESTRING_T(tunnelLink->_middleLine.vertexes);
		point_t centerPoint;
		bg::centroid(curLg_line, centerPoint);
		for (auto path : paths)
		{
			if (path.empty())
			{
					continue;
			}
		
	
			size_t si = UINT64_MAX;
			size_t ei = UINT64_MAX;
			for (ClipperLib::IntPoint& pt : path)
			{

			
				point_t ptCenter = point_t(pt.X, pt.Y, centerPoint.get<2>());
				segment_t tmpCurLg, tmpNearbyLg;
				
				
				bg::closest_points(ptCenter, curLg_line, tmpCurLg);
				bg::closest_points(ptCenter, nearlg_line, tmpNearbyLg);
				size_t si_tmp = 0, ei_tmp = 0;
				MapPoint3D64 ptGrapCurLg = MapPoint3D64_make(tmpCurLg.second.get<0>(), tmpCurLg.second.get<1>(), tmpCurLg.second.get<2>());
				MapPoint3D64 ptGrapNearbyLg = MapPoint3D64_make(tmpNearbyLg.second.get<0>(), tmpNearbyLg.second.get<1>(), tmpNearbyLg.second.get<2>());
			
				// 判定两个车道组的高度，判断上下层，记入有效数据。
				if (ptGrapNearbyLg.z - ptGrapCurLg.z > 100)
				{
					bOverlap = true;
					overlapHeight = min(overlapHeight, abs(ptGrapNearbyLg.z - ptGrapCurLg.z)-300);

					size_t si_tmp = 0, ei_tmp = 0;
					segment_t tmpMiddleLine;
					bg::closest_points(ptCenter, middle_line, tmpMiddleLine);
					MapPoint3D64 ptGrapMiddle = MapPoint3D64_make(tmpMiddleLine.second.get<0>(), tmpMiddleLine.second.get<1>(), tmpMiddleLine.second.get<2>());
				
					if (GrapPointAlgorithm::GeometryUtil_findIndexWithLine(tunnelLink->_middleLine.vertexes.data(), tunnelLink->_middleLine.vertexes.size(), ptGrapMiddle, si_tmp, ei_tmp))

					{

						if (si == UINT64_MAX && ei == UINT64_MAX)
						{
							si = si_tmp;
							ei = ei_tmp;
						}
						else
						{
							si = min(si, si_tmp);
							ei = max(ei, ei_tmp);
						}

					}


				}

				
			}
			if (bOverlap)
			{

				if (si != UINT64_MAX && ei != UINT64_MAX)
				{
					TunnelLinkGlandHeightInfo heighinfo;
					heighinfo.init(si, ei, overlapHeight);
					heighinfo.startPoint = tunnelLink->_middleLine.vertexes[si];
					heighinfo.endPoint = tunnelLink->_middleLine.vertexes[ei];
					heightInfos.emplace_back(heighinfo);
				}
			}
		}
	


		polyTree.Clear();
		return bOverlap;
	}
	bool TunnelCompiler::hasIntersectExtend(
		MapPoint3D64 *pCurPolygon, size_t nCurPolygonPtNum, MapPoint3D64 *pNearPolygon, size_t nNearPolygonPtNum)
	{
		static const double OFFSET_IN_METER = 1.5;
		const float METER_PER_LAT_UNIT = 1.11f;
		const float LAT_UNIT_PER_METER = 1 / METER_PER_LAT_UNIT;

		ClipperLib::Path path_v1;
		for (int i = 0; i < nCurPolygonPtNum; i++)
			path_v1 << ClipperLib::IntPoint(pCurPolygon[i].pos.lon, pCurPolygon[i].pos.lat, 0);

		ClipperLib::Path path_v2;
		for (int i = 0; i < nNearPolygonPtNum; i++)
			path_v2 << ClipperLib::IntPoint(pNearPolygon[i].pos.lon, pNearPolygon[i].pos.lat, 0);

		// 相交
		ClipperLib::Clipper clipper;
		ClipperLib::PolyTree polyTree;
	/*	clipper.AddPath(path_v1, ClipperLib::ptSubject, false);
		clipper.AddPath(path_v2, ClipperLib::ptClip, true);
		
		clipper.Execute(ClipperLib::ctIntersection, polyTree, ClipperLib::pftNonZero);
		if (polyTree.Total() > 0)
		{
			return false;
		}*/
		if (path_v1.empty() || path_v2.empty())
		{
			return false;
		}
		// 扩大
		ClipperLib::ClipperOffset offseter1;
		offseter1.AddPath(path_v1, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
		ClipperLib::Paths extObjePaths1;
		offseter1.Execute(extObjePaths1, OFFSET_IN_METER * LAT_UNIT_PER_METER * 400);
		if (extObjePaths1.empty())
			return false;
		ClipperLib::Path &extObjPath1 = extObjePaths1[0];

		// 扩大
		ClipperLib::ClipperOffset offseter2;
		offseter2.AddPath(path_v2, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
		ClipperLib::Paths extObjePaths2;
		offseter2.Execute(extObjePaths2, OFFSET_IN_METER * LAT_UNIT_PER_METER * 400);
		if (extObjePaths2.empty())
			return false;
		ClipperLib::Path &extObjPath2 = extObjePaths2[0];

		// 相交
		// ClipperLib::Clipper clipper;
		clipper.AddPath(extObjPath1, ClipperLib::ptSubject, false);
		clipper.AddPath(extObjPath2, ClipperLib::ptClip, true);
		// ClipperLib::PolyTree polyTree;
		clipper.Execute(ClipperLib::ctIntersection, polyTree, ClipperLib::pftNonZero);
		if (polyTree.Total() > 0)
		{
			ClipperLib::Paths polyPaths;
			ClipperLib::PolyTreeToPaths(polyTree, polyPaths);
			double area_polyTree = cq_abs(ClipperLib::Area(polyPaths[0]));
			if (area_polyTree > 0)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		return false;
	}

	int TunnelCompiler::getDistance(
		MapPoint3D64 *pCurPolygon, size_t nCurPolygonPtNum, MapPoint3D64 *pNearPolygon, size_t nNearPolygonPtNum)
	{
		typedef boost::geometry::model::point<double, 2, boost::geometry::cs::geographic<boost::geometry::degree>> point;
		typedef boost::geometry::model::linestring<point> linestring;

		linestring linestring1;
		for (int32 i = 0; i < nCurPolygonPtNum; i++)
		{
			linestring1.push_back(point(pCurPolygon[i].pos.lon / 1e8, pCurPolygon[i].pos.lat / 1e8));
		}
		linestring linestring2;
		for (int32 i = 0; i < nNearPolygonPtNum; i++)
		{
			linestring2.push_back(point(pNearPolygon[i].pos.lon / 1e8, pNearPolygon[i].pos.lat / 1e8));
		}

		// GrapPointAlgorithm::geoLengthD(curRange.endPoint.pos.toNdsPoint(), heighInfo[i].startPoint.pos.toNdsPoint())

		int32 distance = (int32)(boost::geometry::distance(linestring1, linestring2) * 100) * 10;
		int32 dist = 0;
		if (distance > 5)
		{
			dist = distance - 5;
		}

		return dist;
	}

	int TunnelCompiler::getDistance2(

		MapPoint3D64 *pCurPolygon, size_t nCurPolygonPtNum, MapPoint3D64 *pNearPolygon, size_t nNearPolygonPtNum)
	{
		
		typedef bg::model::d2::point_xy<double> BgPoint;
		typedef bg::model::linestring<BgPoint> BgLinestring;
		typedef bg::model::segment<BgPoint> BgSegment;

		BgLinestring linestring1;
		for (int32 i = 0; i < nCurPolygonPtNum; i++)
		{
			linestring1.push_back(BgPoint(pCurPolygon[i].pos.lon, pCurPolygon[i].pos.lat));
		}
		BgLinestring linestring2;
		for (int32 i = 0; i < nNearPolygonPtNum; i++)
		{
			linestring2.push_back(BgPoint(pNearPolygon[i].pos.lon, pNearPolygon[i].pos.lat));
		}

		// 2.2.1寻找最近的两个点
		BgSegment clostPoints;
		bg::closest_points(linestring1, linestring2, clostPoints);

		BgPoint pt1 = clostPoints.first;
		BgPoint pt2 = clostPoints.second;
		MapPoint64 ptInLane1{(int64)pt1.x(), (int64)pt1.y()};
		MapPoint64 ptInLane2{(int64)pt2.x(), (int64)pt2.y()};

		int32 distance = GrapPointAlgorithm::geoLengthD(ptInLane1.toNdsPoint(), ptInLane2.toNdsPoint()) * 1000;
		int32 dist = 0;
		if (distance > 5)
		{
			dist = distance - 5;
		}

		return dist;
	}

	bool TunnelCompiler::getClosePoint(
		MapPoint3D64* pCurPolygon, size_t nCurPolygonPtNum, MapPoint3D64* pNearPolygon, size_t nNearPolygonPtNum,MapPoint3D64& pt1Closest, MapPoint3D64& pt2Closest)
	{

		typedef bg::model::d2::point_xy<int64> BgPoint;
		typedef bg::model::linestring<BgPoint> BgLinestring;
		typedef bg::model::segment<BgPoint> BgSegment;
	
		BgLinestring linestring1;
		for (int32 i = 0; i < nCurPolygonPtNum; i++)
		{
			linestring1.push_back(BgPoint(pCurPolygon[i].pos.lon, pCurPolygon[i].pos.lat));
		}
		BgLinestring linestring2;
		for (int32 i = 0; i < nNearPolygonPtNum; i++)
		{
			linestring2.push_back(BgPoint(pNearPolygon[i].pos.lon, pNearPolygon[i].pos.lat));
		}

		if (linestring1.empty() || linestring2.empty())
		{
			return false;
		}
		// 2.2.1寻找最近的两个点
		BgSegment clostPoints;
		bg::closest_points(linestring1, linestring2, clostPoints);
		BgPoint ptF = clostPoints.first;
		BgPoint ptS = clostPoints.second;

		MapPoint64 pt1{ ptF.x(),ptF.y() };
		MapPoint64 pt2{ ptS.x(),ptS.y() };
		bool bRight = false;
		for (int32 i = 0; i < nCurPolygonPtNum-1; i++)
		{
			MapRect64 rect;
			rect.setAsNegtiveMinimum();
			rect.combinePoint(pCurPolygon[i].pos);
			rect.combinePoint(pCurPolygon[i + 1].pos);
			rect.expand(1);
			if (rect.PointInRect(pt1))
			{
				pt1Closest = pCurPolygon[i];
				bRight = true;
				break;
			}
			
		}
		bRight = false;
		for (int32 i = 0; i < nNearPolygonPtNum-1; i++)
		{
			MapRect64 rect;
			rect.setAsNegtiveMinimum();
			rect.combinePoint(pNearPolygon[i].pos);
			rect.combinePoint(pNearPolygon[i + 1].pos);
			rect.expand(1);
			if (rect.PointInRect(pt2))
			{
				pt2Closest = pNearPolygon[i];
				bRight = true;
				break;
			}

		}


		return bRight;
	}




	void TunnelCompiler::mergeHighInfo(std::vector<TunnelLinkGlandHeightInfo> &heighInfo)
	{

		if (heighInfo.empty())
		{
			return;
		}

		std::sort(heighInfo.begin(), heighInfo.end(), [](const TunnelLinkGlandHeightInfo &s1, const TunnelLinkGlandHeightInfo &s2)
				  {
					  if (s1.startIndex != s2.startIndex)
						  return s1.startIndex < s2.startIndex;

					  else
						  return s1.endIndex < s2.endIndex;
				  });
		std::vector<TunnelLinkGlandHeightInfo> mergedheighInfo;
		TunnelLinkGlandHeightInfo curRange = heighInfo[0];
		for (int i = 1; i < heighInfo.size(); i++)
		{
			if (curRange.endIndex >= heighInfo[i].startIndex || GrapPointAlgorithm::geoLengthD(curRange.endPoint.pos.toNdsPoint(), heighInfo[i].startPoint.pos.toNdsPoint()) < 5.0)
			{
				curRange.endIndex = max(curRange.endIndex, heighInfo[i].endIndex);
				curRange.height = min(curRange.height, heighInfo[i].height);
				curRange.endPoint = heighInfo[i].endPoint;
			}
			else
			{
				mergedheighInfo.push_back(curRange);
				curRange = heighInfo[i];
			}
		}

		mergedheighInfo.emplace_back(curRange);
		heighInfo = mergedheighInfo;


	}
	void TunnelCompiler::getCourHeightInfo(std::vector<TunnelLink *> &tunnelLinks, LineString3d &line,
										   std::vector<RdsTunnel::HeightInfo> &rdsHeighInfo, bool bKeepTail)
	{
		std::vector<TunnelLinkGlandHeightInfo> heighInfos;
		for (auto tunnel : tunnelLinks)
		{
			for (auto &heigh : tunnel->linkGlandHeightInfos)
			{
				point_t tmpPointS = POINT_T(heigh.startPoint);
				point_t tmpPointE = POINT_T(heigh.endPoint);
				segment_t tmpSegS, tmpSegE;
				bg::closest_points(tmpPointS, LINESTRING_T(line.vertexes), tmpSegS);
				bg::closest_points(tmpPointE, LINESTRING_T(line.vertexes), tmpSegE);

				MapPoint3D64 mapPtS = MapPoint3D64_make(tmpSegS.second.get<0>(), tmpSegS.second.get<1>(), 0);
				MapPoint3D64 mapPtE = MapPoint3D64_make(tmpSegE.second.get<0>(), tmpSegE.second.get<1>(), 0);
				size_t si1 = 0, ei1 = 0;
				size_t si2 = 0, ei2 = 0;
				MapPoint3D64 ptGrap1, ptGrap2;


				if (GrapPointAlgorithm::GeometryUtil_findIndexWithLine(line.vertexes.data(), line.vertexes.size(), mapPtS, si1, ei1)
					&& GrapPointAlgorithm::GeometryUtil_findIndexWithLine(line.vertexes.data(), line.vertexes.size(), mapPtE, si2, ei2))
				{
					TunnelLinkGlandHeightInfo h;
					h.startIndex = min(si1, si2);
					h.endIndex = max(ei1, ei2);

					h.startPoint = line.vertexes.at(h.startIndex);
					h.endPoint = line.vertexes.at(h.endIndex);
					h.height = heigh.height;

					heighInfos.emplace_back(h);
				}


			}
		}

		mergeHighInfo(heighInfos);

		if (bKeepTail)
		{
			if (!tunnelLinks.front()->linkGlandHeightInfos.empty() && tunnelLinks.front()->linkGlandHeightInfos.front().startIndex == 0)
			{
				if (!heighInfos.empty() && heighInfos.front().startIndex == 0)
				{
					heighInfos.front().startIndex += 1;
				}
				TunnelLinkGlandHeightInfo info = tunnelLinks.front()->linkGlandHeightInfos.front();
				info.endIndex = 1;
				heighInfos.insert(heighInfos.begin(), info);
				
			}

			if (!tunnelLinks.back()->linkGlandHeightInfos.empty() && 
				tunnelLinks.back()->linkGlandHeightInfos.back().endIndex+1 == tunnelLinks.back()->_middleLine.vertexes.size())
			{
				if (!heighInfos.empty() && heighInfos.back().endIndex +1== line.vertexes.size())
				{
					heighInfos.back().endIndex -= 1;
				}
				TunnelLinkGlandHeightInfo info = tunnelLinks.back()->linkGlandHeightInfos.back();
				info.startIndex = line.vertexes.size() - 2;
				info.endIndex = line.vertexes.size()-1;
				heighInfos.emplace_back(info);

			}


		}
		
		for (auto height : heighInfos)
		{
			RdsTunnel::HeightInfo rdsheight;
			rdsheight.startIndex = height.startIndex;
			rdsheight.endIndex = height.endIndex;
			rdsheight.height = height.height;

			rdsHeighInfo.emplace_back(rdsheight);
		}
	}
	bool TunnelCompiler::isTouchWithHadLaneGroup(HadLaneGroup *laneGroup1, HadLaneGroup *laneGroup2)
	{
		for (auto lg : laneGroup1->next)
		{
			if (lg == laneGroup2)
			{
				return true;
			}
		}
		for (auto lg : laneGroup1->previous)
		{
			if (lg == laneGroup2)
			{
				return true;
			}
		}
		return false;
	}
	bool TunnelCompiler::DilutionTunnel(std::vector<MapPoint3D64>& points)
	{
		std::vector<MapPoint3D64> points_temp(points);
		PolylineSimplifier::simplifyT(points_temp, 8000000, 10);//偏差1米
		if (points_temp.size() >= 2 && points_temp.size() < points.size())
		{
			points = points_temp;
			return true;
		}

		return false;
	}
	//void  TunnelCompiler::getNerbyHeightInfo(TunnelLink* tunnnel, bool bfromSelf, bool bNext, double& higninfo)
	//{
	//	higninfo = TUNNEL_HEIGHT;
	//	if (!tunnnel)
	//	{
	//		return;
	//	}
	//	std::vector<TunnelLink*> tunnnels;
	//	if (bfromSelf)
	//	{
	//		tunnnels.emplace_back(tunnnel);
	//	}

	//	if (bNext)
	//	{
	//		while (tunnnel->_next.size() == 1)
	//		{
	//			tunnnels.emplace_back(tunnnel->_next[0]);
	//			tunnnel = tunnnel->_next[0];
	//		}
	//	}
	//	else
	//	{
	//		while (tunnnel->_previous.size() == 1)
	//		{
	//			tunnnels.emplace_back(tunnnel->_previous[0]);
	//			tunnnel = tunnnel->_previous[0];
	//		}
	//		std::reverse(tunnnels.begin(), tunnnels.end());
	//	}

	//	std::vector<MapPoint3D64> middleLinkPoints;
	//	for (int32 i = 0; i < tunnnels.size(); i++)
	//	{
	//		TunnelLink* tunnelLink = tunnnels.at(i);
	//		middleLinkPoints.insert(middleLinkPoints.end(), tunnelLink->_middleLane->location.vertexes.begin(), tunnelLink->_middleLane->location.vertexes.end());
	//	}


	//	//去重（中轴线形状点）
	//	LineString3d line;
	//	auto ip = std::unique(middleLinkPoints.begin(), middleLinkPoints.end(), mapPoint3D64_compare);
	//	middleLinkPoints.resize(std::distance(middleLinkPoints.begin(), ip));
	//	line.vertexes = middleLinkPoints;

	//	std::vector<RdsTunnel::HeightInfo> heightInfos;
	//	getCourHeightInfo(tunnnels, line, heightInfos);


	//	if (heightInfos.empty())
	//	{
	//		return;
	//	}
	//	if (bNext)
	//	{
	//		if (heightInfos[0].startIndex == 0)
	//		{
	//			higninfo = heightInfos[0].height;
	//		}
	//	}
	//	else {
	//		if (heightInfos.back().endIndex == middleLinkPoints.size() - 1)
	//		{
	//			higninfo = heightInfos.back().height;
	//		}
	//	}


	//	return;

	//}
	void  TunnelCompiler::getNerbyHeightInfo(TunnelLink* tunnnel, bool bfromSelf, bool bNext, double& higninfo)
	{
		higninfo = TUNNEL_HEIGHT;
		if (!tunnnel)
		{
			return;
		}
		//TunnelLink* tunnnel_temp = nullptr;
	

		if (bNext)
		{
			std::vector<TunnelLink*> nextTunnels;
			if (bfromSelf)
			{
				nextTunnels.emplace_back(tunnnel);
			}
			else
			{
				for (auto t : tunnnel->_next)
				{
					nextTunnels.emplace_back(t);
				}
				
			}
			
			if (nextTunnels.size() == 1)
			{
				if (!nextTunnels[0]->linkGlandHeightInfos.empty() && nextTunnels[0]->linkGlandHeightInfos.front().startIndex == 0)
				{
					higninfo = nextTunnels[0]->linkGlandHeightInfos.front().height;
				}
			}
			else
			{
				getNearbyHeightInfo(nextTunnels, higninfo);
			}
		
			
		}
		else
		{
			std::vector<TunnelLink*> frontTunnels;
			if (bfromSelf)
			{
				frontTunnels.emplace_back(tunnnel);
			}
			else
			{
				for (auto t : tunnnel->_previous)
				{
					frontTunnels.emplace_back(t);
				}
	
			}

			if (frontTunnels.size() == 1)
			{
				if (!frontTunnels[0]->linkGlandHeightInfos.empty() && frontTunnels[0]->linkGlandHeightInfos.front().startIndex == 0)
				{
					higninfo = frontTunnels[0]->linkGlandHeightInfos.front().height;
				}
			}
			else
			{
				getNearbyHeightInfo(frontTunnels, higninfo);
			}

		}

	}

	void TunnelCompiler::getNearbyHeightInfo(std::vector<TunnelLink*>& tunnnels,double& higninfo) {

		higninfo = TUNNEL_HEIGHT;
		//查找Y隧道
		YTunnel* yTunnel = nullptr;
		for (YTunnel* y : m_YTunnelArray)
		{
			auto iter_left = find_if(tunnnels.begin(), tunnnels.end(), [&](TunnelLink * t)->bool{
				return t == y->main->mixYtunnel->left;
				});
			auto iter_right = find_if(tunnnels.begin(), tunnnels.end(), [&](TunnelLink* t)->bool {
				return t == y->main->mixYtunnel->right;
				});
			if (iter_left != tunnnels.end() && iter_right != tunnnels.end())
			{
				yTunnel = y;
				break;
			}
		}

		if (!yTunnel)
		{
			return;
		}
		// 设置主要线
		setYTunnelMixExtendTunnelGeoLine(yTunnel->main);

		if (yTunnel->main->_leftLine.vertexes.empty() || yTunnel->main->_rightLine.vertexes.empty())
		{
			return ;
		}
		std::vector<RdsTunnel::HeightInfo> heightInfoMain;


		std::vector<TunnelLink*> tunnelLinksMain;
		yTunnel->main->foreachTunnel([&](TunnelLink* tunnel)
			{ tunnelLinksMain.emplace_back(tunnel); });


		getCourHeightInfo(tunnelLinksMain, yTunnel->main->_middleLine, heightInfoMain, false);
		if (!heightInfoMain.empty() && heightInfoMain.front().startIndex == 0)
		{
			higninfo = heightInfoMain.front().height;
		}

	}
	void  TunnelCompiler::mergeHighInfo(std::vector<RdsTunnel::HeightInfo>& heightInfos, std::vector<MapPoint3D64>& middelPoint, std::vector<RdsTunnel::HeightInfo>& mergeHeighInfo)
	{
		middelPoint;
		auto HeightInfoStartIndex = [](const   RdsTunnel::HeightInfo& s1, const   RdsTunnel::HeightInfo& s2)->bool
		{
			if (s1.startIndex != s2.startIndex)
				return   s1.startIndex < s2.startIndex;
			else
				return s1.endIndex < s2.endIndex;
		};
		
		//排序
		sort(heightInfos.begin(), heightInfos.end(), HeightInfoStartIndex);


		RdsTunnel::HeightInfo curRange = heightInfos[0];
		for (int i = 1; i < heightInfos.size(); i++)
		{
			if (curRange.endIndex > heightInfos[i].startIndex
				/*|| GrapPointAlgorithm::geoLengthD(middelPoint.at(curRange.endIndex).pos.toNdsPoint(), middelPoint.at(heightInfos[i].startIndex).pos.toNdsPoint()) < 5.0*/) {
				curRange.endIndex = max(curRange.endIndex, heightInfos[i].endIndex);
				curRange.height = min(curRange.height, heightInfos[i].height);
			}
			else {
				mergeHeighInfo.push_back(curRange);
				curRange = heightInfos[i];
			}
		}
		mergeHeighInfo.emplace_back(curRange);

	}

}
