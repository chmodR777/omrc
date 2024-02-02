#include "stdafx.h"

#include <algorithm>
#include "algorithm/grap_point_algorithm.h"
#include "../framework/SpatialSeacher.h"
#include "algorithm/grap_point_algorithm.h"
#include "GradeSeparationCompiler.h"

//#define __DEBUG_GRADE_SEPARATION__		//用于打印输出立交调试信息


using namespace RDS;
namespace OMDB
{

	const double GradeSeparationCompiler::SEMITRANSPARENT_ABOVE_LG_EXPAND_DISTANCE = 150.0;
	const double GradeSeparationCompiler::SEMITRANSPARENT_BELOW_LG_EXPAND_DISTANCE = 150.0;

	/// @brief 编译立体交叉道路
	/// @param pDatabase 
	/// @param pRDSDatabase 
	void GradeSeparationCompiler::compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile)
	{
		UNREFERENCED_PARAMETER(nearby);
		m_mapLgIntersectLg.clear();
		m_mapBelow2AboveExpandLgs.clear();
		m_mapBelow2BelowExpandLgs.clear();

		//遍历图幅内所有Link，如果Link几何自相交则生成立交关系。
		for (auto obj : pGrid->query(ElementType::HAD_LINK))
		{
			HadLink* pCurLink = (HadLink*)obj;
			_generateCrossRelationByLinkSelfIntersection(pCurLink);
		}

		//遍历图幅内所有车道组，计算车道组间的立交关系。
		for (auto obj : pGrid->query(ElementType::HAD_LANE_GROUP))
		{
			HadLaneGroup* pCurLaneGroup = (HadLaneGroup*)obj;
			_generateCrossRelationByLaneGroups(pGrid, pCurLaneGroup);
		}

		//依车道面拓扑关系向两侧延展压盖区域，补充相交车道组延展信息
		_expandOverlapArea();

		//生成RDS车道组立体交叉数据（RDS_RELATIONSHIP）
		for (auto itB2A : m_mapBelow2AboveExpandLgs)
		{
			HadLaneGroup* pBelowlg = (HadLaneGroup*)itB2A.first;
			SetLg& setAboveLg = itB2A.second;

			RdsGroup* pRdsGroup = queryGroup(pBelowlg->originId, pTile);
			if (!pRdsGroup)
				continue;

			//下层车道组赋值
			auto itBelowExpand = m_mapBelow2BelowExpandLgs.find(pBelowlg);
			if (itBelowExpand != m_mapBelow2BelowExpandLgs.end())
			{
				SetLg& setBelowLg = itBelowExpand->second;
				for (auto lg : setBelowLg)
				{
					//////////////////////////////////////////////////////
					/// @warning 注意调试图幅接边处。
					//////////////////////////////////////////////////////
					RdsRelationship* pRel = (RdsRelationship*)createObject(pTile, EntityType::RDS_RELATIONSHIP);
					pRel->type = RdsRelationship::RelationshipType::OVERLAP;

					//下层车道组赋值
					pRel->first = lg->originId;

					//上层车道组赋值
					_addRdsGroup(setAboveLg, pRel->second);

					//存入RDS数据
					pRdsGroup->objects.push_back(pRel);
				}
			}
		}
	}

	void GradeSeparationCompiler::_addRdsGroup(const SetLg& setLg, std::vector<RdsRelationship::ObjectInfo>& vctRdsGroup)
	{
		for (auto lg : setLg)
		{
			RdsRelationship::ObjectInfo info;
			info.type = EntityType::RDS_GROUP;
			info.id = lg->originId;
			vctRdsGroup.emplace_back(info);
			//RdsGroup* pRdsGroup = queryGroup(lg->originId, pTile);
			//if (pRdsGroup)
			//	vctRdsGroup.emplace_back(pRdsGroup);
			//else
			//	printWarning("\t警告：车道组%I64不存在！不应该存在这种情况！", lg->originId);
		}
	}

	void GradeSeparationCompiler::_generateCrossRelationByLaneGroups(HadGrid* pGrid, HadLaneGroup* pGroup)
	{
		bool bCollide = false;
		MapPoint3D64* pCurPolygon = NULL;
		size_t nCurPolygonPtNum = 0;

		//空间搜索。仅通过地物对象的BoundingBox2d空间关系搜索。
		std::vector<HadLaneGroup*> pNearbyLinkGroups = SpatialSeacher::seachNearby2d(pGrid, pGroup);
		if (pNearbyLinkGroups.size() > 0)
			nCurPolygonPtNum = makeLaneGroupPolygon(pGroup, pCurPolygon);	//创建当前车道组面闭合多边形。

		for (HadLaneGroup* pNearbyLinkGroup : pNearbyLinkGroups)
		{
			//通过存储已经计算过的一对pGroup和pNearbyLinkGroup信息，来减少一半的空间计算量。
			//注意后文记入有效数据的条件：无论上下方都记入。
			LaneGroupPair lgPair(pGroup, pNearbyLinkGroup);
			auto result = m_setLgPair.insert(lgPair);
			if (result.second)	//如果没计算过，则计算空间关系
			{
				//生成两个车道组的相交数据
				_lgIntersectProcess(pGroup, pCurPolygon, nCurPolygonPtNum, pNearbyLinkGroup);
			}
		}
	}

	/*
	size_t GradeSeparationCompiler::_makeLaneGroupPolygon(HadLaneGroup* pGroup, MapPoint3D64*& polygon)
	{
		if (pGroup->laneBoundaries.size() < 2)
			return 0;

		size_t laneBoundaryNum = pGroup->laneBoundaries.size();
		size_t nPolyline1PtNum = pGroup->laneBoundaries[0]->location.vertexes.size();
		size_t nPolyline2PtNum = pGroup->laneBoundaries[laneBoundaryNum - 1]->location.vertexes.size();
		size_t nPtNum = nPolyline1PtNum + nPolyline2PtNum + 1 + (laneBoundaryNum-2)*2;

		//添加路面左边线几何点。
		polygon = new MapPoint3D64[nPtNum];
		MapPoint3D64* pCurPt = polygon;
		memcpy(pCurPt, pGroup->laneBoundaries[0]->location.vertexes.data(), nPolyline1PtNum * sizeof(MapPoint3D64));
		pCurPt += nPolyline1PtNum;

		//添加除道路边线外的各车道边界线末点。
		for( int i=1; i <= laneBoundaryNum-2; i++)
		{
			*pCurPt = pGroup->laneBoundaries[i]->location.vertexes.back();
			pCurPt++;
		}

		//添加路面右边线几何点。
		std::vector<MapPoint3D64> Line2Reverse(nPolyline2PtNum);
		std::reverse_copy(pGroup->laneBoundaries[laneBoundaryNum - 1]->location.vertexes.begin(), pGroup->laneBoundaries[laneBoundaryNum - 1]->location.vertexes.end(), Line2Reverse.begin());
		memcpy(pCurPt, Line2Reverse.data(), nPolyline2PtNum * sizeof(MapPoint3D64));
		pCurPt += nPolyline2PtNum;

		//添加除道路边线外的各车道边界线首点。
		for (size_t i = laneBoundaryNum - 2; i >=1; i--)
		{
			*pCurPt = pGroup->laneBoundaries[i]->location.vertexes.front();
			pCurPt++;
		}

		polygon[nPtNum - 1] = pGroup->laneBoundaries[0]->location.vertexes[0];

		return nPtNum;
	}
	*/

	

	void GradeSeparationCompiler::_lgIntersectProcess(HadLaneGroup* pCurLg, MapPoint3D64* pCurPolygon, size_t nCurPolygonPtNum, HadLaneGroup* pNearbyLg)
	{
		UNUSED_VAR(pCurLg);

		//计算几何中心点函数。
		auto getCenterPt = [](const ClipperLib::Path& path) {
			MapPoint3D64 ptCenter = { 0 };
			size_t size = path.size();

			int64 nTotalX = 0, nTotalY = 0;
			for( int i=0; i<size; i++)
			{
				nTotalX += path[i].X;
				nTotalY += path[i].Y;
			}
			ptCenter.pos.lon = nTotalX / size;
			ptCenter.pos.lat = nTotalY / size;
			return ptCenter;
		};


		//两个车道面相交判定
		MapPoint3D64* pNearbyPolygon = NULL;
		size_t nNearbyPolygonPtNum = makeLaneGroupPolygon(pNearbyLg, pNearbyPolygon);	//创建附近的车道组面闭合多边形。
		if (nNearbyPolygonPtNum == 0)
			return;

		ClipperLib::PolyTree polyTree;
		bool isIntersect = _intersect(pCurPolygon, nCurPolygonPtNum, pNearbyPolygon, nNearbyPolygonPtNum, polyTree);
		delete[] pNearbyPolygon;

		if (!isIntersect)
			return;

		//整理相交部分
#ifdef	__DEBUG_GRADE_SEPARATION__
		printInfo("-->车道组%I64d与车道组%I64d相交，共%d个相交区域，tree子节点数%d ",
			pCurLg->originId, pNearbyLg->originId, polyTree.Total(), polyTree.ChildCount());
#endif	//__DEBUG_GRADE_SEPARATION__

		ClipperLib::Paths paths;
		ClipperLib::ClosedPathsFromPolyTree(polyTree, paths);	//得到闭合轮廓
#ifdef	__DEBUG_GRADE_SEPARATION__
		printInfo("\t闭合轮廓数%d", paths.size());
#endif	//__DEBUG_GRADE_SEPARATION__
		ClipperLib::CleanPolygons(paths);		//清理对后续处理无用的几何

		//遍历所有相交的轮廓，计算车道面空间关系
		for( auto path : paths)
		{
			if (path.size() <= 0)
			{
				//printWarning("\t车道组相交结果中path坐标数<=0，不应该存在这种情况！图幅：%d，curLG：%I64d，nearbyLG：%I64d", pCurLg->owner->getId(), pCurLg->originId, pNearbyLg->originId );
				continue;
			}

			//计算中心点
			MapPoint3D64 ptCenter = getCenterPt(path);

			//得到中心点投影/匹配到车道边界线上的点。（因为存在1个LG对应多个Link情况，故这里使用车道边界线而没有使用Link的形点）
			MapPoint3D64 ptGrapCurLg = { 0 }, ptGrapNearbyLg = { 0 };
			size_t si = 0, ei = 0;
			if (!GrapPointAlgorithm::grapPoint(ptCenter, pCurLg->lanes[0]->location.vertexes.data(), pCurLg->lanes[0]->location.vertexes.size(), ptGrapCurLg, si, ei, 50.0))
				ptGrapCurLg = _nearestLineStringEnd(ptCenter, pCurLg->lanes[0]->location);
			if (!GrapPointAlgorithm::grapPoint(ptCenter, pNearbyLg->lanes[0]->location.vertexes.data(), pNearbyLg->lanes[0]->location.vertexes.size(), ptGrapNearbyLg, si, ei, 50.0))
				ptGrapNearbyLg = _nearestLineStringEnd(ptCenter, pNearbyLg->lanes[0]->location);

			//判定两个车道组的高度，判断上下层，记入有效数据。
			if (ptGrapNearbyLg.z - ptGrapCurLg.z > SEMITRANSPARENT_HEIGHT_DIFF_THRESHOLD)
			{
				_insertLgIntersection(pCurLg, ptGrapCurLg, pNearbyLg, ptGrapNearbyLg);
			}
			else if (ptGrapCurLg.z - ptGrapNearbyLg.z >= SEMITRANSPARENT_HEIGHT_DIFF_THRESHOLD)
			{
				_insertLgIntersection(pNearbyLg, ptGrapNearbyLg, pCurLg, ptGrapCurLg);
			}

		}

		polyTree.Clear();
	}

	void GradeSeparationCompiler::_insertLgIntersection(HadLaneGroup* pBelowLg, const MapPoint3D64& ptBelowGrap, HadLaneGroup* pAboveLg, const MapPoint3D64& ptAboveGrap)
	{
		//插入下方车道组交叉信息
		auto pairBelow = m_mapLgIntersectLg.try_emplace(pBelowLg);
		MapLgIntersection& mapAboveLgInt = pairBelow.first->second;

		//插入对应的上方车道组交叉信息
		auto pairAbove = mapAboveLgInt.try_emplace(pAboveLg);
		VectorOverlapArea& vctOverlapArea = pairAbove.first->second;
		OverlapArea overlapArea = { 0 };
		overlapArea.ptAboveGrap = ptAboveGrap;
		overlapArea.ptBelowGrap = ptBelowGrap;
		vctOverlapArea.emplace_back(overlapArea);
	}

	void GradeSeparationCompiler::_expandOverlapArea()
	{
#ifdef	__DEBUG_GRADE_SEPARATION__
		printInfo("\t------------------ 开始遍历立交区域 -----------------------------"); 
#endif //__DEBUG_GRADE_SEPARATION__

		for( auto itBelow = m_mapLgIntersectLg.begin(); itBelow != m_mapLgIntersectLg.end(); itBelow++ )
		{
			const HadLaneGroup* pBelowLg = itBelow->first;
			MapLgIntersection& mapAboveLgInt = itBelow->second;
			for(auto itAbove = mapAboveLgInt.begin(); itAbove!= mapAboveLgInt.end(); itAbove++)
			{
				const HadLaneGroup* pAboveLg = itAbove->first;
				VectorOverlapArea& vctOverlapArea = itAbove->second;

				////////////// test ///////////////////
				//if (vctOverlapArea.size() > 1)
				//	int a = 0;

				///////////////////// begin /////////////////////////////////////////////////////
				//对每一个相交区（包括上层和下层），根据拓扑关系向两侧延展车道面至指定距离
				auto pairMapAboveIter = m_mapBelow2AboveExpandLgs.try_emplace(pBelowLg);
				SetLg& setAboveLgs = pairMapAboveIter.first->second;

				auto pairMapBelowIter = m_mapBelow2BelowExpandLgs.try_emplace(pBelowLg);
				SetLg& setBelowLgs = pairMapBelowIter.first->second;

				for(auto itOverlap = vctOverlapArea.begin(); itOverlap!= vctOverlapArea.end(); itOverlap++)
				{
					OverlapArea& ov = *itOverlap;
#ifdef	__DEBUG_GRADE_SEPARATION__
					printInfo("-->下方车道组%I64d与上方车道组%I64d立体相交，重合区域内坐标点(%I64d,%I64d) ",
						pBelowLg->originId, pAboveLg->originId, ov.ptAboveGrap.pos.lon, ov.ptAboveGrap.pos.lat);
#endif	//__DEBUG_GRADE_SEPARATION__

					//上方车道拓扑延展
					_recursivePreviousLaneGroup(pAboveLg, GradeSeparationCompiler::SEMITRANSPARENT_ABOVE_LG_EXPAND_DISTANCE, setAboveLgs, true, ov.ptAboveGrap);
					_recursiveNextLaneGroup(pAboveLg, GradeSeparationCompiler::SEMITRANSPARENT_ABOVE_LG_EXPAND_DISTANCE, setAboveLgs, true, ov.ptAboveGrap);

					//下方车道拓扑延展
					_recursivePreviousLaneGroup(pBelowLg, GradeSeparationCompiler::SEMITRANSPARENT_BELOW_LG_EXPAND_DISTANCE, setBelowLgs, true, ov.ptBelowGrap);
					_recursiveNextLaneGroup(pBelowLg, GradeSeparationCompiler::SEMITRANSPARENT_BELOW_LG_EXPAND_DISTANCE, setBelowLgs, true, ov.ptBelowGrap);
				}

#ifdef	__DEBUG_GRADE_SEPARATION__
				for (auto lg : setAboveLgs)
					printInfo("\t-->下方车道组%I64d，上方车道组延长 + %I64d ", pBelowLg->originId, lg->originId);
				printInfo("\t-->===================================================== ");
				for (auto lg : setBelowLgs)
					printInfo("\t-->下方车道组%I64d，下方车道组延长 + %I64d ", pBelowLg->originId, lg->originId);
#endif	//__DEBUG_GRADE_SEPARATION__
				////////////////////// end ////////////////////////////////////////////////

			}
		}
#ifdef	__DEBUG_GRADE_SEPARATION__
		printInfo("\t------------------ 遍历完成 -----------------------------");
#endif	//__DEBUG_GRADE_SEPARATION__
	}

	/// 遍历得到前序车道组
	bool GradeSeparationCompiler::_recursivePreviousLaneGroup(const HadLaneGroup* pGroup, double dRemainDis, SetLg& setExpandLg, bool bFirst/* = false*/, MapPoint3D64 ptLocation/* = { 0 }*/)
	{
		setExpandLg.insert(pGroup);

		const MapPoint3D64* pPolylinePts = pGroup->lanes[0]->location.vertexes.data();
		size_t szPolylinePtsCount = pGroup->lanes[0]->location.vertexes.size();
		size_t si = 0, ei = 0;
		if (bFirst)
		{
			MapPoint3D64 ptGrap = { 0 };
			if (!GrapPointAlgorithm::grapPoint(ptLocation, pPolylinePts, szPolylinePtsCount, ptGrap, si, ei, 50.0))
			{
				//容错。这种情况不一定发生。
				ptGrap = _nearestLineStringEnd(ptLocation, pGroup->lanes[0]->location);
				if (ptGrap == pPolylinePts[0])
					si = 0;
				else if (ptGrap == pPolylinePts[szPolylinePtsCount - 1])
					si = szPolylinePtsCount - 1;
				else
					printError("\t\t匹路错误：坐标点(%I64d,%I64d)匹路到车道组%I64d", ptLocation, pGroup->originId);
			}

			//判定匹配到当前segment上的剩余距离
			double tailDis = _getLength(ptGrap, pPolylinePts[si]);
			dRemainDis -= tailDis;
			if (dRemainDis <= 0)
				return false;
		}
		else
			si = szPolylinePtsCount - 1;

		for (size_t i = si; i > 0; i--)
		{
			double dSegmentDis = _getLength(pPolylinePts[si], pPolylinePts[si - 1]);
			dRemainDis -= dSegmentDis;
			if (dRemainDis <= 0)
				return false;
		}

		//递归调用，遍历前序
		for (auto obj : pGroup->previous)
		{
			HadLaneGroup* pPreLg = (HadLaneGroup*)obj;
			_recursivePreviousLaneGroup(pPreLg, dRemainDis, setExpandLg);
		}

		return true;
	}

	/// 遍历得到后序车道组
	bool GradeSeparationCompiler::_recursiveNextLaneGroup(const HadLaneGroup* pGroup, double dRemainDis, SetLg& setExpandLg, bool bFirst/* = false*/, MapPoint3D64 ptLocation/* = { 0 }*/)
	{
		setExpandLg.insert(pGroup);

		const MapPoint3D64* pPolylinePts = pGroup->lanes[0]->location.vertexes.data();
		size_t szPolylinePtsCount = pGroup->lanes[0]->location.vertexes.size();
		size_t si = 0, ei = 0;
		if (bFirst)
		{
			MapPoint3D64 ptGrap = { 0 };
			if (!GrapPointAlgorithm::grapPoint(ptLocation, pPolylinePts, szPolylinePtsCount, ptGrap, si, ei, 50.0))
			{
				//容错。这种情况不一定发生。
				ptGrap = _nearestLineStringEnd(ptLocation, pGroup->lanes[0]->location);
				if (ptGrap == pPolylinePts[0])
					ei = 0;
				else if (ptGrap == pPolylinePts[szPolylinePtsCount - 1])
					ei = szPolylinePtsCount - 1;
				else
					printError("\t\t匹路错误：坐标点(%I64d,%I64d)匹路到车道组%I64d", ptLocation, pGroup->originId);
			}

			//判定匹配到当前segment上的剩余距离
			double tailDis = _getLength(ptGrap, pPolylinePts[ei]);
			dRemainDis -= tailDis;
			if (dRemainDis <= 0)
				return false;
		}
		else
			ei = 0;

		for (size_t i = ei; i < szPolylinePtsCount - 1; i++)
		{
			double dSegmentDis = _getLength(pPolylinePts[ei], pPolylinePts[ei + 1]);
			dRemainDis -= dSegmentDis;
			if (dRemainDis <= 0)
				return false;
		}

		//递归调用，遍历前序
		for (auto obj : pGroup->next)
		{
			HadLaneGroup* pNextLg = (HadLaneGroup*)obj;
			_recursiveNextLaneGroup(pNextLg, dRemainDis, setExpandLg);
		}

		return true;
	}

	double GradeSeparationCompiler::_getLength(const MapPoint3D64& point1, const MapPoint3D64& point2)
	{
		double distance = 0.0;
		distance = GrapPointAlgorithm::geoLengthD(point1.pos.toNdsPoint(), point2.pos.toNdsPoint());
		return distance;
	}

	bool GradeSeparationCompiler::_generateCrossRelationByLinkSelfIntersection(HadLink* pLink)
	{
		BgLinestring bgLine1;
		_mapPointsToBgLine(pLink->location.vertexes.data(), (int32)pLink->location.vertexes.size(), bgLine1);

		//自相交判断
		bool is_self_intersects = boost::geometry::intersects(bgLine1);
		if (is_self_intersects)
		{
#ifdef	__DEBUG_GRADE_SEPARATION__
			printInfo("\tLink %I64d自相交", pLink->originId);
#endif //#ifdef	__DEBUG_GRADE_SEPARATION__

			//使用中间点而非使用自相交点，是因为boost和ClipperLib没有方法求自相交点。
			//由于后续处理方法中拓扑延展策略的存在，理论上使用中间点对计算结果的影响很小。同时，计算自相交点，也需要耗费算力。
			MapPoint3D64 center = pLink->location.vertexes[pLink->location.vertexes.size() / 2];
			for(auto lgBelow : pLink->groups)
			{
				for (auto lgAbove : pLink->groups)
				{
					_insertLgIntersection(lgBelow, center, lgAbove, center);
				}
			}
		}
		return false;
	}

	void GradeSeparationCompiler::_mapPointsToBgLine(const MapPoint3D64* points, int32 count, BgLinestring& resultOut)
	{
		resultOut.resize(count);
		for (int i = 0; i < count; i++)
		{
			resultOut[i].x((const double)points[i].pos.lon);
			resultOut[i].y((const double)points[i].pos.lat);
		}
	}

}

