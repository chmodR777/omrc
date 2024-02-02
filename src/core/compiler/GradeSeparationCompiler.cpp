#include "stdafx.h"

#include <algorithm>
#include "algorithm/grap_point_algorithm.h"
#include "../framework/SpatialSeacher.h"
#include "algorithm/grap_point_algorithm.h"
#include "GradeSeparationCompiler.h"

//#define __DEBUG_GRADE_SEPARATION__		//���ڴ�ӡ�������������Ϣ


using namespace RDS;
namespace OMDB
{

	const double GradeSeparationCompiler::SEMITRANSPARENT_ABOVE_LG_EXPAND_DISTANCE = 150.0;
	const double GradeSeparationCompiler::SEMITRANSPARENT_BELOW_LG_EXPAND_DISTANCE = 150.0;

	/// @brief �������彻���·
	/// @param pDatabase 
	/// @param pRDSDatabase 
	void GradeSeparationCompiler::compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile)
	{
		UNREFERENCED_PARAMETER(nearby);
		m_mapLgIntersectLg.clear();
		m_mapBelow2AboveExpandLgs.clear();
		m_mapBelow2BelowExpandLgs.clear();

		//����ͼ��������Link�����Link�������ཻ������������ϵ��
		for (auto obj : pGrid->query(ElementType::HAD_LINK))
		{
			HadLink* pCurLink = (HadLink*)obj;
			_generateCrossRelationByLinkSelfIntersection(pCurLink);
		}

		//����ͼ�������г����飬���㳵������������ϵ��
		for (auto obj : pGrid->query(ElementType::HAD_LANE_GROUP))
		{
			HadLaneGroup* pCurLaneGroup = (HadLaneGroup*)obj;
			_generateCrossRelationByLaneGroups(pGrid, pCurLaneGroup);
		}

		//�����������˹�ϵ��������չѹ�����򣬲����ཻ��������չ��Ϣ
		_expandOverlapArea();

		//����RDS���������彻�����ݣ�RDS_RELATIONSHIP��
		for (auto itB2A : m_mapBelow2AboveExpandLgs)
		{
			HadLaneGroup* pBelowlg = (HadLaneGroup*)itB2A.first;
			SetLg& setAboveLg = itB2A.second;

			RdsGroup* pRdsGroup = queryGroup(pBelowlg->originId, pTile);
			if (!pRdsGroup)
				continue;

			//�²㳵���鸳ֵ
			auto itBelowExpand = m_mapBelow2BelowExpandLgs.find(pBelowlg);
			if (itBelowExpand != m_mapBelow2BelowExpandLgs.end())
			{
				SetLg& setBelowLg = itBelowExpand->second;
				for (auto lg : setBelowLg)
				{
					//////////////////////////////////////////////////////
					/// @warning ע�����ͼ���ӱߴ���
					//////////////////////////////////////////////////////
					RdsRelationship* pRel = (RdsRelationship*)createObject(pTile, EntityType::RDS_RELATIONSHIP);
					pRel->type = RdsRelationship::RelationshipType::OVERLAP;

					//�²㳵���鸳ֵ
					pRel->first = lg->originId;

					//�ϲ㳵���鸳ֵ
					_addRdsGroup(setAboveLg, pRel->second);

					//����RDS����
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
			//	printWarning("\t���棺������%I64�����ڣ���Ӧ�ô������������", lg->originId);
		}
	}

	void GradeSeparationCompiler::_generateCrossRelationByLaneGroups(HadGrid* pGrid, HadLaneGroup* pGroup)
	{
		bool bCollide = false;
		MapPoint3D64* pCurPolygon = NULL;
		size_t nCurPolygonPtNum = 0;

		//�ռ���������ͨ����������BoundingBox2d�ռ��ϵ������
		std::vector<HadLaneGroup*> pNearbyLinkGroups = SpatialSeacher::seachNearby2d(pGrid, pGroup);
		if (pNearbyLinkGroups.size() > 0)
			nCurPolygonPtNum = makeLaneGroupPolygon(pGroup, pCurPolygon);	//������ǰ��������պ϶���Ρ�

		for (HadLaneGroup* pNearbyLinkGroup : pNearbyLinkGroups)
		{
			//ͨ���洢�Ѿ��������һ��pGroup��pNearbyLinkGroup��Ϣ��������һ��Ŀռ��������
			//ע����ļ�����Ч���ݵ��������������·������롣
			LaneGroupPair lgPair(pGroup, pNearbyLinkGroup);
			auto result = m_setLgPair.insert(lgPair);
			if (result.second)	//���û������������ռ��ϵ
			{
				//����������������ཻ����
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

		//���·������߼��ε㡣
		polygon = new MapPoint3D64[nPtNum];
		MapPoint3D64* pCurPt = polygon;
		memcpy(pCurPt, pGroup->laneBoundaries[0]->location.vertexes.data(), nPolyline1PtNum * sizeof(MapPoint3D64));
		pCurPt += nPolyline1PtNum;

		//��ӳ���·������ĸ������߽���ĩ�㡣
		for( int i=1; i <= laneBoundaryNum-2; i++)
		{
			*pCurPt = pGroup->laneBoundaries[i]->location.vertexes.back();
			pCurPt++;
		}

		//���·���ұ��߼��ε㡣
		std::vector<MapPoint3D64> Line2Reverse(nPolyline2PtNum);
		std::reverse_copy(pGroup->laneBoundaries[laneBoundaryNum - 1]->location.vertexes.begin(), pGroup->laneBoundaries[laneBoundaryNum - 1]->location.vertexes.end(), Line2Reverse.begin());
		memcpy(pCurPt, Line2Reverse.data(), nPolyline2PtNum * sizeof(MapPoint3D64));
		pCurPt += nPolyline2PtNum;

		//��ӳ���·������ĸ������߽����׵㡣
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

		//���㼸�����ĵ㺯����
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


		//�����������ཻ�ж�
		MapPoint3D64* pNearbyPolygon = NULL;
		size_t nNearbyPolygonPtNum = makeLaneGroupPolygon(pNearbyLg, pNearbyPolygon);	//���������ĳ�������պ϶���Ρ�
		if (nNearbyPolygonPtNum == 0)
			return;

		ClipperLib::PolyTree polyTree;
		bool isIntersect = _intersect(pCurPolygon, nCurPolygonPtNum, pNearbyPolygon, nNearbyPolygonPtNum, polyTree);
		delete[] pNearbyPolygon;

		if (!isIntersect)
			return;

		//�����ཻ����
#ifdef	__DEBUG_GRADE_SEPARATION__
		printInfo("-->������%I64d�복����%I64d�ཻ����%d���ཻ����tree�ӽڵ���%d ",
			pCurLg->originId, pNearbyLg->originId, polyTree.Total(), polyTree.ChildCount());
#endif	//__DEBUG_GRADE_SEPARATION__

		ClipperLib::Paths paths;
		ClipperLib::ClosedPathsFromPolyTree(polyTree, paths);	//�õ��պ�����
#ifdef	__DEBUG_GRADE_SEPARATION__
		printInfo("\t�պ�������%d", paths.size());
#endif	//__DEBUG_GRADE_SEPARATION__
		ClipperLib::CleanPolygons(paths);		//����Ժ����������õļ���

		//���������ཻ�����������㳵����ռ��ϵ
		for( auto path : paths)
		{
			if (path.size() <= 0)
			{
				//printWarning("\t�������ཻ�����path������<=0����Ӧ�ô������������ͼ����%d��curLG��%I64d��nearbyLG��%I64d", pCurLg->owner->getId(), pCurLg->originId, pNearbyLg->originId );
				continue;
			}

			//�������ĵ�
			MapPoint3D64 ptCenter = getCenterPt(path);

			//�õ����ĵ�ͶӰ/ƥ�䵽�����߽����ϵĵ㡣����Ϊ����1��LG��Ӧ���Link�����������ʹ�ó����߽��߶�û��ʹ��Link���ε㣩
			MapPoint3D64 ptGrapCurLg = { 0 }, ptGrapNearbyLg = { 0 };
			size_t si = 0, ei = 0;
			if (!GrapPointAlgorithm::grapPoint(ptCenter, pCurLg->lanes[0]->location.vertexes.data(), pCurLg->lanes[0]->location.vertexes.size(), ptGrapCurLg, si, ei, 50.0))
				ptGrapCurLg = _nearestLineStringEnd(ptCenter, pCurLg->lanes[0]->location);
			if (!GrapPointAlgorithm::grapPoint(ptCenter, pNearbyLg->lanes[0]->location.vertexes.data(), pNearbyLg->lanes[0]->location.vertexes.size(), ptGrapNearbyLg, si, ei, 50.0))
				ptGrapNearbyLg = _nearestLineStringEnd(ptCenter, pNearbyLg->lanes[0]->location);

			//�ж�����������ĸ߶ȣ��ж����²㣬������Ч���ݡ�
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
		//�����·������齻����Ϣ
		auto pairBelow = m_mapLgIntersectLg.try_emplace(pBelowLg);
		MapLgIntersection& mapAboveLgInt = pairBelow.first->second;

		//�����Ӧ���Ϸ������齻����Ϣ
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
		printInfo("\t------------------ ��ʼ������������ -----------------------------"); 
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
				//��ÿһ���ཻ���������ϲ���²㣩���������˹�ϵ��������չ��������ָ������
				auto pairMapAboveIter = m_mapBelow2AboveExpandLgs.try_emplace(pBelowLg);
				SetLg& setAboveLgs = pairMapAboveIter.first->second;

				auto pairMapBelowIter = m_mapBelow2BelowExpandLgs.try_emplace(pBelowLg);
				SetLg& setBelowLgs = pairMapBelowIter.first->second;

				for(auto itOverlap = vctOverlapArea.begin(); itOverlap!= vctOverlapArea.end(); itOverlap++)
				{
					OverlapArea& ov = *itOverlap;
#ifdef	__DEBUG_GRADE_SEPARATION__
					printInfo("-->�·�������%I64d���Ϸ�������%I64d�����ཻ���غ������������(%I64d,%I64d) ",
						pBelowLg->originId, pAboveLg->originId, ov.ptAboveGrap.pos.lon, ov.ptAboveGrap.pos.lat);
#endif	//__DEBUG_GRADE_SEPARATION__

					//�Ϸ�����������չ
					_recursivePreviousLaneGroup(pAboveLg, GradeSeparationCompiler::SEMITRANSPARENT_ABOVE_LG_EXPAND_DISTANCE, setAboveLgs, true, ov.ptAboveGrap);
					_recursiveNextLaneGroup(pAboveLg, GradeSeparationCompiler::SEMITRANSPARENT_ABOVE_LG_EXPAND_DISTANCE, setAboveLgs, true, ov.ptAboveGrap);

					//�·�����������չ
					_recursivePreviousLaneGroup(pBelowLg, GradeSeparationCompiler::SEMITRANSPARENT_BELOW_LG_EXPAND_DISTANCE, setBelowLgs, true, ov.ptBelowGrap);
					_recursiveNextLaneGroup(pBelowLg, GradeSeparationCompiler::SEMITRANSPARENT_BELOW_LG_EXPAND_DISTANCE, setBelowLgs, true, ov.ptBelowGrap);
				}

#ifdef	__DEBUG_GRADE_SEPARATION__
				for (auto lg : setAboveLgs)
					printInfo("\t-->�·�������%I64d���Ϸ��������ӳ� + %I64d ", pBelowLg->originId, lg->originId);
				printInfo("\t-->===================================================== ");
				for (auto lg : setBelowLgs)
					printInfo("\t-->�·�������%I64d���·��������ӳ� + %I64d ", pBelowLg->originId, lg->originId);
#endif	//__DEBUG_GRADE_SEPARATION__
				////////////////////// end ////////////////////////////////////////////////

			}
		}
#ifdef	__DEBUG_GRADE_SEPARATION__
		printInfo("\t------------------ ������� -----------------------------");
#endif	//__DEBUG_GRADE_SEPARATION__
	}

	/// �����õ�ǰ�򳵵���
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
				//�ݴ����������һ��������
				ptGrap = _nearestLineStringEnd(ptLocation, pGroup->lanes[0]->location);
				if (ptGrap == pPolylinePts[0])
					si = 0;
				else if (ptGrap == pPolylinePts[szPolylinePtsCount - 1])
					si = szPolylinePtsCount - 1;
				else
					printError("\t\tƥ·���������(%I64d,%I64d)ƥ·��������%I64d", ptLocation, pGroup->originId);
			}

			//�ж�ƥ�䵽��ǰsegment�ϵ�ʣ�����
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

		//�ݹ���ã�����ǰ��
		for (auto obj : pGroup->previous)
		{
			HadLaneGroup* pPreLg = (HadLaneGroup*)obj;
			_recursivePreviousLaneGroup(pPreLg, dRemainDis, setExpandLg);
		}

		return true;
	}

	/// �����õ����򳵵���
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
				//�ݴ����������һ��������
				ptGrap = _nearestLineStringEnd(ptLocation, pGroup->lanes[0]->location);
				if (ptGrap == pPolylinePts[0])
					ei = 0;
				else if (ptGrap == pPolylinePts[szPolylinePtsCount - 1])
					ei = szPolylinePtsCount - 1;
				else
					printError("\t\tƥ·���������(%I64d,%I64d)ƥ·��������%I64d", ptLocation, pGroup->originId);
			}

			//�ж�ƥ�䵽��ǰsegment�ϵ�ʣ�����
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

		//�ݹ���ã�����ǰ��
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

		//���ཻ�ж�
		bool is_self_intersects = boost::geometry::intersects(bgLine1);
		if (is_self_intersects)
		{
#ifdef	__DEBUG_GRADE_SEPARATION__
			printInfo("\tLink %I64d���ཻ", pLink->originId);
#endif //#ifdef	__DEBUG_GRADE_SEPARATION__

			//ʹ���м�����ʹ�����ཻ�㣬����Ϊboost��ClipperLibû�з��������ཻ�㡣
			//���ں�����������������չ���ԵĴ��ڣ�������ʹ���м��Լ�������Ӱ���С��ͬʱ���������ཻ�㣬Ҳ��Ҫ�ķ�������
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

