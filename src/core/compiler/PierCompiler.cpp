
#include "stdafx.h"
#include "PierCompiler.h"
#include <algorithm>
#include "map-render/glmap/glmap_types.h"
#include "../framework/SpatialSeacher.h"
#include "math3d/nds_point_3d.h"
#include "clipper.hpp"
#include "map_point3d64_converter.h"
#include "algorithm/tangential_algorithm.h"
#include "point_converter.h"
#include "algorithm/grap_point_algorithm.h"

//#define __DEBUG_PIER__		//���ڴ�ӡ����Ŷյ�����Ϣ

using namespace RDS;

namespace OMDB
{
	const float PIER_INTERVAL_METERS = 200.0f;
	const float PIER_HEIGHT_ERROR_METERS = 1.0f;
	const float PIER_START_OFFSET_METERS = PIER_INTERVAL_METERS * 0.5f;

	static NdsPoint _changeNdsPointWithHeading(const NdsPoint& ndsPoint, float heading, float meter)
	{
		Coordinate coord = Coordinate_makeWithNdsPoint(ndsPoint);
		const double x2yRatio = cos(MATH_DEGREE2RADIAN * coord.lat);
		coord.lon *= x2yRatio;
		coord.lon *= 100000;
		coord.lat *= 100000;

		double lenInLat = meter * LAT_UNIT_PER_METER;
		coord.lon += lenInLat * cos((180 - heading - 90) * MATH_DEGREE2RADIAN);
		coord.lat += lenInLat * sin((180 - heading - 90) * MATH_DEGREE2RADIAN);

		coord.lon /= x2yRatio;
		coord.lon /= 100000;
		coord.lat /= 100000;

		NdsPoint result = NdsPoint_makeWithCoordinate(coord);
		return result;
	}

	/**
	* @brief �Ŷձ��롣
	*/
	void PierCompiler::compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile)
	{
		UNREFERENCED_PARAMETER(nearby);

#ifdef __DEBUG_PIER__
		printInfo("\n######## meshId = %d\tNdsGridId = %d ########", grid->getId(), grid->getNdsGridId());
#endif // __DEBUG_PIER__

		//��ʼ��ͼ��
		m_vctResampleResult.clear();
		m_setGroupIdInMesh.clear();
		_initNdsGird(MeshId_toNdsGridId(pGrid->getId()));

		//����ͼ�������г�����
		for (auto obj : pGrid->query(ElementType::HAD_LANE_GROUP))
		{
			HadLaneGroup* pGroup = (HadLaneGroup*)obj;

			//��ͨ·�������Ŷ�
			if (!isProDataLevel(pGroup))
				continue;

			//�����Ѿ��ϲ����ĳ�����
			if (m_setGroupIdInMesh.find(pGroup->originId) != m_setGroupIdInMesh.end())
				continue;

			//���������ĳ�����
			std::list<HadLaneGroup*> listCombineLaneGroup;	//��ǰ��ͺ������޷ֲ�ĳ��������У���Խlink��
			listCombineLaneGroup.push_front(pGroup);
			m_setGroupIdInMesh.insert(pGroup->originId);
			_recursiveForwardLaneGroup(pGroup, listCombineLaneGroup);	//�����õ�ǰ�򳵵���
			_recursiveBackwardLaneGroup(pGroup, listCombineLaneGroup);	//�����õ�����������

#ifdef __DEBUG_PIER__
			printInfo("��ǰpGroup->originId = %I64d����Ϻ󳵵������У�", pGroup->originId);
			for (auto g : listCombineLaneGroup)
			{
				printInfo("pCurGroup->originId = %I64d", g->originId);
			}
			printInfo("----------");
#endif // __DEBUG_PIER__				

			//������������������������ز�����
			_combineLaneGroupRresample(listCombineLaneGroup, PIER_INTERVAL_METERS, PIER_START_OFFSET_METERS);

			//�����������������λ�ã�ʹ֮���ӱƽ�·������λ�á�
			_amendResamplePt();

			//��ײ��⣬���������Ŷա�
			for (size_t i = 0; i < m_vctResampleResult.size(); i++)
			{
				MapPoint3D64 mp = m_vctResampleResult[i].pt;
				NdsPoint ndsPtCenter = mp.pos.toNdsPoint();		//NDS����

				HadLaneGroup* pCurGroup = m_vctResampleResult[i].lg;
				BoundingBox2d pierBoundingBoxNds = { 0 }, pierBoundingBox = { 0 };
				int nRoadWidthCal = pCurGroup->width == 0 ? pCurGroup->lanes.size() * LANE_WIDTH_DEFAULT : pCurGroup->width;

				_makePierBoundingBox(ndsPtCenter, nRoadWidthCal, pierBoundingBoxNds);
				pierBoundingBox.min.fromNdsPoint(NdsPoint_make((int)pierBoundingBoxNds.min.lon, (int)pierBoundingBoxNds.min.lat));
				pierBoundingBox.max.fromNdsPoint(NdsPoint_make((int)pierBoundingBoxNds.max.lon, (int)pierBoundingBoxNds.max.lat));

				//��ײ���
				bool bCollide = _collisionDetection(pGrid, pCurGroup, mp, pierBoundingBox);

				//�����������ײ��������RdsPier��
				if (!bCollide)
				{
					RdsPier* pPier = (RdsPier*)createObject(pTile, EntityType::RDS_PIER);
					convert(mp, pPier->position);
					pPier->angle = (int32)m_vctResampleResult[i].angle;

					//�����г�����ȴ���0�������������ȴ���0��
					//����������Ϊ0ʱ��д��Ŀ��Ϊ��ֵ��ȡ��Сһ�㣨ȥ���������ʵ�����������һ����ڷ���ʵ����������
					if (pCurGroup->width > 0)
						pPier->width = pCurGroup->width;
					else if (pCurGroup->lanes.size() > 2)
						pPier->width = (pCurGroup->lanes.size() - 2) * LANE_WIDTH_DEFAULT;
					else
						pPier->width = LANE_WIDTH_DEFAULT;

					RdsGroup* pRdsGroup = queryGroup(pCurGroup->originId, pTile);
					if (pRdsGroup)
						pRdsGroup->objects.push_back(pPier);		//������������Ŷ�
				}
			}
		}
	}

	void PierCompiler::_initNdsGird(NdsGridId gridId)
	{
		NdsRect ndsRect;
		NdsGridId_getNdsRect(gridId, &ndsRect);
		NdsPoint ndsBase = NdsPoint_make(ndsRect.left, ndsRect.top);

		m_baseHadPoint3D.pos.fromNdsPoint(ndsBase);
		m_baseHadPoint3D.z = 0;

		m_sampleScale.x = METER_PER_LAT_UNIT * 0.001f * coordinate_converter::calcLonLatScale(ndsBase);
		m_sampleScale.y = METER_PER_LAT_UNIT * 0.001f;
		m_sampleScale.z = 0.01f;

		m_sampleScaleInv.set(1.f / m_sampleScale.x, 1.f / m_sampleScale.y, 1.f / m_sampleScale.z);
	}

	void PierCompiler::_pierResampleCallback(Vector3 point, Vector3 dir)
	{
		_PierTemp pier = { 0 };
		pier.pt = PointConverter::Vector3ToMapPoint3D64(point);
		//pier.angle = atan2l(dir.y, dir.x) * 1e6;
		pier.angle = atan2l(dir.x, dir.y) * 1e6;	// x/y,���շ�վ�Ƕȼ��㷽������һ�¡�
		pier.lg = m_pCurLaneGroup;
		m_vctResampleResult.push_back(pier);
	}

	void PierCompiler::_makePierBoundingBox(const NdsPoint& ptCenter, int radius, BoundingBox2d& pierBoundingBox)
	{
		NdsPoint ndsLeft = _changeNdsPointWithHeading(ptCenter, 270, (float)(radius / 100.0));
		NdsPoint ndsRight = _changeNdsPointWithHeading(ptCenter, 90, (float)(radius / 100.0));
		NdsPoint ndsTop = _changeNdsPointWithHeading(ptCenter, 0, (float)(radius / 100.0));
		NdsPoint ndsBottom = _changeNdsPointWithHeading(ptCenter, 180, (float)(radius / 100.0));

		pierBoundingBox.min.lon = ndsLeft.x;
		pierBoundingBox.min.lat = ndsBottom.y;
		pierBoundingBox.max.lon = ndsRight.x;
		pierBoundingBox.max.lat = ndsTop.y;
	}

	size_t PierCompiler::_makeLaneGroupPolygon(HadLaneGroup* pGroup, MapPoint3D64*& polygon)
	{
		assert(pGroup->laneBoundaries.size() >= 2);

		size_t laneBoundaryNum = pGroup->laneBoundaries.size();
		size_t nPolyline1PtNum = pGroup->laneBoundaries[0]->location.vertexes.size();
		size_t nPolyline2PtNum = pGroup->laneBoundaries[laneBoundaryNum - 1]->location.vertexes.size();
		size_t nPtNum = nPolyline1PtNum + nPolyline2PtNum + 1;

		polygon = new MapPoint3D64[nPtNum];
		memcpy(polygon, pGroup->laneBoundaries[0]->location.vertexes.data(), nPolyline1PtNum * sizeof(MapPoint3D64));

		std::vector<MapPoint3D64> Line2Reverse(nPolyline2PtNum);
		std::reverse_copy(pGroup->laneBoundaries[laneBoundaryNum - 1]->location.vertexes.begin(), pGroup->laneBoundaries[laneBoundaryNum - 1]->location.vertexes.end(), Line2Reverse.begin());
		memcpy(polygon + nPolyline1PtNum, Line2Reverse.data(), nPolyline2PtNum * sizeof(MapPoint3D64));

		polygon[nPtNum - 1] = pGroup->laneBoundaries[0]->location.vertexes[0];

		return nPtNum;

		/****************************
		* ��Ϊ����ģ�����´����·��Ĵ�����޷���������������·��߽�������
		* ���󣺵�LGID == 142298754851483137ʱ��pGroup->roadBoundaries.size()Ϊ0��pGroup->laneBoundaries.size()Ϊ6��
		*****************************/
		/*
		assert(pGroup->roadBoundaries.size() == 2);
		//int nPtNum = pGroup->roadBoundaries[0]->location.vertexes.size() + pGroup->roadBoundaries[1]->location.vertexes.size() + 1;
		size_t nPolyline1PtNum = pGroup->roadBoundaries[0]->location.vertexes.size();
		size_t nPolyline2PtNum = pGroup->roadBoundaries[1]->location.vertexes.size();
		size_t nPtNum = nPolyline1PtNum + nPolyline2PtNum + 1;

		polygon = new MapPoint3D64[nPtNum];
		memcpy(polygon, pGroup->roadBoundaries[0]->location.vertexes.data(), nPolyline1PtNum * sizeof(MapPoint3D64));

		std::vector<MapPoint3D64> Line2Reverse(nPolyline2PtNum);
		std::reverse_copy(pGroup->roadBoundaries[1]->location.vertexes.begin(), pGroup->roadBoundaries[1]->location.vertexes.end(), Line2Reverse.begin());
		memcpy(polygon+nPolyline1PtNum, Line2Reverse.data(), nPolyline2PtNum * sizeof(MapPoint3D64));

		polygon[nPtNum - 1] = pGroup->roadBoundaries[0]->location.vertexes[0];

		return nPtNum;
		*/
	}

	size_t PierCompiler::_makeBoundingBoxPolygon(BoundingBox2d boundingBox, MapPoint3D64*& polygon)
	{
		const size_t rectPolygonPtNum = 5;
		polygon = new MapPoint3D64[rectPolygonPtNum];
		polygon[0] = MapPoint3D64_make(boundingBox.min.lon, boundingBox.max.lat, 0);	//����
		polygon[1] = MapPoint3D64_make(boundingBox.max.lon, boundingBox.max.lat, 0);	//����
		polygon[2] = MapPoint3D64_make(boundingBox.max.lon, boundingBox.min.lat, 0);	//����
		polygon[3] = MapPoint3D64_make(boundingBox.min.lon, boundingBox.min.lat, 0);	//����
		polygon[4] = polygon[0];
		return rectPolygonPtNum;
	}

	bool PierCompiler::_hasIntersect(MapPoint3D64* points1, size_t pointCount1, MapPoint3D64* points2, size_t pointCount2)
	{
		MapPoint3D64 basePoint = points1[0];
		ClipperLib::Path path_v1;
		for (int i = 0; i < pointCount1; i++)
		{
			path_v1 << ClipperLib::IntPoint(points1[i].pos.lon - basePoint.pos.lon, points1[i].pos.lat - basePoint.pos.lat, 0);
		}

		ClipperLib::Path path_v2;
		for (int i = 0; i < pointCount2; i++)
		{
			path_v2 << ClipperLib::IntPoint(points2[i].pos.lon - basePoint.pos.lon, points2[i].pos.lat - basePoint.pos.lat, 0);
		}

		ClipperLib::Clipper clipper;
		clipper.AddPath(path_v2, ClipperLib::ptSubject, true);
		clipper.AddPath(path_v1, ClipperLib::ptClip, true);
		ClipperLib::PolyTree polyTree;
		clipper.Execute(ClipperLib::ctIntersection, polyTree, ClipperLib::pftNonZero);
		if (polyTree.Total() > 0)
		{
			return true;
		}
		return false;
	}

	bool PierCompiler::_calTangentialCutWithBoundary(MapPoint3D64 pt, float dir, HadLaneBoundary* laneBoundary, MapPoint3D64& rstPt)
	{
		size_t ptNum = laneBoundary->location.vertexes.size();
		MapPoint3D64* polyline = new MapPoint3D64[ptNum];
		memcpy(polyline, laneBoundary->location.vertexes.data(), ptNum * sizeof(MapPoint3D64));
		bool isSuc = TangentialAlgorithm::calTangentialCut(pt, dvec2(cos(dir), sin(dir)), polyline, (int)ptNum, rstPt);
		delete[] polyline;
		return isSuc;
	}

	bool PierCompiler::_collisionDetection(HadGrid* pGrid, HadLaneGroup* pGroup, MapPoint3D64 ptCenter, BoundingBox2d& pierBoundingBox)
	{
		bool bCollide = false;
		std::vector<HadLaneGroup*> pNearbyLinkGroups = SpatialSeacher::seachNearby2d(pGrid, pGroup);
		for (HadLaneGroup* pNearbyLinkGroup : pNearbyLinkGroups)
		{
			if(pGroup->laneBoundaries.size() == 0 || pNearbyLinkGroup->laneBoundaries.size() == 0)
				continue;

			MapPoint3D64* boxPolygon = NULL, * laneGroupPolygon = NULL;
			size_t boxPolygonPtNum = _makeBoundingBoxPolygon(pierBoundingBox, boxPolygon);	//����rect�պ϶���Ρ�
			size_t LaneGroupPolygonPtNum = _makeLaneGroupPolygon(pNearbyLinkGroup, laneGroupPolygon);	//������������պ϶���Ρ�
			bool isIntersect = _hasIntersect(laneGroupPolygon, LaneGroupPolygonPtNum, boxPolygon, boxPolygonPtNum);

			delete[] boxPolygon;
			delete[] laneGroupPolygon;

			if (isIntersect)
			{
				// ץ·���ж�·��߶����Ŷո߶ȵĹ�ϵ
				MapPoint3D64* polyline = pNearbyLinkGroup->laneBoundaries[0]->location.vertexes.data();
				size_t ptNum = pNearbyLinkGroup->laneBoundaries[0]->location.vertexes.size();
				MapPoint3D64 grappedPt = { 0 };
				size_t si = 0, ei = 0;
				if (GrapPointAlgorithm::grapPoint(ptCenter, polyline, ptNum, grappedPt, si, ei, 500.0))
				{
					// �ų�ͬ��Ŀ����ԣ�Ϊ����ǿ�ݴ�����������һ���������������
					int32 hightDiff = std::abs(ptCenter.z - grappedPt.z);
					if (hightDiff <= 50)	//50����
						continue;

					if (ptCenter.z < grappedPt.z)	//��ǰ�Ŷն�����ƥ����·���
						continue;
					else
					{
						bCollide = true;
						break;
					}
				}
			}
		}

		return bCollide;
		/*************************************************************************************************************
		* ��SpatialSeacher::seachNearby(const BoundingBox3d& input, int32 tolerance )���ڴ������´��벻���á�
		*
		//pierBoundingBox.min = mp.pos;
		//pierBoundingBox.max = mp.pos;
		std::vector<HadLaneGroup*> pNearbyLinkGroups = SpatialSeacher::instance()->seachNearby(pierBoundingBox, 0);
		for (HadLaneGroup* pNearbyLinkGroup : pNearbyLinkGroups)
		{
			int w = 0;
			printInfo("����������LinkGroup->originId = %I64d, ��ǰ= %I64d", pNearbyLinkGroup->originId, pGroup->originId);
			if (pNearbyLinkGroup->originId != pGroup->originId)
			{
				MapPoint3D64* boxPolygon = NULL, *laneGroupPolygon = NULL;
				size_t boxPolygonPtNum = _makeBoundingBoxPolygon(pierBoundingBox, boxPolygon);	//����rect�պ϶���Ρ�
				size_t LaneGroupPolygonPtNum = _makeLaneGroupPolygon(pNearbyLinkGroup, laneGroupPolygon);	//������������պ϶���Ρ�
				bool isIntersect = _hasIntersect(laneGroupPolygon, LaneGroupPolygonPtNum, boxPolygon, boxPolygonPtNum);
				//bool isIntersect2 = _hasIntersect(boxPolygon, boxPolygonPtNum, laneGroupPolygon, LaneGroupPolygonPtNum);

				if(isIntersect)
				{
					printInfo("-->x=%I64d, y=%I64d �� LinkGroup->originId = %I64d �����ཻ", mp.pos.lon, mp.pos.lat, pNearbyLinkGroup->originId);
					///////////////////
					// ץ·�ж�....
					///////////////////

					isValid = false;
					break;
				}

				delete[] boxPolygon;
				delete[] laneGroupPolygon;
			}
			//else
			//{
			//	isValid = true;
			//}
		}
		*************************************************************************************************************/
	}

	bool PierCompiler::_recursiveForwardLaneGroup(HadLaneGroup* pGroup, std::list<HadLaneGroup*>& listCombineLaneGroup)
	{
		auto& tmpPrevious = getLaneGroupSkeletons(pGroup->previous);
		//1��1�������������ǰ����������Ϊ1��ǰ�򳵵���ĳ���Ϊ1��
		if (tmpPrevious.size() == 1 && tmpPrevious[0]->next.size() == 1)
		{
			if (pGroup->owner->getId() != tmpPrevious[0]->owner->getId())
				return false;
			//if (pGroup->owner->getId() != pGroup->previous[0]->next[0]->owner->getId())
			//	return false;

			HadLaneGroup* pCurGroup = (HadLaneGroup*)tmpPrevious[0];
			if (std::find(listCombineLaneGroup.begin(), listCombineLaneGroup.end(), pCurGroup) != listCombineLaneGroup.end())
				return false;

			listCombineLaneGroup.push_front(pCurGroup);
			m_setGroupIdInMesh.insert(pCurGroup->originId);
			return _recursiveForwardLaneGroup(pCurGroup, listCombineLaneGroup);
		}
		else
			return false;
	}

	bool PierCompiler::_recursiveBackwardLaneGroup(HadLaneGroup* pGroup, std::list<HadLaneGroup*>& listCombineLaneGroup)
	{
		auto& tmpNext = getLaneGroupSkeletons(pGroup->next);
		//1��1�������������ǰ������ĳ���Ϊ1�����򳵵�������Ϊ1��
		if (tmpNext.size() == 1 && tmpNext[0]->previous.size() == 1)
		{
			if (pGroup->owner->getId() != tmpNext[0]->owner->getId())
				return false;

			HadLaneGroup* pCurGroup = (HadLaneGroup*)tmpNext[0];
			if (std::find(listCombineLaneGroup.begin(), listCombineLaneGroup.end(), pCurGroup) != listCombineLaneGroup.end())
				return false;

			listCombineLaneGroup.push_back(pCurGroup);
			m_setGroupIdInMesh.insert(pCurGroup->originId);
			return _recursiveBackwardLaneGroup(pCurGroup, listCombineLaneGroup);
		}
		else
			return false;
	}

	float PierCompiler::_combineLaneGroupRresample(std::list<HadLaneGroup*>& listCombineLaneGroup, float interval, float start)
	{
		m_vctResampleResult.clear();

		for (auto pGroup : listCombineLaneGroup)
		{
			size_t laneBoundaryCount = pGroup->laneBoundaries.size();
			if (laneBoundaryCount <= 0)
				continue;

			LineString3d reverseLinestring;

			//ȡ�ó���������·��������ߣ����ǽ���ֵ���Ǽ��������ϵľ��������ߣ���
			//���������ߵĻ��߷����복����ʻ����һ�¡�
			LineString3d* pLine = NULL;
			if (laneBoundaryCount % 2 == 0)
			{
				HadLaneBoundary* pLaneBoundary = pGroup->laneBoundaries[laneBoundaryCount / 2];
				if (directionEqual(pLaneBoundary, pGroup, 3))	//���߷��������ڳ�����ͨ�з����෴��
				{
					LineString3d* curLinestring = &pLaneBoundary->location;
					std::reverse_copy(curLinestring->vertexes.begin(), curLinestring->vertexes.end(), back_inserter(reverseLinestring.vertexes));
					pLine = &reverseLinestring;
				}
				else
				{
					pLine = &pLaneBoundary->location;
				}
			}
			else
				pLine = &pGroup->lanes[laneBoundaryCount / 2]->location;

			//ת�����Ծ���Ϊ��λ��ͼ���ڵѿ�������ϵ�����굥λ���ס�
			cqstd::vector<Vector3> relPoints;
			bool invalid = false;
			for (size_t i = 0; i < pLine->vertexes.size(); i++)
			{
				//�ݴ�pLaneBoundary->location�д��ڸ߳��쳣ֵ��
				//���ڲ�֪���ж���㷢�����쳣����������������
				if (pLine->vertexes[i].z < -15500)	//�й���ͺ���154�ס�
				{
					invalid = true;
					break;
				}
				relPoints.push_back(_relFromMapPoint3D64(pLine->vertexes[i]));
			}
			if (invalid)
				continue;


			//��һ�������������ز���
			//Polyline_resample3D(relPoints.begin(), relPoints.size(), PIER_INTERVAL_METERS, PIER_START_OFFSET_METERS, _pierResampleCallbackWrapper, this);
			m_pCurLaneGroup = pGroup;
			start = Polyline_resample3D(relPoints.begin(), relPoints.size(), interval, start, _pierResampleCallbackWrapper, this);
		}

		return start;
	}

	void PierCompiler::_amendResamplePt()
	{
		//������λ
		for (std::vector<_PierTemp>::iterator it = m_vctResampleResult.begin(); it != m_vctResampleResult.end();)
		{
			MapPoint3D64 ptA = { 0 }, ptB = { 0 };
			float angle = it->angle;
			MapPoint3D64 mp = _mapPoint3D64FromRel(it->pt);		//��ԭ����Ϊԭʼ����
			it->pt = mp;

			bool isSuc = true;
			HadLaneGroup* pGroup = it->lg;

			size_t laneBoundaryCount = pGroup->laneBoundaries.size();
			if (laneBoundaryCount == 0)
				continue;

			size_t si = 0, ei = 0;
			bool roadBoundaryMatched = false;
			if (pGroup->roadBoundaries.size() == 2)
			{
				auto& leftSide = pGroup->roadBoundaries[0]->location;
				auto& rightSide = pGroup->roadBoundaries[1]->location;
				if (GrapPointAlgorithm::grapOrMatchNearestPoint(mp, leftSide.vertexes, ptA, si, ei, 500.0) &&
					GrapPointAlgorithm::grapOrMatchNearestPoint(mp, rightSide.vertexes, ptB, si, ei, 500.0))
					roadBoundaryMatched = true;
			}

			// ���õ�·�߽�,���ó����߽�
			if (!roadBoundaryMatched)
			{
				HadLaneBoundary* leftBoundary = pGroup->laneBoundaries[0];
				HadLaneBoundary* rightBoundary = pGroup->laneBoundaries[laneBoundaryCount - 1];

				if (!GrapPointAlgorithm::grapOrMatchNearestPoint(mp, leftBoundary->location.vertexes, ptA, si, ei, 500.0))
					ptA = _nearestEndPoint(mp, leftBoundary->location.vertexes);

				if (!GrapPointAlgorithm::grapOrMatchNearestPoint(mp, rightBoundary->location.vertexes, ptB, si, ei, 500.0))
					ptB = _nearestEndPoint(mp, rightBoundary->location.vertexes);
			}

			//������λ�����ı�����ϵ���ı���ʱ����ϵΪԭʼ��������ϵ��
			it->pt.pos.lon = (ptA.pos.lon + ptB.pos.lon) / 2;
			it->pt.pos.lat = (ptA.pos.lat + ptB.pos.lat) / 2;
			if (roadBoundaryMatched) {
				it->pt.z = nc_min(ptA.z, ptB.z);
			} else {
				it->pt.z = nc_min(mp.z, nc_min(ptA.z, ptB.z));
			}
			// https://jira.navinfo.com/browse/HQNAVI-3127
			// �Ŷ�����ܶ���������·��,����12cm�߳�
			it->pt.z -= 12;
			it++;
		}
	}

	MapPoint3D64 PierCompiler::_nearestEndPoint(const MapPoint3D64& mp, const std::vector<MapPoint3D64>& vertexes)
	{
		NdsPoint ptCenter = mp.pos.toNdsPoint();
		NdsPoint ptStart = vertexes.front().pos.toNdsPoint();
		NdsPoint ptEnd = vertexes.back().pos.toNdsPoint();
		double toStartDis = GrapPointAlgorithm::geoLengthD(ptCenter, ptStart);
		double toEndDis = GrapPointAlgorithm::geoLengthD(ptCenter, ptEnd);
		return ( toStartDis < toEndDis ? vertexes.front() : vertexes.back() );
	}

}

