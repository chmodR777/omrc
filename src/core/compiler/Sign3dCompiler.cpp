#include "stdafx.h"

#include "Sign3dCompiler.h"
#include "math3d/vector_math.h"
#include "algorithm/grap_point_algorithm.h"
namespace OMDB
{
	//double Sign3dCompiler::SIGN_3D_DISTANCE_TOLERANCE = 2000;
	double Sign3dCompiler::SAME_POLE_DISTANCE_TOLERANCE = 1000;

	void Sign3dCompiler::compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile)
	{
		UNREFERENCED_PARAMETER(nearby);

		m_mapSlbCenterPt.clear();

		//编译限速看板
		for (auto obj : pGrid->query(ElementType::HAD_OBJECT_TRAFFIC_SIGN))
		{
			HadTrafficSign* pTrafficSign = (HadTrafficSign*)obj;
			if (pTrafficSign->signType != HadTrafficSign::SignType::MAX_SPEED_LIMIT_VALUE
				&& pTrafficSign->signType != HadTrafficSign::SignType::MIN_SPEED_LIMIT_VALUE
				&& pTrafficSign->signType != HadTrafficSign::SignType::END_SPEED_LIMIT_VALUE)
				continue;

			RdsGroup* pRdsGroup = nullptr;
			double grappedDis = DBL_MAX;
			MapPoint3D64 grappedPt = { {0, 0}, 0 };
			MapPoint3D64 siPt = grappedPt, eiPt = grappedPt;
			MapPoint3D64 ptCenter = { {0, 0}, 0 };
			for (HadLaneGroup* linkGroup : pTrafficSign->laneGroups) {
				std::vector<HadRoadBoundary*> roadBoundaries = linkGroup->roadBoundaries;
				ptCenter = getCenterPtFromPolygon(pTrafficSign->polygon.vertexes);
				MapPoint3D64 minGrappedPt{ 0 };
				MapPoint3D64 minSiPt{ 0 }, minEiPt{ 0 };
				double minGrappedDis = DBL_MAX;
				grapPoint(ptCenter, (std::vector<HadSkeleton*>&)roadBoundaries, minGrappedPt, minSiPt, minEiPt, minGrappedDis);
				if (minGrappedDis < grappedDis) {
					pRdsGroup = queryGroup(linkGroup->originId, pTile);
					grappedPt = minGrappedPt;
					grappedDis = minGrappedDis;
					siPt = minSiPt;
					eiPt = minEiPt;
				}
			}

			// TODO
			if (pRdsGroup != nullptr) {
				createRdsSLB(pTile, pTrafficSign, ptCenter, siPt, eiPt, grappedPt, pRdsGroup);
			}
		}
	}

	void Sign3dCompiler::createRdsSLB(RdsTile* pTile, HadTrafficSign* pTrafficSign, MapPoint3D64 centerPt, const MapPoint3D64& siPt, const MapPoint3D64& eiPt, const MapPoint3D64& grappedPt, RdsGroup* pRdsGroup)
	{
		//得到限速牌各属性值。
		//int32 speed = std::atoi(pTrafficSign->content.c_str());
		int32 speed = pTrafficSign->speed; 
		int32 angle = atan2l(eiPt.pos.lat - siPt.pos.lat, eiPt.pos.lon - siPt.pos.lon) * 1e5;	//注意，这里的角度单位不是默认的1e6，而是1e5
		RdsSpeedLimitBoard::SpeedLimitType type;
		switch (pTrafficSign->signType)
		{
		case HadTrafficSign::SignType::MAX_SPEED_LIMIT_VALUE:
			type = RdsSpeedLimitBoard::SpeedLimitType::Maximum;
			break;
		case HadTrafficSign::SignType::MIN_SPEED_LIMIT_VALUE:
			type = RdsSpeedLimitBoard::SpeedLimitType::Minimum;
			break;
		case HadTrafficSign::SignType::END_SPEED_LIMIT_VALUE:
			type = RdsSpeedLimitBoard::SpeedLimitType::Release;
			break;
		}

		auto setBelowSlb = [speed, angle, type](RdsSpeedLimitBoard* pRdsSlb) {
			pRdsSlb->setSpeed(pRdsSlb->blow, speed);
			pRdsSlb->setAngle(pRdsSlb->blow, angle);
			pRdsSlb->setSpeedType(pRdsSlb->blow, (int32)type);
		};

		auto setAboveSlb = [speed, angle, type](RdsSpeedLimitBoard* pRdsSlb) {
			pRdsSlb->blow = pRdsSlb->above;
			pRdsSlb->setSpeed(pRdsSlb->above, speed);
			pRdsSlb->setAngle(pRdsSlb->above, angle);
			pRdsSlb->setSpeedType(pRdsSlb->above, (int32)type);
		};

		RdsSpeedLimitBoard* pExistRdsSlb = nullptr;
		if (existPole(pRdsGroup, grappedPt, pExistRdsSlb))	//杆子上已经存在牌子了。
		{	
			//同1杆子上已经2个牌子的情况。
			if (pExistRdsSlb->above != 0 && pExistRdsSlb->blow != 0)
			{
				printWarning("同一根杆子上已经存在两个限速牌了！ msid=%d, orgid=%I64d", pTile->meshId, pTrafficSign->originId);
				return;
			}

			auto iter = m_mapSlbCenterPt.find(pExistRdsSlb->id);
			if (iter == m_mapSlbCenterPt.end())
			{
				printError("异常：RDS限速牌中心点不存在！ msid=%d, orgid=%I64d", pTile->meshId, pExistRdsSlb->id);
				return;
			}
			MapPoint3D64 ptExistRdsSlb = iter->second;
			int diffHigh = ptExistRdsSlb.z - centerPt.z;

			if(diffHigh > PAIR_SIGN_HIGH_DIFFERENCE)
				setBelowSlb(pExistRdsSlb);
			else if(diffHigh < -PAIR_SIGN_HIGH_DIFFERENCE)
				setAboveSlb(pExistRdsSlb);
			else
			{
				//通过类型判定合并后的上下层
				switch (pTrafficSign->signType)
				{
				case HadTrafficSign::SignType::MAX_SPEED_LIMIT_VALUE:
					if (pExistRdsSlb->getType(pExistRdsSlb->above) == RdsSpeedLimitBoard::SpeedLimitType::Minimum)	//最高&最低
						setAboveSlb(pExistRdsSlb);
					if (pExistRdsSlb->getType(pExistRdsSlb->above) == RdsSpeedLimitBoard::SpeedLimitType::Release)	//最高&解除
						setAboveSlb(pExistRdsSlb);
					break;
				case HadTrafficSign::SignType::MIN_SPEED_LIMIT_VALUE:
					if (pExistRdsSlb->getType(pExistRdsSlb->above) == RdsSpeedLimitBoard::SpeedLimitType::Maximum)	//最高&最低
						setBelowSlb(pExistRdsSlb);
					if (pExistRdsSlb->getType(pExistRdsSlb->above) == RdsSpeedLimitBoard::SpeedLimitType::Release)	//最低&解除
						setAboveSlb(pExistRdsSlb);
					break;
				case HadTrafficSign::SignType::END_SPEED_LIMIT_VALUE:
					if (pExistRdsSlb->getType(pExistRdsSlb->above) == RdsSpeedLimitBoard::SpeedLimitType::Maximum)	//最高&解除
						setBelowSlb(pExistRdsSlb);
					if (pExistRdsSlb->getType(pExistRdsSlb->above) == RdsSpeedLimitBoard::SpeedLimitType::Minimum)	//最低&解除
						setBelowSlb(pExistRdsSlb);
					break;
				}
			}
		}
		else
		{
			//创建RdsSpeedLimitBoard，加入RDS数据容器中。
			RdsSpeedLimitBoard* pSlb = (RdsSpeedLimitBoard*)createObject(pTile, EntityType::RDS_SPEEDLIMITBOARD);
			setAboveSlb(pSlb);
			convert(grappedPt, pSlb->position);
			if (pRdsGroup)
				pRdsGroup->objects.push_back(pSlb);
			m_mapSlbCenterPt.insert(std::pair<uint64, MapPoint3D64>(pSlb->id, centerPt));
		}
	}

	void Sign3dCompiler::grapPoint(MapPoint3D64& vertex, std::vector<HadSkeleton*>& boundaries, 
		MapPoint3D64& grappedPt, MapPoint3D64& siPt, MapPoint3D64& eiPt, double& grappedDis)
	{
		if (boundaries.size() != 2) {
			return;
		}

		for (HadSkeleton* boundary : boundaries) 
		{
			size_t si=0, ei=0;
			MapPoint3D64 minGrappedPt{ {0,0},0 };
			std::vector<MapPoint3D64> boundaryVertexes = boundary->location.vertexes;
			GrapPointAlgorithm::grapOrMatchNearestEndPoint(vertex, boundaryVertexes, minGrappedPt, si, ei, 10.0);
			double distance = vertex.pos.distance(minGrappedPt.pos);
			if (distance < grappedDis) {
				grappedPt = minGrappedPt;
				grappedDis = distance;
				siPt = boundaryVertexes[si];
				eiPt = boundaryVertexes[ei];
			}
		}
	}

	bool Sign3dCompiler::existPole(RdsGroup* const currRdsGroup, const MapPoint3D64& grappedPt, RdsSpeedLimitBoard*& pExistSlb)
	{
		auto collectSlbs = [](RdsGroup* const pRdsGroup, std::vector<RdsSpeedLimitBoard*>& slbs) {
			for (auto pRdsObject : pRdsGroup->objects) {
				if (pRdsObject->getEntityType() == RDS::EntityType::RDS_SPEEDLIMITBOARD) {
					RdsSpeedLimitBoard* pSlb = (RdsSpeedLimitBoard*)pRdsObject;
					if (std::find(slbs.begin(), slbs.end(), pSlb) == slbs.end()) {
						slbs.push_back(pSlb);
					}
				}
			}
		};

		std::vector<RdsSpeedLimitBoard*> slbs;
		collectSlbs(currRdsGroup, slbs);

		for (RdsSpeedLimitBoard* pSlb : slbs) {
			MapPoint3D64 pSign3dPt;
			convert(pSlb->position, pSign3dPt);
			double distance = pSign3dPt.pos.distance(grappedPt.pos);
			if (distance < SAME_POLE_DISTANCE_TOLERANCE)
			{
				pExistSlb = pSlb;
				return true;
			}
		}

		return false;
	}

	MapPoint3D64 Sign3dCompiler::getCenterPtFromPolygon(std::vector<MapPoint3D64> vertexes)
	{
		MapPoint3D64 ptCenter = { {0,0},0 };

		//如果是闭合图形则size-1。
		size_t size = vertexes.front() == vertexes.back() ? vertexes.size() - 1 : vertexes.size();

		int64 nTotalX = 0, nTotalY = 0, nTotalZ = 0;
		for (size_t i = 0; i < size; i++)
		{
			nTotalX += vertexes[i].pos.lon;
			nTotalY += vertexes[i].pos.lat;
			nTotalZ += vertexes[i].z;
		}
		ptCenter.pos.lon = nTotalX / size;
		ptCenter.pos.lat = nTotalY / size;
		ptCenter.z = nTotalZ / size;
		return ptCenter;
	}

	/*
	// 车道组可能是跨网格的,目前的方式是可以拿到RdsGroup的
	// 后续如果改成九宫格的方式处理数据想要拿到可能要跨网格车道组得存双份,其他object可能也有这个问题
	bool Sign3dCompiler::containsDuplicateSign3d(HadGrid* const grid, RdsTile* const pTile, RdsGroup* const currRdsGroup, MapPoint3D64& grappedPt)
	{
		auto collectSign3ds = [](RdsGroup* const pRdsGroup, std::vector<RdsSign3d*>& sign3ds) {
			for (auto pRdsObject : pRdsGroup->objects) {
				if (pRdsObject->getEntityType() == RDS::EntityType::RDS_SIGN3D) {
					RdsSign3d* pSign3d = (RdsSign3d*)pRdsObject;
					if (std::find(sign3ds.begin(), sign3ds.end(), pSign3d) == sign3ds.end()) {
						sign3ds.push_back(pSign3d);
					}
				}
			}
		};

		std::vector<RdsSign3d*> sign3ds;
		collectSign3ds(currRdsGroup, sign3ds);
		HadLaneGroup* currLaneGroup = (HadLaneGroup*)grid->query(currRdsGroup->originId, ElementType::HAD_LANE_GROUP);
		for (auto pLaneGroup : currLaneGroup->previous) {
			HadLaneGroup* previousLaneGroup = (HadLaneGroup*)pLaneGroup;
			RdsGroup* previousRdsGroup = queryGroup(previousLaneGroup->originId, pTile);
			if (previousRdsGroup != nullptr) {
				collectSign3ds(previousRdsGroup, sign3ds);
			}
		}

		for (auto pLaneGroup : currLaneGroup->next) {
			HadLaneGroup* nextLaneGroup = (HadLaneGroup*)pLaneGroup;
			RdsGroup* nextRdsGroup = queryGroup(nextLaneGroup->originId, pTile);
			if (nextRdsGroup != nullptr) {
				collectSign3ds(nextRdsGroup, sign3ds);
			}
		}

		for (RdsSign3d* pSign3d : sign3ds) {
			MapPoint3D64 pSign3dPt;
			convert(pSign3d->position, pSign3dPt);
			double distance = pSign3dPt.pos.distance(grappedPt.pos);
			if (distance < SIGN_3D_DISTANCE_TOLERANCE && fabs(pSign3dPt.z - grappedPt.z) < 500) {
				return true;
			}
		}

		return false;
	}
	*/

}