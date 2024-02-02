#include "stdafx.h"

#define _USE_MATH_DEFINES
#include <Math.h>
#include "SpeedLimitBoardCompiler.h"
#include "math3d/vector_math.h"
#include "algorithm/grap_point_algorithm.h"

namespace OMDB
{
	double SpeedLimitBoardCompiler::EXIST_TUNNEL_DIFFERENCE = 5000;		//5;    // 牌子附近存在有效隧道判定长度差。单位：米。
	double SpeedLimitBoardCompiler::PAIR_SIGN_HIGH_DIFFERENCE = 0.0;		//0.5;  // 同一杆子上两个牌子的有效判定高度差。单位：米。
	double SpeedLimitBoardCompiler::SAME_POLE_DISTANCE_DIFFERENCE = 2.0;	//1.0;	// 同一杆子上两个牌子的有效判定距离差。单位：米。

	void SpeedLimitBoardCompiler::compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile)
	{
		UNREFERENCED_PARAMETER(nearby);

		//m_mapSlbCenterPt.clear();
		m_vctPoles.clear();

		//编译限速看板
		for (auto obj : pGrid->query(ElementType::HAD_OBJECT_TRAFFIC_SIGN))
		{
			HadTrafficSign* pTrafficSign = (HadTrafficSign*)obj;

			//判断是否为普通路
			if (CompileSetting::instance()->isNotCompileUrbanData)
			{
				if (!isProDataLevel(pTrafficSign->laneGroups))
					continue;
			}

			if (pTrafficSign->signType != HadTrafficSign::SignType::MAX_SPEED_LIMIT_VALUE
				&& pTrafficSign->signType != HadTrafficSign::SignType::MIN_SPEED_LIMIT_VALUE
				&& pTrafficSign->signType != HadTrafficSign::SignType::END_SPEED_LIMIT_VALUE)
				continue;

			LineString3d guardrailLine;
			MapPoint3D64 minGrappedPt{ 0 };
			double minGrappedDis = DBL_MAX;
			RdsGroup* pMinRdsGroup = nullptr;
			bool existGuardail = false;
			HadLaneGroup* pMinLinkGroup = nullptr;

			//遍历标牌对应的车道组
			for (HadLaneGroup* linkGroup : pTrafficSign->laneGroups)
			{
				//如果限速牌与车道组中最大最小限速差距过大（大于2倍）则抛弃
				if (linkGroup->maxSpeedLimit != 0)
				{
					if (pTrafficSign->signType == HadTrafficSign::SignType::MAX_SPEED_LIMIT_VALUE && pTrafficSign->speed * 2 < linkGroup->maxSpeedLimit)
						continue;
				}
				if (linkGroup->minSpeedLimit != 0)
				{
					if (pTrafficSign->signType == HadTrafficSign::SignType::MIN_SPEED_LIMIT_VALUE && pTrafficSign->speed * 2 < linkGroup->minSpeedLimit)
						continue;
				}

				pTrafficSign->centerPt = getCenterPtFromPolygon(pTrafficSign->polygon.vertexes);
				RdsGroup* pCurRdsGroup = queryGroup(linkGroup->originId, pTile);	//得到该车道组上的object
				if (pCurRdsGroup != nullptr) // 车道组在当前网格
				{
					for (auto rdsObj : pCurRdsGroup->objects)	//遍历车道组上的object
					{
						// 向护栏投影
						if (rdsObj->getEntityType() == EntityType::RDS_GUARDRAIL)
						{
							RdsGuardrail* pGuardrail = (RdsGuardrail*)rdsObj;
							size_t size = pGuardrail->location.vertexes.size();
							if (size < 2)
								continue;

							existGuardail = true;
							LineString3d guardrailLinestring;
							convert(pGuardrail->location, guardrailLinestring);

							//投影，取得最近的投影信息。
							size_t si = 0, ei = 0;
							MapPoint3D64 grappedPt{ 0 };
							bool bGrap = GrapPointAlgorithm::grapOrMatchNearestEndPoint(pTrafficSign->centerPt, guardrailLinestring.vertexes, grappedPt, si, ei, 10.0);
							double grappedDis = pTrafficSign->centerPt.pos.distance(grappedPt.pos);	//投影（平面）距离
							if (grappedDis < minGrappedDis) {
								guardrailLine = guardrailLinestring;
								pMinRdsGroup = pCurRdsGroup;
								minGrappedPt = grappedPt;
								minGrappedDis = grappedDis;
								pMinLinkGroup = linkGroup;
							}
						}
					} // end for (auto rdsObj : pRdsGroup->objects)	//遍历车道组上的object
				} // for (HadLaneGroup* linkGroup : pTrafficSign->laneGroups)
				else
				{	// 车道组数据在其他网格
					MapPoint3D64 grappedPt{ 0 };
					double grappedDis = DBL_MAX;
					LineString3d grappedLinestring;
					std::vector<HadRoadBoundary*> roadBoundaries = linkGroup->roadBoundaries;
					grapPoint(pTrafficSign->centerPt, (std::vector<HadSkeleton*>&)roadBoundaries, grappedPt, grappedLinestring, grappedDis);
					if (grappedDis < minGrappedDis) {
						pMinRdsGroup = queryGroup(linkGroup->originId, pTile);
						minGrappedPt = grappedPt;
						minGrappedDis = grappedDis;
						pMinLinkGroup = linkGroup;
						existGuardail = true;
					}
				}
			}
			if (existGuardail)
			{
				pTrafficSign->grapPt = minGrappedPt;
				if (!existNearbyTunnel(pTrafficSign, pMinLinkGroup))
				{
					createRdsSLB(pTrafficSign, pMinLinkGroup, guardrailLine, pMinRdsGroup);
				}
			}
		}

		//这里RdsSpeedLimitBoard已经被插入了，屏蔽无效，如屏蔽使用HadTrafficSign吧。
		for (auto& pole : m_vctPoles)
		{
			if (pole.vctTS.size() > 2)
				continue;
			
			RdsSpeedLimitBoard* pSlb = (RdsSpeedLimitBoard*)createObject(pTile, EntityType::RDS_SPEEDLIMITBOARD);
			pSlb->angle = pole.rdsSLB.angle;
			pSlb->position = pole.rdsSLB.position;
			pSlb->type = pole.rdsSLB.type;
			pSlb->above = pole.rdsSLB.above;
			pSlb->blow = pole.rdsSLB.blow;
			if (pole.rdsGroup != nullptr)
			{
				pole.rdsGroup->objects.push_back(pSlb);
			}
		}
	}

	double SpeedLimitBoardCompiler::calSLBDirection(HadTrafficSign* pTrafficSign, LineString3d& guardrailLine, HadLaneGroup* linkGroup)
	{
		// 一个车道组有可能属于两条link,选择最近的那条
		auto getNearestLink = [](HadLaneGroup* laneGroup, MapPoint3D64& pt)->HadLink* {
			if (laneGroup->relLinks.size() == 1) {
				return laneGroup->relLinks.begin()->first;
			}

			size_t si, ei;
			HadLink* hadLink = nullptr;
			double minDistance = DBL_MAX;
			for (auto& linkPair : laneGroup->relLinks) {
				MapPoint3D64 grappedPt = {};
				auto& linkVertexes = linkPair.first->location.vertexes;
				GrapPointAlgorithm::grapOrMatchNearestPoint(pt, linkVertexes, grappedPt, si, ei);
				double grappedDistance = grappedPt.pos.distance(pt.pos);
				if (grappedDistance < minDistance) {
					hadLink = linkPair.first;
					minDistance = grappedDistance;
				}
			}
			return hadLink;
		};

		MapPoint3D64 grappedPt{ 0 };
		MapPoint3D64 minSiPt{ 0 }, minEiPt{ 0 };

		std::vector<MapPoint3D64> polyline;
		if (guardrailAverage(guardrailLine) > 0.999)	// arccos(0.999) = 2.5 degree
		{
			// 优先使用护栏计算方向
			size_t size = guardrailLine.vertexes.size();
			polyline.resize(size);
			copy(guardrailLine.vertexes.begin(), guardrailLine.vertexes.end(), polyline.begin());
		}
		else
		{
			// 当车道组只有一个link时，使用link方向；当存在多个link时，使用最近的link方向。
			HadLink* pLink = getNearestLink(linkGroup, pTrafficSign->grapPt);
			size_t size = pLink->location.vertexes.size();
			polyline.resize(size);

			//与通行方向一致的坐标序列
			if (pLink->direct == 3)	//逆向
				reverse_copy(pLink->location.vertexes.begin(), pLink->location.vertexes.end(), polyline.begin());
			else
				copy(pLink->location.vertexes.begin(), pLink->location.vertexes.end(), polyline.begin());
		}

		size_t si = 0, ei = 0;
		if (GrapPointAlgorithm::grapOrMatchNearestEndPoint(pTrafficSign->grapPt, polyline, grappedPt, si, ei, 10.0))
		{
			minSiPt = polyline[si];
			minEiPt = polyline[ei];
		}
		else    //抓路失败，匹配端点时。
		{
			if (grappedPt == polyline[0])
			{
				minSiPt = polyline[0];
				minEiPt = polyline[1];
			}
			else
			{
				size_t size = polyline.size();
				minSiPt = polyline[size - 2];
				minEiPt = polyline[size - 1];
			}
		}

		double angle = atan2l(minEiPt.pos.lat - minSiPt.pos.lat, minEiPt.pos.lon - minSiPt.pos.lon);
		angle = angle < 0 ? angle + 2 * (M_PI) : angle;	//  值域转为(0,2PI]
		angle *= 1e5;

		return angle;
	}

	double SpeedLimitBoardCompiler::guardrailAverage(const LineString3d& originLine)
	{
		if (originLine.vertexes.size() > 1)
		{
			point_t pStart = POINT_T(originLine.vertexes.front());
			point_t pEnd = POINT_T(originLine.vertexes.back());
			vector_t v = V3_N(S3_V3(pStart, pEnd));
			double vSum = 0.0;
			double totalLength = 0.0;
			for (size_t i = 1; i < originLine.vertexes.size(); ++i)
			{
				point_t pa = POINT_T(originLine.vertexes.at(i - 1));
				point_t pb = POINT_T(originLine.vertexes.at(i));
				vector_t tmpV = V3_N(S3_V3(pa, pb));
				vSum += bg::dot_product(v, tmpV);
				totalLength += bg::distance(pa, pb);
			}
			double vAverage = vSum / (originLine.vertexes.size() - 1);
			return totalLength > 5000 ? vAverage : DBL_MIN;
		}
		return DBL_MIN;
	}

	void SpeedLimitBoardCompiler::createRdsSLB(HadTrafficSign* pTrafficSign, 
		HadLaneGroup* linkGroup, LineString3d& guardrailLine, RdsGroup* pRdsGroup)
	{
		int32 speed = pTrafficSign->speed;
		double angle = calSLBDirection(pTrafficSign, guardrailLine, linkGroup);

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

		auto setAboveSlb = [speed, angle, type](RdsSpeedLimitBoard* pRdsSlb) {
			pRdsSlb->blow = pRdsSlb->above;
			pRdsSlb->setSpeed(pRdsSlb->above, speed);
			pRdsSlb->setAngle(pRdsSlb->above, angle);
			pRdsSlb->setSpeedType(pRdsSlb->above, (int32)type);
		};

		Pole* pole = nullptr;
		if (existPole(pTrafficSign->grapPt, pole))	//杆子上已经存在牌子了。
		{
			pole->vctTS.push_back(pTrafficSign);

			//计算中心点【此步忽略，待1杆3牌有需求后，再酌情添加】
			//...

			RdsSpeedLimitBoard* pExistRdsSlb = &pole->rdsSLB;
			int diffHigh = pole->boardsCenterPt.z - pTrafficSign->centerPt.z;
			setPairBoard(pTrafficSign, pExistRdsSlb, diffHigh, speed, angle, type);
		}
		else
		{
			//创建杆和牌。
			Pole pole;
			pole.rdsGroup = pRdsGroup;
			setAboveSlb(&pole.rdsSLB);
			convert(pTrafficSign->grapPt, pole.rdsSLB.position);
			pole.boardsCenterPt = pTrafficSign->centerPt;
			pole.vctTS.push_back(pTrafficSign);
			m_vctPoles.push_back(pole);
		}
	}

	void SpeedLimitBoardCompiler::setPairBoard(HadTrafficSign* pTrafficSign, RdsSpeedLimitBoard* pExistRdsSlb, int diffHigh, int32 speed, double angle, RdsSpeedLimitBoard::SpeedLimitType type)
	{
		//同1杆子上已经2个牌子的情况。
		if (pExistRdsSlb->above != 0 && pExistRdsSlb->blow != 0)
		{
			//printWarning("同一根杆子上已经存在两个限速牌了！ msid=%d, orgid=%I64d", pTile->meshId, pTrafficSign->originId);
			//printWarning("上牌type=%d，speed=%d, angle=%d", pExistRdsSlb->getType(pExistRdsSlb->above), pExistRdsSlb->getSpeed(pExistRdsSlb->above), pExistRdsSlb->getAngle(pExistRdsSlb->above));
			//printWarning("下牌type=%d，speed=%d, angle=%d", pExistRdsSlb->getType(pExistRdsSlb->blow), pExistRdsSlb->getSpeed(pExistRdsSlb->blow), pExistRdsSlb->getAngle(pExistRdsSlb->blow));
			//printWarning("新牌type=%d，speed=%s, grap.x=%I64d, grap.y=%I64d", pTrafficSign->signType, pTrafficSign->content, grappedPt.pos.lon, grappedPt.pos.lat);
			//printWarning("\n");
			return;
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

		auto replaceSlb = [&](RdsSpeedLimitBoard* pRdsSlb, uint32& slb)
		{
			pRdsSlb->setSpeed(slb, speed);
			pRdsSlb->setAngle(slb, angle);
			pRdsSlb->setSpeedType(slb, (int32)type);
		};

		auto setSlbOnPole = [&](RdsSpeedLimitBoard::SpeedLimitType type, HadTrafficSign* pTrafficSign, RdsSpeedLimitBoard* pRdsSlb)
		{
			if (pRdsSlb->getType(pRdsSlb->above) == type)
			{
				if (pTrafficSign->speed > pRdsSlb->getSpeed(pRdsSlb->above))
					replaceSlb(pRdsSlb, pRdsSlb->above);
			}
			else if (pRdsSlb->getType(pRdsSlb->blow) == type)
			{
				if (pTrafficSign->speed > pRdsSlb->getSpeed(pRdsSlb->blow))
					replaceSlb(pRdsSlb, pRdsSlb->blow);
			}
			else
			{
				if (diffHigh > PAIR_SIGN_HIGH_DIFFERENCE)
					setBelowSlb(pRdsSlb);
				else if (diffHigh < -PAIR_SIGN_HIGH_DIFFERENCE)
					setAboveSlb(pRdsSlb);
			}
		};

		if (pTrafficSign->signType == HadTrafficSign::SignType::MAX_SPEED_LIMIT_VALUE)
			setSlbOnPole(RdsSpeedLimitBoard::SpeedLimitType::Maximum, pTrafficSign, pExistRdsSlb);
		else if (pTrafficSign->signType == HadTrafficSign::SignType::MIN_SPEED_LIMIT_VALUE)
			setSlbOnPole(RdsSpeedLimitBoard::SpeedLimitType::Minimum, pTrafficSign, pExistRdsSlb);
		else if (pTrafficSign->signType == HadTrafficSign::SignType::END_SPEED_LIMIT_VALUE)
			setSlbOnPole(RdsSpeedLimitBoard::SpeedLimitType::Release, pTrafficSign, pExistRdsSlb);
	}

	bool SpeedLimitBoardCompiler::existPole(const MapPoint3D64& grappedPt, Pole*& pExistPole)
	{
		for (auto& pole : m_vctPoles) 
		{
			MapPoint3D64 polePt;
			convert(pole.rdsSLB.position, polePt);
			double distance = GrapPointAlgorithm::geoLengthD(polePt.pos.toNdsPoint(), grappedPt.pos.toNdsPoint());
			if (distance < SAME_POLE_DISTANCE_DIFFERENCE)
			{
				pExistPole = &pole;
				return true;
			}
		}

		return false;
	}

	bool SpeedLimitBoardCompiler::existNearbyTunnel(HadTrafficSign* pTrafficSign, HadLaneGroup* pLinkGroup)
	{
		if (existTunnel(pTrafficSign, pLinkGroup))
			return true;

		int connectNum = 2;
		int connectDistance = 100000; // ≈100m
		std::deque<HadRoadBoundary*> connectedBoundarys;
		std::deque<HadLaneGroup*> connectedLaneGroups;
		HadRoadBoundary* boundary = getGrappedBoundary(pLinkGroup, pTrafficSign->grapPt);
		if (boundary != nullptr)
		{
			connectedBoundarys.push_front(boundary);
			connectedLaneGroups.push_front(pLinkGroup);
			connectForwardRoadBoundary(pLinkGroup, boundary, connectedBoundarys, connectedLaneGroups, connectNum);
			connectBackwardRoadBoundary(pLinkGroup, boundary, connectedBoundarys, connectedLaneGroups, connectNum);

			auto begin = connectedLaneGroups.begin();
			auto end = connectedLaneGroups.end();
			auto laneGroupIdx = std::find(begin, end, pLinkGroup) - begin;

			// 往前检查是否有隧道
			for (int idx = laneGroupIdx - 1; idx >= 0; idx--)
			{
				auto connectedLaneGroup = connectedLaneGroups[idx];
				if (existTunnel(pTrafficSign, connectedLaneGroup))
					return true;
			}
			// 往后检查是否有隧道
			for (int idx = laneGroupIdx + 1; idx < connectedLaneGroups.size(); idx++)
			{
				auto connectedLaneGroup = connectedLaneGroups[idx];
				if (existTunnel(pTrafficSign, connectedLaneGroup))
					return true;
			}
		}
		return false;
	}

	bool SpeedLimitBoardCompiler::existTunnel(HadTrafficSign* pTrafficSign, HadLaneGroup* pLinkGroup)
	{
		for (auto& relLink : pLinkGroup->relLinks) {
			for (auto& wayType : relLink.first->wayTypes) {
				if (wayType == LINK_IS_IN_TUNNEL) {
					return true;
				}

				if (wayType == DB_HAD_APPLY_PA_REFERENCE) {
					std::vector<std::pair<double, double>> intervals;
					std::vector<MultiPoint3d> intervalMultiPoints;
					for (auto paValue : relLink.first->attributes) {
						if (paValue->name == LINK_WAY_TYPE_PA_NAME && paValue->value == LINK_IS_IN_TUNNEL) {
							intervals.emplace_back(std::make_pair(paValue->start, paValue->end));
							intervalMultiPoints.push_back(paValue->points);
						}
					}
					if (!intervals.empty())
					{
						// 合并重叠的区间
						std::vector<std::pair<double, double>> merged;
						std::vector<MultiPoint3d> mergedMultiPoints;
						std::pair<double, double> curRange = intervals.front();
						MultiPoint3d curMultiPoint = intervalMultiPoints.front();
						for (int i = 1; i < intervals.size(); i++)
						{
							if (curRange.second + 0.01 >= intervals[i].first)
							{
								curRange.second = max(curRange.second + 0.01, intervals[i].second);
								for (auto& postion : intervalMultiPoints[i].postions) {
									curMultiPoint.postions.push_back(postion);
								}
							}
							else
							{
								merged.push_back(curRange);
								mergedMultiPoints.push_back(curMultiPoint);
								curRange = intervals[i];
								curMultiPoint = intervalMultiPoints[i];
							}
						}
						merged.push_back(curRange);
						mergedMultiPoints.push_back(curMultiPoint);
						for (auto idx = 0; idx < merged.size(); idx++)
						{
							auto& range = merged[idx];
							auto& multiPoint = mergedMultiPoints[idx];
							if (range.first <= relLink.second.start + 0.01 && relLink.second.end <= range.second + 0.01) {
								for (auto& pt : multiPoint.postions) {
									double distance = pt.pos.distance(pTrafficSign->grapPt.pos);	//投影（平面）距离
									if (distance <= EXIST_TUNNEL_DIFFERENCE) {
										return true;
									}
								}
							}
						}

					}

				}


			}
		}
		return false;
	}

HadRoadBoundary* SpeedLimitBoardCompiler::getGrappedBoundary(HadLaneGroup* pGroup, MapPoint3D64& pt)
{
	double minGrappedDis = DBL_MAX;
	HadRoadBoundary* grappedBoundary = nullptr;
	for (auto boundary : pGroup->roadBoundaries) 
	{
		//投影，取得最近的投影信息。
		size_t si = 0, ei = 0;
		MapPoint3D64 grappedPt{ 0 };
		GrapPointAlgorithm::grapOrMatchNearestPoint(pt, boundary->location.vertexes, grappedPt, si, ei);
		double grappedDis = pt.pos.distance(grappedPt.pos);	//投影（平面）距离
		if (grappedDis < minGrappedDis) {
			grappedBoundary = boundary;
			minGrappedDis = grappedDis;
		}
	}
	return grappedBoundary;
}

MapPoint3D64 SpeedLimitBoardCompiler::getCenterPtFromPolygon(std::vector<MapPoint3D64> vertexes)
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


	bool SpeedLimitBoardCompiler::pointInRoad(MapPoint3D64 centerPt, HadLaneGroup* pGroup)
	{
		MapPoint3D64* pRoadPolygon = NULL;
		size_t nRoadPolygonPtNum = makeLaneGroupPolygon(pGroup, pRoadPolygon);	//创建附近的车道组面闭合多边形。
		if (nRoadPolygonPtNum == 0)
			return false;

		ClipperLib::Path pathRoad;
		for (int i = 0; i < nRoadPolygonPtNum; i++)
			pathRoad << ClipperLib::IntPoint(pRoadPolygon[i].pos.lon, pRoadPolygon[i].pos.lat, 0);

		bool bIn = ClipperLib::PointInPolygon(ClipperLib::IntPoint(centerPt.pos.lon, centerPt.pos.lat,0), pathRoad);

		if(pRoadPolygon)
			delete[] pRoadPolygon;

		return bIn;
	}

	void SpeedLimitBoardCompiler::grapPoint(MapPoint3D64& vertex, std::vector<HadSkeleton*>& boundaries, MapPoint3D64& grappedPt, LineString3d& grappedLinestring, double& grappedDis)
	{
		if (boundaries.size() != 2) {
			return;
		}

		for (HadSkeleton* boundary : boundaries)
		{
			size_t si = 0, ei = 0;
			MapPoint3D64 minGrappedPt{ {0,0},0 };
			std::vector<MapPoint3D64> boundaryVertexes = boundary->location.vertexes;
			GrapPointAlgorithm::grapOrMatchNearestPoint(vertex, boundaryVertexes, minGrappedPt, si, ei);
			double distance = vertex.pos.distance(minGrappedPt.pos);
			if (distance < grappedDis) {
				grappedLinestring = boundary->location;
				grappedPt = minGrappedPt;
				grappedDis = distance;
			}
		}
	}

}