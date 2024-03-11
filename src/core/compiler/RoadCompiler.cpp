#include "stdafx.h"

#include "RoadCompiler.h"
#include "math3d/vector_math.h"
#include "grap_point_algorithm.h"
#include "algorithm/linear_interpolation_triangle_surface.h"
#include <algorithm>
using namespace RDS;
namespace OMDB
{
	void RoadCompiler::compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile)
	{
		UNREFERENCED_PARAMETER(nearby);
		auto elements = pGrid->query(ElementType::HAD_LANE_GROUP);

		for each (auto & e in elements)
		{
			int64 disTol = 500;
			HadLaneGroup* pGroup = (HadLaneGroup*)e;
			if (pGroup->next.size() == 1 &&
				pGroup->next.front()->previous.size() == 1 &&
				pGroup->previous.size() == 1 &&
				pGroup->previous.front()->next.size() == 1)
			{
				HadLaneGroup* tmpNextGroup = (HadLaneGroup*)pGroup->next.front();
				HadLaneGroup* tmpPrevGroup = (HadLaneGroup*)pGroup->previous.front();
				if (pGroup->laneBoundaries.size() - tmpNextGroup->laneBoundaries.size() == 1 &&
					tmpNextGroup->laneBoundaries.size() - tmpPrevGroup->laneBoundaries.size() == 0)
				{
					if (pGroup->inIntersection == 0 &&
						tmpNextGroup->inIntersection == 0 &&
						tmpPrevGroup->inIntersection == 0)
					{
						LineString3d leftSide, rightSide, nextLeftSide, nextRightSide, prevLeftSide, prevRightSide;
						getLaneGroupBoundary(pGroup, leftSide, rightSide, false);
						getLaneGroupBoundary(tmpNextGroup, nextLeftSide, nextRightSide, false);
						getLaneGroupBoundary(tmpPrevGroup, prevLeftSide, prevRightSide, false);
						auto frontLeftDis = leftSide.vertexes.back().pos.distance(nextLeftSide.vertexes.front().pos);
						auto frontRightDis = rightSide.vertexes.back().pos.distance(nextRightSide.vertexes.front().pos);
						auto backLeftDis = prevLeftSide.vertexes.back().pos.distance(leftSide.vertexes.front().pos);
						auto backRightDis = prevRightSide.vertexes.back().pos.distance(rightSide.vertexes.front().pos);
						if ((frontLeftDis > disTol && frontRightDis < disTol) && (backLeftDis > disTol && backRightDis < disTol))
						{
							compilerData.hullGroups.push_back(pGroup);
							compilerData.isLeftHulls.push_back(true);
						}
						else if ((frontLeftDis < disTol && frontRightDis > disTol) && (backLeftDis < disTol && backRightDis > disTol))
						{
							compilerData.hullGroups.push_back(pGroup);
							compilerData.isLeftHulls.push_back(false);
						}
					}
				}
				else if (pGroup->laneBoundaries.size() - tmpNextGroup->laneBoundaries.size() == 0 &&
					pGroup->laneBoundaries.size() - tmpPrevGroup->laneBoundaries.size() == 1)
				{
					if (tmpNextGroup->next.size() == 1)
					{
						HadLaneGroup* tmpSecondNextGroup = (HadLaneGroup*)tmpNextGroup->next.front();
						if (tmpPrevGroup->laneBoundaries.size() - tmpSecondNextGroup->laneBoundaries.size() == 0)
						{

							LineString3d leftSide, rightSide, prevLeftSide, prevRightSide;
							getLaneGroupBoundary(pGroup, leftSide, rightSide, false);
							getLaneGroupBoundary(tmpPrevGroup, prevLeftSide, prevRightSide, false);
							auto leftDis = leftSide.vertexes.front().pos.distance(prevLeftSide.vertexes.back().pos);
							auto rightDis = rightSide.vertexes.front().pos.distance(prevRightSide.vertexes.back().pos);
							if (leftDis > disTol && rightDis < disTol)
							{
								compilerData.hullGroups.push_back(pGroup);
								compilerData.isLeftHulls.push_back(true);
							}
							else if (leftDis < disTol && rightDis > disTol)
							{
								compilerData.hullGroups.push_back(pGroup);
								compilerData.isLeftHulls.push_back(false);
							}
						}
					}
				}
				else if (pGroup->laneBoundaries.size() - tmpPrevGroup->laneBoundaries.size() == 0 &&
					pGroup->laneBoundaries.size() - tmpNextGroup->laneBoundaries.size() == 1)
				{
					if (tmpPrevGroup->previous.size() == 1)
					{
						HadLaneGroup* tmpSecondPrevGroup = (HadLaneGroup*)tmpPrevGroup->previous.front();
						if (tmpNextGroup->laneBoundaries.size() - tmpSecondPrevGroup->laneBoundaries.size() == 0)
						{
							LineString3d leftSide, rightSide, nextLeftSide, nextRightSide;
							getLaneGroupBoundary(pGroup, leftSide, rightSide, false);
							getLaneGroupBoundary(tmpNextGroup, nextLeftSide, nextRightSide, false);
							auto leftDis = leftSide.vertexes.back().pos.distance(nextLeftSide.vertexes.front().pos);
							auto rightDis = rightSide.vertexes.back().pos.distance(nextRightSide.vertexes.front().pos);
							if (leftDis > disTol && rightDis < disTol)
							{
								compilerData.hullGroups.push_back(pGroup);
								compilerData.isLeftHulls.push_back(true);
							}
							else if (leftDis < disTol && rightDis > disTol)
							{
								compilerData.hullGroups.push_back(pGroup);
								compilerData.isLeftHulls.push_back(false);
							}
						}
					}
				}
			}
		}

		for each (auto & e in elements)
		{
			HadLaneGroup* pCurrentGroup = (HadLaneGroup*)e;

			m_rdsGroup = queryGroup(pCurrentGroup->originId, pTile);

			//判断是否为普通路
			if (CompileSetting::instance()->isNotCompileUrbanData)
			{
				if (!isProDataLevel(pCurrentGroup))
					continue;
			}

			if (pCurrentGroup->roadBoundaries.size() == 2)
			{
				if (pCurrentGroup->roadBoundaries.at(1)->originId == 236981339749296897)
					printInfo("");
			}

			if (pCurrentGroup->inIntersection)
				continue;

			if (pCurrentGroup->next.size() > 1)
			{
				createRoadFaceAtForkRoadDownStream(pCurrentGroup, pTile);
			}
			else
			{
				createRoadFaceAtNextStraightRoad(pCurrentGroup, pTile);
			}
		}
	}

	void RoadCompiler::createRoadFaceAtNextStraightRoad(HadLaneGroup* pGroup, RDS::RdsTile* pTile)
	{
		// if (pGroup->roadBoundaries.size() != 2)
		if (pGroup->roadBoundaries.size() != 2 && pGroup->laneBoundaries.size() < 2)
			return;

		if (pGroup->next.empty())
		{
			auto tmpItem = createRoadFace(pTile, pGroup);
			if(m_rdsGroup)
				m_rdsGroup->objects.push_back(tmpItem);
			return;
		}

		HadLaneGroup* pNextFirstLaneGroup = (HadLaneGroup*)pGroup->next[0];

		if (pNextFirstLaneGroup->roadBoundaries.size() != 2 && pNextFirstLaneGroup->laneBoundaries.size() < 2)
		{
			auto tmpItem = createRoadFace(pTile, pGroup);
			if(m_rdsGroup)
				m_rdsGroup->objects.push_back(tmpItem);
			return;
		}

		LineString3d leftSide, rightSide;
		getLaneGroupBoundary(pGroup, leftSide, rightSide);

		connectRoadFaceAtNextStraightRoad(pGroup, leftSide, rightSide);

		connectRoadFaceAtPreviousStraightRoad(pGroup, leftSide, rightSide);

		auto tmpItem = createRoadFace(pTile, pGroup, leftSide, rightSide);
		if(m_rdsGroup)
			m_rdsGroup->objects.push_back(tmpItem);
	}

	void RoadCompiler::connectIntersectionRoad(
			LineString3d& leftSide, 
			LineString3d& rightSide, 
			bool isNext)
	{
		auto leftPoints = leftSide.vertexes;
		auto rightPoints = rightSide.vertexes;
		coordinatesTransform.convert(leftPoints.data(), leftPoints.size());
		coordinatesTransform.convert(rightPoints.data(), rightPoints.size());
		auto leftPts = LINESTRING_T(leftPoints);
		auto rightPts = LINESTRING_T(rightPoints);
		std::vector<point_t> tmpPts;
		if (isNext)
		{
			tmpPts.push_back(leftPts.back());
			tmpPts.push_back(rightPts.back());
		}
		else
		{
			tmpPts.push_back(leftPts.front());
			tmpPts.push_back(rightPts.front());
		}
		std::vector<size_t> tmpIds;
		std::vector<segment_t> tmpSegs;
		for (auto& item : tmpPts)
		{
			getIntersectionStopLineSegmentRTree()->query(bgi::nearest(item, 1),
				boost::make_function_output_iterator([&](size_t const& id) {
					tmpIds.push_back(id);
					tmpSegs.push_back(compilerData.gridIntectionStopLines[id]);
					})
			);
		}
		if (tmpIds.size() == 2)
		{
			if (tmpIds.at(0) == tmpIds.at(1))
			{
				for (size_t i = 0; i < tmpPts.size(); ++i)
				{
					segment_t tmpSeg;
					bg::closest_points(tmpPts[i], tmpSegs.front(), tmpSeg);
					auto tmpLength = bg::length(tmpSeg);
					if (tmpLength > 0 &&
						tmpLength < 1000 &&
						!bg::equals(tmpSeg.second, tmpSegs.front().first) &&
						!bg::equals(tmpSeg.second, tmpSegs.front().second))
					{
						for (auto& tmpInfo : compilerData.m_rdsIntersections)
						{
							if (tmpInfo._originId == compilerData.stopLineToOriginIdMaps[tmpIds.front()])
							{
								auto resultPoint = tmpSeg.second;
								auto resultOriginPoint = MapPoint3D64_make(resultPoint.get<0>(), resultPoint.get<1>(), resultPoint.get<2>() / 10);
								coordinatesTransform.invert(&resultOriginPoint, 1);
								std::vector<MapPoint3D64>::iterator ita, itb;
								getClosestSeg(resultOriginPoint, tmpInfo._originIntersctPoly.vertexes, ita, itb);
								tmpInfo._originIntersctPoly.vertexes.insert(itb, resultOriginPoint);
								//std::cout << "intersection problem position: (" << resultOriginPoint.pos.lon / 1000 << "," << resultOriginPoint.pos.lat / 1000 << ")" << std::endl;
								convert(tmpInfo._originIntersctPoly, tmpInfo._intersection->contour);

								if (i == 0)
								{
									if (isNext)
										leftSide.vertexes.push_back(resultOriginPoint);
									else
										leftSide.vertexes.insert(leftSide.vertexes.begin(), resultOriginPoint);
								}
								else if (i == 1)
								{
									if(isNext)
										rightSide.vertexes.push_back(resultOriginPoint);
									else
										rightSide.vertexes.insert(rightSide.vertexes.begin(), resultOriginPoint);
								}
							}
						}
					}
				}
			}
		}
	}

	void RoadCompiler::connectRoadFaceAtNextStraightRoad(HadLaneGroup* pGroup, LineString3d& leftSide, LineString3d& rightSide)
	{
		//先判断是否与路口相连
		auto isIntersectionNextGroup = [](HadLaneGroup* laneGroup)->bool {
			for (auto item : laneGroup->next)
			{
				auto tmpGroup = (HadLaneGroup*)item;
				if (tmpGroup->inIntersection != 0)
					return true;
			}
			return false;
		};

		if (isIntersectionNextGroup(pGroup))
		{
			connectIntersectionRoad(leftSide, rightSide, true);
			return;
		}

		HadLaneGroup* pNextFirstLaneGroup = (HadLaneGroup*)pGroup->next[0];
		if (!pNextFirstLaneGroup->inIntersection)
		{
			LineString3d nextLeftSide, nextRightSide;
			getLaneGroupBoundary(pNextFirstLaneGroup, nextLeftSide, nextRightSide);
			if (pNextFirstLaneGroup->previous.size() > 1)
			{
				// 11304682,2321048
				HadLaneGroup* pLeftLaneGroup = nullptr;
				HadLaneGroup* pRightLaneGroup = nullptr;
				LineString3d forkRoadLeftSideUpStreamStopLine;
				LineString3d forkRoadRightSideUpStreamStopLine;
				connectRoadFaceAtForkRoadUpStream(pNextFirstLaneGroup, nextLeftSide, nextRightSide, pLeftLaneGroup, pRightLaneGroup,
					forkRoadLeftSideUpStreamStopLine, forkRoadRightSideUpStreamStopLine);
			}

			//隧道做了坡度下降.考虑不做连接.
			if (abs(nextLeftSide.vertexes[0].z - leftSide.vertexes.back().z) < 100)
			{
				if (leftSide.vertexes.back() != nextLeftSide.vertexes[0])
					leftSide.vertexes.push_back(nextLeftSide.vertexes[0]);
			}
			if (abs(nextRightSide.vertexes[0].z - rightSide.vertexes.back().z) < 100)
			{
				if (rightSide.vertexes.back() != nextRightSide.vertexes[0])
					rightSide.vertexes.push_back(nextRightSide.vertexes[0]);
			}
		}
	}

	void RoadCompiler::connectRoadFaceAtPreviousStraightRoad(HadLaneGroup* pGroup, LineString3d& leftSide, LineString3d& rightSide)
	{
		//先判断是否与路口相连
		auto isIntersectionPreviousGroup = [](HadLaneGroup* laneGroup)->bool {
			for (auto item : laneGroup->previous)
			{
				auto tmpGroup = (HadLaneGroup*)item;
				if (tmpGroup->inIntersection != 0)
					return true;
			}
			return false;
		};

		if (isIntersectionPreviousGroup(pGroup))
		{
			connectIntersectionRoad(leftSide, rightSide, false);
			return;
		}

		//如果不是与路口相连，继续按照之前的逻辑进行
		auto isStraightPreviousGroup = [](HadLaneGroup* laneGroup)->bool {
			if (laneGroup->previous.size() != 1)
				return false;
			for (auto& pBoundary : laneGroup->roadBoundaries) {
				if (pBoundary->previous.size() > 1)
					return false;
			}
			for (auto& pBoundary : laneGroup->laneBoundaries) {
				if (pBoundary->previous.size() > 1)
					return false;
			}
			return true;
		};

		if (!isStraightPreviousGroup(pGroup)) {
			return;
		}

		HadLaneGroup* pPreviousFirstLaneGroup = (HadLaneGroup*)pGroup->previous[0];
		if (!pPreviousFirstLaneGroup->inIntersection && pPreviousFirstLaneGroup->next.size() > 1)
		{
			LineString3d previousLeftSide;
			LineString3d previousRightSide;
			getLaneGroupBoundary(pPreviousFirstLaneGroup, previousLeftSide, previousRightSide);

			HadLaneGroup* pPreviousLeftLaneGroup = nullptr;
			HadLaneGroup* pPreviousRightLaneGroup = nullptr;
			LineString3d forkRoadLeftSideDownStreamStopLine;
			LineString3d forkRoadRightSideDownStreamStopLine;
			connectRoadFaceAtForkRoadDownStream(pPreviousFirstLaneGroup, previousLeftSide, previousRightSide,
				pPreviousLeftLaneGroup, pPreviousRightLaneGroup, forkRoadLeftSideDownStreamStopLine, forkRoadRightSideDownStreamStopLine);
			//隧道做了坡度下降.考虑不做连接.
			if (abs(leftSide.vertexes.front().z - previousLeftSide.vertexes.back().z) < 100)
			{
				if (leftSide.vertexes.front() != previousLeftSide.vertexes.back())
					leftSide.vertexes.insert(leftSide.vertexes.begin(), previousLeftSide.vertexes.back());
			}
			if (abs(rightSide.vertexes.front().z - previousRightSide.vertexes.back().z) < 100)
			{
				if (rightSide.vertexes.front() != previousRightSide.vertexes.back())
					rightSide.vertexes.insert(rightSide.vertexes.begin(), previousRightSide.vertexes.back());
			}
		}
	}

	void RoadCompiler::createRoadFaceAtForkRoadDownStream(HadLaneGroup* pGroup, RDS::RdsTile* pTile)
	{
		// if (pGroup->roadBoundaries.size() != 2)
		if (pGroup->roadBoundaries.size() != 2 && pGroup->laneBoundaries.size() < 2)
			return;

		RdsGroup* pRdsGroup = queryGroup(pGroup->originId, pTile);
		HadLink* pFirstLink = pGroup->relLinks.begin()->first;
		HadLink* pSecondLink = pGroup->relLinks.rbegin()->first;

		// 处于上游
		HadLaneGroup* pNextFirst = (HadLaneGroup*)pGroup->next[0];
		HadLaneGroup* pNextSecond = (HadLaneGroup*)pGroup->next[1];
		// if (pNextFirst->roadBoundaries.size() != 2 || pNextSecond->roadBoundaries.size() != 2)
		if ((pNextFirst->roadBoundaries.size() != 2 && pNextFirst->laneBoundaries.size() < 2) ||
			(pNextSecond->roadBoundaries.size() != 2 && pNextSecond->laneBoundaries.size() < 2))
		{
			pRdsGroup->objects.push_back(createRoadFace(pTile, pGroup));
			return;
		}

		LineString3d leftSide, rightSide;
		getLaneGroupBoundary(pGroup, leftSide, rightSide);

		HadLaneGroup* pLeftLaneGroup = nullptr;
		HadLaneGroup* pRightLaneGroup = nullptr;
		LineString3d forkRoadLeftSideDownStreamStopLine;
		LineString3d forkRoadRightSideDownStreamStopLine;
		connectRoadFaceAtForkRoadDownStream(pGroup, leftSide, rightSide, pLeftLaneGroup, pRightLaneGroup,
			forkRoadLeftSideDownStreamStopLine, forkRoadRightSideDownStreamStopLine);
		RDS::RdsRoad* pRoad = createRoadFace(pTile, pGroup, leftSide, rightSide);
		pRdsGroup->objects.push_back(pRoad);
	}

	void RoadCompiler::connectRoadFaceAtForkRoadDownStream(HadLaneGroup* pGroup,
		LineString3d& leftSide, LineString3d& rightSide, HadLaneGroup*& pLeftLaneGroup, HadLaneGroup*& pRightLaneGroup,
		LineString3d& forkRoadLeftSideDownStreamStopLine, LineString3d& forkRoadRightSideDownStreamStopLine)
	{
		auto getNextLeftRightPoint = [](HadLaneGroup* pLaneGroup, MapPoint3D64& firstLeftPoint, MapPoint3D64& firstRrightPoint) {
			if (pLaneGroup->roadBoundaries.size() == 2)
			{
				firstLeftPoint = pLaneGroup->roadBoundaries.front()->location.vertexes.front();
				if (directionEqual(pLaneGroup->roadBoundaries.front(), pLaneGroup, 3))
					firstLeftPoint = pLaneGroup->roadBoundaries.front()->location.vertexes.back();

				firstRrightPoint = pLaneGroup->roadBoundaries.back()->location.vertexes.front();
				if (directionEqual(pLaneGroup->roadBoundaries.back(), pLaneGroup, 3))
					firstRrightPoint = pLaneGroup->roadBoundaries.back()->location.vertexes.back();
			}
			else
			{
				firstLeftPoint = pLaneGroup->laneBoundaries.front()->location.vertexes.front();
				if (directionEqual(pLaneGroup->laneBoundaries.front(), pLaneGroup, 3))
					firstLeftPoint = pLaneGroup->laneBoundaries.front()->location.vertexes.back();

				firstRrightPoint = pLaneGroup->laneBoundaries.back()->location.vertexes.front();
				if (directionEqual(pLaneGroup->laneBoundaries.back(), pLaneGroup, 3))
					firstRrightPoint = pLaneGroup->laneBoundaries.back()->location.vertexes.back();
			}
		};

		// 处于上游
		HadLaneGroup* pNextFirst = (HadLaneGroup*)pGroup->next[0];
		HadLaneGroup* pNextSecond = (HadLaneGroup*)pGroup->next[1];

		// 判断左右
		MapPoint3D64 leftEndPoint = *leftSide.vertexes.rbegin();
		MapPoint3D64 rightEndPoint = *rightSide.vertexes.rbegin();

		MapPoint3D64 firstLeftPoint, firstRrightPoint;
		getNextLeftRightPoint(pNextFirst, firstLeftPoint, firstRrightPoint);

		MapPoint3D64 secondLeftPoint, secondRrightPoint;
		getNextLeftRightPoint(pNextSecond, secondLeftPoint, secondRrightPoint);
		auto leftEndFirstPointDistance = leftEndPoint.pos.distanceSquare(firstLeftPoint.pos);
		auto leftEndSecondPointDistance = leftEndPoint.pos.distanceSquare(secondLeftPoint.pos);
		if (leftEndFirstPointDistance < leftEndSecondPointDistance)
		{
			pLeftLaneGroup = pNextFirst;
			pRightLaneGroup = pNextSecond;
		}
		else if (leftEndFirstPointDistance == leftEndSecondPointDistance)
		{
			auto rightEndFirstPointDistance = rightEndPoint.pos.distanceSquare(firstRrightPoint.pos);
			auto rightEndSecondPointDistance = rightEndPoint.pos.distanceSquare(secondRrightPoint.pos);
			if (rightEndFirstPointDistance < rightEndSecondPointDistance)
			{
				pLeftLaneGroup = pNextSecond;
				pRightLaneGroup = pNextFirst;
			}
			else
			{
				pLeftLaneGroup = pNextFirst;
				pRightLaneGroup = pNextSecond;
			}
		}
		else
		{
			pLeftLaneGroup = pNextSecond;
			pRightLaneGroup = pNextFirst;
		}

		// 与下一个车道组左右坐标都相连时返回
		auto& leftSideBack = leftSide.vertexes.back();
		auto& rightSideBack = rightSide.vertexes.back();
		if ((leftSideBack == firstLeftPoint && rightSideBack == firstRrightPoint)
			|| (leftSideBack == secondLeftPoint && rightSideBack == secondRrightPoint))
		{
			return;
		}

		if (pLeftLaneGroup->inIntersection && pRightLaneGroup->inIntersection)
		{
			connectIntersectionRoad(leftSide, rightSide, true);
			return;
		}

		if (!pLeftLaneGroup->inIntersection)
		{
			MapPoint3D64 firstLeftSideConnectPt, secondLeftSideConnectPt;
			if (pLeftLaneGroup->roadBoundaries.size() == 2)
			{
				firstLeftSideConnectPt = getRoadBoundaryFirstPointRelGroup(pLeftLaneGroup, pLeftLaneGroup->roadBoundaries[0]);
				secondLeftSideConnectPt = getRoadBoundaryFirstPointRelGroup(pLeftLaneGroup, pLeftLaneGroup->roadBoundaries[1]);
			}
			else
			{
				auto rightIdx = pLeftLaneGroup->laneBoundaries.size() - 1;
				firstLeftSideConnectPt = getRoadBoundaryFirstPointRelGroup(pLeftLaneGroup, pLeftLaneGroup->laneBoundaries[0]);
				secondLeftSideConnectPt = getRoadBoundaryFirstPointRelGroup(pLeftLaneGroup, pLeftLaneGroup->laneBoundaries[rightIdx]);
			}
			forkRoadLeftSideDownStreamStopLine.vertexes.push_back(firstLeftSideConnectPt);
			forkRoadLeftSideDownStreamStopLine.vertexes.push_back(secondLeftSideConnectPt);
		}

		if (!pRightLaneGroup->inIntersection)
		{
			MapPoint3D64 firstRightSideConnectPt, secondRightSideConnectPt;
			if (pRightLaneGroup->roadBoundaries.size() == 2)
			{
				firstRightSideConnectPt = getRoadBoundaryFirstPointRelGroup(pRightLaneGroup, pRightLaneGroup->roadBoundaries[1]);
				secondRightSideConnectPt = getRoadBoundaryFirstPointRelGroup(pRightLaneGroup, pRightLaneGroup->roadBoundaries[0]);
			}
			else
			{
				auto rightIdx = pRightLaneGroup->laneBoundaries.size() - 1;
				firstRightSideConnectPt = getRoadBoundaryFirstPointRelGroup(pRightLaneGroup, pRightLaneGroup->laneBoundaries[rightIdx]);
				secondRightSideConnectPt = getRoadBoundaryFirstPointRelGroup(pRightLaneGroup, pRightLaneGroup->laneBoundaries[0]);
			}
			forkRoadRightSideDownStreamStopLine.vertexes.push_back(firstRightSideConnectPt);
			forkRoadRightSideDownStreamStopLine.vertexes.push_back(secondRightSideConnectPt);
		}

		// 插入中间坐标点
		addMiddlePoint(forkRoadLeftSideDownStreamStopLine, forkRoadRightSideDownStreamStopLine);
		addMiddlePoint(forkRoadRightSideDownStreamStopLine, forkRoadLeftSideDownStreamStopLine);
		for (auto& vertex : forkRoadLeftSideDownStreamStopLine.vertexes) {
			if (leftSide.vertexes.back() != vertex)
				leftSide.vertexes.push_back(vertex);
		}
		for (auto& vertex : forkRoadRightSideDownStreamStopLine.vertexes) {
			if (rightSide.vertexes.back() != vertex)
				rightSide.vertexes.push_back(vertex);
		}
	}

	void RoadCompiler::connectRoadFaceAtForkRoadUpStream(HadLaneGroup* pGroup,
		LineString3d& leftSide, LineString3d& rightSide, HadLaneGroup*& pLeftLaneGroup, HadLaneGroup*& pRightLaneGroup, 
		LineString3d& forkRoadLeftSideUpStreamStopLine, LineString3d& forkRoadRightSideUpStreamStopLine)
	{
		auto getPreviousLeftRightPoint = [](HadLaneGroup* pLaneGroup, MapPoint3D64& firstLeftPoint, MapPoint3D64& firstRrightPoint) {
			if (pLaneGroup->roadBoundaries.size() == 2)
			{
				firstLeftPoint = pLaneGroup->roadBoundaries.front()->location.vertexes.back();
				if (directionEqual(pLaneGroup->roadBoundaries.front(), pLaneGroup, 3))
					firstLeftPoint = pLaneGroup->roadBoundaries.front()->location.vertexes.front();

				firstRrightPoint = pLaneGroup->roadBoundaries.back()->location.vertexes.back();
				if (directionEqual(pLaneGroup->roadBoundaries.back(), pLaneGroup, 3))
					firstRrightPoint = pLaneGroup->roadBoundaries.back()->location.vertexes.front();
			}
			else
			{
				firstLeftPoint = pLaneGroup->laneBoundaries.front()->location.vertexes.back();
				if (directionEqual(pLaneGroup->laneBoundaries.front(), pLaneGroup, 3))
					firstLeftPoint = pLaneGroup->laneBoundaries.front()->location.vertexes.front();

				firstRrightPoint = pLaneGroup->laneBoundaries.back()->location.vertexes.back();
				if (directionEqual(pLaneGroup->laneBoundaries.back(), pLaneGroup, 3))
					firstRrightPoint = pLaneGroup->laneBoundaries.back()->location.vertexes.front();
			}
		};

		// 处于下游
		HadLaneGroup* pPreviousFirst = (HadLaneGroup*)pGroup->previous[0];
		HadLaneGroup* pPreviousSecond = (HadLaneGroup*)pGroup->previous[1];

		// 判断左右
		MapPoint3D64 leftBeginPoint = *leftSide.vertexes.begin();
		MapPoint3D64 rightBeginPoint = *rightSide.vertexes.begin();

		MapPoint3D64 firstLeftPoint, firstRrightPoint;
		getPreviousLeftRightPoint(pPreviousFirst, firstLeftPoint, firstRrightPoint);

		MapPoint3D64 secondLeftPoint, secondRrightPoint;
		getPreviousLeftRightPoint(pPreviousSecond, secondLeftPoint, secondRrightPoint);
		auto leftBeginFirstPointDistance = leftBeginPoint.pos.distanceSquare(firstLeftPoint.pos);
		auto leftBeginSecondPointDistance = leftBeginPoint.pos.distanceSquare(secondLeftPoint.pos);
		if (leftBeginFirstPointDistance < leftBeginSecondPointDistance)
		{
			pLeftLaneGroup = pPreviousFirst;
			pRightLaneGroup = pPreviousSecond;
		}
		else if (leftBeginFirstPointDistance == leftBeginSecondPointDistance)
		{
			auto rightBeginFirstPointDistance = rightBeginPoint.pos.distanceSquare(firstRrightPoint.pos);
			auto rightBeginSecondPointDistance = rightBeginPoint.pos.distanceSquare(secondRrightPoint.pos);
			if (rightBeginFirstPointDistance < rightBeginSecondPointDistance)
			{
				pLeftLaneGroup = pPreviousSecond;
				pRightLaneGroup = pPreviousFirst;
			}
			else
			{
				pLeftLaneGroup = pPreviousFirst;
				pRightLaneGroup = pPreviousSecond;
			}
		}
		else
		{
			pLeftLaneGroup = pPreviousSecond;
			pRightLaneGroup = pPreviousFirst;
		}

		// 与上一个车道组左右坐标都相连时返回
		auto& leftSideFront = leftSide.vertexes.front();
		auto& rightSideFront = rightSide.vertexes.front();
		if ((leftSideFront == firstLeftPoint && rightSideFront == firstRrightPoint)
			|| (leftSideFront == secondLeftPoint && rightSideFront == secondRrightPoint))
		{
			return;
		}

		if (!pLeftLaneGroup->inIntersection)
		{
			MapPoint3D64 firstLeftSideConnectPt, secondLeftSideConnectPt;
			if (pLeftLaneGroup->roadBoundaries.size() == 2)
			{
				firstLeftSideConnectPt = getRoadBoundaryLastPointRelGroup(pLeftLaneGroup, pLeftLaneGroup->roadBoundaries[1]);
				secondLeftSideConnectPt = getRoadBoundaryLastPointRelGroup(pLeftLaneGroup, pLeftLaneGroup->roadBoundaries[0]);
			}
			else
			{
				auto rightIdx = pLeftLaneGroup->laneBoundaries.size() - 1;
				firstLeftSideConnectPt = getRoadBoundaryLastPointRelGroup(pLeftLaneGroup, pLeftLaneGroup->laneBoundaries[rightIdx]);
				secondLeftSideConnectPt = getRoadBoundaryLastPointRelGroup(pLeftLaneGroup, pLeftLaneGroup->laneBoundaries[0]);
			}
			forkRoadLeftSideUpStreamStopLine.vertexes.push_back(firstLeftSideConnectPt);
			forkRoadLeftSideUpStreamStopLine.vertexes.push_back(secondLeftSideConnectPt);
		}

		if (!pRightLaneGroup->inIntersection)
		{
			MapPoint3D64 firstRightSideConnectPt, secondRightSideConnectPt;
			if (pRightLaneGroup->roadBoundaries.size() == 2)
			{
				firstRightSideConnectPt = getRoadBoundaryLastPointRelGroup(pRightLaneGroup, pRightLaneGroup->roadBoundaries[0]);
				secondRightSideConnectPt = getRoadBoundaryLastPointRelGroup(pRightLaneGroup, pRightLaneGroup->roadBoundaries[1]);
			}
			else
			{
				auto rightIdx = pRightLaneGroup->laneBoundaries.size() - 1;
				firstRightSideConnectPt = getRoadBoundaryLastPointRelGroup(pRightLaneGroup, pRightLaneGroup->laneBoundaries[0]);
				secondRightSideConnectPt = getRoadBoundaryLastPointRelGroup(pRightLaneGroup, pRightLaneGroup->laneBoundaries[rightIdx]);
			}
			forkRoadRightSideUpStreamStopLine.vertexes.push_back(firstRightSideConnectPt);
			forkRoadRightSideUpStreamStopLine.vertexes.push_back(secondRightSideConnectPt);
		}

		// 插入中间坐标点
		addMiddlePoint(forkRoadLeftSideUpStreamStopLine, forkRoadRightSideUpStreamStopLine);
		addMiddlePoint(forkRoadRightSideUpStreamStopLine, forkRoadLeftSideUpStreamStopLine);
		for_each(forkRoadLeftSideUpStreamStopLine.vertexes.rbegin(), forkRoadLeftSideUpStreamStopLine.vertexes.rend(), [&](auto& vertex) {
			if (leftSide.vertexes.front() != vertex)
				leftSide.vertexes.insert(leftSide.vertexes.begin(), vertex);
			}
		);
		for_each(forkRoadRightSideUpStreamStopLine.vertexes.rbegin(), forkRoadRightSideUpStreamStopLine.vertexes.rend(), [&](auto& vertex) {
			if (rightSide.vertexes.front() != vertex)
				rightSide.vertexes.insert(rightSide.vertexes.begin(), vertex);
			}
		);
	}

	void RoadCompiler::addMiddlePoint(LineString3d& firstStopLine, LineString3d& secondStopLine)
	{
		if (firstStopLine.vertexes.size() != 2 || secondStopLine.vertexes.empty())
			return;

		auto isMiddlePoint = [](LineString3d& stopLine, MapPoint3D64& vertex)->bool {
			size_t si, ei;
			MapPoint3D64 grappedPt = {};
			bool grapped = GrapPointAlgorithm::grapOrMatchNearestPoint(vertex, stopLine.vertexes, grappedPt, si, ei);
			if (grappedPt == vertex)
				return false;

			auto grappedPointDistance = grappedPt.pos.distance(vertex.pos);
			if (grapped && grappedPointDistance < 100)
				return true;
			return false;
		};

		auto& frontPt = secondStopLine.vertexes.front();
		auto& backPt = secondStopLine.vertexes.back();
		bool isFrontPtMiddlePoint = isMiddlePoint(firstStopLine, frontPt);
		bool isBackPtMiddlePoint = isMiddlePoint(firstStopLine, backPt);
		if (isFrontPtMiddlePoint && !isBackPtMiddlePoint) {
			auto iter = firstStopLine.vertexes.begin();
			firstStopLine.vertexes.insert(iter + 1, frontPt);
		}
		if (!isFrontPtMiddlePoint && isBackPtMiddlePoint) {
			auto iter = firstStopLine.vertexes.begin();
			firstStopLine.vertexes.insert(iter + 1, backPt);
		}
	}

	RDS::RdsRoad* RoadCompiler::createRoadFace(RdsTile* pTile, HadLaneGroup* pGroup)
	{
		LineString3d leftSide, rightSide;
		getLaneGroupBoundary(pGroup, leftSide, rightSide);
		return createRoadFace(pTile, pGroup, leftSide, rightSide);
	}

	RDS::RdsRoad* RoadCompiler::createRoadFace(RdsTile* pTile, HadLaneGroup* pGroup, LineString3d& leftSide, LineString3d& rightSide)
	{
		if (pGroup->previous.size() > 1)
		{
			HadLaneGroup* pLeftLaneGroup = nullptr;
			HadLaneGroup* pRightLaneGroup = nullptr;
			LineString3d forkRoadLeftSideUpStreamStopLine;
			LineString3d forkRoadRightSideUpStreamStopLine;
			connectRoadFaceAtForkRoadUpStream(pGroup, leftSide, rightSide, pLeftLaneGroup, pRightLaneGroup,
				forkRoadLeftSideUpStreamStopLine, forkRoadRightSideUpStreamStopLine);
		}

		RdsRoad* pRoad = (RdsRoad*)createObject(pTile, EntityType::RDS_ROAD);
		RDS::LineString3d lineLeftSide;
		convert(leftSide, lineLeftSide);
		RDS::LineString3d lineRightSize;
		convert(rightSide, lineRightSize);
		pRoad->contour.lines.resize(2);
		pRoad->contour.lines[0] = lineLeftSide;
		pRoad->contour.lines[1] = lineRightSize;

		std::vector<LineString3d> tmpLines;
		tmpLines.push_back(leftSide);
		tmpLines.push_back(rightSide);
		saveRdsRoad(pRoad, tmpLines, isRoundabout(pGroup), pGroup->originId);
		return pRoad;
	}

	MapPoint3D64 RoadCompiler::getRoadBoundaryFirstPointRelGroup(HadLaneGroup* pGroup, HadRoadBoundary* pBoundary)
	{
		if (directionEqual(pBoundary, pGroup, 3))
		{
			return *pBoundary->location.vertexes.rbegin();
		}
		return pBoundary->location.vertexes.front();
	}

	MapPoint3D64 RoadCompiler::getRoadBoundaryFirstPointRelGroup(HadLaneGroup* pGroup, HadLaneBoundary* pBoundary)
	{
		if (directionEqual(pBoundary, pGroup, 3))
		{
			return *pBoundary->location.vertexes.rbegin();
		}
		return pBoundary->location.vertexes.front();
	}

	MapPoint3D64 RoadCompiler::getRoadBoundaryLastPointRelGroup(HadLaneGroup* pGroup, HadRoadBoundary* pBoundary)
	{
		if (directionEqual(pBoundary, pGroup, 3))
		{
			return *pBoundary->location.vertexes.begin();
		}
		return pBoundary->location.vertexes.back();
	}

	MapPoint3D64 RoadCompiler::getRoadBoundaryLastPointRelGroup(HadLaneGroup* pGroup, HadLaneBoundary* pBoundary)
	{
		if (directionEqual(pBoundary, pGroup, 3))
		{
			return *pBoundary->location.vertexes.begin();
		}
		return pBoundary->location.vertexes.back();
	}


	void RoadCompiler::saveRdsRoad(RdsRoad* pRoad, std::vector<LineString3d>& originRoadLines, bool isRoundabout, const int64& originId)
	{
		if (pRoad->contour.lines.empty())
			return;

		if (originRoadLines.size() == 2)
		{
			rdsRoadInfo tmp;
			tmp.originId = originId;
			tmp._road = pRoad;
			tmp._originRoadLines = originRoadLines;
			std::vector<MapPoint3D64> tmpPoly;
			auto& leftLine = originRoadLines.at(0);
			auto& rightLine = originRoadLines.at(1);

			std::vector<Triangle> roadTriangles;
			LinearInterpolationTriangleSurface::triangularizeStroke(coordinatesTransform, leftLine.vertexes, rightLine.vertexes, roadTriangles);
			compilerData.m_roadTriangles.insert(compilerData.m_roadTriangles.end(), roadTriangles.begin(), roadTriangles.end());
			coordinatesTransform.convert(leftLine.vertexes.data(), leftLine.vertexes.size());
			coordinatesTransform.convert(rightLine.vertexes.data(), rightLine.vertexes.size());
			if (!isRoundabout)
			{
				compilerData.gridRoadStopLines.push_back(segment_t(POINT_T(leftLine.vertexes.front()), POINT_T(rightLine.vertexes.front())));
				compilerData.gridRoadStopLines.push_back(segment_t(POINT_T(leftLine.vertexes.back()), POINT_T(rightLine.vertexes.back())));
			}
			std::reverse(rightLine.vertexes.begin(), rightLine.vertexes.end());
			leftLine.vertexes.insert(leftLine.vertexes.end(), rightLine.vertexes.begin(), rightLine.vertexes.end());
			tmp._roadBox2T = BOX_2T(leftLine.vertexes);
			tmp._roadPoly2T = RING_2T(leftLine.vertexes);
			tmp._roadPoints = LINESTRING_T(leftLine.vertexes);
			bg::correct(tmp._roadPoly2T);
			compilerData.m_rdsRoads.push_back(tmp);
			compilerData.m_rdsRoadBoxes.push_back(tmp._roadBox2T);
			compilerData.m_originIdToRdsRoadInfo.emplace(originId, tmp);
		}
	}
}

