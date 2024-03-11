#include "stdafx.h"
#include "GuardrailCompiler.h"
#include "../CompileSetting.h"
#include "math3d/vector_math.h"
#include "tool_kit/call_stack_walker.h"
#include "point_converter.h"
#include "polyline_intersector.h"
#include "algorithm/grap_point_algorithm.h"

#define ROAD_BOUNDARY_PA_NAME 7
#define MAX_HEIGHT 5000

namespace OMDB
{
    void GuardrailCompiler::compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile) 
    {
        UNREFERENCED_PARAMETER(nearby);
        allGroups = getGridAllGroups(pGrid);
        setLaneGroupData(allGroups);
        setBarrierData(pGrid);
        setDiversionData(pTile);
        setLaRoadFaceData(pTile);
        allForkGroups = getForkGroups(allGroups);
        setLaDiversionData();
        setGoreDiversionData();
        setSingleDiversionData();
        for (auto obj : pGrid->query(ElementType::HAD_LANE_GROUP))
        {
            HadLaneGroup* pGroup = (HadLaneGroup*)obj;
            if (isTunnelArea(pGroup) || isInTollArea(pGroup) || !isProDataLevel(pGroup))
            {
                //printf("(%d,%d)\n", pGroup->laneBoundaries[0]->location.vertexes[0].pos.lon / 1000, pGroup->laneBoundaries[0]->location.vertexes[0].pos.lat / 1000);
                continue;
            }
            if (!pGroup->roadBoundaries.empty())
            {
                auto iter = allGroupMaps.find(pGroup->originId);
                if (iter == allGroupMaps.end()) {
                    continue;
                }
                auto& groupInfo = iter->second;

                //search gore diversion
                std::vector<RdsDiversion*> intersectDiversions;
                getDiversionBoxRTree()->query(bgi::intersects(groupInfo._groupBox),
                    boost::make_function_output_iterator([&](size_t const& id) {
                        intersectDiversions.push_back(gridDiversions[id]);
                        }));

                std::set<uint64> triangleDiversionIds;
                std::vector<goreDiversionInfo> triangleDiversions;
                for (size_t j = 0; j < intersectDiversions.size(); ++j)
                {
                    auto intersectDiversion = intersectDiversions[j];
                    for (auto& tmpGore : allGoreDiversions)
                    {
						if (triangleDiversionIds.find(intersectDiversion->id) != triangleDiversionIds.end())
							continue;

                        if (tmpGore._rdsDiversion->id == intersectDiversion->id && tmpGore._isDeleteGuardrail)
                        {
                            triangleDiversionIds.emplace(intersectDiversion->id);
                            triangleDiversions.push_back(tmpGore);
                            break;
                        }
                    }
                }

                //search single diversion
                std::vector<singleDiversionInfo> intersectSingleDiversions;
                getSingleBoxRTree()->query(bgi::intersects(groupInfo._groupBox),
                    boost::make_function_output_iterator([&](size_t const& id) {
                        std::vector<ring_2t> results;
                        bg::intersection(groupInfo._groupPoly, allSingleDiversions[id]._originRing, results);
                        if (!results.empty())
                        {
                            point_t nearbyPoint = getNearestPoint(groupInfo._groupPoints, allSingleDiversions[id]._centerPoint);
                            if (std::abs(allSingleDiversions[id]._centerPoint.get<2>() - nearbyPoint.get<2>()) < MAX_HEIGHT)
                            {
                                intersectSingleDiversions.push_back(allSingleDiversions[id]);
                            }
                        }
                        }));
                if (!intersectSingleDiversions.empty()) {
                    groupInfo._isCirclePoly = isCirclePoly(groupInfo._groupPoly);
                }

                for (size_t roadIndex = 0; roadIndex < pGroup->roadBoundaries.size(); ++roadIndex)
                {
                    if (roadIndex > 2)
                        continue;
                    auto item = pGroup->roadBoundaries[roadIndex];

                    //convert 
                    LineString3d originLine = item->location;
                    coordinatesTransform.convert(originLine.vertexes.data(), originLine.vertexes.size());
                    if (directionEqual(item, pGroup, 3))
                    {
                        std::reverse(originLine.vertexes.begin(), originLine.vertexes.end());
                    }
                    std::vector<LineString3d> firstRoundGuardrails;
               
                    // lane group connection
                    connectNextRoadBoundary(pGroup, item, originLine);
                    if (pGroup->inIntersection && isMiddleRoadBoundary(pGroup, item, originLine))
                        continue;

                    // build guardrail
                    std::vector<segment_2t> guardrailSegments;
                    if (item->boundaryType == BoundaryType::NONE ||
                        item->boundaryType == BoundaryType::GORE ||
                        item->boundaryType == BoundaryType::MARKING)
                    {
						if (item->linkGroups.size() > 1)
							continue;
                        if (isMiddleRoadBoundary(pGroup, item, originLine))
                            continue;
                        if (!isExistBarrierObjects(originLine, getBarrierSegmentRTree(), allBarrierSegs))
                            continue;
                        firstRoundGuardrails.push_back(originLine);
                        setBoundaryGuardrailSegments(originLine, guardrailSegments);
                    }
                    else if (!item->attributes.empty())
                    {
                        auto lineLength = bg::length(LINESTRING_T(originLine.vertexes));
                        auto tmpGuardrailPoints = buildGuardrailPoints(originLine.vertexes);
                        for (auto& attributeItem : item->attributes)
                        {
                            if (attributeItem->name == ROAD_BOUNDARY_PA_NAME)
                            {
                                double startFraction = attributeItem->start;
                                double endFraction = attributeItem->end;
                                auto resultLine = getNewGuardrailByFraction(startFraction * lineLength, endFraction * lineLength, tmpGuardrailPoints, originLine.vertexes);
								if (item->linkGroups.size() > 1)
									continue;
								if (isMiddleRoadBoundary(pGroup, item, resultLine))
									continue;
                                if (attributeItem->value == (int)BoundaryType::NONE ||
                                    attributeItem->value == (int)BoundaryType::GORE ||
                                    attributeItem->value == (int)BoundaryType::MARKING)
                                {
                                    if (endFraction - startFraction < 0.11 || isExistBarrierObjects(resultLine, getBarrierSegmentRTree(), allBarrierSegs))
                                    {
                                        firstRoundGuardrails.push_back(resultLine);
                                        setBoundaryGuardrailSegments(resultLine, guardrailSegments);
                                    }
                                }
                                else
                                {
                                    firstRoundGuardrails.push_back(resultLine);
                                    if (attributeItem->value == (int)BoundaryType::WALL ||
                                        attributeItem->value == (int)BoundaryType::GURADRAIL ||
                                        isExistBarrierObjects(resultLine, getBarrierSegmentRTree(), allBarrierSegs)) {
                                        setBoundaryGuardrailSegments(resultLine, guardrailSegments);
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        firstRoundGuardrails.push_back(originLine);
                        if (item->boundaryType == BoundaryType::WALL ||
                            item->boundaryType == BoundaryType::GURADRAIL ||
                            isExistBarrierObjects(originLine, getBarrierSegmentRTree(), allBarrierSegs)) {
                            setBoundaryGuardrailSegments(originLine, guardrailSegments);
                        }
                    }

                    // modify guardrail
                    if (firstRoundGuardrails.size() > 1)
                    {
                        for (auto it = firstRoundGuardrails.begin() + 1; it != firstRoundGuardrails.end();)
                        {
                            if ((it - 1)->vertexes.back() == it->vertexes.front())
                            {
                                (it - 1)->vertexes.insert((it -1)->vertexes.end(), it->vertexes.begin(), it->vertexes.end());
                                it = firstRoundGuardrails.erase(it);
                            }
                            else
                            {
                                it++;
                            }
                        }
                    }

                    std::vector<LineString3d> secondRoundGuardrails;
                    for (auto tmpGuardrail : firstRoundGuardrails)
                    {
                        if (tmpGuardrail.vertexes.size() < 2)
                            continue;
                        std::vector<LineString3d> resultLines;
                        if (!groupInfo._isCirclePoly && !intersectSingleDiversions.empty())
                        {
                            if (!modifyGuardrailByRdsSingleDiversion(groupInfo, intersectSingleDiversions, tmpGuardrail, roadIndex, resultLines))
                                resultLines = std::vector<LineString3d>{ tmpGuardrail };
                        }
                        else
                        {
                            resultLines = std::vector<LineString3d>{ tmpGuardrail };
                        }
                        secondRoundGuardrails.insert(secondRoundGuardrails.end(), resultLines.begin(), resultLines.end());
                    }

                    std::vector<LineString3d> thirdRoundGuardrails;
                    for (auto tmpGuardrail : secondRoundGuardrails)
                    {
                        if (tmpGuardrail.vertexes.size() < 2)
                            continue;
                        std::vector<LineString3d> resultLines;
                        if (!triangleDiversions.empty())
                        {
                            resultLines = modifyGuardrailByRdsTriDiversion(triangleDiversions, guardrailSegments, tmpGuardrail);
                        }
                        else
                        {
                            resultLines = std::vector<LineString3d>{ tmpGuardrail };
                        }
                        thirdRoundGuardrails.insert(thirdRoundGuardrails.end(), resultLines.begin(), resultLines.end());
                    }

                    if (!thirdRoundGuardrails.empty())
                    {
                        std::vector<LineString3d> filteredLines;
                        removeDuplicatePoint(thirdRoundGuardrails, filteredLines);

                        if (!filteredLines.empty())
                        {
                            RdsGroup* rdsGroup = queryGroup(pGroup->originId, pTile);
                            for (auto& tmpLine : filteredLines)
                                coordinatesTransform.invert(tmpLine.vertexes.data(), tmpLine.vertexes.size());
                            createGuardrail(rdsGroup->objects, pTile, filteredLines);
                        }

                      
                    }
                }
            }
        }
        return;
    }

    void GuardrailCompiler::connectNextRoadBoundary(HadLaneGroup* pGroup, HadRoadBoundary* pBoundary, LineString3d& originLine)
    {
        if (!pBoundary->danglingEndNode) {
            return;
        }

        // 路口点不需要连
        double minDistance = DBL_MAX;
        HadRoadBoundary* nextRoadBoundary = nullptr;
        point_t p0 = POINT_T(originLine.vertexes.back());
        for (auto tBoundary : pBoundary->next) {
			auto tmpRoadBoundary = (HadRoadBoundary*)tBoundary;
			for (auto pair : tmpRoadBoundary->linkGroups) {
				if (pair.second->inIntersection)
					return;
			}

			MapPoint3D64 p1 = tmpRoadBoundary->location.vertexes.back();
			MapPoint3D64 p2 = tmpRoadBoundary->location.vertexes.front();
			coordinatesTransform.convert(&p1, 1);
			coordinatesTransform.convert(&p2, 1);
            double distance = DBL_MAX;
			if (directionEqual(nextRoadBoundary, pGroup, 3)) {
                distance = bg::distance(P3_P2(p0), P3_P2(POINT_T(p1)));
			} else {
                distance = bg::distance(P3_P2(p0), P3_P2(POINT_T(p2)));
			}
			if (distance < minDistance) {
				nextRoadBoundary = tmpRoadBoundary;
				minDistance = distance;
			}
        }

        if (nextRoadBoundary != nullptr && !nextRoadBoundary->barrierObjects.empty()) {
			MapPoint3D64 p1 = nextRoadBoundary->location.vertexes.back();
			MapPoint3D64 p2 = nextRoadBoundary->location.vertexes.front();
			coordinatesTransform.convert(&p1, 1);
			coordinatesTransform.convert(&p2, 1);
			if (directionEqual(nextRoadBoundary, pGroup, 3))
			{
				if (bg::distance(P3_P2(p0), P3_P2(POINT_T(p1))) < 3000)
					originLine.vertexes.push_back(p1);
			}
			else
			{
				if (bg::distance(P3_P2(p0), P3_P2(POINT_T(p2))) < 3000)
					originLine.vertexes.push_back(p2);
			}
        }
    }

    bool GuardrailCompiler::isMiddleRoadBoundary(HadLaneGroup* pGroup, HadRoadBoundary* pBoundary, LineString3d& segLine)
    {
        UNREFERENCED_PARAMETER(pBoundary);
		ring_2t tmpExpandRing;
        linestring_2t tmpLine = LINESTRING_2T(segLine.vertexes);
        bg::unique(tmpLine);
		bg::correct(tmpLine);
		if (!getExpandPolyByOffset(tmpLine, 500, tmpExpandRing))
            return false;
		point_t tmpPoint;
		linestring_t tmpLine3D = LINESTRING_T(segLine.vertexes);
		bg::centroid(tmpLine3D, tmpPoint);

        // lanegroup
        std::vector<size_t> groupIds;
        std::vector<ring_2t> groupPolys;
		getLaneGroupBoxRTree()->query(bgi::intersects(BOX_T(segLine.vertexes)),
			boost::make_function_output_iterator([&](size_t const& id) {
                std::vector<ring_2t> results;
				bg::intersection(allGroupDatas[id]._groupPoly, tmpExpandRing, results);
				if (pGroup != allGroupDatas[id]._laneGroup && !results.empty())
				{
					point_t nearbyPoint = getNearestPoint(allGroupDatas[id]._groupPoints, tmpPoint);
					if (std::abs(tmpPoint.get<2>() - nearbyPoint.get<2>()) < MAX_HEIGHT)
					{
                        groupIds.push_back(id);
					}
				}
			}));
		for (auto groupId : groupIds)
			groupPolys.push_back(allGroupDatas[groupId]._groupPoly);
        if (!groupPolys.empty() && getLineInPolyPercentage(tmpLine, groupPolys) > 0.3) {
            return true;
        }

        // intersection
        std::vector<size_t> intersectionIds;
		std::vector<ring_2t> intersectionPolys;
        auto& gridIntersectionDatas = compilerData.m_rdsIntersections;
        getIntersectionBox2TRTree()->query(bgi::intersects(BOX_2T(segLine.vertexes)),
			boost::make_function_output_iterator([&](size_t const& id) {
				std::vector<ring_2t> results;
				bg::intersection(gridIntersectionDatas[id]._intersectionPoly2T, tmpExpandRing, results);
				if (!results.empty())
				{
					point_t nearbyPoint = getNearestPoint(gridIntersectionDatas[id]._intersectionPoints, tmpPoint);
					if (std::abs(tmpPoint.get<2>() - nearbyPoint.get<2>()) < MAX_HEIGHT)
					{
                        intersectionIds.push_back(id);
					}
				}
			}));
		for (auto intersectionId : intersectionIds)
            intersectionPolys.push_back(gridIntersectionDatas[intersectionId]._intersectionPoly2T);
		if (!intersectionPolys.empty() && getLineInPolyPercentage(tmpLine, intersectionPolys) > 0.3) {
			return true;
		}

        return false;
    }

    double GuardrailCompiler::getLineInPolyPercentage(linestring_2t& tmpLine, std::vector<ring_2t>& polys)
    {
        auto withinPoly = [](point_2t& pt, ring_2t& poly, linestring_2t& polyString)->bool {
			double ptDistance = bg::distance(pt, polyString);
			if (bg::within(pt, poly) && ptDistance > 10) {
				return true;
			}
            return false;
        };

		auto withinOrCrossPoly = [&](linestring_2t& line, ring_2t& poly)->bool {
            auto polyString = LINESTRING_2T(poly);
            for (int idx = 0; idx < line.size() - 1; idx++) {
                auto& currPt = line[idx];
                auto& nextPt = line[idx + 1];
                if (withinPoly(currPt, poly, polyString) || withinPoly(nextPt, poly, polyString)) {
                    return true;
                }

                auto intersectPt = currPt;
				bg::add_point(intersectPt, nextPt);
				bg::divide_value(intersectPt, 2.0);
				if (withinPoly(intersectPt, poly, polyString)) {
					return true;
				}

				linestring_2t clipper;
				clipper.push_back(currPt);
				clipper.push_back(nextPt);
				std::vector<point_2t> intersectPoints;
				bg::intersection(poly, clipper, intersectPoints);
				for (auto iter = intersectPoints.begin(); iter != intersectPoints.end(); ) {
					double d1 = bg::distance(*iter, currPt);
					double d2 = bg::distance(*iter, nextPt);
					if (floatEqual(d1, 0) || floatEqual(d2, 0)) {
						iter = intersectPoints.erase(iter);
					}
					else {
						iter++;
					}
				}
                for (auto& intersectionPt : intersectPoints) {
                    auto currTmpPt = intersectionPt;
					bg::add_point(currTmpPt, currPt);
					bg::divide_value(currTmpPt, 2.0);
					if (withinPoly(currTmpPt, poly, polyString)) {
						return true;
					}

					auto nextTmpPt = intersectionPt;
					bg::add_point(nextTmpPt, nextPt);
					bg::divide_value(nextTmpPt, 2.0);
					if (withinPoly(nextTmpPt, poly, polyString)) {
						return true;
					}
                }
            }
			return false;
		};

		double totalDistance = 0.0;
		for (int idx = 0; idx < tmpLine.size() - 1; idx++) {
			auto& currPt = tmpLine[idx];
			auto& nextPt = tmpLine[idx + 1];
			totalDistance += bg::distance(currPt, nextPt);
		}

		double inPolyDistance = 0.0;
		for (auto poly : polys) {
            if (!withinOrCrossPoly(tmpLine, poly))
                continue;
			for (int idx = 0; idx < tmpLine.size() - 1; idx++) {
				auto& currPt = tmpLine[idx];
				auto& nextPt = tmpLine[idx + 1];
				linestring_2t clipper;
				clipper.push_back(currPt);
				clipper.push_back(nextPt);
				std::vector<point_2t> intersectPoints;
				bg::intersection(poly, clipper, intersectPoints);
				for (auto iter = intersectPoints.begin(); iter != intersectPoints.end(); ) {
					double d1 = bg::distance(*iter, currPt);
					double d2 = bg::distance(*iter, nextPt);
					if (floatEqual(d1, 0) || floatEqual(d2, 0)) {
						iter = intersectPoints.erase(iter);
					}
					else {
						iter++;
					}
				}

				if (intersectPoints.empty())
				{
					if (bg::covered_by(currPt, poly) && bg::covered_by(nextPt, poly)) {
						inPolyDistance += bg::distance(currPt, nextPt);
					}
				}
				else if (intersectPoints.size() == 1)
				{
					if (bg::covered_by(currPt, poly)) {
						inPolyDistance += bg::distance(currPt, intersectPoints[0]);
					}
					if (bg::covered_by(nextPt, poly)) {
						inPolyDistance += bg::distance(nextPt, intersectPoints[0]);
					}
				}
				else if (intersectPoints.size() == 2)
				{
					inPolyDistance += bg::distance(intersectPoints[0], intersectPoints[1]);
				}

			}
		}
		return inPolyDistance / totalDistance;
    }

    void GuardrailCompiler::createGuardrail(
        std::vector<RdsObject*>& rdsObjects,
        RdsTile* pTile,
        std::vector<LineString3d>& lines)
    {
        for (auto& originLine : lines)
        {
            RdsGuardrail* pGuardrail = (RdsGuardrail*)createObject(pTile, EntityType::RDS_GUARDRAIL);
            RDS::LineString3d line;
            convert(originLine, line);
            pGuardrail->location = line;
            rdsObjects.push_back(pGuardrail);
        }
    }

    std::vector<guardrailPoint> GuardrailCompiler::buildGuardrailPoints(
        const std::vector<MapPoint3D64>& originPoints)
    {
        std::vector<guardrailPoint> tmpGuardrailPoints;
        if (originPoints.empty())
            return tmpGuardrailPoints;
        double tmpLength = 0;
        tmpGuardrailPoints.emplace_back(originPoints.front(), 0.0, 0);
        for (size_t i = 1; i < originPoints.size(); ++i)
        {
            double twoPointsDis = bg::distance(POINT_T(originPoints[i - 1]), POINT_T(originPoints[i]));
            tmpLength += twoPointsDis;
            tmpGuardrailPoints.emplace_back(originPoints[i], tmpLength, i);
        }
        return tmpGuardrailPoints;
    }

    LineString3d GuardrailCompiler::getNewGuardrailByFraction(
        double startFractionDis,
        double endFractionDis,
        std::vector<guardrailPoint> originGuardrails,
        std::vector<MapPoint3D64>& originPoints)
    {
        LineString3d tmpLine;
        std::vector<MapPoint3D64> tmpPoints;
        if (originPoints.empty())
            return tmpLine;
        point_t sp;
        bg::line_interpolate(LINESTRING_T(originPoints), startFractionDis, sp);
        originGuardrails.emplace_back(MapPoint3D64_make(sp.get<0>(), sp.get<1>(), sp.get<2>() / 10), startFractionDis, -1);
        point_t ep;
        bg::line_interpolate(LINESTRING_T(originPoints), endFractionDis, ep);
        originGuardrails.emplace_back(MapPoint3D64_make(ep.get<0>(), ep.get<1>(), ep.get<2>() / 10), endFractionDis, -2);
        std::sort(originGuardrails.begin(), originGuardrails.end(), [&](guardrailPoint& a, guardrailPoint& b)->bool { return a._fractionDistance < b._fractionDistance; });
        for (auto it = std::begin(originGuardrails); it != std::end(originGuardrails); ++it)
        {
            if (it->_id == -1)
            {
                tmpPoints.push_back(it->_originPoint);
                while (it != std::end(originGuardrails))
                {
                    it++;
                    tmpPoints.push_back(it->_originPoint);
                    if (it->_id == -2)
                        break;
                };
                break;
            }
        }
        tmpLine.vertexes = tmpPoints;
        return tmpLine;
    }

    void GuardrailCompiler::setBoundaryGuardrailSegments(
        const LineString3d& originLine, 
        std::vector<segment_2t>& guardrailSegments)
    {
		for (size_t i = 0; i < originLine.vertexes.size() - 1; ++i)
		{
			segment_2t tmpSeg(POINT_2T(originLine.vertexes[i]), POINT_2T(originLine.vertexes[i + 1]));
            guardrailSegments.push_back(tmpSeg);
		}
    }

    bool GuardrailCompiler::existGuardrailSegment(
        const point_2t& pa, const point_2t& pb,
        const rtree_type_2segment& guardrailRtree,
        const std::vector<segment_2t>& guardrailSegments)
    {
		double tmpDis = 1.0e10;
		double ptsDis = 1.0e10;
        segment_2t pabSeg{pa, pb};
        const segment_2t* nearestSeg = nullptr;
        guardrailRtree.query(bgi::nearest(pabSeg, 6),
			boost::make_function_output_iterator([&](size_t const& id) {
				segment_2t tmpSeg;
				auto& guardrailSegment = guardrailSegments[id];
				bg::closest_points(pa, guardrailSegment, tmpSeg);
				double dist = bg::length(tmpSeg);
                bg::closest_points(pb, guardrailSegment, tmpSeg);
                dist = min(dist, bg::length(tmpSeg));
				if (dist == 0.0)
				{
					double segmentPointDis = 0.0;
					bg::closest_points(guardrailSegment.first, pabSeg, tmpSeg);
                    segmentPointDis += bg::length(tmpSeg);
					bg::closest_points(guardrailSegment.second, pabSeg, tmpSeg);
                    segmentPointDis += bg::length(tmpSeg);
					if (segmentPointDis < ptsDis)
					{
                        nearestSeg = &guardrailSegment;
						ptsDis = segmentPointDis;
						tmpDis = 0.0;
					}
				}
				else if (dist < tmpDis)
				{
					tmpDis = dist;
                    nearestSeg = &guardrailSegment;
				}
			})
		);
        const static double tolerance = 1000;
        return ptsDis < tolerance && nearestSeg != nullptr;
    }

    bool GuardrailCompiler::isExistBarrierObjects(
        const LineString3d& line,
        const SegmentRTree::Ptr& rtreeSegs,
        const std::vector<segment_t>& allBarrierSegs)
    {
        auto grabBarrier = [&](const point_t& p, const point_t& p1, const point_t& p2, size_t& count)-> bool {
            segment_t seg;
            rtreeSegs->query(bgi::nearest(p, 1),
                boost::make_function_output_iterator([&](size_t const& id) {
                    seg = allBarrierSegs[id];
                    }));
            if (std::abs(seg.first.get<2>() - p.get<2>()) > 5000)
                return true;
            auto _p1 = P2_V2(P3_P2(p1));
            auto _p2 = P2_V2(P3_P2(p2));
            double _l1 = bg::distance(_p2, _p1);
            if (_l1 < 1000.0)
                return false;
            vector_2t _v1 = _p2;
            bg::subtract_point(_v1, _p1);
            bg::divide_value(_v1, _l1);
            vector_2t _v2(_v1.get<1>(), -_v1.get<0>());
            point_2t _p3 = P3_P2(p);
            bg::multiply_value(_v2, 3000);
            bg::add_point(_p3, V2_P2(_v2));
            vector_2t _v3(-_v1.get<1>(), _v1.get<0>());
            point_2t _p4 = P3_P2(p);
            bg::multiply_value(_v3, 3000);
            bg::add_point(_p4, V2_P2(_v3));
            segment_2t seg1 = S3_S2(seg);
            segment_2t seg2(_p3, _p4);
            if (bg::intersects(seg1, seg2))
                count++;
            return true;
        };

        auto grabPoint = [&](const linestring_t& line, std::vector<double>& factors, std::vector<point_t>& points)-> void {
            double lineLength = bg::length(line);
            for (auto& f : factors)
            {
                point_t p;
                int64 l = lineLength * f;
                bg::line_interpolate(line, l, p);
                points.push_back(p);
            }
        };

        linestring_t resultLinestring = LINESTRING_T(line.vertexes);
        bg::unique(resultLinestring);
        if (bg::length(resultLinestring) > 2500)
        {
            size_t count = 0;
            if (resultLinestring.size() < 2)
                return false;
            size_t totalSize = resultLinestring.size() - 1;
            for(size_t i = 1; i < resultLinestring.size(); ++i)
            {
                point_t tmp = resultLinestring[i];
                bg::add_point(tmp, resultLinestring[i - 1]);
                bg::divide_value(tmp, 2);
                if (!grabBarrier(tmp, resultLinestring[i - 1], resultLinestring[i], count))
                    totalSize--;
            }

            if (resultLinestring.size() == 2 || resultLinestring.size() == 3)
            {
                std::vector<point_t> ps;
                std::vector<double> pf = std::vector<double>{0.4, 0.5, 0.6};
                grabPoint(resultLinestring, pf, ps);
                totalSize += pf.size();
                for (auto& gp : ps)
                {
                    if (!grabBarrier(gp, resultLinestring[0], resultLinestring[1], count))
                        totalSize--;
                }
            }
            if (totalSize == 0)
                return false;
            if (((double)count / (double)totalSize) < 0.5)
                return false;
            return true;
        }
        return true;
    }

    std::vector<LineString3d> GuardrailCompiler::modifyGuardrailByRdsTriDiversion(
        const std::vector<goreDiversionInfo>& triangleDiversions,
        const std::vector<segment_2t>& guardrailSegments,
        const LineString3d& originLine)
    {
        std::vector<guardrailVertex> tmpVertices;
        std::vector<guardrailEdge> tmpEdges;
        auto guardrailRtree = getBoundarySegmentRTree(guardrailSegments);
        for (size_t i = 0; i < originLine.vertexes.size(); ++i)
            tmpVertices.emplace_back(originLine.vertexes[i], i);
        auto it = std::begin(tmpVertices);
        auto end = std::end(tmpVertices);
        int index = 0;
        for (auto previous = it++; it != end; ++previous, ++it, ++index)
        {
            auto& v0 = *previous;
            auto& v1 = *it;
            tmpEdges.emplace_back(v0, v1, index);
        }
        for (size_t i = 0; i < triangleDiversions.size(); ++i)
        {
            if (std::all_of(tmpEdges.begin(), tmpEdges.end(), [](guardrailEdge& edge) {return edge._id == -1;}))
                continue;

            auto& triangleDiversion = triangleDiversions[i];
            ring_2t tmpDiversionRing = triangleDiversion._deleteGuardrailPoly;
            bg::correct(tmpDiversionRing);
            for (auto& item : tmpEdges)
            {
                double tolerance = 1000;
				auto& firstPt = item._vertices[0]->_point2d;
				auto& secondPt = item._vertices[1]->_point2d;
                if (!isLaDiversion(triangleDiversion._rdsDiversion) && 
                    existGuardrailSegment(firstPt, secondPt, guardrailRtree, guardrailSegments))
                    continue;

				double oneDis = bg::distance(firstPt, tmpDiversionRing);
				double twoDis = bg::distance(secondPt, tmpDiversionRing);
                bool oneVertexInPoly = bg::intersects(firstPt, tmpDiversionRing);
                bool twoVertexInPoly = bg::intersects(secondPt, tmpDiversionRing);
                if (oneVertexInPoly == true && twoVertexInPoly == true)
                {
                    item._id = -1;      // 删除
                }
                else if (oneVertexInPoly == true && twoVertexInPoly == false)
                {
                    if (twoDis < tolerance)
                    {
                        item._id = -1;      // 删除
                    }
                    else
                    {
                        item._id = -2;      // 起点被裁
                        std::vector<linestring_2t> resultSegs;
                        bg::difference(item._line2d, tmpDiversionRing, resultSegs);
                        if (!resultSegs.empty())
                        {
                            for (size_t k = 0; k < 2; ++k)
                            {
                                auto x = resultSegs[0][k].get<0>();
                                auto y = resultSegs[0][k].get<1>();
                                item._vertices[k]->_point2d.set<0>(x);
                                item._vertices[k]->_point2d.set<1>(y);
                                item._vertices[k]->_originPoint.pos.lon = x;
                                item._vertices[k]->_originPoint.pos.lat = y;
                                item._vertices[k]->_originPoint.z = item._vertices[k]->_originPoint.z;
                            }
                        }
                    }
                }
                else if (oneVertexInPoly == false && twoVertexInPoly == true)
                {
                    if (oneDis < tolerance)
                    {
                        item._id = -1;      // 删除
                    }
                    else
                    {
                        item._id = -3;      // 终点被裁
                        std::vector<linestring_2t> resultSegs;
                        bg::difference(item._line2d, tmpDiversionRing, resultSegs);
                        if (!resultSegs.empty())
                        {
                            for (size_t k = 0; k < 2; ++k)
                            {
                                auto x = resultSegs[0][k].get<0>();
                                auto y = resultSegs[0][k].get<1>();
                                item._vertices[k]->_point2d.set<0>(x);
                                item._vertices[k]->_point2d.set<1>(y);
                                item._vertices[k]->_originPoint.pos.lon = x;
                                item._vertices[k]->_originPoint.pos.lat = y;
                                item._vertices[k]->_originPoint.z = item._vertices[k]->_originPoint.z;
                            }
                        }
                    }
                }
                else
                {
                    if (oneDis < tolerance && twoDis < tolerance) {
                        item._id = -1;  // 删除
                    }
                }
            }
        }

        auto itEdge = std::begin(tmpEdges);
        auto endEdge = std::end(tmpEdges);
        for (auto prevEdge = itEdge++; itEdge != endEdge; ++prevEdge, ++itEdge)
        {
            auto& e0 = *prevEdge;
            auto& e1 = *itEdge;
            if (e0._id >= 0 && e1._id >= 0)
            {
                e0.next = &e1;
            }
            else if (e0._id == -2 && e1._id >= 0)
            {
                e0.next = &e1;
            }
            else if (e0._id >= 0 && e1._id == -3)
            {
                e0.next = &e1;
            }
        }

        std::vector<LineString3d> resultLines;
        for (size_t j = 0; j < tmpEdges.size(); ++j)
        {
            LineString3d tmpLine;
            if (tmpEdges[j]._id >= 0 || tmpEdges[j]._id == -2 ||  tmpEdges[j]._id == -3)
            {
                guardrailEdge* currentEdge = &tmpEdges[j];
                tmpLine.vertexes.push_back(currentEdge->_vertices[0]->_originPoint);
                tmpLine.vertexes.push_back(currentEdge->_vertices[1]->_originPoint);
                while (currentEdge->next)
                {
                    tmpLine.vertexes.push_back(currentEdge->next->_vertices[1]->_originPoint);
                    currentEdge = currentEdge->next;
                    j++;
                }
                resultLines.push_back(tmpLine);
            }
          
        }
        return resultLines;
    }

    bool GuardrailCompiler::modifyGuardrailByRdsSingleDiversion(
        const laneGroupInfo& groupInfo,
        const std::vector<singleDiversionInfo>& singleDiversions,
        const LineString3d& originLine,
        const size_t& index,
        std::vector<LineString3d>& resultLines)
    {
        if (originLine.vertexes.size() < 2)
            return false;
        linestring_2t line = LINESTRING_2T(originLine.vertexes);
        std::vector<segment_2t> tmpSegs;
        for (auto it = bg::segments_begin(line); it != bg::segments_end(line); ++it)
            tmpSegs.push_back(segment_2t(*it->first, *it->second));
        auto segRtree = getBoundarySegmentRTree(tmpSegs);
        bg::unique(line);
        ring_2t tmpExpandRing;
        if (!getExpandPolyByOffset(line, 500, tmpExpandRing))
            return false;

        linestring_2t resultLine_totle;
        for (size_t i = 0; i < singleDiversions.size(); ++i)
        {
            auto tmpDiversion = singleDiversions[i];
            if (tmpDiversion._isCirclePoly)
                continue;

            auto tmpDiversionRing = tmpDiversion._originRing;
            ring_2t tmppExpandDiversionRing;
            if (!getExpandPolyByOffset(tmpDiversionRing, 200, tmppExpandDiversionRing))
                continue;

            if (bg::intersects(tmpExpandRing, tmppExpandDiversionRing))
            {
                linestring_2t tmpLine = line;
                ring_2t tmpRing;
                point_2t startOffsetPoint;
                point_2t endOffsetPoint;
                if (index == 0)
                {
                    startOffsetPoint = point_2t(
                        line.front().get<0>() + (int64)(groupInfo._leftVerUnitDir.get<0>() * 1.0e5),
                        line.front().get<1>() + (int64)(groupInfo._leftVerUnitDir.get<1>() * 1.0e5));
                    endOffsetPoint = point_2t(
                        line.back().get<0>() + (int64)(groupInfo._leftVerUnitDir.get<0>() * 1.0e5),
                        line.back().get<1>() + (int64)(groupInfo._leftVerUnitDir.get<1>() * 1.0e5));
                }
                else if (index == 1)
                {

                    startOffsetPoint = point_2t(
                        line.front().get<0>() + (int64)(groupInfo._rightVerUnitDir.get<0>() * 1.0e5),
                        line.front().get<1>() + (int64)(groupInfo._rightVerUnitDir.get<1>() * 1.0e5));
                    endOffsetPoint = point_2t(
                        line.back().get<0>() + (int64)(groupInfo._rightVerUnitDir.get<0>() * 1.0e5),
                        line.back().get<1>() + (int64)(groupInfo._rightVerUnitDir.get<1>() * 1.0e5));
                }
                tmpLine.push_back(endOffsetPoint);
                tmpLine.push_back(startOffsetPoint);
                bg::append(tmpRing, tmpLine);
                bg::correct(tmpRing);
                std::vector<ring_2t> resultRings;
                bg::difference(tmpRing, tmppExpandDiversionRing, resultRings);

                ring_2t maxPolygon;
                if (!resultRings.empty())
                {
                    maxPolygon = resultRings.front();
                    double maxArea = 0.0;
                    if (resultRings.size() > 1)
                    {
                        for (auto temp : resultRings)
                        {
                            double area = 0.0;
                            area = std::abs(bg::area(temp));
                            if (area > maxArea) {
                                maxArea = area;
                                maxPolygon = temp;
                            }
                        }
                    }
                }
            
                if (!resultRings.empty() && !maxPolygon.empty())
                {
                    resultLine_totle.clear();
                    linestring_2t resultLine;
                    bg::correct(maxPolygon);
                    bg::append(resultLine, maxPolygon);
                    linestring_2t firstLine;
                    linestring_2t secondLine;
                    bool isShift = false;
                    if (index == 0)
                    {
                        for (auto& obj : resultLine)
                        {
                            if (!isShift)
                            {
                                firstLine.push_back(obj);
                                if (bg::equals(obj, startOffsetPoint) || bg::equals(obj, endOffsetPoint))
                                {
                                    firstLine.pop_back();
                                    isShift = true;
                                }
                            }
                            else
                            {
                                secondLine.push_back(obj);
                                if (bg::equals(obj, endOffsetPoint) || bg::equals(obj, startOffsetPoint))
                                {
                                    secondLine.pop_back();
                                }
                            }
                        }
                        resultLine.assign(secondLine.begin(), secondLine.end());
                        resultLine.insert(resultLine.end(), firstLine.begin(), firstLine.end());
                    }
                    else if (index == 1)
                    {
                        for (auto& obj : resultLine)
                        {
                            if (!isShift)
                            {
                                secondLine.push_back(obj);
                                if (bg::equals(obj, endOffsetPoint) || bg::equals(obj, startOffsetPoint))
                                {
                                    secondLine.pop_back();
                                    isShift = true;
                                }
                            }
                            else
                            {
                                firstLine.push_back(obj);
                                if (bg::equals(obj, startOffsetPoint) || bg::equals(obj, endOffsetPoint))
                                {
                                    firstLine.pop_back();
                                }
                            }
                        }
                        resultLine.assign(firstLine.begin(), firstLine.end());
                        resultLine.insert(resultLine.end(), secondLine.begin(), secondLine.end());
                    }
                    bg::unique(resultLine);

                    line.clear();

                    line.insert(line.end(), resultLine.begin(), resultLine.end());

                    resultLine_totle.insert(resultLine_totle.end(), resultLine.begin(), resultLine.end());

                }


            }
        }



        if (!resultLine_totle.empty())
        {
            LineString3d resultMapPoints;
            for (auto& obj : resultLine_totle)
            {
                segRtree.query(bgi::nearest(obj, 1),
                    boost::make_function_output_iterator([&](size_t const& id) {
                        segment_2t nearbySeg = tmpSegs[id];
                        segment_2t verSeg;
                        bg::closest_points(obj, nearbySeg, verSeg);

                        double tmpLength = std::abs(bg::distance(nearbySeg.first, verSeg.second)) + std::abs(bg::distance(nearbySeg.second, verSeg.second));// (double)(std::abs(bg::length(nearbySeg)));
                        if (tmpLength == 0.0)
                        {
                            resultMapPoints.vertexes.push_back(MapPoint3D64_make(obj.get<0>(), obj.get<1>(), originLine.vertexes[id].z));
                            return;
                        }
                        double wa = std::abs(bg::distance(nearbySeg.first, verSeg.second)) / tmpLength;
                        double wb = std::abs(bg::distance(nearbySeg.second, verSeg.second)) / tmpLength;
                        double tmpZ = wa * originLine.vertexes[id + 1].z + wb * originLine.vertexes[id].z;
                        resultMapPoints.vertexes.push_back(MapPoint3D64_make(obj.get<0>(), obj.get<1>(), tmpZ));
                        }));
            }
            resultLines.push_back(resultMapPoints);
            return true;
        }



        return false;
    }

    bool GuardrailCompiler::isCirclePoly(ring_2t& tmpRing)
    {
        ring_2t convexRing;
		bg::convex_hull(tmpRing, convexRing);
        double tmpDiversionRingArea = std::abs(bg::area(tmpRing));
        double convexRingArea = std::abs(bg::area(convexRing));
        double tmpDiversionRingAreaRate = tmpDiversionRingArea / convexRingArea;
        return tmpDiversionRingAreaRate < 0.5;
    }

    std::vector<forkGroupInfo> GuardrailCompiler::getForkGroups(
        const std::vector<HadLaneGroup*>& groups)
    {
        std::vector<forkGroupInfo> forkGroups(0);
        const int64 basePointBoundary = 15000;
        for (auto obj : groups)
        {
            HadLaneGroup* pGroup = (HadLaneGroup*)obj;
            if (pGroup->roadBoundaries.size() < 2)
                continue;

            auto currentRoadBoundaries = pGroup->roadBoundaries;
            if (pGroup->next.size() > 1)
            {
                std::vector<MapPoint3D64> tmpPoints;
                std::vector<MapPoint3D64> refPoints;
                if (currentRoadBoundaries.size() < 2)
                    continue;
                for (auto& obj : currentRoadBoundaries)
                {
                    if (directionEqual(obj, pGroup, 3))
                        refPoints.push_back(obj->location.vertexes.front());
                    else
                        refPoints.push_back(obj->location.vertexes.back());
                }
                coordinatesTransform.convert(refPoints.data(), refPoints.size());

                for (auto& obj : pGroup->next)
                {
                    HadLaneGroup* nextGroup = (HadLaneGroup*)(obj);
                    if (nextGroup == nullptr)
                        continue;
                    for (auto& item : nextGroup->roadBoundaries)
                    {
                        if (!item->location.vertexes.empty())
                        {
                            if (directionEqual(item, nextGroup, 3))
                                tmpPoints.push_back(item->location.vertexes.back());
                            else
                                tmpPoints.push_back(item->location.vertexes.front());
                        }
                    }
                }
                coordinatesTransform.convert(tmpPoints.data(), tmpPoints.size());

                std::vector<MapPoint3D64> resultsPoint;
                if (tmpPoints.size() > 3)
                {
                    for (auto& tmpPoint : tmpPoints)
                    {
                        bool isAdd = true;
                        for (auto& tmpRefPoint : refPoints)
                        {
                            if (std::abs(bg::distance(P3_P2(POINT_T(tmpRefPoint)), P3_P2(POINT_T(tmpPoint)))) < 1000)
                            {
                                isAdd = false;
                                break;
                            }
                        }
                        if (isAdd)
                            resultsPoint.push_back(tmpPoint);
                    }
                }

                if (!resultsPoint.empty())
                {
                    forkGroupInfo tmpForkGroup;
                    point_t bp(0, 0);
                    segment_t seg;
                    for (auto& gBasePoint : resultsPoint)
                    {
                        bg::add_point(bp, POINT_T(gBasePoint));
                    }
                    bg::divide_value(bp, resultsPoint.size());
                    vector_t dir1;
                    vector_t dir2;
                    vector_t dirZ(0.0, 0.0, 1.0);
                    bg::convert(POINT_T(refPoints[0]), dir1);
                    bg::convert(POINT_T(refPoints[1]), dir2);
                    bg::subtract_point(dir2, dir1);
                    tmpForkGroup._baseLine = dir2;
                    tmpForkGroup._baseDir = bg::cross_product(dirZ, dir2);
                    tmpForkGroup._basePoint = bp;
                    tmpForkGroup._basePointExpandBox = expandPoint(bp, basePointBoundary);
                    tmpForkGroup._basePointExpandPoly = getExpandPolyByVector(P3_P2(bp), V3_V2(tmpForkGroup._baseLine), V3_V2(tmpForkGroup._baseDir), 1000, basePointBoundary);
                    tmpForkGroup._basePointExpandPoly2 = getExpandPolyByVector_twoway(P3_P2(bp), V3_V2(tmpForkGroup._baseLine), V3_V2(tmpForkGroup._baseDir), 1000, 4000);
                    forkGroups.push_back(tmpForkGroup);
                }
            }

            if (pGroup->previous.size() > 1)
            {
                std::vector<MapPoint3D64> tmpPoints;
                std::vector<MapPoint3D64> refPoints;
                for (auto& obj : currentRoadBoundaries)
                {
                    if (directionEqual(obj, pGroup, 3))
                        refPoints.push_back(obj->location.vertexes.back());
                    else
                        refPoints.push_back(obj->location.vertexes.front());
                }
                coordinatesTransform.convert(refPoints.data(), refPoints.size());

                for (auto& obj : pGroup->previous)
                {
                    HadLaneGroup* prevGroup = (HadLaneGroup*)(obj);
                    if (prevGroup == nullptr)
                        continue;
                    for (auto& item : prevGroup->roadBoundaries)
                    {
                        if (!item->location.vertexes.empty())
                        {
                            if (directionEqual(item, prevGroup, 3))
                                tmpPoints.push_back(item->location.vertexes.front());
                            else
                                tmpPoints.push_back(item->location.vertexes.back());
                        }
                    }
                }
                coordinatesTransform.convert(tmpPoints.data(), tmpPoints.size());

                std::vector<MapPoint3D64> resultsPoint;
                if (tmpPoints.size() > 3)
                {
                    for (auto& tmpPoint : tmpPoints)
                    {
                        bool isAdd = true;
                        for (auto& tmpRefPoint : refPoints)
                        {
                            if (std::abs(bg::distance(P3_P2(POINT_T(tmpRefPoint)), P3_P2(POINT_T(tmpPoint)))) < 1000)
                            {
                                isAdd = false;
                                break;
                            }
                        }
                        if (isAdd)
                            resultsPoint.push_back(tmpPoint);
                    }
                }

                if (!resultsPoint.empty())
                {
                    forkGroupInfo tmpForkGroup;
                    point_t bp(0, 0);
                    for (auto& gBasePoint : resultsPoint)
                    {
                        bg::add_point(bp, POINT_T(gBasePoint));
                    }
                    bg::divide_value(bp, resultsPoint.size());
                    vector_t dir1;
                    vector_t dir2;
                    vector_t dirZ(0.0, 0.0, 1.0);
                    bg::convert(POINT_T(refPoints[0]), dir1);
                    bg::convert(POINT_T(refPoints[1]), dir2);
                    bg::subtract_point(dir2, dir1);
                    tmpForkGroup._baseLine = dir2;
                    tmpForkGroup._baseDir = bg::cross_product(dir2, dirZ);
                    tmpForkGroup._basePoint = bp;
                    tmpForkGroup._basePointExpandBox = expandPoint(bp, basePointBoundary);
                    tmpForkGroup._basePointExpandPoly = getExpandPolyByVector(P3_P2(bp), V3_V2(tmpForkGroup._baseLine), V3_V2(tmpForkGroup._baseDir), 1000, basePointBoundary);
                    tmpForkGroup._basePointExpandPoly2 = getExpandPolyByVector_twoway(P3_P2(bp), V3_V2(tmpForkGroup._baseLine), V3_V2(tmpForkGroup._baseDir), 1000, 4000);
                    forkGroups.push_back(tmpForkGroup);
                }
            }
        }
        return forkGroups;
    }

    box_t GuardrailCompiler::expandPoint(
        point_t p,
        int64 len)
    {
        box_t tmp;
        linestring_t tmpR;
        int64 heightTol = 3000;
        tmpR.push_back(point_t(len + p.get<0>(), p.get<1>(), p.get<2>()));
        tmpR.push_back(point_t(-len + p.get<0>(), p.get<1>(), p.get<2>()));
        tmpR.push_back(point_t(p.get<0>(), len + p.get<1>(), p.get<2>()));
        tmpR.push_back(point_t(p.get<0>(), -len + p.get<1>(), p.get<2>()));
        tmpR.push_back(point_t(len + p.get<0>(), p.get<1>(), p.get<2>() + heightTol));
        tmpR.push_back(point_t(-len + p.get<0>(), p.get<1>(), p.get<2>() + heightTol));
        tmpR.push_back(point_t(p.get<0>(), len + p.get<1>(), p.get<2>() + heightTol));
        tmpR.push_back(point_t(p.get<0>(), -len + p.get<1>(), p.get<2>() + heightTol));
        tmpR.push_back(point_t(len + p.get<0>(), p.get<1>(), p.get<2>() - heightTol));
        tmpR.push_back(point_t(-len + p.get<0>(), p.get<1>(), p.get<2>() - heightTol));
        tmpR.push_back(point_t(p.get<0>(), len + p.get<1>(), p.get<2>() - heightTol));
        tmpR.push_back(point_t(p.get<0>(), -len + p.get<1>(), p.get<2>() - heightTol));
        bg::envelope(tmpR, tmp);
        return tmp;
    }

    box_t GuardrailCompiler::expandBoxZValue(box_t b, int64 zVal)
    {
        auto& min_corner = b.min_corner();
        auto& max_corner = b.max_corner();
        if (min_corner.get<2>() <= max_corner.get<2>()) 
        {
            min_corner.set<2>(min_corner.get<2>() - zVal);
            max_corner.set<2>(max_corner.get<2>() + zVal);
        }
        else
        {
            min_corner.set<2>(min_corner.get<2>() + zVal);
            max_corner.set<2>(max_corner.get<2>() - zVal);
        }
        return b;
    }

    void GuardrailCompiler::updateGoreDiversionInfo(
        goreDiversionInfo& goreDiversion, ring_2t _basePointExpandPoly)
    {
        if (goreDiversion._rdsDiversion->id == 2392318903104569347)
        {
            printf("");
        }
        std::vector<kneePoint> pts;
        ring_2t gorePoly = goreDiversion._originRing;
        linestring_2t linePoly;
        bg::unique(gorePoly);
        size_t n = gorePoly.size();
        if (n < 3)
            return;
        std::vector<vector_2t> tmpDirs;
        if (isKneePoint(gorePoly[0], gorePoly[1], gorePoly[n - 1], 0.7))
        {
            kneePoint tmpKneePoint;
            tmpKneePoint._point = gorePoly[0];
            tmpKneePoint._prevSeg = segment_2t(gorePoly[1], gorePoly[0]);
            tmpKneePoint._nextSeg = segment_2t(gorePoly[n -1], gorePoly[1]);
            pts.push_back(tmpKneePoint);
        }
        for (size_t i = 1; i < n - 1; ++i)
        {
            if (isKneePoint(gorePoly[i], gorePoly[i - 1], gorePoly[i + 1], 0.7))
            {
                kneePoint tmpKneePoint;
                tmpKneePoint._point = gorePoly[i];
                tmpKneePoint._prevSeg = segment_2t(gorePoly[i - 1], gorePoly[i]);
                tmpKneePoint._nextSeg = segment_2t(gorePoly[i], gorePoly[i + 1]);
                pts.push_back(tmpKneePoint);
            }
        }
        std::sort(pts.begin(), pts.end(), [&](const kneePoint& a, const kneePoint& b)->bool {
            return bg::comparable_distance(a._point, P3_P2(goreDiversion._basePoint)) < bg::comparable_distance(b._point, P3_P2(goreDiversion._basePoint));
            });
        for (auto it = pts.begin(); it != pts.end() - 1;)
        {
            if (std::abs(bg::distance((*it)._point, (*(it + 1))._point)) < 1000)
                it = pts.erase(it);
            else
                it++;
        }
        if (pts.size() > 1)
        {
            auto& ptsFront = pts.front();
            auto allPtsDis = bg::distance(ptsFront._point, pts.back()._point);
            for (auto ptsIdx = 1; ptsIdx < pts.size(); ptsIdx++)
            {
                auto ptsDis = bg::distance(ptsFront._point, pts[ptsIdx]._point);
                if (ptsDis / allPtsDis > 0.5)
                {
                    goreDiversion._endPoint = pts[ptsIdx]._point;
                    break;
                }
            }
            ring_2t tmpPoly = getExpandPolyByVector(goreDiversion._endPoint, V3_V2(goreDiversion._baseLine), V3_V2(goreDiversion._baseDir), 1e7, 1e7);
            std::vector<ring_2t> resultPolys;
            bg::difference(gorePoly, tmpPoly, resultPolys);
            if (resultPolys.empty())
                return;
            ring_2t tmpExpandRing;
            if (getExpandPolyByOffset(resultPolys.front(), 200, tmpExpandRing))
            {
                std::vector<ring_2t> _deleteGuardrailPoly;;
                bg::union_(tmpExpandRing, _basePointExpandPoly, _deleteGuardrailPoly);
                if (_deleteGuardrailPoly.size() == 1)
                {
                    goreDiversion._deleteGuardrailPoly = _deleteGuardrailPoly[0];
                    goreDiversion._isDeleteGuardrail = true;
                }
            }
            return;
        }
        return;
    }

    ring_2t GuardrailCompiler::getExpandPolyByVector(
        const point_2t& basePoint,
        const vector_2t& dirH,
        const vector_2t& dirV,
        const double width,
        const double length)
    {
        ring_2t ring;
        point_2t p1, p2, p3, p4;
        vector_2t dirHN = V2_N(dirH);
        vector_2t dirVN = V2_N(dirV);
        vector_2t v1 = dirHN;
        vector_2t v2 = dirHN;
        vector_2t v3 = dirVN;
        vector_2t v4 = dirVN;
        bg::multiply_value(v1, width);
        bg::multiply_value(v3, length);
        bg::add_point(v3, v1);
        bg::multiply_value(v2, -1.0);
        bg::multiply_value(v2, width);
        bg::multiply_value(v4, length);
        bg::add_point(v4, v2);
        bg::strategy::transform::translate_transformer<double, 2, 2> trans1(v1.get<0>(), v1.get<1>());
        bg::strategy::transform::translate_transformer<double, 2, 2> trans2(v2.get<0>(), v2.get<1>());
        bg::strategy::transform::translate_transformer<double, 2, 2> trans3(v3.get<0>(), v3.get<1>());
        bg::strategy::transform::translate_transformer<double, 2, 2> trans4(v4.get<0>(), v4.get<1>());
        bg::transform(basePoint, p1, trans1);
        bg::transform(basePoint, p2, trans2);
        bg::transform(basePoint, p3, trans3);
        bg::transform(basePoint, p4, trans4);
        ring.push_back(p1);
        ring.push_back(p2);
        ring.push_back(p4);
        ring.push_back(p3);
        bg::correct(ring);
        return ring;
    }

    ring_2t GuardrailCompiler::getExpandPolyByVector_twoway(
        const point_2t& basePoint,
        const vector_2t& dirH,
        const vector_2t& dirV,
        const double width,
        const double length)
    {
        ring_2t ring;
        point_2t p1, p2, p3, p4;
        vector_2t dirHN = V2_N(dirH);
        vector_2t dirVN = V2_N(dirV);
        vector_2t v1 = dirHN;
        vector_2t v2 = dirHN;
        vector_2t v3 = dirVN;
        vector_2t v4 = dirVN;
        vector_2t v5 = dirVN;
        vector_2t v6 = dirVN;
        bg::multiply_value(v1, width);
        bg::multiply_value(v3, length);
        bg::add_point(v3, v1);
        bg::multiply_value(v2, -1.0);
        bg::multiply_value(v2, width);
        bg::multiply_value(v4, length);
        bg::add_point(v4, v2);

        bg::multiply_value(v5, -1.0);
        bg::multiply_value(v5, length);
        bg::add_point(v5, v1);

        bg::multiply_value(v6, -1.0);
        bg::multiply_value(v6, length);
        bg::add_point(v6, v2);

        bg::strategy::transform::translate_transformer<double, 2, 2> trans3(v3.get<0>(), v3.get<1>());
        bg::strategy::transform::translate_transformer<double, 2, 2> trans4(v4.get<0>(), v4.get<1>());
        bg::strategy::transform::translate_transformer<double, 2, 2> trans5(v5.get<0>(), v5.get<1>());
        bg::strategy::transform::translate_transformer<double, 2, 2> trans6(v6.get<0>(), v6.get<1>());

        bg::transform(basePoint, p1, trans3);
        bg::transform(basePoint, p2, trans4);
        bg::transform(basePoint, p3, trans6);
        bg::transform(basePoint, p4, trans5);
        ring.push_back(p1);
        ring.push_back(p2);
        ring.push_back(p3);
        ring.push_back(p4);
        bg::correct(ring);
        return ring;
    }

    bool GuardrailCompiler::getExpandPolyByOffset(
        const linestring_2t& line,
        const int64& offsetSize,
        ring_2t& resultRing)
    {
        multi_polygon_2t bufferResults;
        bg::strategy::buffer::distance_symmetric<int64> distance_strategy(offsetSize);
        bg::buffer(line, bufferResults, distance_strategy, side_strategy, join_strategy, end_strategy, circle_strategy);
        if (bufferResults.empty())
            return false;
        resultRing = bufferResults[0].outer();
        return true;
    }

    bool GuardrailCompiler::getExpandPolyByOffset(
        const ring_2t& ring,
        const int64& offsetSize,
        ring_2t& resultRing)
    {
        multi_polygon_2t bufferResults;
        bg::strategy::buffer::distance_symmetric<int64> distance_strategy(offsetSize);
        bg::buffer(ring, bufferResults, distance_strategy, side_strategy, join_strategy, end_strategy, circle_strategy);
        if (bufferResults.empty())
            return false;
        resultRing = bufferResults[0].outer();
        return true;
    }

    std::vector<HadLaneGroup*> GuardrailCompiler::getGridAllGroups(
        HadGrid* const pGrid)
    {
        std::vector<HadLaneGroup*> allGroups;
        for (auto obj : pGrid->query(ElementType::HAD_LANE_GROUP))
        {
            HadLaneGroup* pGroup = (HadLaneGroup*)obj;
            allGroups.push_back(pGroup);
            if (pGroup->crossGrid)
            {
                int count = 0;
                int totalNum = 5;
                getPrevGroups(pGroup, allGroups, totalNum, count);
                count = 0;
                getNextGroups(pGroup, allGroups, totalNum, count);
            }
        }
        std::set<HadLaneGroup*> groupSets(allGroups.begin(), allGroups.end());
        allGroups.assign(groupSets.begin(), groupSets.end());
        return allGroups;
    }

    void GuardrailCompiler::getPrevGroups(
        HadLaneGroup* const currentGroup,
        std::vector<HadLaneGroup*>& resultGroups,
        const int& num,
        int& count)
    {
        if (count > num)
            return;
        for (const auto& prev : currentGroup->previous)
        {
            count++;
            HadLaneGroup* tmpPrev = (HadLaneGroup*)prev;
            resultGroups.push_back(tmpPrev);
            getPrevGroups(tmpPrev, resultGroups, num, count);
        }
    }

    void GuardrailCompiler::getNextGroups(
        HadLaneGroup* const currentGroup,
        std::vector<HadLaneGroup*>& resultGroups,
        const int& num,
        int& count)
    {
        if (count > num)
            return;
        for (const auto& next : currentGroup->next)
        {
            count++;
            HadLaneGroup* tmpNext = (HadLaneGroup*)next;
            resultGroups.push_back(tmpNext);
            getNextGroups(tmpNext, resultGroups, num, count);
        }
    }

    void GuardrailCompiler::setLaneGroupData(
        const std::vector<HadLaneGroup*>& groups)
    {
        for (auto& pGroup : groups)
        {
            if (!pGroup->roadBoundaries.empty())
            {
                if (pGroup->originId == 186431113724903425)
                {
                    printf("");
                }
                laneGroupInfo tmp;
                std::vector<MapPoint3D64> groupPoints;
                std::vector<MapPoint3D64> leftBoundary(pGroup->roadBoundaries[0]->location.vertexes.begin(), pGroup->roadBoundaries[0]->location.vertexes.end());
                if (directionEqual(pGroup->roadBoundaries[0], pGroup, 3))
                    std::reverse(leftBoundary.begin(), leftBoundary.end());
                for (size_t i = 0; i < pGroup->roadBoundaries.size(); ++i)
                {
                    std::vector<MapPoint3D64> tmpRoadVertexes(pGroup->roadBoundaries[i]->location.vertexes.begin(), pGroup->roadBoundaries[i]->location.vertexes.end());
                    if (directionEqual(pGroup->roadBoundaries[i], pGroup, 3))
                        std::reverse(tmpRoadVertexes.begin(), tmpRoadVertexes.end());
                    if (i % 2 == 1)
                        std::reverse(tmpRoadVertexes.begin(), tmpRoadVertexes.end());
                    groupPoints.insert(groupPoints.end(), tmpRoadVertexes.begin(), tmpRoadVertexes.end());
                }
                coordinatesTransform.convert(groupPoints.data(), groupPoints.size());
                tmp._laneGroup = pGroup;
                tmp._groupBox = BOX_T(groupPoints);
                tmp._groupPoly = RING_2T(groupPoints);
                tmp._groupPoints = LINESTRING_T(groupPoints);
                bg::correct(tmp._groupPoly);
                bg::centroid(tmp._groupBox, tmp._centerPoint);
                point_t pa = POINT_T(leftBoundary.front());
                point_t pb = POINT_T(leftBoundary.back());
                tmp._parallelDir = vector_t(pb.get<0>() - pa.get<0>(), pb.get<1>() - pa.get<1>(), pb.get<2>() - pa.get<2>());
                vector_t tmpUnitDir = V3_N(tmp._parallelDir);
                vector_t tmpVerUnitDirA = bg::cross_product(vector_t(0.0, 0.0, 1.0), tmpUnitDir);
                vector_t tmpVerUnitDirB = bg::cross_product(tmpUnitDir, vector_t(0.0, 0.0, 1.0));
                tmp._leftVerUnitDir = tmpVerUnitDirA;
                tmp._rightVerUnitDir = tmpVerUnitDirB;
                double expandDis = 30000.0;
                point_t tmpPointA = point_t(
                    tmp._centerPoint.get<0>() + tmpVerUnitDirA.get<0>() * expandDis,
                    tmp._centerPoint.get<1>() + tmpVerUnitDirA.get<1>() * expandDis,
                    tmp._centerPoint.get<2>() + tmpVerUnitDirA.get<2>() * expandDis);
                point_t tmpPointB = point_t(
                    tmp._centerPoint.get<0>() + tmpVerUnitDirB.get<0>() * expandDis,
                    tmp._centerPoint.get<1>() + tmpVerUnitDirB.get<1>() * expandDis,
                    tmp._centerPoint.get<2>() + tmpVerUnitDirB.get<2>() * expandDis);
                tmp._verticalSegment = segment_t(tmpPointA, tmpPointB);
                allGroupDatas.push_back(tmp);
                allGroupBoxes.push_back(tmp._groupBox);
                allGroupMaps.emplace(pGroup->originId, tmp);
            }
        }
    }

    void GuardrailCompiler::setBarrierData(
        HadGrid* const pGrid)
    {
        for (auto obj : pGrid->query(ElementType::HAD_LANE_GROUP))
        {
            HadLaneGroup* pGroup = (HadLaneGroup*)obj;
            for (size_t i = 0; i < pGroup->objects.size(); i++)
            {
                std::vector<LineString3d> tmpLines;
                HadObject* pObject = pGroup->objects[i];
                if (pObject->objectType == ElementType::HAD_OBJECT_BARRIER)
                {
                    tmpLines = ((HadBarrier*)pObject)->location.lines;
                }
                else if (pObject->objectType == ElementType::HAD_OBJECT_WALL)
                {
                    tmpLines = ((HadWall*)pObject)->location.lines;
                }

                if (!tmpLines.empty()) 
                {
                    setBarrierObjects(pGroup, pObject, tmpLines);
                    allBarrierLines.insert(allBarrierLines.end(), tmpLines.begin(), tmpLines.end());
                }
            }
        }
        for (auto& item : allBarrierLines)
        {
            if (item.vertexes.size() > 1)
            {
                auto vertexes = item.vertexes;
                coordinatesTransform.convert(vertexes.data(), vertexes.size());
                for (size_t i = 1; i < vertexes.size(); ++i)
                {
                    allBarrierSegs.emplace_back(segment_t(POINT_T(vertexes[i - 1]), POINT_T(vertexes[i])));
                }
            }
        }
    }

    void GuardrailCompiler::setBarrierObjects(HadLaneGroup* pGroup, HadObject* pObject, std::vector<LineString3d>& tmpLines)
    {
        if (pGroup->roadBoundaries.size() != 2 || tmpLines.empty())
            return;

        auto& tmpLine = tmpLines[0];
        double nearbyDistance = DBL_MAX;
        HadRoadBoundary* nearbyBoundary = nullptr;
        for (auto pBoundary : pGroup->roadBoundaries) {
            MapPoint3D64 frontPt = pBoundary->location.vertexes.front();
            if (directionEqual(pBoundary, pGroup, 3)) {
                frontPt = pBoundary->location.vertexes.back();
            }
			size_t si, ei;
			MapPoint3D64 grappedPt = {};
			GrapPointAlgorithm::grapOrMatchNearestPoint(frontPt, tmpLine.vertexes, grappedPt, si, ei);
            double distance = frontPt.pos.distance(grappedPt.pos);
            if (distance < nearbyDistance) {
                nearbyBoundary = pBoundary;
                nearbyDistance = distance;
            }
        }
        if (nearbyBoundary) {
            nearbyBoundary->barrierObjects.push_back(pObject);
        }
    }

    void GuardrailCompiler::setDiversionData(
        RdsTile* const pTile)
    {
        std::vector<RdsObject*> pRdsDiversionupObjects = pTile->query(RDS::EntityType::RDS_DIVERSION);
        for (auto p : pRdsDiversionupObjects)
        {
            RdsDiversion* tmpDiversion = (RdsDiversion*)p;
            gridDiversions.push_back(tmpDiversion);
            OMDB::Polygon3d tmpDiversionPoly;
            convert(tmpDiversion->contour, tmpDiversionPoly);
            coordinatesTransform.convert(tmpDiversionPoly.vertexes.data(), tmpDiversionPoly.vertexes.size());
            gridDiversionPolys.push_back(tmpDiversionPoly);
            gridDiversionBoxes.push_back(BOX_T(tmpDiversionPoly.vertexes));
        }

    }

    void GuardrailCompiler::setLaRoadFaceData(RdsTile* const pTile)
    {
		std::vector<RdsObject*> pRdsLaRoadObjects = pTile->query(RDS::EntityType::RDS_ROAD);
		for (auto p : pRdsLaRoadObjects)
		{
            RdsRoad* tmpRoad = (RdsRoad*)p;
            if (tmpRoad->roadType == RdsRoad::RoadType::LA) {
				OMDB::MultiLineString3d tmpLaEdges;
				convert(tmpRoad->contour, tmpLaEdges);
                if (tmpLaEdges.lines.size() != 2) {
                    continue;
                }

                OMDB::Polygon3d tmpDiversionPoly;
                LineString3d& rightSide = tmpLaEdges.lines[1];
                std::reverse(rightSide.vertexes.begin(), rightSide.vertexes.end());
                buildLaPolygon(tmpLaEdges.lines, tmpDiversionPoly);

                // 用LA面构造一个导流带
				auto newDiversion = std::make_shared<RdsDiversion>();
                newDiversion->diversionType = RdsDiversion::DiversionType::GORE;
                convert(tmpDiversionPoly, newDiversion->contour);
                newDiversion->id = tmpRoad->id;
                gridLaDiversions.push_back(newDiversion);

                RdsDiversion* tmpDiversion = newDiversion.get();
                gridDiversions.push_back(tmpDiversion);
				coordinatesTransform.convert(tmpDiversionPoly.vertexes.data(), tmpDiversionPoly.vertexes.size());
				gridDiversionPolys.push_back(tmpDiversionPoly);
				gridDiversionBoxes.push_back(BOX_T(tmpDiversionPoly.vertexes));
            }
		}

    }

    void GuardrailCompiler::setLaDiversionData()
    {
        auto doUpdateGoreDiversionInfo = [&](RdsDiversion* rdsDiversion, Polygon3d& tmpDiversionPoly) {
			coordinatesTransform.convert(tmpDiversionPoly.vertexes.data(), tmpDiversionPoly.vertexes.size());
			ring_2t tmpRing = RING_2T(tmpDiversionPoly.vertexes);
			bg::correct(tmpRing);

			goreDiversionInfo tmpDiversionInfo;
			tmpDiversionInfo._rdsDiversion = rdsDiversion;
			tmpDiversionInfo._diversionPoly = tmpDiversionPoly;
			tmpDiversionInfo._originRing = tmpRing;
			tmpDiversionInfo._isDeleteGuardrail = true;
			tmpDiversionInfo._deleteGuardrailPoly = tmpRing;
			allGoreDiversions.push_back(tmpDiversionInfo);
        };

        // LA关系构建的三角岛
		for (auto gridDiversion : gridDiversions)
		{
            if (gridDiversion->isLaDiversion) {
				OMDB::Polygon3d tmpDiversionPoly;
				convert(gridDiversion->contour, tmpDiversionPoly);
				doUpdateGoreDiversionInfo(gridDiversion, tmpDiversionPoly);
            }
		}

        // LA关系构建的路面
		for (auto laDiversion : gridLaDiversions)
		{
			OMDB::Polygon3d tmpDiversionPoly;
			convert(laDiversion->contour, tmpDiversionPoly);
            doUpdateGoreDiversionInfo(laDiversion.get(), tmpDiversionPoly);
		}
    }

    void GuardrailCompiler::setGoreDiversionData()
    {
        auto doUpdateGoreDiversionInfo = [&](forkGroupInfo& obj, std::vector<size_t>& ids, std::vector<ring_2t>& rings) {
			goreDiversionInfo tmpDiversionInfo;
			tmpDiversionInfo._rdsDiversion = gridDiversions[ids.front()];
			tmpDiversionInfo._diversionPoly = gridDiversionPolys[ids.front()];
			tmpDiversionInfo._basePoint = obj._basePoint;
			tmpDiversionInfo._baseLine = obj._baseLine;
			tmpDiversionInfo._baseDir = obj._baseDir;
			tmpDiversionInfo._originRing = rings.front();
			updateGoreDiversionInfo(tmpDiversionInfo, obj._basePointExpandPoly2);
			allGoreDiversions.push_back(tmpDiversionInfo);
        };

        auto fillIntersectPolys = [&](box_t& bbox, ring_2t& poly,
            std::vector<size_t>& ids, std::vector<ring_2t>& idRings,
            std::vector<size_t>& las, std::vector<ring_2t>& laRings) {
				getDiversionBoxRTree()->query(bgi::intersects(bbox),
					boost::make_function_output_iterator([&](size_t const& id) {
                        if (std::find(ids.begin(), ids.end(), id) == ids.end() && std::find(las.begin(), las.end(), id) == las.end())
                        {
							std::vector<ring_2t> results;
							ring_2t tmpRing = RING_2T(gridDiversionPolys[id].vertexes);
							bg::correct(tmpRing);
							bg::intersection(poly, tmpRing, results);
							if (!results.empty())
							{
								auto gridDiversion = gridDiversions[id];
								if (!isLaDiversion(gridDiversion))
								{
									ids.push_back(id);
									idRings.push_back(tmpRing);
								}
								else
								{
									las.push_back(id);
									laRings.push_back(tmpRing);
								}
							}
                        }
					}));
        };

        for (auto& obj : allForkGroups)
        {
            std::vector<size_t> ids;
            std::vector<ring_2t> idRings;

			std::vector<size_t> las;
			std::vector<ring_2t> laRings;
            fillIntersectPolys(obj._basePointExpandBox, obj._basePointExpandPoly, ids, idRings, las, laRings);
			if (ids.size() == 1)
			{
                ring_2t tmpRing = RING_2T(gridDiversionPolys[ids.front()].vertexes);
				ring_2t tmpExpandRing;
				bg::correct(tmpRing);
                if (getExpandPolyByOffset(tmpRing, 3000, tmpExpandRing)) {
                    box_t tmpBox = BOX_T(gridDiversionPolys[ids.front()].vertexes);
                    tmpBox = expandBoxZValue(tmpBox, 500); // 解决两个box高程差的问题
                    fillIntersectPolys(tmpBox, tmpExpandRing, ids, idRings, las, laRings);
                }
			}

            if (ids.size() == 1)
            {
                doUpdateGoreDiversionInfo(obj, ids, idRings);
            }

			if (las.size() == 1)
			{
				doUpdateGoreDiversionInfo(obj, las, laRings);
			}
        }
    }

	bool GuardrailCompiler::isLaDiversion(RdsDiversion* rdsDiversion)
	{
        if (rdsDiversion->isLaDiversion)
            return true;

        for (auto laDiversion : gridLaDiversions) 
        {
            if (laDiversion.get() == rdsDiversion)
            {
                return true;
            }
        }
		return false;
	}

    void GuardrailCompiler::setSingleDiversionData()
    {
        auto isMiddleLaneGroup = [&](HadLaneGroup* pGroup)->bool 
        {
            for (auto pBoundary : pGroup->roadBoundaries) 
            {
				//convert 
				LineString3d originLine = pBoundary->location;
				coordinatesTransform.convert(originLine.vertexes.data(), originLine.vertexes.size());
				if (directionEqual(pBoundary, pGroup, 3))
				{
					std::reverse(originLine.vertexes.begin(), originLine.vertexes.end());
				}
                if (isMiddleRoadBoundary(pGroup, pBoundary, originLine))
                {
                    return true;
                }
            }
            return false;
        };

        for (size_t diversionIndex = 0; diversionIndex < gridDiversions.size(); ++diversionIndex)
        {
            auto obj = gridDiversions[diversionIndex];
            if (obj->id == 2392469063751172140)
            {
                printf("");
            }
            std::vector<size_t> intersectGroupIds;
            std::vector<ring_2t> results;
            ring_2t tmpExpandRing;
            ring_2t tmpRing = RING_2T(gridDiversionPolys[diversionIndex].vertexes);
            bg::correct(tmpRing);
            if (!getExpandPolyByOffset(tmpRing, 500, tmpExpandRing))
                continue;
            point_t tmpPoint;
            bool isDeleteDiversion = true;
            linestring_t tmpLine3D = LINESTRING_T(gridDiversionPolys[diversionIndex].vertexes);
            bg::centroid(tmpLine3D, tmpPoint);
            getLaneGroupBoxRTree()->query(bgi::intersects(gridDiversionBoxes[diversionIndex]),
                boost::make_function_output_iterator([&](size_t const& id) {
                    bg::intersection(allGroupDatas[id]._groupPoly, tmpRing, results);
                    if (!results.empty())
                    {
                        double totalArea = 0.0;
                        for (auto obj : results)
                            totalArea += bg::area(obj);
                        if(totalArea > 1.0e4)
                            isDeleteDiversion = false;
                    }
                    results.clear();
                    bg::intersection(allGroupDatas[id]._groupPoly, tmpExpandRing, results);
                    if (!results.empty() )
                    {
                        point_t nearbyPoint = getNearestPoint(allGroupDatas[id]._groupPoints, tmpPoint);
                        if (std::abs(tmpPoint.get<2>() - nearbyPoint.get<2>()) < MAX_HEIGHT)
                        {
                            intersectGroupIds.push_back(id);
                        }
                    }
                    }));
            bool isSingleDiversion = false;
            if (intersectGroupIds.size() == 1)
                isSingleDiversion = true;
            if (intersectGroupIds.size() > 1)
            {
                bool isContinue = true;
                isSingleDiversion = true;
                for (size_t i = 0; i < intersectGroupIds.size(); ++i)
                {
                    auto& iGroupInfo = allGroupDatas[intersectGroupIds[i]];
                    if (isMiddleLaneGroup(iGroupInfo._laneGroup))
                    {
                        continue;
                    }

                    auto seg = S3_S2(iGroupInfo._verticalSegment);
                    for (size_t j = 0; j < intersectGroupIds.size(); ++j)
                    {
                        if (i == j)
                        {
                            continue;
                        }

                        auto& jGroupInfo = allGroupDatas[intersectGroupIds[j]];
                        if (isMiddleLaneGroup(jGroupInfo._laneGroup))
                        {
                            continue;
                        }
                        
                        auto& poly = jGroupInfo._groupPoly;
                        if (bg::intersects(seg, poly))
                        {
							isContinue = false;
							isSingleDiversion = false;
							break;
                        }
                    }
                    if (!isContinue)
                        break;
                }
            }
            if (isSingleDiversion)
            {
                singleDiversionInfo tmpDiversionInfo;
                tmpDiversionInfo._rdsDiversion = gridDiversions[diversionIndex];
                tmpDiversionInfo._diversionPoly = gridDiversionPolys[diversionIndex];
                tmpDiversionInfo._originRing = RING_2T(gridDiversionPolys[diversionIndex].vertexes);
                tmpDiversionInfo._expandRing = tmpExpandRing;
                tmpDiversionInfo._centerPoint = tmpPoint;
                tmpDiversionInfo._isWillDelete = isDeleteDiversion;
                tmpDiversionInfo._isCirclePoly = isCirclePoly(tmpDiversionInfo._originRing);
                bg::correct(tmpDiversionInfo._originRing);
                tmpDiversionInfo._diversionBox = gridDiversionBoxes[diversionIndex];
                allSingleDiversions.push_back(tmpDiversionInfo);
                gridSingleDiversionBoxes.push_back(gridDiversionBoxes[diversionIndex]);
            }
        }
    }


    void GuardrailCompiler::removeDuplicatePoint(std::vector<LineString3d> roundGuardrails, std::vector<LineString3d>& filteredLines)
    {
     
        for (auto iter : roundGuardrails)
        {
            MapPoint3D64 current = iter.vertexes[0];
            for (int i = 1; i < iter.vertexes.size(); i++)
            {
                MapPoint3D64 next = iter.vertexes[i];

                point_2t c(current.pos.lon, current.pos.lat);
                point_2t n(next.pos.lon, next.pos.lat);
                if (bg::distance(c, n) < 100)
                {
                    iter.vertexes.erase(iter.vertexes.begin() + i);
                    i--;
                    
                    continue;
                }
                else
                {
                    current = next;
                }

            }
            filteredLines.push_back(iter);
        }
        
    }

    void GuardrailCompiler::buildLaPolygon(std::vector<LineString3d>& laEdges, Polygon3d& polygon)
    {
		polygon.vertexes.clear();
		MapPoint3D64 prevVertex = {};
		for (auto& laEdge : laEdges) {
			for (auto& vertex : laEdge.vertexes) {
				if (!floatEqual(vertex.pos.distanceSquare(prevVertex.pos), 0)) {
					polygon.vertexes.push_back(vertex);
					prevVertex = vertex;
				}
			}
		}

		// 闭环
		MapPoint3D64& startPt = polygon.vertexes.front();
		MapPoint3D64& endPt = polygon.vertexes.back();
		if (!floatEqual(startPt.pos.distanceSquare(endPt.pos), 0)) {
			polygon.vertexes.push_back(startPt);
		}
    }

    point_t GuardrailCompiler::getNearestPoint(
        const std::vector<point_t>& points,
        const point_t& originPoint)
    {
        parameters param;
        index_getter originInd(points);
        rtree_type rtree(boost::irange<std::size_t>(0lu, points.size()), param, originInd);
        std::vector<point_t> resultPoints;
        rtree.query(bgi::nearest(originPoint, 1),
            boost::make_function_output_iterator([&](size_t const& id) {
                resultPoints.push_back(points[id]);
                }));
        if(!resultPoints.empty())
            return resultPoints.front();
        return point_t(0, 0);
    }

    rtree_type_2segment GuardrailCompiler::getBoundarySegmentRTree(
        const std::vector<segment_2t>& segs)
    {
        parameters param;
        index_getter_2segment originInd(segs);
        rtree_type_2segment	rtree(boost::irange<std::size_t>(0lu, segs.size()), param, originInd);
        return rtree;
    }

    SegmentRTree::Ptr GuardrailCompiler::getBarrierSegmentRTree()
    {
        auto& tmp = guardrailInd.rtreeBarrierSegs;
        if (tmp)
            return tmp;
        SegmentRTree::guardParam param;
        SegmentRTree::indexGetterSegment originIndSeg(allBarrierSegs);
        auto rtreeBarrierSegs = std::make_shared<SegmentRTree::RTree>(boost::irange<std::size_t>(0lu, allBarrierSegs.size()), param, originIndSeg);
        guardrailInd.rtreeBarrierSegs = rtreeBarrierSegs;
        return rtreeBarrierSegs;
    }

    BoxRTree::Ptr GuardrailCompiler::getDiversionBoxRTree()
    {
        auto& tmp = guardrailInd.rtreeDiversionBox;
        if (tmp)
            return tmp;
        BoxRTree::guardParam param;
        BoxRTree::indexGetterBox originIndBox(gridDiversionBoxes);
        auto rtreeBox = std::make_shared<BoxRTree::RTree>(boost::irange<std::size_t>(0lu, gridDiversionBoxes.size()), param, originIndBox);
        guardrailInd.rtreeDiversionBox = rtreeBox;
        return rtreeBox;
    }

    BoxRTree::Ptr GuardrailCompiler::getSingleBoxRTree()
    {
        auto& tmp = guardrailInd.rtreeSingleBox;
        if (tmp)
            return tmp;
        BoxRTree::guardParam param;
        BoxRTree::indexGetterBox originIndBox(gridSingleDiversionBoxes);
        auto rtreeBox = std::make_shared<BoxRTree::RTree>(boost::irange<std::size_t>(0lu, gridSingleDiversionBoxes.size()), param, originIndBox);
        guardrailInd.rtreeSingleBox = rtreeBox;
        return rtreeBox;
    }

    BoxRTree::Ptr GuardrailCompiler::getLaneGroupBoxRTree()
    {
        auto& tmp = guardrailInd.rtreeLaneGroupBox;
        if (tmp)
            return tmp;
        BoxRTree::guardParam param;
        BoxRTree::indexGetterBox originIndBox(allGroupBoxes);
        auto rtreeBox = std::make_shared<BoxRTree::RTree>(boost::irange<std::size_t>(0lu, allGroupBoxes.size()), param, originIndBox);
        guardrailInd.rtreeLaneGroupBox = rtreeBox;
        return rtreeBox;
    }

}

