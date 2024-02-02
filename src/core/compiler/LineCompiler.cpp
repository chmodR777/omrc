#include "stdafx.h"
#include "LineCompiler.h"
#include "algorithm/grap_point_algorithm.h"
#include "algorithm/polyline_intersector.h"
#include "algorithm/linear_interpolation_triangle_surface.h"
#include "CompileSetting.h"

namespace OMDB
{
	void LineCompiler::compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile)
	{
		UNREFERENCED_PARAMETER(nearby);
		// 路口三角化结果
		std::vector<Triangle> surfaceTriangles{ compilerData.m_intersectionTriangles.begin(), compilerData.m_intersectionTriangles.end() };

		// 路面三角化结果
		surfaceTriangles.insert(surfaceTriangles.end(), compilerData.m_roadTriangles.begin(), compilerData.m_roadTriangles.end());

		// rtree
		parameters surfaceParam;
		std::vector<box_2t> surfaceTriangleBoxes;
		LinearInterpolationTriangleSurface::getTriangleBoxes(surfaceTriangles, surfaceTriangleBoxes);
		index_getter_2box surfaceOriginInd(surfaceTriangleBoxes);
		rtree_type_2box surfaceRTree2T(boost::irange<std::size_t>(0lu, surfaceTriangleBoxes.size()), surfaceParam, surfaceOriginInd);

		compileStopLocation(pGrid, surfaceTriangles, surfaceRTree2T, pTile);

		compileLaneBoundary(pGrid, surfaceTriangles, surfaceRTree2T, pTile);
	}

	void LineCompiler::compileLaneBoundary(HadGrid* const pGrid, const std::vector<Triangle>& surfaceTriangles, const rtree_type_2box& surfaceRTree2T, RdsTile* pTile)
	{
		//路口屏蔽
		std::set<HadLaneBoundary*> inIntersectionBoundary;
		for (auto obj : pGrid->query(ElementType::HAD_LANE))
		{
			HadLane* pLane = (HadLane*)obj;
			if (pLane->linkGroup && pLane->linkGroup->inIntersection)
			{
				inIntersectionBoundary.insert(pLane->leftBoundary);
				inIntersectionBoundary.insert(pLane->rightBoundary);
			}
		
		}

		for (auto obj : pGrid->query(ElementType::HAD_LANE_BOUNDARY))
		{
			HadLaneBoundary* pLaneBoundary = (HadLaneBoundary*)obj;

			//判断是否为普通路
			std::vector<HadLaneGroup*> tmpGroups;
			for (auto itemGroup : pLaneBoundary->linkGroups)
				tmpGroups.push_back(itemGroup.second);
			if (CompileSetting::instance()->isNotCompileUrbanData)
			{
				if (!isProDataLevel(tmpGroups))
					continue;
			}

			if (CompileSetting::instance()->isDaimlerShangHai)
			{
				if (pLaneBoundary->turnwaitingPAs.empty() && inIntersectionBoundary.count(pLaneBoundary))
					continue;
			}
			
			//待转区边界线
			if (CompileSetting::instance()->isCompileTurnWaiting)
			{
				if (!pLaneBoundary->turnwaitingPAs.empty())
				{

					std::vector<RangeF> rangeOffsets;
					mergeTurnwaitingPAs(pLaneBoundary->turnwaitingPAs, rangeOffsets);

					for (auto offet : rangeOffsets)
					{
						LineString3d subLocation;
						truncateLineSegment(pLaneBoundary->location.vertexes, offet.lower, offet.upper, subLocation.vertexes);

						RdsLine* pLine = (RdsLine*)createObject(pTile, EntityType::RDS_LINE);
						pLine->lineType = RdsLine::LineType::eShortThickDot;
						pLine->color = RdsLine::LineColor::eWhite;
						pLine->width = 0;
						pLine->side = 0;

						convert(subLocation, pLine->location);
					}
					continue;
				}
			}

			if (!isCreateRdsForIntersection(pGrid, tmpGroups))
				continue;

			std::map<int64, std::vector<HadPartAttribute*>> groupAttributes;
			groupByLaneBoundaryRange(pLaneBoundary, groupAttributes);

			// 使用道路边界和车道边界
			Polygon3d polygon;
			if (pLaneBoundary->linkGroups.size() == 1)
			{
				LineString3d leftSide, rightSide;
				auto pLaneGroup = pLaneBoundary->linkGroups.begin()->second;
				getLaneGroupBoundary(pLaneGroup, leftSide, rightSide);
				makeLaneGroupPolygon(leftSide.vertexes, rightSide.vertexes, polygon);
			}

			std::set<int> markWidths;
			for (size_t i = 0; i < pLaneBoundary->markings.size(); i++) {
				HadLaneBoundary::HadLaneBoundaryMarking& marking = pLaneBoundary->markings[i];
				int width = markingValue(marking.markWidth);
				if (width != 0)
					markWidths.emplace(width);
			}
			for (size_t i = 0; i < pLaneBoundary->markings.size(); i++)
			{
				LineString3d location = pLaneBoundary->location;

				// 车道边界类型、颜色、宽度为-99时使用PAValue
				HadLaneBoundary::HadLaneBoundaryMarking& marking = pLaneBoundary->markings[i];
				if (!groupAttributes.empty() && (marking.markType == DB_HAD_APPLY_PA_REFERENCE || 
					marking.markColor == DB_HAD_APPLY_PA_REFERENCE || marking.lateralOffset == DB_HAD_APPLY_PA_REFERENCE)) {
					for (auto attributes : groupAttributes) {
						std::map<int64, std::vector<HadPartAttribute*>> markSeqNumAttributes;
						groupByMarkSeqNum(attributes.second, markSeqNumAttributes);
						for (auto seqNumAttributes : markSeqNumAttributes) {
							LineString3d subLocation = createSubLaneBoundary(location, (*seqNumAttributes.second.begin())->points);
							RdsLine* pLine = (RdsLine*)createObject(pTile, EntityType::RDS_LINE);
							pLine->lineType = (RdsLine::LineType)markingValue(marking.markType);
							pLine->color = (RdsLine::LineColor)markingValue(marking.markColor);
							pLine->width = markingValue(marking.markWidth);
							pLine->side = 0;
							// offset不为0时,矫正车道边界偏移
							for (HadPartAttribute* paValue : seqNumAttributes.second) {
								HadLaneBoundaryPAType paType = (HadLaneBoundaryPAType)paValue->name;
								if (marking.lateralOffset == DB_HAD_APPLY_PA_REFERENCE &&
									paType == HadLaneBoundaryPAType::LATERAL_OFFSET &&
									paValue->value != DB_HAD_NOT_APPLY_PA_REFERENCE && paValue->value != 0) {
									adjustLaneBoundaryOffset(subLocation, paValue->value, polygon);
									pLine->side = paValue->value;
								}
								if (marking.markType == DB_HAD_APPLY_PA_REFERENCE &&
									paType == HadLaneBoundaryPAType::MARK_TYPE) {
									pLine->lineType = (RdsLine::LineType)paValue->value;
								}
								if (marking.markColor == DB_HAD_APPLY_PA_REFERENCE &&
									paType == HadLaneBoundaryPAType::MARK_COLOR) {
									pLine->color = (RdsLine::LineColor)paValue->value;
								}
							}
							if (pLine->width == DB_HAD_NOT_APPLY_PA_REFERENCE) {
								pLine->width = 0;
							}

							// 车道线线投影到路面
							std::vector<MapPoint3D64> lineOnRoadSurface;
							LinearInterpolationTriangleSurface::interpolationLine(
								coordinatesTransform, surfaceTriangles, surfaceRTree2T, subLocation.vertexes, lineOnRoadSurface);
							subLocation.vertexes = lineOnRoadSurface;

							saveRdsLine(pLine, subLocation);
							convert(subLocation, pLine->location);
							for (auto iter : pLaneBoundary->linkGroups) {
								RdsGroup* pRdsGroup = queryGroup(iter.first, pTile);
								if (pRdsGroup)
								{
									pRdsGroup->objects.push_back(pLine);
								}
							}

						}

					}
				}
				else {
					RdsLine* pLine = (RdsLine*)createObject(pTile, EntityType::RDS_LINE);
					pLine->lineType = (RdsLine::LineType)markingValue(marking.markType);
					pLine->color = (RdsLine::LineColor)markingValue(marking.markColor);
					pLine->width = markingValue(marking.markWidth);
					pLine->side = 0;
					if (pLine->width == 0 && markWidths.size() == 1) {
						pLine->width = *markWidths.begin();
					}
					// offset不为0时,矫正车道边界偏移
					if (marking.lateralOffset != 0 && marking.lateralOffset != DB_HAD_APPLY_PA_REFERENCE
						&& marking.lateralOffset != DB_HAD_NOT_APPLY_PA_REFERENCE) {
						MultiPoint3d points;
						points.postions.push_back(*location.vertexes.begin());
						points.postions.push_back(*location.vertexes.rbegin());
						adjustLaneBoundaryOffset(location, marking.lateralOffset, polygon);
						pLine->side = marking.lateralOffset;
					}

					// 车道线线投影到路面
					std::vector<MapPoint3D64> lineOnRoadSurface;
					LinearInterpolationTriangleSurface::interpolationLine(
						coordinatesTransform, surfaceTriangles, surfaceRTree2T, location.vertexes, lineOnRoadSurface);
					location.vertexes = lineOnRoadSurface;

					saveRdsLine(pLine, location);
					convert(location, pLine->location);
					for (auto iter : pLaneBoundary->linkGroups) {
						RdsGroup* pRdsGroup = queryGroup(iter.first, pTile);
						if (pRdsGroup)
						{
							pRdsGroup->objects.push_back(pLine);
						}
					}
				}

			}

			// 没有markings信息,默认宽度、类型和颜色赋值为0
			if (pLaneBoundary->markings.empty()) {
				RdsLine* pLine = (RdsLine*)createObject(pTile, EntityType::RDS_LINE);
				pLine->width = 0;
				pLine->side = 0;
				pLine->lineType = (RdsLine::LineType)0;
				pLine->color = (RdsLine::LineColor)0;

				// 车道线线投影到路面
				LineString3d location = pLaneBoundary->location;
				std::vector<MapPoint3D64> lineOnRoadSurface;
				LinearInterpolationTriangleSurface::interpolationLine(
					coordinatesTransform, surfaceTriangles, surfaceRTree2T, location.vertexes, lineOnRoadSurface);
				location.vertexes = lineOnRoadSurface;

				saveRdsLine(pLine, location);
				convert(location, pLine->location);
				for (auto iter : pLaneBoundary->linkGroups) {
					RdsGroup* pRdsGroup = queryGroup(iter.first, pTile);
					if (pRdsGroup)
					{
						pRdsGroup->objects.push_back(pLine);
					}
				}

			}

		}
	}

	void LineCompiler::groupByLaneBoundaryRange(HadLaneBoundary* const pLaneBoundary, std::map<int64, std::vector<HadPartAttribute*>>& groupAttributes)
	{
		int64 lastOriginId = -1;
		for (HadPartAttribute* attribute : pLaneBoundary->attributes) {
			// 如果将来枚举遍历值不是连续的这里需要跟着改动
			HadLaneBoundaryPAType paType = (HadLaneBoundaryPAType)attribute->name;
			if (paType >= HadLaneBoundaryPAType::BOUNDARY_TYPE && paType <= HadLaneBoundaryPAType::LATERAL_OFFSET)
			{
				if (lastOriginId != attribute->originId) {
					lastOriginId = attribute->originId;
				}

				auto iter = groupAttributes.find(lastOriginId);
				if (iter == groupAttributes.end()) {
					std::vector<HadPartAttribute*> vec;
					vec.push_back(attribute);
					groupAttributes.emplace(lastOriginId, vec);
				}
				else {
					iter->second.push_back(attribute);
				}
			}
		}
	}

	void LineCompiler::groupByMarkSeqNum(std::vector<HadPartAttribute*>& attributes, std::map<int64, std::vector<HadPartAttribute*>>& groupAttributes)
	{
		// markSeqNum分左右
		int64 lastSeqNum = -1;
		for (HadPartAttribute* attribute : attributes) {
			if (lastSeqNum != attribute->seqNum) {
				lastSeqNum = attribute->seqNum;
			}

			auto iter = groupAttributes.find(lastSeqNum);
			if (iter == groupAttributes.end()) {
				std::vector<HadPartAttribute*> vec;
				vec.push_back(attribute);
				groupAttributes.emplace(lastSeqNum, vec);
			}
			else {
				iter->second.push_back(attribute);
			}
		}
	}

	LineString3d LineCompiler::createSubLaneBoundary(LineString3d& location, MultiPoint3d& points)
	{
		if (points.postions.size() < 2) {
			throw std::runtime_error{ "CreateSubLaneBoundary points.postions.size() != 2." };
		}

		LineString3d line;
		MapPoint3D64 grappedPt{};
		auto& startPt = points.postions.front();
		auto& entPt = points.postions.back();

		int startIdx = -1, endIdx = -1;
		size_t startSi, startEi, entSi, entEi;
		// 存在圆弧时最近点不是本侧的,改成抓点的方式
		if (GrapPointAlgorithm::grapOrMatchNearestPoint(startPt, location.vertexes, grappedPt, startSi, startEi) 
			&& GrapPointAlgorithm::grapOrMatchNearestPoint(entPt, location.vertexes, grappedPt, entSi, entEi)) {
			startIdx = startSi;
			endIdx = entEi;
		}
		else
		{
			startIdx = GrapPointAlgorithm::findNearestPoint(location.vertexes, startPt);
			endIdx = GrapPointAlgorithm::findNearestPoint(location.vertexes, entPt);
		}
		if (startIdx == endIdx) {
			line.vertexes.push_back(startPt);
			line.vertexes.push_back(entPt);
			return line;
		}

		for (int idx = startIdx; idx <= endIdx; idx++) {
			auto& currPt = location.vertexes[idx];
			if (idx == startIdx) {
				auto& nextPt = location.vertexes[idx + 1];
				auto currNextDis = currPt.pos.distance(nextPt.pos);
				auto startNextDis = startPt.pos.distance(nextPt.pos);
				line.vertexes.push_back(startPt);
				if (startNextDis > currNextDis)  {
					line.vertexes.push_back(currPt);
				}
			} else if (idx == endIdx) {
				auto& prevPt = location.vertexes[idx - 1];
				auto currPrevDis = currPt.pos.distance(prevPt.pos);
				auto endPrevDis = entPt.pos.distance(prevPt.pos);
				if (endPrevDis > currPrevDis) {
					line.vertexes.push_back(currPt);
				}
				line.vertexes.push_back(entPt);
			} else {
				line.vertexes.push_back(currPt);
			}
		}

		if (line.vertexes.size() == 0) {
			//printf("vertexes");
		}
		return line;
	}

	void LineCompiler::adjustLaneBoundaryOffset(LineString3d& location, int offset, Polygon3d& polygon)
	{
		auto ip = std::unique(location.vertexes.begin(), location.vertexes.end(), mapPoint3D64_compare);
		location.vertexes.resize(std::distance(location.vertexes.begin(), ip));

		auto newLocation = location;
		static const double tolerance = 10;
		int endIdx = newLocation.vertexes.size() - 1;
		for (int idx = 0; idx < endIdx; idx++) {
			auto& currPt = newLocation.vertexes[idx];
			auto& nextPt = newLocation.vertexes[idx + 1];
			auto r = std::atan2(nextPt.pos.lon - currPt.pos.lon, nextPt.pos.lat - currPt.pos.lat) + MATH_PI_D / 2;
			auto x = std::sin(r) * offset;
			auto y = std::cos(r) * offset;
			currPt.pos.lon = currPt.pos.lon + x;
			currPt.pos.lat = currPt.pos.lat + y;
			if (idx == endIdx - 1) {
				nextPt.pos.lon = nextPt.pos.lon + x;
				nextPt.pos.lat = nextPt.pos.lat + y;
			}
		}
		// 共享车道边界,直接返回
		if (polygon.vertexes.empty()) {
			location = newLocation;
			return;
		}

		// 检查线是否在路面内,在路面内直接返回
		for (int idx = 0; idx < endIdx; idx++) {
			auto& currPt = newLocation.vertexes[idx];
			auto& nextPt = newLocation.vertexes[idx + 1];
			if (GrapPointAlgorithm::isPointInPolygon(currPt, polygon.vertexes)) {
				location = newLocation;
				return;
			}
			if (idx == endIdx - 1) {
				if (GrapPointAlgorithm::isPointInPolygon(nextPt, polygon.vertexes)) {
					location = newLocation;
					return;
				}
			}
			size_t si, ei;
			MapPoint3D64 intersectPt = {};
			if (PolylineIntersector::intersect(currPt, nextPt, polygon.vertexes, tolerance, intersectPt, si, ei)) {
				location = newLocation;
				return;
			}
		}
	}

	void LineCompiler::saveRdsLine(RdsLine* pLine, LineString3d& location)
	{
		//去重
		auto ip = std::unique(location.vertexes.begin(), location.vertexes.end(), mapPoint3D64_compare);
		location.vertexes.resize(std::distance(location.vertexes.begin(), ip));
		if (location.vertexes.size() < 2)
			return;

		rdsLineInfo tmp;
		tmp._line = pLine;
		tmp._originPoints = location.vertexes;
		std::vector<MapPoint3D64> tmpLine = location.vertexes;
		coordinatesTransform.convert(tmpLine.data(), tmpLine.size());
		tmp._lineBox2T = BOX_2T(tmpLine);
		tmp._linePoints = LINESTRING_T(tmpLine);

		//push line
		compilerData.m_rdsLines.push_back(tmp);
	}

	void LineCompiler::saveRdsStopLine(RdsLine* pLine, LineString3d& location)
	{
		//去重
		auto ip = std::unique(location.vertexes.begin(), location.vertexes.end(), mapPoint3D64_compare);
		location.vertexes.resize(std::distance(location.vertexes.begin(), ip));
		if (location.vertexes.size() < 2)
			return;

		rdsLineInfo tmp;
		tmp._line = pLine;
		tmp._originPoints = location.vertexes;
		std::vector<MapPoint3D64> tmpLine = location.vertexes;
		coordinatesTransform.convert(tmpLine.data(), tmpLine.size());
		tmp._lineBox2T = BOX_2T(tmpLine);
		tmp._linePoints = LINESTRING_T(tmpLine);

		//push stopline
		compilerData.m_rdsStopLineBoxes.push_back(tmp._lineBox2T);
		compilerData.m_rdsStopLines.push_back(tmp);
	}



    void LineCompiler::compileStopLocation(HadGrid* const pGrid, const std::vector<Triangle>& surfaceTriangles, const rtree_type_2box& surfaceRTree2T, RdsTile* pTile)
	{
		//路口屏蔽
		std::set<HadObject*> inIntersectionObjects;
		for (auto obj : pGrid->query(ElementType::HAD_LANE))
		{
			HadLane* pLane = (HadLane*)obj;
			if (pLane->linkGroup && pLane->linkGroup->inIntersection 
				&& pLane->leftBoundary->turnwaitingPAs.empty()
				&& pLane->rightBoundary->turnwaitingPAs.empty()
				)
			{
				for (HadObject* obj : pLane->objects)
				{
					if (obj->objectType == ElementType::HAD_OBJECT_STOPLOCATION)
					{
						inIntersectionObjects.insert(obj);
					}
				}
			}
		}

		for (auto obj : pGrid->query(ElementType::HAD_OBJECT_STOPLOCATION))
		{
			if (CompileSetting::instance()->isDaimlerShangHai)
			{
				if (inIntersectionObjects.count((HadObject*)obj))
					continue;
			}

			HadStopLocation* pStopLocation = (HadStopLocation*)obj;
			LineString3d location = pStopLocation->location;

			//判断是否为普通路
			if (CompileSetting::instance()->isNotCompileUrbanData)
			{
				if (!isProDataLevel(pStopLocation->laneGroups))
					continue;
			}


			if (!CompileSetting::instance()->isCompileTurnWaiting)
			{
				if (!isCreateRdsForIntersection(pGrid, pStopLocation->laneGroups))
					continue;
			}

			// 停止线投影到路面
			std::vector<MapPoint3D64> lineOnSurface;
			LinearInterpolationTriangleSurface::interpolationLine(
				coordinatesTransform, surfaceTriangles, surfaceRTree2T, location.vertexes, lineOnSurface);
			location.vertexes = lineOnSurface;

			// 裁减路面外的停止线
			clipOutOfRoadFace(surfaceTriangles, surfaceRTree2T, location.vertexes);

			RdsLine* pLine = (RdsLine*)createObject(pTile, EntityType::RDS_LINE);
			saveRdsStopLine(pLine, location);
			convert(location, pLine->location);
			pLine->width = pStopLocation->width;
			pLine->side = 0;
			// TODO merge locationType into lineType, color?
			// pLine->lineType = (RdsLine::LineType)pStopLocation->locationType;
			pLine->lineType = RdsLine::LineType::eStopLine;
			pLine->color = (RdsLine::LineColor)pStopLocation->color;
			pLine->isStopLocation = true;

			for (HadLaneGroup* linkGroup : pStopLocation->laneGroups) {
				RdsGroup* pRdsGroup = queryGroup(linkGroup->originId, pTile);
				if (pRdsGroup)
				{
					pRdsGroup->objects.push_back(pLine);
				}
			}
		}
	}

	void LineCompiler::clipOutOfRoadFace(const std::vector<Triangle>& surfaceTriangles, const rtree_type_2box& surfaceRTree2T, std::vector<MapPoint3D64>& outPts)
	{
		if (outPts.size() < 2)
			return;
		auto convertedPts = outPts;
		std::vector<Triangle> triangles;
		static const double tolerance = 10;
		coordinatesTransform.convert(convertedPts.data(), convertedPts.size());
		segment_t segment{ POINT_T(convertedPts.front()), POINT_T(convertedPts.back()) };
		if (LinearInterpolationTriangleSurface::getTrianglesBySegment(segment, surfaceRTree2T, surfaceTriangles, triangles)) {
			std::vector<Polygon3d> roadFacePolygons;
			for (auto& triangle : triangles) {
				Polygon3d roadFacePolygon;
				for (auto& pt : triangle.vertexes) {
					MapPoint3D64 tmpPos = MapPoint3D64_make(pt.get<0>(), pt.get<1>(), pt.get<2>() / 10);
					roadFacePolygon.vertexes.push_back(tmpPos);
				}
				coordinatesTransform.invert(roadFacePolygon.vertexes.data(), roadFacePolygon.vertexes.size());
				if (roadFacePolygon.vertexes.front() != roadFacePolygon.vertexes.back()) {
					roadFacePolygon.vertexes.push_back(roadFacePolygon.vertexes.front());
				}
				roadFacePolygons.push_back(roadFacePolygon);
			}
			
			if (!roadFacePolygons.empty()) {
				bool containsIntersectPoint = false;
				for (auto& roadFacePolygon : roadFacePolygons) {
					for (size_t i = 0; i < outPts.size() - 1; i++) {
						auto& startPt = outPts[i];
						auto& endPt = outPts[i + 1];
						std::vector<MapPoint3D64> intersectPoints;
						if (PolylineIntersector::intersect(startPt, endPt, roadFacePolygon.vertexes, tolerance, intersectPoints)) {
							auto pStart = POINT_T(startPt);
							auto pEnd = POINT_T(endPt);
							segment_t segment{ pStart, pEnd };
							vector_t pabVec = V3_N(S3_V3(segment));
							std::sort(intersectPoints.begin(), intersectPoints.end(), [&](MapPoint3D64& pa, MapPoint3D64& pb) {
								vector_t tmp0V = S3_V3(pStart, POINT_T(pa));
								auto v0Sum = bg::dot_product(pabVec, tmp0V);

								vector_t tmp1V = S3_V3(pStart, POINT_T(pb));
								auto v1Sum = bg::dot_product(pabVec, tmp1V);
								return v0Sum < v1Sum;
							});
							auto insertIter = mapPoint3D64_iterator(outPts, endPt);
							for (auto& intersectPoint : intersectPoints) {
								containsIntersectPoint = true;
								auto intersectPt = POINT_2T(intersectPoint);
								if (isEqualPoint2T(P3_P2(pStart), intersectPt) || isEqualPoint2T(P3_P2(pEnd), intersectPt)) {
									continue;
								}
								insertIter = outPts.insert(insertIter, intersectPoint);
								insertIter++;// 指向插入点下一个位置
							}
						}
					}
				}
				if (!containsIntersectPoint) {
					return;
				}

				// clip
				for (auto iter = outPts.begin(); iter != outPts.end(); ) {
					bool pointInPolygon = false;
					for (auto& roadFacePolygon : roadFacePolygons) {
						if (GrapPointAlgorithm::isPointInPolygon(*iter, roadFacePolygon.vertexes)) {
							pointInPolygon = true;
							break;
						}
						size_t si, ei;
						MapPoint3D64 minGrappedPt = {};
						GrapPointAlgorithm::grapOrMatchNearestPoint(*iter, roadFacePolygon.vertexes, minGrappedPt, si, ei);
						if (minGrappedPt.pos.distance(iter->pos) < 50) {
							pointInPolygon = true;
							break;
						}
					}

					if (pointInPolygon) {
						iter++;
						continue;
					}
					iter = outPts.erase(iter);
				}
			}
		}
	}


    void LineCompiler::truncateLineSegment(const std::vector<MapPoint3D64>& line, double startOffset, double endOffset, std::vector<MapPoint3D64>& outline)
    {
		auto scale = [](const MapPoint3D64& sPt, const MapPoint3D64& ePt, double scale)->MapPoint3D64
		{
			MapPoint3D64 p = ePt - sPt;
			p.pos.lon *= scale;
			p.pos.lat *= scale;
			p.z *= scale;

			return sPt + p;
		};
		if (startOffset <0 || startOffset >1 || endOffset < 0 || endOffset >1 || startOffset > endOffset)
		{
			return;
		}
		//计算总长
		std::vector<double> sections;
		double line_length = _getLength(line, sections);

		double currentOffset = 0.0;
		for (int i = 0; i < line.size(); i++)
		{
			double NextOffset = currentOffset + sections[i] / line_length;
			if (startOffset >= currentOffset && startOffset <= NextOffset)
			{
				MapPoint3D64 ptOne = scale(line[i], line[i + 1], sections[i]?(startOffset- currentOffset)/(sections[i] / line_length):0);
				outline.emplace_back(ptOne);
			}

            if (startOffset < currentOffset && currentOffset < endOffset)
            {
                outline.emplace_back(line[i]);
            }


			if(endOffset >= currentOffset && endOffset <= NextOffset)
			{
				MapPoint3D64 ptLast = scale(line[i], line[i + 1], sections[i] ? (endOffset - currentOffset) / (sections[i] / line_length):0);
				outline.emplace_back(ptLast);
				break;
			}
		
			
			currentOffset = NextOffset;
		}
		std::unique(outline.begin(), outline.end());
    }

    double LineCompiler::_getLength(const std::vector<MapPoint3D64>& line, std::vector<double>& sections)
    {
        double distance = 0.0;
        std::vector<NdsPoint> ndsPoints;
        ndsPoints.resize(line.size());
        for (size_t i = 0; i < line.size(); i++)
        {
            ndsPoints[i] = line[i].pos.toNdsPoint();
        }

        sections.resize(line.size());
        NdsPoint current = ndsPoints[0];
        for (size_t i = 1; i < line.size(); i++)
        {
            NdsPoint next = ndsPoints[i];
            double d = GrapPointAlgorithm::geoLengthD(current, next);
            distance += d;
            current = next;

            sections[i - 1] = d;
        }
        sections[line.size() - 1] = 0.0;
        return distance;
    }

    void LineCompiler::mergeTurnwaitingPAs(const std::vector<HadLaneTurnwaiting*>& turnwaitinPA,std::vector<RangeF>& out)
    {

        if (turnwaitinPA.empty())
        {
            return;
        }
		std::vector<RangeF> rangF;
		for (auto pa : turnwaitinPA)
		{
			rangF.emplace_back(RangeF_make(pa->startOffset, pa->endOffset));
		}

        std::sort(rangF.begin(), rangF.end(), [](RangeF s1, RangeF s2)
            {
                if (s1.lower != s2.lower)
                    return s1.lower < s2.lower;

                else
                    return s1.upper < s2.upper;
            });

		RangeF curRange = rangF[0];
        for (int i = 1; i < rangF.size(); i++)
        {
            if (curRange.upper>= rangF[i].lower)
            {
                curRange.upper = max(curRange.upper, rangF[i].upper);

            }
            else
            {
                out.emplace_back(curRange);
                curRange = rangF[i];
            }
        }

        out.emplace_back(curRange);
    


    }

}
