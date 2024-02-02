#include "stdafx.h"
#include "CrossWalkCompiler.h"
#include "algorithm/poly2tri.h"

#define PI       3.14159265358979323846   // pi
#define MAX_HEIGHT 5000

 namespace OMDB
 {
	 void CrossWalkCompiler::compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile)
	 {
		 UNREFERENCED_PARAMETER(nearby);
		 std::vector<segment_t> gridStopLines;
		 gridStopLines.insert(gridStopLines.end(), compilerData.gridRoadStopLines.begin(), compilerData.gridRoadStopLines.end());
		 parameters param;
		 index_getter_segment originInd(gridStopLines);
		 rtree_type_segment	rtree(boost::irange<std::size_t>(0lu, gridStopLines.size()), param, originInd);

		 // 路口三角化结果
		 std::vector<Triangle> surfaceTriangles{ compilerData.m_intersectionTriangles.begin(), compilerData.m_intersectionTriangles.end() };

		 // 路面三角化结果
		 surfaceTriangles.insert(surfaceTriangles.end(), compilerData.m_roadTriangles.begin(), compilerData.m_roadTriangles.end());

		 // 跨网格时RdsRoad数据不存在情况
		 //for (auto& laneGroup : laneGroups)
		 //{
			// if (laneGroup->inIntersection)
			//	 continue;
			// if (!laneGroup->crossGrid)
			//	 continue;

			// LineString3d leftSide, rightSide;
			// getLaneGroupBoundary(laneGroup, leftSide, rightSide);
			// if (leftSide.vertexes.empty() || rightSide.vertexes.empty())
			//	 continue;

			// std::vector<LinearInterpolationTriangleSurface::Triangle> roadTriangles;
			// LinearInterpolationTriangleSurface::triangularizeStroke(coordinatesTransform, leftSide.vertexes, rightSide.vertexes, roadTriangles);
			// surfaceTriangles.insert(surfaceTriangles.end(), roadTriangles.begin(), roadTriangles.end());
		 //}

		 for (auto obj : pGrid->query(ElementType::HAD_OBJECT_CROSS_WALK))
		 {
			 HadCrossWalk* pcw = (HadCrossWalk*)obj;

			 //判断是否为普通路
			 if (CompileSetting::instance()->isNotCompileUrbanData)
			 {
				 if (!isProDataLevel(pcw->laneGroups))
					 continue;
			 }

			 if (pcw->originId == 259303923134024231)
				 printInfo("");

			 //多余进行裁剪
			 std::set<size_t> rdsRoadIds;
			 std::vector<ring_2t> intersectResults;
			 auto pCrossWalkPoly = pcw->polygon.vertexes;
			 coordinatesTransform.convert(pCrossWalkPoly.data(), pCrossWalkPoly.size());
			 point_t crossWalkCenter;
			 auto tmpCrossWalkPoints = LINESTRING_T(pCrossWalkPoly);
			 bg::centroid(tmpCrossWalkPoints, crossWalkCenter);
			 auto tmpCroassWalkRing = RING_2T(pCrossWalkPoly);
			 bg::correct(tmpCroassWalkRing);

			 auto& gridRoadDatas = compilerData.m_rdsRoads;
			 std::vector<ring_2t> allRings;
			 getRoadBox2TRTree()->query(bgi::intersects(BOX_2T(pCrossWalkPoly)),
				 boost::make_function_output_iterator([&](size_t const& id) {
					 auto tmpRing = gridRoadDatas[id]._roadPoly2T;
					 std::vector<ring_2t> results;
					 bg::intersection(tmpCroassWalkRing, tmpRing, results);
					 if (!results.empty())
					 {
						 point_t nearbyPoint = getNearestPoint(gridRoadDatas[id]._roadPoints, crossWalkCenter);
						 if (std::abs(crossWalkCenter.get<2>() - nearbyPoint.get<2>()) < MAX_HEIGHT)
						 {
							 allRings.push_back(tmpRing);
						 }
					 }
					 }));

			 auto& gridIntersectDatas = compilerData.m_rdsIntersections;
			 getIntersectionBox2TRTree()->query(bgi::intersects(BOX_2T(pCrossWalkPoly)),
				 boost::make_function_output_iterator([&](size_t const& id) {
					 auto tmpRing = gridIntersectDatas[id]._intersectionPoly2T;
					 std::vector<ring_2t> results;
					 bg::intersection(tmpCroassWalkRing, tmpRing, results);
					 if (!results.empty())
					 {
						 point_t nearbyPoint = getNearestPoint(gridIntersectDatas[id]._intersectionPoints, crossWalkCenter);
						 if (std::abs(crossWalkCenter.get<2>() - nearbyPoint.get<2>()) < MAX_HEIGHT)
						 {
							 allRings.push_back(tmpRing);
						 }
					 }
					 }));

			 if (allRings.empty())
				 continue;

			 std::vector<ring_2t> allNewRings;
			 while (!allRings.empty())
			 {
				 ring_2t resultRing = allRings.front();
				 auto copyAllRings = allRings;
				 std::vector<int> indexList;
				 indexList.push_back(0);
				 for (size_t i = 1; i < copyAllRings.size(); ++i)
				 {
					 if (bg::intersects(allRings[i], resultRing))
					 {
						 std::vector<ring_2t> results;
						 bg::union_(allRings[i], resultRing, results);
						 if (results.size() == 1)
						 {
							 resultRing = results.front();
							 indexList.push_back(i);
						 }
						 else
						 {
							//std::cout << "corsswalk reuslts size: " << results.size() << std::endl;
						 }
					 }
				 }
				 allNewRings.push_back(resultRing);

				 std::sort(indexList.rbegin(), indexList.rend());
				 for (const auto& index : indexList) {
					 if (index >= 0 && index < allRings.size()) {
						 allRings.erase(allRings.begin() + index);
					 }
				 }
			 };

			 for (auto item : allNewRings)
			 {
				 std::vector<ring_2t> results;
				 bg::intersection(item, tmpCroassWalkRing, results);
				 intersectResults.insert(intersectResults.end(), results.begin(), results.end());
			 }

			 if (intersectResults.empty())
				 continue;

			 std::vector<Polygon3d> intersectNews;
			 for (size_t i = 0; i < intersectResults.size(); ++i)
			 {
				 Polygon3d tmpRing;
				 for (size_t j = 0; j < intersectResults.at(i).size(); ++j)
				 {
					 point_t p = point_t(
						 intersectResults.at(i).at(j).get<0>(),
						 intersectResults.at(i).at(j).get<1>(),
						 crossWalkCenter.get<2>());
					 std::vector<point_t>::iterator ita, itb;
					 getClosestSeg(p, tmpCrossWalkPoints, ita, itb);
					 segment_t tmpSeg;
					 bg::closest_points(p, segment_t(*ita, *itb), tmpSeg);
					 MapPoint3D64 tmpPoint = MapPoint3D64_make(
						 p.get<0>(),
						 p.get<1>(),
						 tmpSeg.second.get<2>() / 10);
					 tmpRing.vertexes.push_back(tmpPoint);
				 }
				 coordinatesTransform.invert(tmpRing.vertexes.data(), tmpRing.vertexes.size());
				 intersectNews.push_back(tmpRing);
			 }

			 for (size_t i = 0; i < intersectNews.size(); ++i)
			 {
				 Polygon3d polygon = intersectNews.at(i);

				 //确定方向
				 auto tmpPoints = polygon.vertexes;
				 coordinatesTransform.convert(tmpPoints.data(), tmpPoints.size());
				 auto tmpPolyPoints = LINESTRING_T(tmpPoints);
				 point_t tmpCenter;
				 bg::centroid(tmpPolyPoints, tmpCenter);
				 std::vector<segment_t> resultSegments;
				 rtree.query(bgi::nearest(tmpCenter, 1),
					 boost::make_function_output_iterator([&](size_t const& id) {
						 resultSegments.push_back(gridStopLines.at(id));
						 }));

				 double rad = 0.0;
				 if (!resultSegments.empty())
				 {
					 auto tmp = resultSegments.front();

					 //旋转90度
					 //vector_t tmpV = S3_V3(tmp);
					 //vector_t tmpVV = bg::cross_product(tmpV, vector_t(0, 0, 1));
					 //point_t op2 = point_t(
						// tmpVV.get<0>() + tmp.first.get<0>(),
						// tmpVV.get<1>() + tmp.first.get<1>(),
						// tmpVV.get<2>() + tmp.first.get<2>());

					 auto p1 = MapPoint3D64_make(tmp.first.get<0>(), tmp.first.get<1>(), 0);
					 auto p2 = MapPoint3D64_make(tmp.second.get<0>(), tmp.second.get<1>(), 0);
					 coordinatesTransform.invert(&p1, 1);
					 coordinatesTransform.invert(&p2, 1);
					 auto mapPt1 = MapPoint64::make(p1.pos.lon, p1.pos.lat);
					 auto mapPt2 = MapPoint64::make(p2.pos.lon, p2.pos.lat);
					 rad = Math_getDirectionFromTwoCoordinatesNds(mapPt1.toNdsPoint(), mapPt2.toNdsPoint());
				 }

				 // 斑马线三角化结果
				 std::vector<uint16> polyTriIndex;
				 std::vector<Triangle> polyTriangles;
				 Poly2Tri::triangularize(coordinatesTransform, polygon.vertexes, polyTriIndex, polyTriangles);
				 LinearInterpolationTriangleSurface::triangularizeWMT(coordinatesTransform, polygon.vertexes, polyTriangles);

				 // 基于路面和路口三角化结果对斑马线高度调整
				 std::vector<MapPoint3D64> lineOnSurface;
				 LinearInterpolationTriangleSurface::interpolationPolygon(coordinatesTransform, surfaceTriangles, polyTriangles, polygon.vertexes, lineOnSurface);
				 polygon.vertexes = lineOnSurface;

				 //判断自相交，打印日志并不输出rds
				 if (Poly2Tri::isSelfIntersections(polygon.vertexes))
				 {
					 //std::cout << "crosswalk is self intersection: " << "(" << polygon.vertexes.front().pos.lon / 1000 << "," << polygon.vertexes.front().pos.lat / 1000 << ")" << std::endl;
					 continue;
				 }

				 //生成RDS
				 RdsCrossWalk* pCrossWalk = (RdsCrossWalk*)createObject(pTile, EntityType::RDS_CROSSWALK);
				 convert(polygon, pCrossWalk->contour);
				 rad = rad < 0 ? rad + 2 * PI : rad;
				 pCrossWalk->zebraHeading = rad * 1e5;
				 for (HadLaneGroup* laneGroup : pcw->laneGroups) {
					 RdsGroup* pRdsGroup = queryGroup(laneGroup->originId, pTile);
					 if (pRdsGroup) {
						 pRdsGroup->objects.push_back(pCrossWalk);
					 }
				 }
			 }
		 }
	 }

 }