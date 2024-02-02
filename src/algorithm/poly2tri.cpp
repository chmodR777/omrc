#include "stdafx.h"
#include <algorithm>
#include <unordered_map>
#include "poly2tri.h"
#include "poly2tri/poly2tri.h"

bool Poly2Tri::triangularize(MapPoint3D64Converter& mapPointConverter, std::vector<MapPoint3D64>& polygon, std::vector<uint16>& triangleIndex, std::vector<Triangle>& polyTriangles)
{
	auto ip = std::unique(polygon.begin(), polygon.end(), [](auto& a, auto& b) {return a.pos == b.pos; });
	polygon.resize(std::distance(polygon.begin(), ip));

	if (!isSelfIntersections(polygon))
	{
		auto tmpPolygon = polygon;
		mapPointConverter.convert(tmpPolygon.data(), tmpPolygon.size());
		try
		{
			return doPoly2tri(tmpPolygon, triangleIndex, polyTriangles);
		}
		catch (const std::exception&)
		{
			return false;
		}
	}
	return false;
}

bool Poly2Tri::triangularize(std::vector<MapPoint3D64>& polygon, std::vector<uint16>& triangleIndex, std::vector<Triangle>& polyTriangles)
{
	auto ip = std::unique(polygon.begin(), polygon.end(), [](auto& a, auto& b) {return a.pos == b.pos; });
	polygon.resize(std::distance(polygon.begin(), ip));

	if (!isSelfIntersections(polygon))
	{
		try
		{
			return doPoly2tri(polygon, triangleIndex, polyTriangles);
		}
		catch (const std::exception&)
		{
			return false;
		}
	}
	return false;
}

bool Poly2Tri::isSelfIntersections(std::vector<MapPoint3D64>& polygon)
{
	auto polygon2t = LINESTRING_2T(polygon);
	if (polygon2t.front() == polygon2t.back()) {
		polygon2t.pop_back();
	}
	return bg::intersects(polygon2t);
}

bool Poly2Tri::doPoly2tri(std::vector<MapPoint3D64>& polygon, std::vector<uint16>& triangleIndex, std::vector<Triangle>& polyTriangles)
{
	auto tmpPolygonPoints = polygon;
	int pSize = polygon.size();
	if (polygon.front().pos == polygon.back().pos) {
		pSize--;
	}

	std::vector<p2t::Point*> polyline;
	std::unordered_map<p2t::Point*, int> pointToId;
	for (auto idx = 0; idx < pSize; idx++) {
		auto& pos = polygon[idx].pos;
		auto point = new p2t::Point(pos.lon, pos.lat);
		pointToId.emplace(point, idx);
		polyline.push_back(point);
	}

	p2t::CDT cdt{ polyline };
	cdt.Triangulate();
	for (auto& tri : cdt.GetTriangles()) {
		Triangle triangle{};
		for (int i = 0; i < 3; i++) {
			auto idx = pointToId[tri->GetPoint(i)];
			triangleIndex.push_back(idx);

			auto vertex = POINT_T(polygon[idx]);
			triangle.vertexes[i] = vertex;
			auto tmpPt = P3_P2(vertex);
			triangle.triangleLine.push_back(tmpPt);
			triangle.trianglePoly.push_back(tmpPt);
		}
		bg::correct(triangle.trianglePoly);
		if (triangle.trianglePoly.size() == 4)
			polyTriangles.push_back(triangle);
	}

	//std::vector<ring_2t> tmpRings;
	//for (auto& tri : cdt.GetTriangles()) {
	//	ring_2t tmpRing;
	//	for (int i = 0; i < 3; i++) {
	//		auto pt = tri->GetPoint(i);
	//		tmpRing.push_back(point_2t(pt->x, pt->y));
	//	}
	//	bg::correct(tmpRing);
	//	tmpRings.push_back(tmpRing);
	//}

	for (auto point : polyline) {
		delete point;
	}

	// (12065991,3138557)
	// 三角化函数有bug,此处做一个简单的过滤
	if (pSize > 3 && triangleIndex.size() <= 3) {
		triangleIndex.clear();
		polyTriangles.clear();
		return false;
	}
	return true;
}

