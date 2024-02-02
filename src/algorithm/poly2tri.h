#pragma once
#include <vector>
#include "math3d/vector2.h"
#include "map_point3d64_converter.h"
#include "geometry/map_point3d64.h"
#include "tool_kit/boost_geometry_utils.h"
class Poly2Tri
{
public:
	static bool triangularize(
		MapPoint3D64Converter& mapPointConverter,
		std::vector<MapPoint3D64>& polygon,
		std::vector<uint16>& triangleIndex,
		std::vector<Triangle>& polyTriangles);

	static bool triangularize(
		std::vector<MapPoint3D64>& polygon,
		std::vector<uint16>& triangleIndex,
		std::vector<Triangle>& polyTriangles);

	static bool isSelfIntersections(
		std::vector<MapPoint3D64>& polygon);

private:
	static bool doPoly2tri(
		std::vector<MapPoint3D64>& polygon,
		std::vector<uint16>& triangleIndex,
		std::vector<Triangle>& polyTriangles);
};