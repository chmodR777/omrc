#pragma once
#include "math3d/vector2.h"
#include <vector>
#include "geometry/map_point3d64.h"
class PolylineIntersector
{
public:
	PolylineIntersector();
	~PolylineIntersector();

	static bool intersect(const Vector2& pointStart, const Vector2& pointEnd, const std::vector<Vector2>& points, float tolerance, Vector2& point, size_t& si, size_t& ei);
	static int intersect(const Vector2& p0, const Vector2& p1, const Vector2& p2, const Vector2 p3, float tolerance, Vector2& point, bool extend/* = true*/);

	static bool intersect(const MapPoint3D64& pointStart, const MapPoint3D64& pointEnd, const std::vector<MapPoint3D64>& points, double tolerance, std::vector<MapPoint3D64>& intersectPoints);
	static bool intersect(const MapPoint3D64& pointStart, const MapPoint3D64& pointEnd, const std::vector<MapPoint3D64>& points, double tolerance, MapPoint3D64& point, size_t& si, size_t& ei);
	static int intersect(const MapPoint3D64& p0, const MapPoint3D64& p1, const MapPoint3D64& p2, const MapPoint3D64 p3, double tolerance, MapPoint3D64& point, bool extend/* = true*/);
};