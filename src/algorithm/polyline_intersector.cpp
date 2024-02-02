#include "stdafx.h"
#include "polyline_intersector.h"
#include "point_converter.h"
PolylineIntersector::PolylineIntersector()
{
}

PolylineIntersector::~PolylineIntersector()
{
}

bool PolylineIntersector::intersect(const Vector2& pointStart, const Vector2& pointEnd, const std::vector<Vector2>& points, float tolerance, Vector2& point, size_t& si, size_t& ei)
{
	if (points.size() < 2)
	{
		return false;
	}
	for (size_t i = 0; i < points.size() - 1; i++)
	{
		Vector2 start = points[i];
		Vector2 end = points[i + 1];

		if (intersect(pointStart, pointEnd, start, end,tolerance, point, false) == 1)
		{
			si = i;
			ei = i + 1;
			return true;
		}
	}

	return false;
}



int PolylineIntersector::intersect(const Vector2& p0, const Vector2& p1, const Vector2& p2, const Vector2 p3, float tolerance, Vector2& point, bool extend/* = true*/)
{
	float A1 = p1.y - p0.y;
	float B1 = p0.x - p1.x;
	float C1 = A1 * p0.x + B1 * p0.y;


	float A2 = p3.y - p2.y;
	float B2 = p2.x - p3.x;
	float C2 = A2 * p2.x + B2 * p2.y;

	if (std::abs(A1 - A2) < tolerance && std::abs(B1 - B2) < tolerance)
	{
		// 平行
		if (std::abs(C1 - C2) < tolerance)
		{
			// 重合
			return 2;
		}
		return 0;
	}

	float denominator = A1 * B2 - A2 * B1;
	float x = (B2 * C1 - B1 * C2) / denominator;
	float y = (A1 * C2 - A2 * C1) / denominator;
	if (denominator == 0)
	{
		return 4;
	}

	float dt = tolerance * tolerance;
	if (extend)
	{
		point.x = x;
		point.y = y;
	}
	else
	{
		if ((x - p0.x) * (x - p1.x) > dt ||
			(y - p0.y) * (y - p1.y) > dt)
		{
			return 3;
		}

		if ((x - p2.x) * (x - p3.x) > dt ||
			(y - p2.y) * (y - p3.y) > dt)
		{
			return 3;
		}
		point.x = x;
		point.y = y;
	}

	return 1;

}

bool PolylineIntersector::intersect(const MapPoint3D64& pointStart, const MapPoint3D64& pointEnd, const std::vector<MapPoint3D64>& points, double tolerance, std::vector<MapPoint3D64>& intersectPoints)
{
	bool flag = false;
	if (points.size() < 2)
	{
		return flag;
	}
	for (size_t i = 0; i < points.size() - 1; i++)
	{
		MapPoint3D64 start = points[i];
		MapPoint3D64 end = points[i + 1];

		MapPoint3D64 point = {};
		if (intersect(pointStart, pointEnd, start, end, tolerance, point, false) == 1)
		{
			intersectPoints.push_back(point);
			flag = true;
		}
	}

	return flag;
}

bool PolylineIntersector::intersect(const MapPoint3D64& pointStart, const MapPoint3D64& pointEnd, const std::vector<MapPoint3D64>& points, double tolerance, MapPoint3D64& point, size_t& si, size_t& ei)
{
	if (points.size() < 2)
	{
		return false;
	}
	for (size_t i = 0; i < points.size() - 1; i++)
	{
		MapPoint3D64 start = points[i];
		MapPoint3D64 end = points[i + 1];

		if (intersect(pointStart, pointEnd, start, end, tolerance, point, false) == 1)
		{
			si = i;
			ei = i + 1;
			return true;
		}
	}

	return false;
}

int PolylineIntersector::intersect(const MapPoint3D64& p0, const MapPoint3D64& p1, const MapPoint3D64& p2, const MapPoint3D64 p3, double tolerance, MapPoint3D64& point, bool extend)
{
	double A1 = p1.pos.lat - p0.pos.lat;
	double B1 = p0.pos.lon - p1.pos.lon;
	double C1 = A1 * p0.pos.lon + B1 * p0.pos.lat;

	double A2 = p3.pos.lat - p2.pos.lat;
	double B2 = p2.pos.lon - p3.pos.lon;
	double C2 = A2 * p2.pos.lon + B2 * p2.pos.lat;

	if (std::abs(A1 - A2) < tolerance && std::abs(B1 - B2) < tolerance)
	{
		// 平行
		if (std::abs(C1 - C2) < tolerance)
		{
			// 重合
			return 2;
		}
		return 0;
	}

	double denominator = A1 * B2 - A2 * B1;
	double x = (B2 * C1 - B1 * C2) / denominator;
	double y = (A1 * C2 - A2 * C1) / denominator;
	if (denominator == 0)
	{
		return 4;
	}

	double dt = tolerance * tolerance;
	if (extend)
	{
		point.pos.lon = x;
		point.pos.lat = y;
	}
	else
	{
		if ((x - p0.pos.lon) * (x - p1.pos.lon) > dt ||
			(y - p0.pos.lat) * (y - p1.pos.lat) > dt)
		{
			return 3;
		}

		if ((x - p2.pos.lon) * (x - p3.pos.lon) > dt ||
			(y - p2.pos.lat) * (y - p3.pos.lat) > dt)
		{
			return 3;
		}
		point.pos.lon = x;
		point.pos.lat = y;
	}

	double d1 = p0.pos.distance(point.pos);
	double d2 = p1.pos.distance(point.pos);
	double sum = d1 + d2;
	point.z = p0.z * d2 / sum + p1.z * d1 / sum;
	return 1;
}
