#pragma once
#include "geometry/map_point3d64.h"
#include <vector>
class PolylineSimplifier
{
	struct PointEx
	{
		MapPoint3D64* Point;
		int VertexIndex;
	};
	using PointExPtr = std::shared_ptr<PointEx>;

public:
	static void simplify(std::vector<MapPoint3D64>& points,double hor = 2500,double ver = 10);
	static void compressWithZ(const PointExPtr& startPoint, const PointExPtr& endPoint, const std::vector<PointExPtr>& points, const std::vector<double> lengthes, double ver);
	static void compress(const PointExPtr& startPoint, const PointExPtr& endPoint, const std::vector<PointExPtr>& points, const std::vector<double> lengthes, double hor, double ver);
	static void simplifyT(std::vector<MapPoint3D64>& points, double hor, double ver);
	static double distance(const MapPoint64& pt, const MapPoint64& startPoint, const MapPoint64& endPoint);
	static void simplify(const PointExPtr& startPoint, const PointExPtr& endPoint, const std::vector<PointExPtr>& points, const std::vector<double> lengthes, double hor, double ver);
	static void simplifyZ(const PointExPtr& startPoint, const PointExPtr& endPoint, const std::vector<PointExPtr>& points, const std::vector<double> lengthes, double ver);

};

