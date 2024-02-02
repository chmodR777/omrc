#pragma once
#include "geometry\map_point3d64.h"
class GrapPointAlgorithm
{
public:
	static double geoLengthD(const NdsPoint& l, const NdsPoint& r);
	//static bool grapPoint(const MapPoint3D64&pt, MapPoint3D64* points, size_t pointCount, MapPoint3D64& grappedPt, size_t& si, size_t& ei);
	static bool grapPoint(const MapPoint3D64& pt, const MapPoint3D64* points, size_t pointCount, MapPoint3D64& grappedPt, size_t& si, size_t& ei, double tolerance = 10.0);
	static bool grapPoint(const MapPoint3D64&pt, const MapPoint3D64& pt1, const MapPoint3D64& pt2, MapPoint3D64& grappedPt, double tolerance = 10.0);

	static bool isOnLeft(const MapPoint3D64& point, MapPoint3D64* points, size_t pointCount);

	static bool isOnLeft(MapPoint3D64* const ps,size_t psc, MapPoint3D64* points, size_t pointCount);
	static bool isOnLeft(const std::vector<MapPoint3D64>& ps, const std::vector<MapPoint3D64>& points);

	static bool grapPoint(const MapPoint3D64& pt, const std::vector<MapPoint3D64>& points, MapPoint3D64& grappedPt, size_t& si, size_t& ei, double tolerance = 10.0);


	//////////////////////////////////////////////////////////
	/// @brief 匹配得到折线两个端点中最近的端点。
	//////////////////////////////////////////////////////////
	static MapPoint3D64 nearestEndPoint(const MapPoint3D64& mp, const std::vector<MapPoint3D64>& vertexes);
	static MapPoint3D64 nearestEndPoint(const MapPoint3D64& mp, const MapPoint3D64* points, size_t pointCount);

	//////////////////////////////////////////////////////////
	/// @brief 抓路或者匹配最近的端点。
	/// @return 抓路成功返回true；抓路失败返回false。
	/// @warring 
	///		仅当抓路失败时，才会匹配端点，此时grappedPt为匹配结果，即points一侧的端点。
	///		返回值成功与否代表抓路成功与否！与匹配结果无关。
	//////////////////////////////////////////////////////////
	static bool grapOrMatchNearestEndPoint(const MapPoint3D64& pt, const std::vector<MapPoint3D64>& points, MapPoint3D64& grappedPt, size_t& si, size_t& ei, double tolerance = 10.0);
	static bool grapOrMatchNearestEndPoint(const MapPoint3D64& pt, const MapPoint3D64* points, size_t pointCount, MapPoint3D64& grappedPt, size_t& si, size_t& ei, double tolerance = 10.0);

	// 找最近的点
	static bool grapOrMatchNearestPoint(const MapPoint3D64& pt, const std::vector<MapPoint3D64>& points, MapPoint3D64& grappedPt, size_t& si, size_t& ei, double tolerance = 10.0);
	static bool grapOrMatchNearestPoint(const MapPoint3D64& pt, const std::vector<MapPoint3D64>& points, MapPoint3D64& grappedPt, MapPoint3D64& nearestPt, size_t& si, size_t& ei, double tolerance = 10.0);
	static bool grapOrMatchNearestPoint(const MapPoint3D64& pt, const std::vector<MapPoint3D64>& points, MapPoint3D64& grappedPt, int& nearestPtIdx, size_t& si, size_t& ei, double tolerance = 10.0);
	static bool grapOrMatchNearestPoint(const MapPoint3D64& pt, const MapPoint3D64* points, size_t pointCount, MapPoint3D64& grappedPt, int& nearestPtIdx, size_t& si, size_t& ei, double tolerance = 10.0);
	static int findNearestPoint(const std::vector<MapPoint3D64>& points, const MapPoint3D64& p);
	static int findNearestPoint(const MapPoint3D64* points, size_t pointCount, const MapPoint3D64& p);

	// 是否点在面内
	static bool isPointInPolygon(MapPoint3D64& refPt, const std::vector<MapPoint3D64>& polygonVertexes);
	static bool GeometryUtil_findIndexWithLine(const MapPoint3D64* points, size_t ptNum, MapPoint3D64& p, size_t& si, size_t& ei);
};

