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
	/// @brief ƥ��õ����������˵�������Ķ˵㡣
	//////////////////////////////////////////////////////////
	static MapPoint3D64 nearestEndPoint(const MapPoint3D64& mp, const std::vector<MapPoint3D64>& vertexes);
	static MapPoint3D64 nearestEndPoint(const MapPoint3D64& mp, const MapPoint3D64* points, size_t pointCount);

	//////////////////////////////////////////////////////////
	/// @brief ץ·����ƥ������Ķ˵㡣
	/// @return ץ·�ɹ�����true��ץ·ʧ�ܷ���false��
	/// @warring 
	///		����ץ·ʧ��ʱ���Ż�ƥ��˵㣬��ʱgrappedPtΪƥ��������pointsһ��Ķ˵㡣
	///		����ֵ�ɹ�������ץ·�ɹ������ƥ�����޹ء�
	//////////////////////////////////////////////////////////
	static bool grapOrMatchNearestEndPoint(const MapPoint3D64& pt, const std::vector<MapPoint3D64>& points, MapPoint3D64& grappedPt, size_t& si, size_t& ei, double tolerance = 10.0);
	static bool grapOrMatchNearestEndPoint(const MapPoint3D64& pt, const MapPoint3D64* points, size_t pointCount, MapPoint3D64& grappedPt, size_t& si, size_t& ei, double tolerance = 10.0);

	// ������ĵ�
	static bool grapOrMatchNearestPoint(const MapPoint3D64& pt, const std::vector<MapPoint3D64>& points, MapPoint3D64& grappedPt, size_t& si, size_t& ei, double tolerance = 10.0);
	static bool grapOrMatchNearestPoint(const MapPoint3D64& pt, const std::vector<MapPoint3D64>& points, MapPoint3D64& grappedPt, MapPoint3D64& nearestPt, size_t& si, size_t& ei, double tolerance = 10.0);
	static bool grapOrMatchNearestPoint(const MapPoint3D64& pt, const std::vector<MapPoint3D64>& points, MapPoint3D64& grappedPt, int& nearestPtIdx, size_t& si, size_t& ei, double tolerance = 10.0);
	static bool grapOrMatchNearestPoint(const MapPoint3D64& pt, const MapPoint3D64* points, size_t pointCount, MapPoint3D64& grappedPt, int& nearestPtIdx, size_t& si, size_t& ei, double tolerance = 10.0);
	static int findNearestPoint(const std::vector<MapPoint3D64>& points, const MapPoint3D64& p);
	static int findNearestPoint(const MapPoint3D64* points, size_t pointCount, const MapPoint3D64& p);

	// �Ƿ��������
	static bool isPointInPolygon(MapPoint3D64& refPt, const std::vector<MapPoint3D64>& polygonVertexes);
	static bool GeometryUtil_findIndexWithLine(const MapPoint3D64* points, size_t ptNum, MapPoint3D64& p, size_t& si, size_t& ei);
};

