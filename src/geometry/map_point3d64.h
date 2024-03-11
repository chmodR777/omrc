#pragma once

#include <functional>

#include "geometry/map_point64.h"
#include "math3d/dvector3.h"

/*
	高精度3D坐标点。
	经纬度单位：1/10^8 度；海拔单位：厘米。
*/
struct MapPoint3D64
{
	MapPoint64 pos;
	int32 z;

	forceinline MapPoint3D64& operator = (const MapPoint3D64& mp3D)//赋值运算符
	{
		if (this != &mp3D)
		{
			this->pos = mp3D.pos;
			this->z = mp3D.z;
		}
		return *this;
	}

	forceinline DVector3 toDVector3() const { return dvec3(double(pos.lon), double(pos.lat), double(z)); }

	forceinline void invalidate() { pos.lon = pos.lat = 0; z = INT_MAX; }

	forceinline bool valid() { return z != INT_MAX; }
};


using MapPoint3D64Ref = std::reference_wrapper<MapPoint3D64>;

static forceinline MapPoint3D64 MapPoint3D64_make(int64 lon, int64 lat, int32 z) { MapPoint3D64 mp; mp.pos.lon = lon; mp.pos.lat = lat; mp.z = z; return mp; }

static forceinline MapPoint3D64 MapPoint3D64_make(const DVector3& pt) { return MapPoint3D64{ MapPoint64{int64(pt.x), int64(pt.y)}, int32(pt.z) }; }

forceinline std::vector<MapPoint3D64>::iterator mapPoint3D64_iterator(std::vector<MapPoint3D64>& points, MapPoint3D64& pt) {
	return std::find_if(points.begin(), points.end(), [&](MapPoint3D64& p) {
		return &p == &pt;
		});
};

forceinline bool operator == (MapPoint3D64 l, MapPoint3D64 r) {
	return l.pos == r.pos && l.z == r.z;
}

forceinline bool operator != (MapPoint3D64 l, MapPoint3D64 r) {
	return l.pos != r.pos || l.z != r.z;
}

forceinline bool mapPoint3D64_compare(MapPoint3D64 a, MapPoint3D64 b) { 
	return a == b; 
}

forceinline MapPoint3D64 operator - (const MapPoint3D64& l, const MapPoint3D64& r)
{
	MapPoint3D64 point3d;
	point3d.pos = MapPoint64::make(l.pos.lon - r.pos.lon, l.pos.lat - r.pos.lat);
	point3d.z = l.z - r.z;
	return point3d;
}

forceinline MapPoint3D64 operator + (const MapPoint3D64& l, const MapPoint3D64& r)
{
	MapPoint3D64 point3d;
	point3d.pos = MapPoint64::make(l.pos.lon + r.pos.lon, l.pos.lat + r.pos.lat);
	point3d.z = l.z + r.z;
	return point3d;
}

forceinline bool operator < (const MapPoint3D64& l, const MapPoint3D64& r)
{
	if (l.pos.lon != r.pos.lon)
		return l.pos.lon < r.pos.lon;
	if (l.pos.lat != r.pos.lat)
		return l.pos.lat < r.pos.lat;
	return l.z < r.z;
}

forceinline MapPoint3D64 operator * (MapPoint3D64 pt, double scalar) {
	return MapPoint3D64_make(pt.pos.lon * scalar, pt.pos.lat * scalar, pt.z * scalar);
}

forceinline double dot(const MapPoint3D64& l, const MapPoint3D64& r) {
	return l.pos.lon * r.pos.lon + l.pos.lat * r.pos.lat;
}

/**
 * @param perc 范围在[0.0, 1.0]之间
*/
forceinline MapPoint3D64 mapPoint3D64_lerp(MapPoint3D64 point, MapPoint3D64 nextPoint, double perc)
{
	double dx = double(nextPoint.pos.lon) - double(point.pos.lon);
	double dy = double(nextPoint.pos.lat) - double(point.pos.lat);
	double dz = double(nextPoint.z) - double(point.z);
	double newx = double(point.pos.lon) + dx * perc;
	double newy = double(point.pos.lat) + dy * perc;
	double newZ = double(point.z) + dz * perc;
	return MapPoint3D64{ MapPoint64{int64(std::round(newx)), int64(std::round(newy))}, int32(std::round(newZ)) };
}

/**
 * @brief 判断两点在平面上距离是否小于指定容许距离(tolerance)
 * @return 
 */
forceinline bool mapPoint3D64_equalEpsilon2D(const MapPoint3D64& p1, const MapPoint3D64& p2, int64 tolerance)
{
	int64 dx = p1.pos.lon - p2.pos.lon;
	int64 dy = p1.pos.lat - p2.pos.lat;
	int64 sqDist = dx * dx + dy * dy;
	return sqDist < (tolerance * tolerance);
}
