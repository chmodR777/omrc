#pragma once

#include "geometry/map_point64.h"

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
};
static forceinline MapPoint3D64 MapPoint3D64_make(int64 lon, int64 lat, int32 z) { MapPoint3D64 mp; mp.pos.lon = lon; mp.pos.lat = lat; mp.z = z; return mp; }

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

forceinline MapPoint3D64 operator * (MapPoint3D64 pt, float scalar) {
	return MapPoint3D64_make(pt.pos.lon * scalar, pt.pos.lat * scalar, pt.z * scalar);
}

forceinline double dot(const MapPoint3D64& l, const MapPoint3D64& r) {
	return l.pos.lon * r.pos.lon + l.pos.lat * r.pos.lat;
}
