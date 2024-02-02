#include "stdafx.h"
#include "point_converter.h"
PointConverter::PointConverter()
{

}

PointConverter::~PointConverter()
{

}

Vector2 PointConverter::ConvertToVector2(const MapPoint64& pt,const MapPoint64& basePt)
{
	Vector2 v;
	v.set(pt.lon - basePt.lon, pt.lat - basePt.lat);
	return v;
}

MapPoint64 PointConverter::ConvertToMapPoint64(const Vector2& pt)
{
	MapPoint64 pos;
	pos.lon = (int64)pt.x;
	pos.lat = (int64)pt.y;

	return pos;
}

Vector3 PointConverter::MapPoint3D64ToVector3(const MapPoint3D64& pt)
{
	Vector3 v;
	v.set(pt.pos.lon, pt.pos.lat, pt.z);
	return v;
}

MapPoint3D64 PointConverter::Vector3ToMapPoint3D64(const Vector3& pt)
{
	return MapPoint3D64_make(pt.x+0.5, pt.y+0.5, pt.z+0.5);
}
