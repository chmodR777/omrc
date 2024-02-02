#include "stdafx.h"
#include "map_point3d64_converter.h"
#include "math3d/coordinate_converter.h"

void MapPoint3D64Converter::setBasePoint(const MapPoint3D64& basePt)
{
	m_basePt = basePt;
	m_x2yRatio = coordinate_converter::calcLonLatScale(m_basePt.pos.toNdsPoint());
}

void MapPoint3D64Converter::convert(MapPoint3D64* pts, size_t num)
{
	for (size_t i = 0; i < num; i++)
	{
		pts[i].pos.lon = pts[i].pos.lon - m_basePt.pos.lon;
		pts[i].pos.lon = int64(pts[i].pos.lon * m_x2yRatio);
		pts[i].pos.lat = pts[i].pos.lat - m_basePt.pos.lat;
	}
}

void MapPoint3D64Converter::invert(MapPoint3D64* pts, size_t num)
{
	for (size_t i = 0; i < num; i++)
	{
		pts[i].pos.lon = int64(pts[i].pos.lon / m_x2yRatio);
		pts[i].pos.lon = pts[i].pos.lon + m_basePt.pos.lon;
		pts[i].pos.lat = pts[i].pos.lat + m_basePt.pos.lat;
	}
}