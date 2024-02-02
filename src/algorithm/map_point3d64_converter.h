#pragma once

#include "geometry/map_point64.h"
#include "geometry/map_point3d64.h"

/**
	@brief
		subtract the plane-coordinate of a MapPoint3D64 point with a base-point, than change the x-coordinate unit to that of y-coordinate.
*/
class MapPoint3D64Converter
{
public:
	MapPoint3D64Converter() {};
	~MapPoint3D64Converter() {};

	void setBasePoint(const MapPoint3D64& basePt);
	void convert(MapPoint3D64* pts, size_t num);
	void invert(MapPoint3D64* pts, size_t num);

private:
	double m_x2yRatio;
	MapPoint3D64 m_basePt;
};
