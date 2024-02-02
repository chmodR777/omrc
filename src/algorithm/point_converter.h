#pragma once
#include "math3d/vector2.h"
#include "math3d/vector3.h"
#include <vector>
#include "geometry/map_point64.h"
#include "geometry/map_point3d64.h"

class PointConverter
{
public:
	PointConverter();
	~PointConverter();

	static Vector2 ConvertToVector2(const MapPoint64& pt, const MapPoint64& basePt);

	static MapPoint64 ConvertToMapPoint64(const Vector2& pt);

	static Vector3 MapPoint3D64ToVector3(const MapPoint3D64& pt);
	static MapPoint3D64 Vector3ToMapPoint3D64(const Vector3& pt);

};