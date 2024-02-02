#pragma once
#include "geometry\map_point3d64.h"
#include "math3d\dvector2.h"

class TangentialAlgorithm
{
public:
	static bool calTangentialCut(MapPoint3D64 pt, DVector2 dir, MapPoint3D64* segPts, int segPtNum, MapPoint3D64& rstPt);

	static bool calRayCutWithLine(const MapPoint3D64* points, int ptNum, const MapPoint3D64& p, 
		const DVector2& dir, MapPoint3D64& rstPt, double& rstDis);

	static bool calIntersection(const MapPoint3D64& rayPt, const DVector2& rayDir,
		const MapPoint3D64& aPt, const MapPoint3D64& bPt, MapPoint3D64& rstPt, double& rstDis);

};

