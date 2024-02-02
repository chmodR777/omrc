#include "stdafx.h"
#include "tangential_algorithm.h"
#include "map_point3d64_converter.h"
#include "math3d/coordinate_converter.h"

bool TangentialAlgorithm::calTangentialCut(MapPoint3D64 pt, DVector2 dir, MapPoint3D64* segPts, int segPtNum, MapPoint3D64& rstPt)
{
	MapPoint3D64Converter c;
	c.setBasePoint(pt);
	c.convert(&pt, 1);
	c.convert(segPts, segPtNum);

	double rstDis;
	bool isSuc = calRayCutWithLine(segPts, segPtNum, pt, dvec2(-dir.y, dir.x), rstPt, rstDis);
	c.invert(&rstPt, 1);

	double disInMeter = rstDis * 0.001f * METER_PER_LAT_UNIT;
	return isSuc && (disInMeter >= 0 && disInMeter <= 15.0);
}

bool TangentialAlgorithm::calRayCutWithLine(const MapPoint3D64* points, int ptNum, const MapPoint3D64& p, const DVector2& dir, MapPoint3D64& rstPt, double& rstDis)
{
	bool isSuc = false;
	double minDis = 1000000.0;
	double tmpDis; MapPoint3D64 tmpPt;
	for (int i = 0; i < (int)ptNum - 1; i++)
	{
		if (calIntersection(p, dir, points[i], points[i + 1], tmpPt, tmpDis))
		{
			if (tmpDis >= 0)
			{
				if (tmpDis < minDis)
				{
					minDis = tmpDis;
					rstDis = minDis;
					rstPt = tmpPt;
					isSuc = true;
				}
			}
		}
	}
	return isSuc;
}

bool TangentialAlgorithm::calIntersection(const MapPoint3D64& rayPt, const DVector2& rayDir,
	const MapPoint3D64& aPt, const MapPoint3D64& bPt, MapPoint3D64& rstPt, double& rstDis)
{
	//https://rootllama.wordpress.com/2014/06/20/ray-line-segment-intersection-test-in-2d/
	const DVector2& v1 = dvec2(double(rayPt.pos.lon - aPt.pos.lon), double(rayPt.pos.lat - aPt.pos.lat));
	const DVector2& v2 = dvec2(double(bPt.pos.lon - aPt.pos.lon), double(bPt.pos.lat - aPt.pos.lat));
	DVector2 v3 = dvec2(-rayDir.y, rayDir.x);
	v3.normalize();

	double prodInv = 1.0 / dot(v2, v3);
	double t1 = DVector2_cross(v2, v1) * prodInv;
	double t2 = dot(v1, v3) * prodInv;
	if (t2 >= 0 && t2 <= 1)
	{
		const DVector2& clippedVec = v2 * t2;
		rstPt.pos.lon = (int64)(aPt.pos.lon + clippedVec.x);
		rstPt.pos.lat = (int64)(aPt.pos.lat + clippedVec.y);
		rstPt.z = (int32)(aPt.z + (bPt.z - aPt.z) * t2);
		rstDis = t1;
		return true;
	}

	return false;
}