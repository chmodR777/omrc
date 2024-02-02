#include "stdafx.h"
#include "geometry_utils.h"
#include "math3d/coordinate_converter.h"
#include "cqstl/cq_algorithm.h"
#include "geometry/map_rect64.h"

namespace hadc
{
	int GeometryUtil_findNearestPoint(const Vector3* points, int n, Vector2 p)
	{
		float minLengthSquared = FLT_MAX;
		int index = 0;
		for (int i = 0; i < n; i++)
		{
			float lengthSquared = (vec2(points[i]) - p).lengthSquared();
			if (floatEqual(lengthSquared, 0))
				return i;
			if (lengthSquared < minLengthSquared)
			{
				index = i;
				minLengthSquared = lengthSquared;
			}
		}

		return index;
	}
	int32 GeometryUtil_calcHeightAdjustmentByPosition(double lon, double lat)
	{
		PointD ptD1 = { lon, lat };
		PointD ptD2 = Math_marsToWgsD(ptD1);
		return (int32)((cq_abs(ptD1.x - ptD2.x) + cq_abs(ptD1.y - ptD2.y)) * 100000);
	}

	static bool _calIntersection(const MapPoint3D64& rayPt, const DVector2& rayDir,
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

	bool GeometryUtil_calRayCutWithLine(const MapPoint3D64* points, size_t ptNum,
		const MapPoint3D64& p, const DVector2& dir, MapPoint3D64& rstPt, double& rstDis)
	{
		bool isSuc = false;
		double minDis = 1000000.0;
		double tmpDis; MapPoint3D64 tmpPt;
		for (size_t i = 0; i < (int)ptNum - 1; i++)
		{
			if (_calIntersection(p, dir, points[i], points[i + 1], tmpPt, tmpDis))
			{
				if (nc_abs(tmpDis) < minDis)
				{
					minDis = nc_abs(tmpDis);
					rstDis = minDis;
					rstPt = tmpPt;
					isSuc = true;
				}
			}
		}

		return isSuc;
	}

	bool GeometryUtil_testIsPointWithinDistanceFromPolyline(const MapPoint3D64& p, const DVector2& dir,
		const MapPoint3D64* points, size_t ptNum, double testDistanceInMeter, MapPoint3D64& rstPt, double& rstDis)
	{
		bool isSuc = true;
		if (GeometryUtil_calRayCutWithLine(points, ptNum, p,
			dir, rstPt, rstDis) && (nc_abs(rstDis) * 0.001f * METER_PER_LAT_UNIT < testDistanceInMeter)) {
		}
		else if (GeometryUtil_calRayCutWithLine(points, ptNum, p,
			dir * -1, rstPt, rstDis) && (nc_abs(rstDis) * 0.001f * METER_PER_LAT_UNIT < testDistanceInMeter)) {
		}
		else
			isSuc = false;

		rstDis = nc_abs(rstDis);
		return isSuc;
	}

	// 计算线段和点之间的距离，返回值单位：cm
	static int64 _segmentPointDistance(MapPoint64 l0, MapPoint64 l1, MapPoint64 p)
	{
		NdsPoint projectionPos;
		Math_segmentPointDistanceSquaredFNds(l0.toNdsPoint(), l1.toNdsPoint(), p.toNdsPoint(), &projectionPos);
		return int64(Math_segGeoLengthNdsF(p.toNdsPoint(), projectionPos) * 100 + 0.5);
	}

	// 计算Polyline和点之间的最近距离，返回值单位：cm
	int64 GeometryUtil_calcDistanceBetweenPointAndPolyline(MapPoint64 pos, const MapPoint3D64* pts, size_t ptCount)
	{
		CQ_ASSERT(ptCount > 1);
		int64 distance = INT64_MAX;
		for (size_t i = 0; i < ptCount - 1; i++)
		{
			int64 dist = _segmentPointDistance(pts[i].pos, pts[i + 1].pos, pos);
			if (dist < distance)
				distance = dist;
		}
		return distance;
	}

	forceinline static int32 _calcIntersectionHeight(double backward, double length, int32 prevHeight, int32 nextHeight)
	{
		if (cq_abs(length) < CQ_EPSILON)
			return prevHeight;

		return int32(backward / length * (nextHeight - prevHeight) + prevHeight);
	}

	void PolylineGrabParams::reset()
	{
		position.lon = 0;
		position.lat = 0;
		queryAngle = JUST_ANY_ANGLE;
		deviationFactor = 1;
		angleDeviationFactor = 0;
	}

	static void _grabPolyline(const PolylineGrabParams& params, const MapPoint3D64* pts, size_t ptCount, PolylineGrabResult& resultOut)
	{
		CQ_ASSERT(pts != nullptr && ptCount > 1);

		int interPointId = 0;
		// 初始化为一个较大值
		double minDeviation = 1e50;
		NdsPoint intersectionPoint = NdsPoint_make(0, 0);
		double totalLength = 0;
		cqstd::vector<double> segmentLenghtList;
		segmentLenghtList.resize(ptCount - 1);

		for (size_t i = 0; i < ptCount - 1; ++i)
		{
			NdsPoint projectionPoint;
			Math_segmentPointDistanceSquaredFNds(pts[i].pos.toNdsPoint(), pts[i + 1].pos.toNdsPoint(), params.position.toNdsPoint(), &projectionPoint);
			double deviation = Math_segGeoLengthNdsF(projectionPoint, params.position.toNdsPoint());
			segmentLenghtList[i] = Math_segGeoLengthNdsF(pts[i].pos.toNdsPoint(), pts[i + 1].pos.toNdsPoint());
			totalLength += segmentLenghtList[i];
			if (deviation < minDeviation)
			{
				minDeviation = deviation;
				intersectionPoint = projectionPoint;
				interPointId = (int)i;
			}
		}

		resultOut.interPointId = (uint16)interPointId;
		resultOut.deviation = int(minDeviation * 100 + 0.5); // 转成 cm 为单位
		resultOut.intersection.pos.fromNdsPoint(intersectionPoint);
		float radianAngle = 0;
		if (interPointId != 0 && intersectionPoint == pts[interPointId].pos.toNdsPoint())
		{
			// link的每一对相邻的shapePoint组成一个segment，如果intersectionPoint是中间segment连接的点
			// 则intersectionDir取前后两个segment的方向的均值
			float prevAngle = Math_getDirectionFromTwoCoordinatesNds(pts[interPointId - 1].pos.toNdsPoint(), pts[interPointId].pos.toNdsPoint());
			float nextAngle = Math_getDirectionFromTwoCoordinatesNds(pts[interPointId].pos.toNdsPoint(), pts[interPointId + 1].pos.toNdsPoint());
			radianAngle = (prevAngle + nextAngle) / 2;
		}
		else if (interPointId != (int)(ptCount - 2) && intersectionPoint == pts[interPointId + 1].pos.toNdsPoint())
		{
			float prevAngle = Math_getDirectionFromTwoCoordinatesNds(pts[interPointId].pos.toNdsPoint(), pts[interPointId + 1].pos.toNdsPoint());
			float nextAngle = Math_getDirectionFromTwoCoordinatesNds(pts[interPointId + 1].pos.toNdsPoint(), pts[interPointId + 2].pos.toNdsPoint());
			radianAngle = (prevAngle + nextAngle) / 2;
		}
		else
		{
			radianAngle = Math_getDirectionFromTwoCoordinatesNds(pts[interPointId].pos.toNdsPoint(), pts[interPointId + 1].pos.toNdsPoint());
		}
		// 转成角度
		resultOut.intersectionDir = (uint16)(((int32)(radianAngle * 180 / MATH_PI) % 360 + 360) % 360);
		CQ_ASSERT(resultOut.intersectionDir >= 0 && resultOut.intersectionDir < 360);

		resultOut.angleDeviation = 0;
		if (params.queryAngle != JUST_ANY_ANGLE)
		{
			// 确保转到 [0, 360) 区间
			int32 fromAngle = (((int32)params.queryAngle % 360) + 360) % 360;
			// angleDeviation 的区间是 [-180, 180]
			resultOut.angleDeviation = Math_getTurnAngle(fromAngle, resultOut.intersectionDir);
			CQ_ASSERT(resultOut.angleDeviation <= 180 && resultOut.angleDeviation >= -180);
		}

		double disBackward = 0;
		for (int i = 0; i < resultOut.interPointId; ++i)
		{
			disBackward += segmentLenghtList[i];
		}
		double sectionBackward = Math_segGeoLengthNdsF(pts[interPointId].pos.toNdsPoint(), intersectionPoint);
		disBackward += sectionBackward;
		resultOut.disBackward = uint32(disBackward * 100 + 0.5);  // 转成 cm 为单位
		resultOut.disForward = uint32(totalLength * 100 + 0.5) - resultOut.disBackward;
		resultOut.intersection.z = _calcIntersectionHeight(sectionBackward, segmentLenghtList[interPointId], pts[interPointId].z, pts[interPointId + 1].z);

		CQ_ASSERT(resultOut.disForward >= 0);
		if (resultOut.deviation == 0)
		{
			resultOut.sideness = Sideness_unknown;
		}
		else
		{
			const MapPoint64& pre = pts[resultOut.interPointId].pos;
			const MapPoint64& latter = pts[resultOut.interPointId + 1].pos;
			MapPoint64 v1 = { pre.lon - params.position.lon, pre.lat - params.position.lat };
			MapPoint64 v2 = { latter.lon - params.position.lon, latter.lat - params.position.lat };
			int64 crossProduct = v1.lon * v2.lat - v2.lon * v1.lat;
			if (crossProduct < 0)
				resultOut.sideness = Sideness_right;
			else if (crossProduct > 0)
				resultOut.sideness = Sideness_left;
			else
				resultOut.sideness = Sideness_unknown;
		}
		resultOut.score = params.deviationFactor * resultOut.deviation + params.angleDeviationFactor * resultOut.angleDeviation * 100;
	}

	void GeometryUtil_grabPolyline(const PolylineGrabParams& params, const MapPoint3D64* pts, size_t ptCount, PolylineGrabResult& resultOut)
	{
		_grabPolyline(params, pts, ptCount, resultOut);
	}

	void GeometryUtil_grabPolyline(const PolylineGrabParams& params, const Polyline& polyline, PolylineGrabResult& resultOut)
	{
		GeometryUtil_grabPolyline(params, polyline.pts, polyline.ptCount, resultOut);
	}

	void GeometryUitl_grabMultiPolylines(const PolylineGrabParams& params, 
		const Polyline* polylines, size_t lineCount, cqstd::vector<PolylineGrabResult>& resultsOut)
	{
		resultsOut.resize(lineCount);

		for (size_t i = 0; i < lineCount; ++i)
		{
			const MapPoint3D64* pts = polylines[i].pts;
			size_t ptCount = polylines[i].ptCount;
			_grabPolyline(params, pts, ptCount, resultsOut[i]);
		}

		if (resultsOut.empty())
			return;

		// 按照score排序
		cq::sortWithComparator<PolylineGrabResult>(resultsOut.begin(), resultsOut.end(),
			[](const PolylineGrabResult& l, const PolylineGrabResult& r, void* /*context*/)
			{ return (int)((int64)l.score - (int64)r.score); }, nullptr);
	}
}

