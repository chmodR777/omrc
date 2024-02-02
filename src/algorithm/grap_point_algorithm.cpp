#include "stdafx.h"
#include "grap_point_algorithm.h"
#include "math3d\dvector2.h"
#include "math3d/vector_math.h"
#include "map_rect64.h"
double GrapPointAlgorithm::geoLengthD(const NdsPoint& l, const NdsPoint& r)
{
	double lon1, lat1, lon2, lat2;
	NdsPoint_toDouble(l, &lon1, &lat1);
	NdsPoint_toDouble(r, &lon2, &lat2);
	const static double degree2Radian = MATH_PI_D / 180.0;
	double radLat1 = lat1 * degree2Radian;
	double radLat2 = lat2 * degree2Radian;
	double a = radLat1 - radLat2;
	double b = lon1 * degree2Radian - lon2 * degree2Radian;

	double sina = sin(a / 2);
	double sinb = sin(b / 2);

	double s = 2 * asin(sqrt(sina * sina + cos(radLat1) * cos(radLat2) * sinb * sinb));
	s = s * 6378137.0;// 取WGS84标准参考椭球中的地球长半径(单位:m)
	return s;
}

bool GrapPointAlgorithm::grapPoint(const MapPoint3D64&pt, const MapPoint3D64* points, size_t pointCount, MapPoint3D64& grappedPt, size_t& si, size_t& ei, double tolerance/* = 10.0*/)
{
	NdsPoint ndsPt = pt.pos.toNdsPoint();
	int32 dz = pt.z;
	double distance = 1e20;

	auto length = [&](const MapPoint3D64& point)-> double {

		NdsPoint ndsPoint = point.pos.toNdsPoint();
		double hor = geoLengthD(ndsPt, ndsPoint) * 100.0;
		int ver = point.z - pt.z;

		return hor * hor + (double)ver * ver;
	};
	bool status = false;

	for (size_t i = 0; i < pointCount; i++)
	{
		MapPoint3D64 point = points[i];
		if (i == pointCount - 1)
		{
			MapPoint3D64 p;
			if (grapPoint(pt, points[i], points[i - 1], p, tolerance))
			{
				double d = length(p);
				if (d < distance)
				{
					si = i - 1;
					ei = i;
					distance = d;
					grappedPt = p;
					status = true;
				}
			}
		}
		else
		{
			MapPoint3D64 p;
			if (grapPoint(pt, points[i], points[i + 1], p, tolerance))
			{
				double d = length(p);
				if (d < distance)
				{
					si = i;
					ei = i + 1;
					distance = d;
					grappedPt = p;
					status = true;
				}
			}
		}
	}
	return status;
}

bool GrapPointAlgorithm::grapPoint(const MapPoint3D64&pt, const MapPoint3D64& st, const MapPoint3D64& end, MapPoint3D64& grappedPt, double tolerance/* = 10.0*/)
{
	if (st.pos == end.pos) {
		grappedPt = st;
		return false;
	}

	DVector2 v1;
	v1.x = end.pos.lon - st.pos.lon;
	v1.y = end.pos.lat - st.pos.lat;

	DVector2 v2;
	v2.x = pt.pos.lon - st.pos.lon;
	v2.y = pt.pos.lat - st.pos.lat;

	double length = v1.length();

	v1.normalize();
	double l = dot(v1, v2);

	double al = std::abs(l);

	//if (st == end) {
	//	grappedPt = st;
	//	return false;
	//}

	if (al < tolerance)
	{
		grappedPt = st;
		return true;
	}

	if (l < 0)
	{
		grappedPt = st;
		return false;
	}

	if (std::abs(l - length) < tolerance)
	{
		grappedPt = end;
		return true;
	}

	if (l > length)
	{
		grappedPt = end;
		return false;
	}

	DVector2 p = v1*l;
	p.x += st.pos.lon;
	p.y += st.pos.lat;

	grappedPt.pos.lon = p.x;
	grappedPt.pos.lat = p.y;

	grappedPt.z = st.z + (end.z - st.z)*l / length;

	return true;
}

bool GrapPointAlgorithm::grapPoint(const MapPoint3D64& pt, const std::vector<MapPoint3D64>& vPoints, MapPoint3D64& grappedPt, size_t& si, size_t& ei, double tolerance/* = 10.0*/)
{
	size_t pointCount = vPoints.size();
	return GrapPointAlgorithm::grapPoint(pt, (MapPoint3D64*)vPoints.data(), pointCount, grappedPt, si, ei, tolerance);
}


bool GrapPointAlgorithm::isOnLeft(const MapPoint3D64& point, MapPoint3D64* points, size_t pointCount)
{
	size_t si = 0;
	size_t ei = 0;
	double distance = 1e10;

	NdsPoint ndsPoint = point.pos.toNdsPoint();
	int32 dz = point.z;
	for (size_t i = 0; i < pointCount; i++)
	{
		MapPoint3D64 pl = points[i];
		NdsPoint ndsPl = pl.pos.toNdsPoint();
		double hor = GrapPointAlgorithm::geoLengthD(ndsPl, ndsPoint) * 100.0;
		int ver = point.z - pl.z;
		double d = hor * hor + (double)ver * ver;
		if (d < distance)
		{
			distance = d;
			if (i == pointCount - 1)
			{
				si = i - 1;
				ei = i;
			}
			else
			{
				si = i;
				ei = i + 1;
			}
		}

	}
	MapPoint3D64 st = points[si];
	MapPoint3D64 end = points[ei];


	return ((end.pos.lon - point.pos.lon) * (st.pos.lat - point.pos.lat) - (st.pos.lon - point.pos.lon) * (end.pos.lat - point.pos.lat)) < 0;
}


bool GrapPointAlgorithm::isOnLeft(MapPoint3D64* const ps, size_t psc, MapPoint3D64* points, size_t pointCount)
{
	size_t si = 0;
	size_t ei = 0;
	size_t oi = 0;
	double distance = 1e10;

	for (size_t i = 0; i < pointCount; i++)
	{
		MapPoint3D64 pl = points[i];
		int32 dz = pl.z;
		NdsPoint ndsPt = pl.pos.toNdsPoint();

		for (int j = 0; j < psc; j++)
		{
			MapPoint3D64 point = ps[j];
			NdsPoint ndsPoint = point.pos.toNdsPoint();
			double hor = GrapPointAlgorithm::geoLengthD(ndsPt, ndsPoint) * 100.0;
			int ver = point.z - pl.z;
			double d = hor * hor + (double)ver * ver;

			if (d < distance)
			{
				distance = d;
				oi = j;
				if (i == pointCount - 1)
				{
					si = i - 1;
					ei = i;
				}
				else
				{
					si = i;
					ei = i + 1;
				}
			}
		}
	}
	MapPoint3D64 st = points[si];
	MapPoint3D64 end = points[ei];

	MapPoint3D64 pt = ps[oi];
	return ((end.pos.lon - pt.pos.lon) * (st.pos.lat - pt.pos.lat) - (st.pos.lon - pt.pos.lon) * (end.pos.lat - pt.pos.lat)) < 0;
}

bool GrapPointAlgorithm::isOnLeft(const std::vector<MapPoint3D64>& ps, const std::vector<MapPoint3D64>& points)
{
	size_t si = 0;
	size_t ei = 0;
	size_t oi = 0;
	double distance = 1e10;
	std::set<size_t> intersectPointIndex;
	for (size_t i = 0; i < points.size(); i++)
	{
		MapPoint3D64 pl = points[i];
		int32 dz = pl.z;
		NdsPoint ndsPt = pl.pos.toNdsPoint();

		for (int j = 0; j < ps.size(); j++)
		{

			//去除重复点
			if (intersectPointIndex.find(j) != intersectPointIndex.end())
			{
				continue;
			}
			if (pl == ps[j])
			{
				intersectPointIndex.emplace(j);
				continue;
			}


			MapPoint3D64 point = ps[j];
			NdsPoint ndsPoint = point.pos.toNdsPoint();
			double hor = GrapPointAlgorithm::geoLengthD(ndsPt, ndsPoint) * 100.0;
			int ver = point.z - pl.z;
			double d = hor * hor + (double)ver * ver;

			if (d < distance)
			{
				distance = d;
				oi = j;
				if (i == points.size() - 1)
				{
					si = i - 1;
					ei = i;
				}
				else
				{
					si = i;
					ei = i + 1;
				}
			}
		}
	}
	MapPoint3D64 st = points[si];
	MapPoint3D64 end = points[ei];

	MapPoint3D64 pt = ps[oi];
	return ((end.pos.lon - pt.pos.lon) * (st.pos.lat - pt.pos.lat) - (st.pos.lon - pt.pos.lon) * (end.pos.lat - pt.pos.lat)) < 0;
}


MapPoint3D64 GrapPointAlgorithm::nearestEndPoint(const MapPoint3D64& mp, const std::vector<MapPoint3D64>& vertexes)
{
	return GrapPointAlgorithm::nearestEndPoint(mp, vertexes.data(), vertexes.size());
}

MapPoint3D64 GrapPointAlgorithm::nearestEndPoint(const MapPoint3D64& mp, const MapPoint3D64* points, size_t pointCount)
{
	NdsPoint ptCenter = mp.pos.toNdsPoint();
	NdsPoint ptStart = points[0].pos.toNdsPoint();
	NdsPoint ptEnd = points[pointCount-1].pos.toNdsPoint();
	double toStartDis = GrapPointAlgorithm::geoLengthD(ptCenter, ptStart);
	double toEndDis = GrapPointAlgorithm::geoLengthD(ptCenter, ptEnd);
	return (toStartDis < toEndDis ? points[0] : points[pointCount - 1]);
}

bool GrapPointAlgorithm::grapOrMatchNearestEndPoint(const MapPoint3D64& pt, const std::vector<MapPoint3D64>& points, MapPoint3D64& grappedPt, size_t& si, size_t& ei, double tolerance/* = 10.0*/)
{
	return grapOrMatchNearestEndPoint(pt, points.data(), points.size(), grappedPt, si, ei, tolerance);
}

bool GrapPointAlgorithm::grapOrMatchNearestEndPoint(const MapPoint3D64& pt, const MapPoint3D64* points, size_t pointCount, MapPoint3D64& grappedPt, size_t& si, size_t& ei, double tolerance/* = 10.0*/)
{
	bool grapSucceed = GrapPointAlgorithm::grapPoint(pt, points, pointCount, grappedPt, si, ei, tolerance);

	if (!grapSucceed)
		grappedPt = GrapPointAlgorithm::nearestEndPoint(pt, points, pointCount);

	return grapSucceed;
}

bool GrapPointAlgorithm::grapOrMatchNearestPoint(const MapPoint3D64& pt, const std::vector<MapPoint3D64>& points, MapPoint3D64& grappedPt, size_t& si, size_t& ei, double tolerance)
{
	MapPoint3D64 nearestPt = {};
	return grapOrMatchNearestPoint(pt, points, grappedPt, nearestPt, si, ei, tolerance);
}

bool GrapPointAlgorithm::grapOrMatchNearestPoint(const MapPoint3D64& pt, const std::vector<MapPoint3D64>& points, MapPoint3D64& grappedPt, MapPoint3D64& nearestPt, size_t& si, size_t& ei, double tolerance)
{
	int nearestPtIdx = -1;
	bool status = grapOrMatchNearestPoint(pt, points.data(), points.size(), grappedPt, nearestPtIdx, si, ei, tolerance);
	nearestPt = points[nearestPtIdx];
	return status;
}

bool GrapPointAlgorithm::grapOrMatchNearestPoint(const MapPoint3D64& pt, const std::vector<MapPoint3D64>& points, MapPoint3D64& grappedPt, int& nearestPtIdx, size_t& si, size_t& ei, double tolerance)
{
	return grapOrMatchNearestPoint(pt, points.data(), points.size(), grappedPt, nearestPtIdx, si, ei, tolerance);
}

bool GrapPointAlgorithm::grapOrMatchNearestPoint(const MapPoint3D64& pt, const MapPoint3D64* points, size_t pointCount, MapPoint3D64& grappedPt, int& nearestPtIdx, size_t& si, size_t& ei, double tolerance)
{
	float minLengthSquared = FLT_MAX;
	NdsPoint ndsPt = pt.pos.toNdsPoint();
	int32 dz = pt.z;
	double distance = 1e20;

	auto length = [&](const MapPoint3D64& point)-> double {

		NdsPoint ndsPoint = point.pos.toNdsPoint();
		double hor = geoLengthD(ndsPt, ndsPoint) * 100.0;
		int ver = point.z - pt.z;

		return hor * hor + (double)ver * ver;
	};
	bool status = false;

	for (size_t i = 0; i < pointCount; i++)
	{
		MapPoint3D64 point = points[i];
		if (i == pointCount - 1)
		{
			MapPoint3D64 p;
			if (grapPoint(pt, points[i], points[i - 1], p, tolerance))
			{
				double d = length(p);
				if (d < distance)
				{
					si = i - 1;
					ei = i;
					distance = d;
					grappedPt = p;
					status = true;
				}
			}
		}
		else
		{
			MapPoint3D64 p;
			if (grapPoint(pt, points[i], points[i + 1], p, tolerance))
			{
				double d = length(p);
				if (d < distance)
				{
					si = i;
					ei = i + 1;
					distance = d;
					grappedPt = p;
					status = true;
				}
			}
		}

		MapPoint3D64 map3d = point - pt;
		float lengthSquared = vec2(map3d.pos.lon, map3d.pos.lat).lengthSquared();
		if (lengthSquared < minLengthSquared)
		{
			nearestPtIdx = i;
			minLengthSquared = lengthSquared;
		}
	}

	auto& nearestPt = points[nearestPtIdx];
	if (status && nearestPt.pos.distance(pt.pos) < grappedPt.pos.distance(pt.pos)) {
		status = false; // 大圆弧时可能抓到对侧点,此时状态改为false
	}
	if (!status) {
		grappedPt = nearestPt;
	}
	return status;
}

int GrapPointAlgorithm::findNearestPoint(const std::vector<MapPoint3D64>& points, const MapPoint3D64& p)
{
	return findNearestPoint(points.data(), points.size(), p);
}

int GrapPointAlgorithm::findNearestPoint(const MapPoint3D64* points, size_t pointCount, const MapPoint3D64& p)
{
	float minLengthSquared = FLT_MAX;
	int index = -1;
	for (int i = 0; i < pointCount; i++)
	{
		MapPoint3D64 map3d = points[i] - p;
		float lengthSquared = vec2(map3d.pos.lon, map3d.pos.lat).lengthSquared();
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

bool GrapPointAlgorithm::isPointInPolygon(MapPoint3D64& refPt, const std::vector<MapPoint3D64>& polygonVertexes)
{
	int counter = 0;
	for (int idx = 0; idx < polygonVertexes.size() - 1; idx++) {
		auto& currPt = polygonVertexes[idx];
		auto& nextPt = polygonVertexes[idx + 1];
		if ((refPt.pos.lat >= currPt.pos.lat && refPt.pos.lat <= nextPt.pos.lat) ||
			(refPt.pos.lat >= nextPt.pos.lat && refPt.pos.lat <= currPt.pos.lat)) {
			double t = double(refPt.pos.lat - currPt.pos.lat) / double(nextPt.pos.lat - currPt.pos.lat);
			double xt = currPt.pos.lon + t * (nextPt.pos.lon - currPt.pos.lon);
			if (refPt.pos.lon == xt) { // online
				return true;
			}
			if (refPt.pos.lon < xt) {
				++counter;
			}
		}
	}

	return counter % 2;
}

bool GrapPointAlgorithm::GeometryUtil_findIndexWithLine(const MapPoint3D64* points, size_t ptNum, MapPoint3D64& p, size_t& si, size_t& ei)
{
	MapRect64 rect;
	for (size_t i = 0; i < ptNum - 1; i++)
	{
		if (points[i].pos == p.pos || points[i + 1].pos == p.pos)
		{
			if (points[i].pos == p.pos)
			{
				p.z = points[i].z;
			}
			if (points[i + 1].pos == p.pos)
			{
				p.z = points[i + 1].z;
			}

			si = i;
			ei = i + 1;
			return true;
		}

		rect.setAsNegtiveMinimum();
		rect.combinePoint(points[i].pos);
		rect.combinePoint(points[i + 1].pos);


		rect.expand(1);

		if (rect.PointInRect(p.pos))
		{
			double l = 1.0;
			{
				DVector2 v1 = dvec2(points[i].pos.lon, points[i].pos.lat);
				DVector2 v2 = dvec2(points[i + 1].pos.lon, points[i + 1].pos.lat);
				DVector2 vp = dvec2(p.pos.lon, p.pos.lat);

				double lp = (vp - v1).length();
				double lv12 = (v2 - v1).length();
				l = (vp - v1).length() / (v2 - v1).length();
			}



			DVector3 v1 = dvec3(points[i].pos.lon, points[i].pos.lat, points[i].z);
			DVector3 v2 = dvec3(points[i + 1].pos.lon, points[i + 1].pos.lat, points[i + 1].z);
			DVector3 vs = v2 - v1;
			vs = vs * l;
			vs = vs + v1;

			p.z = vs.z;

			si = i;
			ei = i + 1;


			return true;
		}
	}

	return false;
}
