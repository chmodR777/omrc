#include "stdafx.h"
#include "polyline_simplifier.h"
#include "math3d/dvector2.h"
#include <algorithm>

double PolylineSimplifier::distance(const MapPoint64& pt, const MapPoint64& startPoint, const MapPoint64& endPoint)
{
	DVector2 v1;
	v1.x = endPoint.lon - startPoint.lon;
	v1.y = endPoint.lat - startPoint.lat;

	DVector2 v2;
	v2.x = pt.lon - startPoint.lon;
	v2.y = pt.lat - startPoint.lat;

	v1.normalize();
	double length = dot(v1, v2);

	DVector2 p = v1 * length;
	p.x += startPoint.lon;
	p.y += startPoint.lat;

	MapPoint64 grappedPt;

	grappedPt.lon = p.x;
	grappedPt.lat = p.y;

	double dx = p.x - pt.lon;
	double dy = p.y - pt.lat;
	
	return dx * dx + dy * dy;
}

void PolylineSimplifier::simplifyZ(const PointExPtr& startPoint, const PointExPtr& endPoint, const std::vector<PointExPtr>& points, const std::vector<double> lengthes, double ver)
{
	auto getLength = [&](int start, int end)->double
	{
		if (start > end)
		{
			throw std::invalid_argument("percent : input error argument!");
		}

		double length = 0.0;
		for_each(lengthes.begin() + start, lengthes.begin() + end, [&](auto& d)->void { length += d; });
		return length;
	};

	double max_v = 0.0;
	int max_v_i = -1;

	double total = getLength(startPoint->VertexIndex, endPoint->VertexIndex);
	for (int i = startPoint->VertexIndex + 1; i < endPoint->VertexIndex; i++)
	{
		// 插值计算该点高度
		double p = getLength(startPoint->VertexIndex, i) / total;
		double z = startPoint->Point->z * (1 - p) + endPoint->Point->z * p;
		double dz = std::abs(points[i]->Point->z - z);
		if (dz > max_v)
		{
			max_v = dz;
			max_v_i = i;
		}
	}

	if (max_v < ver)
	{
		for_each(points.begin() + startPoint->VertexIndex + 1, points.begin() + endPoint->VertexIndex, [&](const PointExPtr& point)->void {point->VertexIndex = -1; });
	}
	else
	{
		PointExPtr middle = points[max_v_i];
		simplifyZ(startPoint, middle, points, lengthes, ver);
		simplifyZ(middle, endPoint, points, lengthes, ver);
	}
}

void PolylineSimplifier::simplify(const PointExPtr& startPoint,const PointExPtr& endPoint,const std::vector<PointExPtr>& points,const std::vector<double> lengthes,double hor,double ver)
{
	double max_h = 0.0;
	int max_h_i = -1;

	for (int i = startPoint->VertexIndex + 1; i < endPoint->VertexIndex; i++)
	{
		double d = distance(points[i]->Point->pos, startPoint->Point->pos, endPoint->Point->pos);
		if (d > max_h)
		{
			max_h_i = i;
			max_h = d;
		}
	}

	if (max_h < hor)
	{
		simplifyZ(startPoint, endPoint, points, lengthes, ver);
	}
	else
	{
		PointExPtr middle = points[max_h_i];
		simplify(startPoint, middle, points,lengthes, hor, ver);
		simplify(middle, endPoint, points, lengthes,hor, ver);
	}
}

void PolylineSimplifier::simplify(std::vector<MapPoint3D64>& points,double hor,double ver)
{
	auto execute = [&](std::vector<MapPoint3D64>& points) ->void {
		
		if (points.empty())
			return;

		size_t size = points.size();
		if (size < 3)
			return;

		std::vector<PointExPtr> pointexs;
		pointexs.resize(size);

		std::vector<double> lengthes;
		lengthes.resize(size);
		for (int i = 0; i < size; i++)
		{
			if (i < size - 1)
			{
				int64 dx = points[i + 1].pos.lon - points[i].pos.lon;
				int64 dy = points[i + 1].pos.lat - points[i].pos.lat;
				lengthes[i] = std::sqrt(dx * dx + dy * dy);
				// 			x_lengthes[i] = length;
				// 			length += std::sqrt(dx * dx + dy * dy);
			}

			PointExPtr p(new PointEx());
			p->Point = &(points[i]);
			p->VertexIndex = i;
			pointexs[i] = p;
		}
		lengthes[size - 1] = 0.0;
		// 	x_lengthes[size - 1] = length;
		simplify(pointexs[0], pointexs[size - 1], pointexs, lengthes, hor, ver);
		std::vector<MapPoint3D64> res;
		for_each(pointexs.begin(), pointexs.end(), [&](const PointExPtr& p) {
			if (p->VertexIndex != -1)
			{
				res.push_back(*(p->Point));
			}
			}
		);

		points.clear();
		points.assign(res.begin(), res.end());
	};

	if (points.empty())
		return;

	size_t size = points.size();
	if (size < 3)
		return;

	if (points[0].pos.lon == points[size-1].pos.lon && points[0].pos.lat == points[size - 1].pos.lat)
	{
		std::vector<std::vector<MapPoint3D64>> vmps;

		size_t middle = size / 2;
		std::vector<MapPoint3D64> one(points.begin(),points.begin() + middle + 1);
		std::vector<MapPoint3D64> two(points.begin() + middle + 1,points.end());

		execute(one);
		execute(two);
		
		points.clear();

		points.reserve(one.size() + two.size());

		points.insert(points.end(),one.begin(),one.end());
		points.insert(points.end(), two.begin(), two.end());
	}
	else
	{
		execute(points);
	}
}
double distance(const MapPoint64& pt, const MapPoint64& startPoint, const MapPoint64& endPoint)
{
	DVector2 v1;
	v1.x = endPoint.lon - startPoint.lon;
	v1.y = endPoint.lat - startPoint.lat;
	DVector2 v2;
	v2.x = pt.lon - startPoint.lon;
	v2.y = pt.lat - startPoint.lat;
	v1.normalize();
	double length = dot(v1, v2);
	DVector2 p = v1 * length;
	p.x += startPoint.lon;
	p.y += startPoint.lat;
	MapPoint64 grappedPt;
	grappedPt.lon = p.x;
	grappedPt.lat = p.y;
	double dx = p.x - pt.lon;
	double dy = p.y - pt.lat;

	return dx * dx + dy * dy;
}

void PolylineSimplifier::compressWithZ(const PointExPtr& startPoint, const PointExPtr& endPoint, const std::vector<PointExPtr>& points, const std::vector<double> lengthes, double ver)
{
	auto getLength = [&](int start, int end)->double
	{
		if (start > end)
		{
			throw std::invalid_argument("percent : input error argument!");
		}

		double length = 0.0;
		for_each(lengthes.begin() + start, lengthes.begin() + end, [&](auto& d)->void { length += d; });
		return length;
	};

	double max_v = 0.0;
	int max_v_i = -1;

	double total = getLength(startPoint->VertexIndex, endPoint->VertexIndex);
	for (int i = startPoint->VertexIndex + 1; i < endPoint->VertexIndex; i++)
	{
		// 插值计算该点高度
		double p = getLength(startPoint->VertexIndex, i) / total;
		double z = startPoint->Point->z * (1 - p) + endPoint->Point->z * p;
		double dz = std::abs(points[i]->Point->z - z);
		if (dz > max_v)
		{
			max_v = dz;
			max_v_i = i;
		}
	}

	if (max_v < ver)
	{
		for_each(points.begin() + startPoint->VertexIndex + 1, points.begin() + endPoint->VertexIndex, [&](const PointExPtr& point)->void {point->VertexIndex = -1; });
	}
	else
	{
		PointExPtr middle = points[max_v_i];
		compressWithZ(startPoint, middle, points, lengthes, ver);
		compressWithZ(middle, endPoint, points, lengthes, ver);
	}
}

void PolylineSimplifier::compress(const PointExPtr& startPoint, const PointExPtr& endPoint, const std::vector<PointExPtr>& points, const std::vector<double> lengthes, double hor, double ver)
{
	double max_h = 0.0;
	int max_h_i = -1;

	for (int i = startPoint->VertexIndex + 1; i < endPoint->VertexIndex; i++)
	{
		double d = distance(points[i]->Point->pos, startPoint->Point->pos, endPoint->Point->pos);
		if (d > max_h)
		{
			max_h_i = i;
			max_h = d;
		}
	}

	if (max_h < hor)
	{
		compressWithZ(startPoint, endPoint, points, lengthes, ver);
	}
	else
	{
		PointExPtr middle = points[max_h_i];
		compress(startPoint, middle, points, lengthes, hor, ver);
		compress(middle, endPoint, points, lengthes, hor, ver);
	}
}


void PolylineSimplifier::simplifyT(std::vector<MapPoint3D64>& points, double hor, double ver)
{
	auto execute = [&](std::vector<MapPoint3D64>& points) ->void {

		if (points.empty())
			return;

		size_t size = points.size();
		if (size < 3)
			return;

		std::vector<PointExPtr> pointexs;
		pointexs.resize(size);

		std::vector<double> lengthes;
		lengthes.resize(size);
		for (int i = 0; i < size; i++)
		{
			if (i < size - 1)
			{
				int dx = points[i + 1].pos.lon - points[i].pos.lon;
				int dy = points[i + 1].pos.lat - points[i].pos.lat;
				lengthes[i] = std::sqrt(dx * dx + dy * dy);
				// 			x_lengthes[i] = length;
				// 			length += std::sqrt(dx * dx + dy * dy);
			}

			PointExPtr p(new PointEx());
			p->Point = &(points[i]);
			p->VertexIndex = i;
			pointexs[i] = p;
		}
		lengthes[size - 1] = 0.0;
		// 	x_lengthes[size - 1] = length;
		compress(pointexs[0], pointexs[size - 1], pointexs, lengthes, hor, ver); //偏差1米
		std::vector<MapPoint3D64> res;

		res.push_back(*(pointexs[0]->Point));
		for_each(pointexs.begin() + 1, pointexs.end() - 1, [&](const PointExPtr& p) {
			if (p->VertexIndex == -1)
			{
				res.push_back(*(p->Point));
			}
			}
		);

		res.push_back(*(pointexs[size - 1]->Point));

		points.clear();
		points.assign(res.begin(), res.end());
	};

	if (points.empty())
		return;

	size_t size = points.size();
	if (size < 3)
		return;

	if (points[0].pos.lon == points[size - 1].pos.lon && points[0].pos.lat == points[size - 1].pos.lat)
	{
		std::vector<std::vector<MapPoint3D64>> vmps;

		size_t middle = size / 2;
		std::vector<MapPoint3D64> one(points.begin(), points.begin() + middle + 1);
		std::vector<MapPoint3D64> two(points.begin() + middle + 1, points.end());

		execute(one);
		execute(two);

		points.clear();

		points.reserve(one.size() + two.size());

		points.insert(points.end(), one.begin(), one.end());
		points.insert(points.end(), two.begin(), two.end());
	}
	else
	{
		execute(points);
	}
}
