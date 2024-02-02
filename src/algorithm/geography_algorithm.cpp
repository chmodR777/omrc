#include "stdafx.h"
#include "geography_algorithm.h"
#include "math.h"
#include "math3d\dvector2.h"
// 防止引擎重定义的 cq_stdlib.h 中的 memcpy 宏影响到下面的代码（主要是 CGAL / boost 库中的 "std::memcpy"）
#pragma push_macro("memcpy")
#undef memcpy
#include "boost/geometry.hpp"
#include "boost/geometry/geometry.hpp"
#include <tool_kit\call_stack_walker.h>

#define EARTH_RADIUS 6367447.5;
#define M_PI       3.14159265358979323846   // pi

//地理经纬度下的距离
double geoLengthD(const bgPoint3d& pt1, const bgPoint3d& pt2)
{
	auto rad = [&](double d)->double {
		return d * M_PI / 180.0;
	};
	const double& lat1 = pt1.y();
	const double& lng1 = pt1.x();
	const double& lat2 = pt2.y();
	const double& lng2 = pt2.x();
		
	double radLat1 = rad(lat1);
	double radLat2 = rad(lat2);
	double a = radLat1 - radLat2;
	double b = rad(lng1) - rad(lng2);
	double s = 2 * asin(sqrt(pow(sin(a / 2), 2) + cos(radLat1) * cos(radLat2) * pow(sin(b / 2), 2)));
	s *= EARTH_RADIUS;
	s = round(s * 10000) / 10000.0;
	return s;
}

double GeographyAlgorithm::getGeoLength(const bg_line_string_3d& points) {
	double geoLength = 0.0;
	if (points.size() < 2)
	{
		return 0.0;
	}
	for (size_t i = 0; i < points.size() - 1; i++)
	{
		auto& pt1 = points.at(i);
		auto& pt2 = points.at(i + 1);
		geoLength += geoLengthD(pt1, pt2);
	}
	return geoLength;
}

bool GeographyAlgorithm::readLineStringZ(const char* wkt, bg_line_string_3d& point3d)
{
	//LineStringZ example "LINESTRING Z (117.5608894 38.7120701 -2.146,117.5609058 38.7120064 -2.18)"
	try
	{
		if (wkt != nullptr)
		{
			bg::read_wkt(wkt, point3d);
			return true;
		}

		return false;

	}
	catch (CallStackWalker& walker)
	{
		//walker.showExceptionInfo();
		return false;
	}
	catch (const std::exception& e)
	{
		//printError("Catch C++ exception: %s", e.what());
		return false;
	}

	return false;
}

bool GeographyAlgorithm::grapPoint(const bgPoint3d& pt, const bg_line_string_3d& points, bgPoint3d& grappedPt, size_t& si, size_t& ei)
{
	int32 dz = pt.z();
	double distance = 1e20;
	bool status = false;
	size_t pointCount = points.size();

	auto length = [&](const bgPoint3d& point)-> double {

		double hor = geoLengthD(pt, point) * 100.0;
		int ver = point.z() - pt.z();

		return hor * hor + (double)ver * ver;
	};


	for (size_t i = 0; i < points.size(); i++)
	{
		const bgPoint3d& point = points.at(i);
		if (i == pointCount - 1)
		{
			bgPoint3d p;
			if (grapPoint(pt, points[i], points[i - 1], p))
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
			bgPoint3d p;
			if (grapPoint(pt, points[i], points[i + 1], p))
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

	return false;
}

bool GeographyAlgorithm::grapPoint(const bgPoint3d& pt, const bgPoint3d& st, const bgPoint3d& end, bgPoint3d& grappedPt)
{
	DVector2 v1;
	v1.x = end.x() - st.x();
	v1.y = end.y() - st.y();

	DVector2 v2;
	v2.x = pt.x() - st.x();
	v2.y = pt.y() - st.y();

	double length = v1.length();

	v1.normalize();
	double l = dot(v1, v2);

	double al = std::abs(l);

	if (al < 10)
	{
		grappedPt = st;
		return true;
	}

	if (l < 0)
	{
		grappedPt = st;
		return false;
	}


	if (std::abs(l - length) < 20)
	{
		grappedPt = end;
		return true;
	}

	if (l > length)
	{
		grappedPt = end;
		return false;
	}

	DVector2 p = v1 * l;
	p.x += st.x();
	p.y += st.y();

	grappedPt.x(p.x);
	grappedPt.y(p.y);

	grappedPt.z(st.z() + (end.z() - st.z()) * l / length);

	return true;
}
