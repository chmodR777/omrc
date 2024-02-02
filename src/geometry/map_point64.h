#pragma once

#include "cq_types_basic.h"
#include "cq_math_basic.h"

/*
	高精度经纬度坐标点，单位：1/10^8 度
*/
struct MapPoint64
{
	int64 lon;
	int64 lat;

	static MapPoint64 make(int64 x, int64 y)
	{
		MapPoint64 pt; pt.lon = x; pt.lat = y;
		return pt;
	}

	void fromPointD(const PointD& pt)
	{
		lon = (int64)(pt.x * 1e8 + 0.5);
		lat = (int64)(pt.y * 1e8 + 0.5);
	}

	PointD toPointD()
	{
		return PointD{ this->lon * (1.0 / 1e8), this->lat * (1.0 / 1e8) };
	}

	double distance(const MapPoint64& point) const
	{
		return std::sqrt(distanceSquare(point));
	}

	int64 distanceSquare(const MapPoint64& point) const
	{
		int64 dx = point.lon - this->lon;
		int64 dy = point.lat - this->lat;

		return dx * dx + dy * dy;
	}

	double geodistance(const MapPoint64& other) const
	{
		return geodistance(*this, other);
	}
	/**
	 * @brief 计算以球体为参考的ECEF坐标系下球体表面两经纬度点的直线距离（非大圆距离），单位米。
	 *        球体半径取WGS84标准参考椭球中的地球长半径(单位:m)
	 * @param  pa 点a
	 * @param  pb 点b
	 * @return 
	*/
	static double geodistance(const MapPoint64& pa, const MapPoint64& pb)
	{
		PointD pa1 = pa.toPointD();
		PointD pb1 = pb.toPointD();
		return Math_segGeoLengthD(&pa1, &pb1);
	}

	PointD toPointD() const
	{
		return PointD_make(lon / 1e8, lat / 1e8);
	}

	void fromNdsPoint(const NdsPoint& pt)
	{
		double x, y;
		NdsPoint_toDouble(pt, &x, &y);
		lon = (int64)(x * 1e8 + 0.5);
		lat = (int64)(y * 1e8 + 0.5);
	}

	NdsPoint toNdsPoint() const
	{
		Coordinate c;
		c.lat = lat / 1e8;
		c.lon = lon / 1e8;
		return NdsPoint_makeWithLonLat(c.lon, c.lat);
	}

	void fromPoint(const Point& pt)
	{
		lon = pt.x * 1000;
		lat = pt.y * 1000;
	}

	Point toPoint() const
	{
		Point pt;
		pt.x = (int32)((lon + 500) / 1000);
		pt.y = (int32)((lat + 500) / 1000);
		return pt;
	}
};

forceinline MapPoint64 MapPoint64_make(int64 x, int64 y)
{
	MapPoint64 pt; pt.lon = x; pt.lat = y;
	return pt;
}

forceinline bool operator == (const MapPoint64& l, const MapPoint64& r)
{
	return l.lon == r.lon && l.lat == r.lat;
}

forceinline bool operator != (const MapPoint64& l, const MapPoint64& r)
{
	return !(l == r);
}

forceinline MapPoint64 operator - (const MapPoint64& l, const MapPoint64& r)
{
	return MapPoint64::make(l.lon - r.lon, l.lat - r.lat);
}
