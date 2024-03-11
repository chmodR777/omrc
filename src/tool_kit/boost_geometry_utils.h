#pragma once
#include "algorithm/geography_algorithm.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/function_output_iterator.hpp>
#include <boost/range/irange.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/regex.hpp>

namespace bg = boost::geometry;
namespace bnu = boost::numeric::ublas;
namespace bgi = boost::geometry::index;

template <typename Container>
class VectorIndexable {
    using size_type = typename Container::size_type;
    using cref = const typename Container::value_type&;
    Container const& container;

public:
    using result_type = cref;

    explicit VectorIndexable(Container const& c)
        : container(c)
    {
    }

    result_type operator()(size_type i) const
    {
        return container[i];
    }
};

typedef bg::model::point<int64, 3, bg::cs::cartesian> point_t;
typedef bg::model::point<double, 3, bg::cs::cartesian> vector_t;
typedef bg::model::linestring<point_t> linestring_t;
typedef bg::model::multi_linestring<linestring_t> multi_linestring_t;
typedef bg::model::box<point_t> box_t;
typedef bg::model::segment<point_t> segment_t;
typedef bg::model::ring<point_t> ring_t;
typedef bg::model::polygon<point_t> polygon_t;
typedef bgi::quadratic<16> parameters;
typedef VectorIndexable<std::vector<point_t>> index_getter;
typedef VectorIndexable<std::vector<segment_t>> index_getter_segment;
typedef VectorIndexable<std::vector<box_t>> index_getter_box;
typedef bgi::rtree<std::vector<point_t>::size_type, bgi::quadratic<16>, index_getter> rtree_type;
typedef bgi::rtree<std::vector<segment_t>::size_type, bgi::quadratic<16>, index_getter_segment> rtree_type_segment;
typedef bgi::rtree<std::vector<box_t>::size_type, bgi::quadratic<16>, index_getter_box> rtree_type_box;
typedef bg::strategy::side::side_by_triangle<> side_tri_type;

#define POINT_T(pt) point_t(pt.pos.lon, pt.pos.lat, static_cast<int64>(pt.z) * static_cast<int64>(10))

forceinline bool operator == (const point_t& a, const point_t& b) 
{
	return a.get<0>() == b.get<0>() && a.get<1>() == b.get<1>() && a.get<2>() == b.get<2>();
}

forceinline bool operator != (const point_t& a, const point_t& b)
{
    return !(a == b);
}

forceinline bool operator < (const point_t& a, const point_t& b)
{
	if (a.get<0>() != b.get<0>())
		return a.get<0>() < b.get<0>();
	if (a.get<1>() != b.get<1>())
		return a.get<1>() < b.get<1>();
	return a.get<2>() < b.get<2>();
}

forceinline bool point_t_compare(const point_t& a, const point_t& b)
{
    return a == b;
}

inline linestring_t LINESTRING_T(const std::vector<MapPoint3D64>& pts)
{
    linestring_t line;
    for (auto& p : pts)
        line.push_back(POINT_T(p));
    return line;
}

inline ring_t RING_T(const std::vector<MapPoint3D64>& pts)
{
    ring_t ring;
    for (auto& p : pts)
        ring.push_back(POINT_T(p));
    return ring;
}

inline polygon_t POLYGON_T(const std::vector<MapPoint3D64>& pts)
{
    polygon_t poly;
    for (auto& p : pts)
        poly.outer().push_back(POINT_T(p));
    return poly;
}

inline box_t BOX_T(const std::vector<MapPoint3D64>& pts)
{
    box_t box;
    bg::envelope(LINESTRING_T(pts), box);
    return box;
}

inline box_t BOX_T(const linestring_t& line)
{
	box_t box;
	bg::envelope(line, box);
	return box;
}

inline box_t BOX_T(const point_t& point, int32 posOffset = 10, int32 zOffset = 10)
{
	box_t box;
    point_t minPoint{ point.get<0>() - posOffset, point.get<1>() - posOffset, point.get<2>() - zOffset };
    point_t maxPoint{ point.get<0>() + posOffset, point.get<1>() + posOffset, point.get<2>() + zOffset };
    linestring_t line;
    line.push_back(minPoint);
	line.push_back(maxPoint);
	bg::envelope(line, box);
	return box;
}

typedef bg::model::point<int64, 2, bg::cs::cartesian> point_2t;
typedef bg::model::point<double, 2, bg::cs::cartesian> vector_2t;
typedef bg::model::linestring<point_2t> linestring_2t;
typedef bg::model::multi_linestring<linestring_2t> multi_linestring_2t;
typedef bg::model::box<point_2t> box_2t;
typedef bg::model::segment<point_2t> segment_2t;
typedef bg::model::ring<point_2t> ring_2t;
typedef bg::model::polygon<point_2t> polygon_2t;
typedef bg::model::multi_polygon<polygon_2t> multi_polygon_2t;
typedef VectorIndexable<std::vector<point_2t>> index_2getter;
typedef VectorIndexable<std::vector<segment_2t>> index_getter_2segment;
typedef VectorIndexable<std::vector<box_2t>> index_getter_2box;
typedef bgi::rtree<std::vector<point_2t>::size_type, bgi::quadratic<16>, index_2getter> rtree_2type;
typedef bgi::rtree<std::vector<segment_2t>::size_type, bgi::quadratic<16>, index_getter_2segment> rtree_type_2segment;
typedef bgi::rtree<std::vector<box_2t>::size_type, bgi::quadratic<16>, index_getter_2box> rtree_type_2box;

struct Triangle
{
	std::array<point_t, 3> vertexes;
	linestring_2t triangleLine;
	ring_2t trianglePoly;
};


forceinline bool operator == (const point_2t& a, const point_2t& b) 
{
	return a.get<0>() == b.get<0>() && a.get<1>() == b.get<1>();
}

forceinline bool operator != (const point_2t& a, const point_2t& b)
{
	return !(a == b);
}

forceinline bool operator < (const point_2t& a, const point_2t& b)
{
	if (a.get<0>() != b.get<0>())
		return a.get<0>() < b.get<0>();
	return a.get<1>() < b.get<1>();
}

forceinline bool point_2t_compare(const point_2t& a, const point_2t& b)
{
    return a == b;
}

inline point_2t POINT_2T(const MapPoint64& pt)
{
	return point_2t(pt.lon, pt.lat);
}

inline point_2t POINT_2T(const MapPoint3D64& pt)
{
	return POINT_2T(pt.pos);
}

inline linestring_2t LINESTRING_2T(const ring_2t& pts)
{
	linestring_2t line;
	for (auto& p : pts)
		line.push_back(p);
	return line;
}

inline linestring_2t LINESTRING_2T(const std::vector<MapPoint3D64>& pts)
{
    linestring_2t line;
    for (auto& p : pts)
        line.push_back(POINT_2T(p));
    return line;
}

inline ring_2t RING_2T(const std::vector<MapPoint3D64>& pts)
{
    ring_2t ring;
    for (auto& p : pts)
        ring.push_back(POINT_2T(p));
    return ring;
}

inline ring_2t RING_2T(const MapPoint3D64& pt, int32 posOffset = 10)
{
	ring_2t ring;
    point_2t point = POINT_2T(pt);
	point_2t leftDownPt{ point.get<0>() - posOffset, point.get<1>() - posOffset };
	point_2t rightDownPt{ point.get<0>() + posOffset, point.get<1>() - posOffset };
	point_2t rightUpPt{ point.get<0>() + posOffset, point.get<1>() + posOffset };
	point_2t leftUpPt{ point.get<0>() - posOffset, point.get<1>() + posOffset };
    ring.push_back(leftDownPt);
    ring.push_back(rightDownPt);
    ring.push_back(rightUpPt);
    ring.push_back(leftUpPt);
	return ring;
}

inline polygon_2t POLYGON_2T(const std::vector<MapPoint3D64>& pts)
{
    polygon_2t poly;
    for (auto& p : pts)
        poly.outer().push_back(POINT_2T(p));
    return poly;
}

inline box_2t BOX_2T(const std::vector<MapPoint3D64>& pts)
{
    box_2t box;
    bg::envelope(LINESTRING_2T(pts), box);
    return box;
}

inline box_2t BOX_2T(const point_2t& point, int32 posOffset = 10)
{
	box_2t box;
	point_2t minPoint{ point.get<0>() - posOffset, point.get<1>() - posOffset };
	point_2t maxPoint{ point.get<0>() + posOffset, point.get<1>() + posOffset };
	linestring_2t line;
	line.push_back(minPoint);
	line.push_back(maxPoint);
	bg::envelope(line, box);
	return box;
}

//others
inline point_2t P3_P2(const point_t& p3)
{
    return point_2t(p3.get<0>(), p3.get<1>());
}

inline point_t P2_P3(const point_2t& p2)
{
	return point_t(p2.get<0>(), p2.get<1>(), 0);
}

inline linestring_2t L3_L2(const linestring_t& l3)
{
	linestring_2t line;
	for (auto& p : l3) {
		line.push_back(P3_P2(p));
	}
	return line;
}

inline linestring_t L2_L3(const linestring_2t& l2)
{
	linestring_t line;
	for (auto& p : l2) {
		line.push_back(P2_P3(p));
	}
	return line;
}

inline segment_2t S3_S2(const segment_t& s3)
{
    return segment_2t(P3_P2(s3.first), P3_P2(s3.second));
}

inline vector_2t V3_V2(const vector_t& v3)
{
    return vector_2t(v3.get<0>(), v3.get<1>());
}

inline vector_2t P2_V2(const point_2t& p)
{
    return vector_2t((double)p.get<0>(), (double)p.get<1>());
}

inline point_t V3_P3(const vector_t& v)
{
	return point_t((int64)v.get<0>(), (int64)v.get<1>(), (int64)v.get<2>());
}

inline point_2t V2_P2(const vector_2t& v)
{
    return point_2t((int64)v.get<0>(), (int64)v.get<1>());
}

inline vector_2t S2_V2(const point_2t& p1, const point_2t& p2)
{
    return vector_2t(p2.get<0>() - p1.get<0>(), p2.get<1>() - p1.get<1>());
}

inline vector_2t S2_V2(const segment_2t& seg)
{
    return vector_2t(seg.second.get<0>() - seg.first.get<0>(), seg.second.get<1>() - seg.first.get<1>());
}

inline vector_t S3_V3(
    const point_t& p1, 
    const point_t& p2)
{
    return vector_t(
        p2.get<0>() - p1.get<0>(), 
        p2.get<1>() - p1.get<1>(), 
        p2.get<2>() - p1.get<2>());
}

inline vector_t S3_V3(const segment_t& seg)
{
    return vector_t(
        seg.second.get<0>() - seg.first.get<0>(), 
        seg.second.get<1>() - seg.first.get<1>(),
        seg.second.get<2>() - seg.first.get<2>());
}

inline vector_2t V2_N(const vector_2t& v2)
{
    vector_2t tmp = v2;
    double dis = std::abs(bg::distance(v2, point_2t(0.0, 0.0)));
    if (!bg::math::equals(dis, 0.0))
        bg::divide_value(tmp, dis);
    return tmp;
}

inline vector_t V3_N(const vector_t& v3)
{
    vector_t tmp = v3;
    double dis = std::abs(bg::distance(v3, point_t(0.0, 0.0, 0.0)));
    if (!bg::math::equals(dis, 0.0))
        bg::divide_value(tmp, dis);
    return tmp;
}

inline ring_2t R3_R2(const ring_t& r3)
{
	ring_2t ring;
	for (auto& p : r3)
        ring.push_back(P3_P2(p));
	return ring;
}

inline box_2t B3_B2(const box_t& b3)
{
    point_2t minP(b3.min_corner().get<0>(), b3.min_corner().get<1>());
    point_2t maxP(b3.max_corner().get<0>(), b3.max_corner().get<1>());
    box_2t tmp(minP, maxP);
    return tmp;
}

inline box_2t BOX_2T(const segment_t& s3)
{
	box_2t box;
	linestring_2t line;
	line.push_back(P3_P2(s3.first));
	line.push_back(P3_P2(s3.second));
	bg::envelope(line, box);
	return box;
}

inline box_2t BOX_2T(const point_t& pt, const int64 r)
{
	box_2t box = box_2t(
		point_2t(pt.get<0>() - r, pt.get<1>() - r),
		point_2t(pt.get<0>() + r, pt.get<1>() + r));
	return box;
}

inline segment_2t SEGMENT_2T_EX(const segment_2t& seg, const double length)
{
    vector_2t v = V2_N(S2_V2(seg.first, seg.second));
    point_2t pa = point_2t(seg.first.get<0>() - v.get<0>() * length, seg.first.get<1>() - v.get<1>() * length);
    point_2t pb = point_2t(seg.second.get<0>() + v.get<0>() * length, seg.second.get<1>() + v.get<1>() * length);
    return segment_2t(pa, pb);
}

inline point_t POINT_EX(const point_t& pt, const vector_t& v, const double length)
{
	vector_t vn = V3_N(v);
	point_t p = point_t(
		pt.get<0>() + vn.get<0>() * length,
		pt.get<1>() + vn.get<1>() * length,
		pt.get<2>() + vn.get<2>() * length);
	return p;
}

inline segment_t SEGMENT_EX(const segment_t& seg, const double length)
{
    vector_t v = V3_N(S3_V3(seg));
    point_t pa = point_t(
        seg.first.get<0>() - v.get<0>() * length, 
        seg.first.get<1>() - v.get<1>() * length,
        seg.first.get<2>() - v.get<2>() * length);
    point_t pb = point_t(
        seg.second.get<0>() + v.get<0>() * length, 
        seg.second.get<1>() + v.get<1>() * length, 
        seg.second.get<2>() + v.get<2>() * length);
    return segment_t(pa, pb);
}

inline segment_2t SEGMENT_2T_EX_FRONT(const segment_2t& seg, const double length)
{
    vector_2t v = V2_N(S2_V2(seg.first, seg.second));
    point_2t pb = point_2t(seg.second.get<0>() + v.get<0>() * length, seg.second.get<1>() + v.get<1>() * length);
    return segment_2t(seg.first, pb);
}

inline segment_2t SEGMENT_2T_EX_BACK(const segment_2t& seg, const double length)
{
    vector_2t v = V2_N(S2_V2(seg.first, seg.second));
    point_2t pb = point_2t(seg.second.get<0>() - v.get<0>() * length, seg.second.get<1>() - v.get<1>() * length);
    return segment_2t(seg.first, pb);
}

inline segment_2t SEGMENT_2T_IN(const segment_2t& seg, const double factor)
{
    double length = factor * bg::length(seg);
    vector_2t v = V2_N(S2_V2(seg.first, seg.second));
    point_2t pa = point_2t(seg.first.get<0>() + v.get<0>() * length, seg.first.get<1>() + v.get<1>() * length);
    point_2t pb = point_2t(seg.second.get<0>() - v.get<0>() * length, seg.second.get<1>() - v.get<1>() * length);
    return segment_2t(pa, pb);
}

template <typename POINT>
inline bool GRAP_POINT(const POINT& pt, const POINT& p1, const POINT& p2, POINT& grappedPt, double tolerance = 10.0)
{
    POINT w, v, projected;
	bg::convert(p2, v);
	bg::convert(pt, w);
	bg::convert(p1, projected);

	bg::subtract_point(v, projected);
	bg::subtract_point(w, projected);

	double const zero = 0;
	double c1 = bg::dot_product(w, v);
	if (c1 <= zero) {
        double d1 = bg::distance(p1, pt);
        if (abs(d1) < tolerance) {
            grappedPt = p1;
            return true;
        }
		return false;
	}

	double c2 = bg::dot_product(v, v);
	if (c2 < c1) {
		double d2 = bg::distance(p2, pt);
		if (abs(d2) < tolerance) {
			grappedPt = p2;
			return true;
		}
		return false;
	}

	double b = c1 / c2;
    // bg::multiply_value(v, b);
	bg::detail::for_each_dimension<POINT>([&](auto index) {
        bg::set<index>(v, bg::get<index>(v) * b);
	});
	bg::add_point(projected, v);

    grappedPt = projected;
	return true;
}

template <typename POINT, typename SEGMENT>
inline bool GRAP_POINT(const POINT& pt, const SEGMENT& seg, POINT& grappedPt, double tolerance = 10.0)
{
	return GRAP_POINT(pt, seg.first, seg.second, grappedPt, tolerance);
}

template<typename POINT,
	template<typename, typename> class Container = std::vector,
	template<typename> class Allocator = std::allocator>
inline bool GRAP_POINT(const POINT& pt, const Container<POINT, Allocator<POINT>>& points, POINT& grappedPt, size_t& si, size_t& ei, double tolerance = 10.0)
{
	double distance = 1e20;
	auto length = [&](const POINT& point)-> double {
		return bg::distance(pt, point);
	};
	bool status = false;

    size_t pointCount = points.size();
	for (size_t i = 0; i < pointCount; i++)
	{
		if (i == pointCount - 1)
		{
            POINT p;
			if (GRAP_POINT(pt, points[i], points[i - 1], p, tolerance))
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
            POINT p;
			if (GRAP_POINT(pt, points[i], points[i + 1], p, tolerance))
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

inline bool IS_LEFT_POINT_OF_SEGMENT_2T(segment_2t seg, point_2t p)
{
    vector_2t v1 = S2_V2(seg);
    vector_2t v2 = S2_V2(seg.first, p);
    return v1.get<0>() * v2.get<1>() - v1.get<1>() * v2.get<0>() > 0.0;
}

forceinline bool isEqualPoint2T(const point_2t& pa, const point_2t& pb)
{
	double tmpWidth = std::abs(bg::distance(pa, pb));
	return bg::equals(pa, pb) || tmpWidth < 10;
}

forceinline bool isEqualPoint(const point_t& pa, const point_t& pb)
{
	double tmpWidth = std::abs(bg::distance(pa, pb));
	return bg::equals(pa, pb) || tmpWidth < 10;
}

// 计算两个向量的最小夹角
forceinline uint16 minimalDegree(point_2t dir1, point_2t dir2)
{
	double angle1 = std::atan2(dir1.get<1>(), dir1.get<0>());
	angle1 = int(angle1 * 180 / MATH_PI);
	double angle2 = std::atan2(dir2.get<1>(), dir2.get<0>());
	angle2 = int(angle2 * 180 / MATH_PI);
	double angle = DBL_MAX;
	if (angle1 * angle2 >= 0) {
		angle = abs(angle1 - angle2);
	}
	else {
		angle = abs(angle1) + abs(angle2);
		if (angle > 180) {
			angle = 360 - angle;
		}
	}
	angle = (int32)angle % 180;
	if (angle > 90) {
		angle = 180 - angle;
	}
	return angle;
}

forceinline point_t calculateBezierPoint(
	const point_t& p0,
	const point_t& p1,
	const point_t& p2,
	double t)
{
	double x = (1 - t) * (1 - t) * p0.get<0>() + 2 * (1 - t) * t * p1.get<0>() + t * t * p2.get<0>();
	double y = (1 - t) * (1 - t) * p0.get<1>() + 2 * (1 - t) * t * p1.get<1>() + t * t * p2.get<1>();
	double z = (1 - t) * (1 - t) * p0.get<2>() + 2 * (1 - t) * t * p1.get<2>() + t * t * p2.get<2>();
	return point_t(x, y, z);
}

 forceinline bool isKneePoint(
	const point_2t& currentPoint,
	const point_2t& prevPoint,
	const point_2t& nextPoint,
	const double& factor)
{
	vector_2t d1 = S2_V2(prevPoint, currentPoint);
	vector_2t d2 = S2_V2(currentPoint, nextPoint);
	if (bg::dot_product(V2_N(d1), V2_N(d2)) < factor)
		return true;
	return false;
}