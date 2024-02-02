#pragma once


// 防止引擎重定义的 cq_stdlib.h 中的 memcpy 宏影响到下面的代码（主要是 CGAL / boost 库中的 "std::memcpy"）
#pragma push_macro("memcpy")
#undef memcpy
#pragma push_macro("malloc")
#undef malloc
#pragma push_macro("free")
#undef free
#pragma push_macro("realloc")
#undef realloc

#include "boost/geometry.hpp"
#include "Eigen/Dense"

namespace bg = boost::geometry;
using namespace bg;
typedef bg::model::d3::point_xyz<double> bgPoint3d;
typedef bg::model::linestring <bgPoint3d> bg_line_string_3d;

//此算法类是使用boost处理特殊虚线补丁的WKT几何数据
class GeographyAlgorithm
{
public:
	//读取LINESTRING Z格式
	static bool readLineStringZ(const char* wkt, bg_line_string_3d& point3d);
	//获取地理上的长度,单位：米
	static double getGeoLength(const bg_line_string_3d& points);
	static bool grapPoint(const bgPoint3d&pt,const bg_line_string_3d& points, bgPoint3d& grappedPt, size_t& si, size_t& ei);
	static bool grapPoint(const bgPoint3d& pt, const bgPoint3d& st, const bgPoint3d& end, bgPoint3d& grappedPt);
};


