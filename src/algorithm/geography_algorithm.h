#pragma once


// ��ֹ�����ض���� cq_stdlib.h �е� memcpy ��Ӱ�쵽����Ĵ��루��Ҫ�� CGAL / boost ���е� "std::memcpy"��
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

//���㷨����ʹ��boost�����������߲�����WKT��������
class GeographyAlgorithm
{
public:
	//��ȡLINESTRING Z��ʽ
	static bool readLineStringZ(const char* wkt, bg_line_string_3d& point3d);
	//��ȡ�����ϵĳ���,��λ����
	static double getGeoLength(const bg_line_string_3d& points);
	static bool grapPoint(const bgPoint3d&pt,const bg_line_string_3d& points, bgPoint3d& grappedPt, size_t& si, size_t& ei);
	static bool grapPoint(const bgPoint3d& pt, const bgPoint3d& st, const bgPoint3d& end, bgPoint3d& grappedPt);
};


