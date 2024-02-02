#pragma once
#include <vector>
#include <array>
#include <algorithm>
#include "math3d/vector_math.h"
#include "map_point3d64_converter.h"
#include "tool_kit/boost_geometry_utils.h"
#include "algorithm/polygon_triangularizer.h"

class LinearInterpolationTriangleSurface
{
public:
	static void interpolationLine(
		MapPoint3D64Converter& mapPointConverter,
		const std::vector<Triangle>& surfaceTriangles,
		const std::vector<MapPoint3D64>& line,
		std::vector<MapPoint3D64>& lineOnSurface);

	static void interpolationLine(
		MapPoint3D64Converter& mapPointConverter,
		const std::vector<Triangle>& surfaceTriangles,
		const rtree_type_2box& surfaceRTree2T,
		const std::vector<MapPoint3D64>& line,
		std::vector<MapPoint3D64>& lineOnSurface);

    static void interpolationPolygon(
        MapPoint3D64Converter& mapPointConverter,
        const std::vector<Triangle>& surfaceTriangles,
		const std::vector<Triangle>& polyTriangles,
        const std::vector<MapPoint3D64>& polygon,
        std::vector<MapPoint3D64>& polygonOnSurface);

	static void getTriangleBoxes(
		const std::vector<Triangle>& polyTriangles, 
		std::vector<box_2t>& triangleBoxes);

	static bool interpolationTriangle(
		std::vector<point_t> trianglePts,
		DVector4& coeffs);

	static double interpolatePt(
		DVector4& coeffs, const point_t& pt);

	static bool calculateTrianglePt(
		point_t& point,
		const rtree_type_2box& rtree,
		const std::vector<Triangle>& polyTriangles);

	static bool calculateTrianglePts(
		const segment_t& segment,
		const rtree_type_2box& rtree,
		const std::vector<Triangle>& polyTriangles,
		std::vector<point_t>& trianglePts);

	static bool getTrianglesByPt(
		const point_t& point,
		const rtree_type_2box& rtree,
		const std::vector<Triangle>& polyTriangles,
		std::vector<Triangle>& triangles);

	static bool getTrianglesBySegment(
		const segment_t& segment,
		const rtree_type_2box& rtree,
		const std::vector<Triangle>& polyTriangles,
		std::vector<Triangle>& triangles);

	static double calculateZOnTriangle(
		const point_2t& pt,
		const Triangle& triangle);

    // 路口面三角化
    static void triangularize(
        MapPoint3D64Converter& mapPointConverter,
        const std::vector<MapPoint3D64>& poly,
		const PolygonDirection polygonDir,
        std::vector<Triangle>& polyTriangles);

	// 导流带面三角化
	static void triangularizeWMT(
		MapPoint3D64Converter& mapPointConverter,
		const std::vector<MapPoint3D64>& poly,
		std::vector<Triangle>& polyTriangles);

    // 路面三角化
	static void triangularizeStroke(
		MapPoint3D64Converter& mapPointConverter,
		const std::vector<MapPoint3D64>& leftBoundary,
		const std::vector<MapPoint3D64>& rightBoundary,
		std::vector<Triangle>& polyTriangles);

	static void appendTriangulationResult(
		PolygonTriangularizer& trizer,
		const std::vector<Vector3>& contour,
		std::vector<Triangle>& polyTriangles);

	// 获取原始坐标点的索引
	static std::vector<size_t> getPointIndexes(
		const point_t& point, rtree_type& rtree, 
		std::vector<point_t>& originPoints);

	static point_t getClosestPoint(
		const Triangle& triangle,
		const point_t& originPoint);

    static int findNearestPoint(
		const Vector3* points, int n, Vector2 p);

	forceinline static double minimalDegree(
		point_t& dir1, point_t& dir2);

	forceinline static bool coveredByTriangle(
		const point_2t& pt,
		const Triangle& triangle) 
	{
		double tmpDistance = bg::distance(pt, triangle.trianglePoly);
		return bg::covered_by(pt, triangle.trianglePoly) || tmpDistance < 10;
	}

	forceinline static bool isStraightTriangle(const Triangle& triangle)
	{
		auto ver01 = S3_V3(triangle.vertexes[0], triangle.vertexes[1]);
		auto ver02 = S3_V3(triangle.vertexes[0], triangle.vertexes[2]);
		auto ver12 = S3_V3(triangle.vertexes[1], triangle.vertexes[2]);
		double ver0Degree = minimalDegree(V3_P3(ver01), V3_P3(ver02));
		double ver1Degree = minimalDegree(V3_P3(ver01), V3_P3(ver12));
		double ver2Degree = minimalDegree(V3_P3(ver02), V3_P3(ver12));
		if (ver0Degree < 1 || ver1Degree < 1 || ver2Degree < 1) {
			// 角度太小时认为是直线的三角化面
			return true;
		}
		return false;
	}

	forceinline static bool isEqualPoint2T(
		const point_2t& pa,
		const point_2t& pb)
	{
		double tmpWidth = std::abs(bg::distance(pa, pb));
		return bg::equals(pa, pb) || tmpWidth < 10;
	}

	forceinline static bool isEqualPoint(
		const point_t& pa,
		const point_t& pb)
	{
		double tmpWidth = std::abs(bg::distance(pa, pb));
		return bg::equals(pa, pb) || tmpWidth < 10;
	}
};