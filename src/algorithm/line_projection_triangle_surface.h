#pragma once
#include <vector>
#include <array>
#include <algorithm>
#include "geometry/map_point3d64.h"
#include "tool_kit/boost_geometry_utils.h"

class LineProjectionTriangleSurface
{
public:
    static void projection(
        const std::vector<MapPoint3D64>& leftRdBoundary,
        const std::vector<MapPoint3D64>& rightRdBoundary,
        const std::vector<MapPoint3D64>& line,
        std::vector<MapPoint3D64>& lineOnRoadSurface);

    static bool isGetValue(
	    const segment_t& originSeg,
        const segment_2t& seg,
        const point_2t& pc,
        MapPoint3D64& point);

    //路面三角化算法
    //contour：道路两边边界点，rightN：右边界点数量，leftN：左边界点数量，out_edges：输出的边
    static bool triangularizeStroke(
        MapPoint3D64* contour,
        int rightN,
        int leftN,
        std::vector<std::array<int, 2>>& out_edges);

};