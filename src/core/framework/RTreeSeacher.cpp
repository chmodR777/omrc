#include "stdafx.h"
#include "RTreeSeacher.h"
#include <algorithm>
#include "cq_math.h"
#include "geometry/map_point3d64.h"
#include "grap_point_algorithm.h"
#define HAD_GRID_NDS_LEVEL	13

namespace OMDB
{
    std::vector<DbLink*> RTreeSeacher::seachNearby2d(DbMesh* pMesh, DbLink* pLink, MapPoint3D64 point, int32 tolerance /*= 10*/)
    {
		point_2t point2T = POINT_2T(point);
		box_2t box = BOX_2T(point2T, tolerance);

		std::vector<DbLink*> v;
		auto& linkInfos = pMesh->getLinkInfos();
		pMesh->getLinkRtree()->query(bgi::intersects(box),
			boost::make_function_output_iterator([&](size_t const& id) {
				if (pLink != linkInfos[id]._link)
				{
					segment_2t tmpSeg;
					auto tmpPoints = linkInfos[id]._linkPoints2T;
					bg::closest_points(point2T, tmpPoints, tmpSeg);
					double const dist = bg::length(tmpSeg);
					if (dist < tolerance)
					{
						v.push_back(linkInfos[id]._link);
					}
				}
				}));
		return v;
    }

	std::vector<DbLink*> RTreeSeacher::seachNearby2d(const std::vector<DbMesh*>& meshes, DbLink* pLink, MapPoint3D64 point, int32 tolerance /*= 10*/)
	{
		std::vector<DbLink*> v;
		if (meshes.empty())
			return v;
		for (auto pMesh : meshes)
		{
			auto vs = seachNearby2d(pMesh, pLink, point, tolerance);
			v.insert(v.end(), vs.begin(), vs.end());
		}
		return v;
	}
	
	std::vector<DbLink*> RTreeSeacher::seachNearby(const std::vector<DbMesh*>& meshes, const BoundingBox3d& input, int32 posTolerance /*= 10*/, int32 zTolerance /*= 10*/)
	{
		std::vector<DbLink*> v;
		if (meshes.empty())
			return v;
		for (auto pMesh : meshes)
		{
			auto vs = seachNearby(pMesh, input, posTolerance, zTolerance);
			v.insert(v.end(), vs.begin(), vs.end());
		}
		return v;
	}
	
	std::vector<DbLink*> RTreeSeacher::seachNearby(const std::vector<DbMesh*>& meshes, const BoundingBox2d& input, int32 tolerance /*= 10*/)
	{
		std::vector<DbLink*> v;
		if (meshes.empty())
			return v;
		for (auto pMesh : meshes)
		{
			auto vs = seachNearby(pMesh, input, tolerance);
			v.insert(v.end(), vs.begin(), vs.end());
		}
		return v;
	}
	
	std::vector<DbLink*> RTreeSeacher::seachNearby3d(const std::vector<DbMesh*>& meshes, DbLink* pLink, MapPoint3D64 point, int32 posTolerance /*= 10*/, int32 zTolerance /*= 10*/)
	{
		std::vector<DbLink*> v;
		if (meshes.empty())
			return v;
		for (auto pMesh : meshes)
		{
			auto vs = seachNearby3d(pMesh, pLink, point, posTolerance, zTolerance);
			v.insert(v.end(), vs.begin(), vs.end());
		}
		return v;
	}
    
	std::vector<DbLink*> RTreeSeacher::seachNearby3d(DbMesh* pMesh, DbLink* pLink, MapPoint3D64 point, int32 posTolerance /*= 10*/, int32 zTolerance /*= 10*/)
    {
		point_2t point2T = POINT_2T(point);
		box_2t box = BOX_2T(point2T, posTolerance);

		std::vector<DbLink*> v;
		auto& linkInfos = pMesh->getLinkInfos();
		pMesh->getLinkRtree()->query(bgi::intersects(box),
			boost::make_function_output_iterator([&](size_t const& id) {
				if (pLink != linkInfos[id]._link)
				{
					segment_2t tmpSeg;
					auto tmpPoints = linkInfos[id]._linkPoints2T;
					bg::closest_points(point2T, tmpPoints, tmpSeg);
					double const dist = bg::length(tmpSeg);
					if (dist < posTolerance)
					{
						size_t si, ei;
						MapPoint3D64 grappedPt;
						auto& lineVertexes = linkInfos[id]._link->geometry.vertexes;
						if (GrapPointAlgorithm::grapOrMatchNearestPoint(point, lineVertexes, grappedPt, si, ei, posTolerance))
						{
							if (abs(point.z - grappedPt.z) < zTolerance)
							{
								v.push_back(linkInfos[id]._link);
							}
						}
					}
				}
				}));
		return v;
    }
	
	std::vector<DbLink*> RTreeSeacher::seachNearby(DbMesh* pMesh, const BoundingBox3d& input, int32 posTolerance /*= 10*/, int32 zTolerance /*= 10*/)
    {
		box_t box;
		point_t minPoint{ input.min.pos.lon, input.min.pos.lat, input.min.z };
		point_t maxPoint{ input.max.pos.lon, input.max.pos.lat, input.max.z };
		linestring_t line;
		line.push_back(minPoint);
		line.push_back(maxPoint);
		bg::envelope(line, box);

		point_t pos;
		bg::centroid(box, pos);
		MapPoint3D64 point = MapPoint3D64_make(pos.get<0>(), pos.get<1>(), pos.get<2>());

		std::vector<DbLink*> v;
		auto& linkInfos = pMesh->getLinkInfos();
		pMesh->getLinkRtree()->query(bgi::intersects(B3_B2(box)),
			boost::make_function_output_iterator([&](size_t const& id) {
				segment_2t tmpSeg;
				auto tmpPoints = linkInfos[id]._linkPoints2T;
				bg::closest_points(P3_P2(pos), tmpPoints, tmpSeg);
				double const dist = bg::length(tmpSeg);
				if (dist < posTolerance)
				{
					size_t si, ei;
					MapPoint3D64 grappedPt;
					auto& lineVertexes = linkInfos[id]._link->geometry.vertexes;
					if (GrapPointAlgorithm::grapOrMatchNearestPoint(point, lineVertexes, grappedPt, si, ei, posTolerance))
					{
						if (abs(point.z - grappedPt.z) < zTolerance)
						{
							v.push_back(linkInfos[id]._link);
						}
					}
				}
				}));
		return v;
    }
    
	std::vector<DbLink*> RTreeSeacher::seachNearby(DbMesh* pMesh, const BoundingBox2d& input, int32 tolerance /*= 10*/)
    {
		box_2t box;
		point_2t minPoint{ input.min.lon, input.min.lat };
		point_2t maxPoint{ input.max.lon, input.max.lat };
		linestring_2t line;
		line.push_back(minPoint);
		line.push_back(maxPoint);
		bg::envelope(line, box);

		point_2t tmpCenterPoint;
		bg::centroid(box, tmpCenterPoint);

		std::vector<DbLink*> v;
		auto& linkInfos = pMesh->getLinkInfos();
		pMesh->getLinkRtree()->query(bgi::intersects(box),
			boost::make_function_output_iterator([&](size_t const& id) {
				segment_2t tmpSeg;
				auto tmpPoints = linkInfos[id]._linkPoints2T;
				bg::closest_points(tmpCenterPoint, tmpPoints, tmpSeg);
				double const dist = bg::length(tmpSeg);
				if (dist < tolerance)
				{
					v.push_back(linkInfos[id]._link);
				}
				}));
        return v;
    }

	bool RTreeSeacher::intersectsWithLinks2d(DbMesh* pMesh, const MapPoint64& segmentStart, const MapPoint64& segmentEnd)
	{
		segment_2t segment{ POINT_2T(segmentStart), POINT_2T(segmentEnd) };
		box_2t box;
		bg::envelope(segment, box);

		bool intersects = false;
		auto& linkInfos = pMesh->getLinkInfos();
		pMesh->getLinkRtree()->query(bgi::intersects(box),
			boost::make_function_output_iterator([&](size_t const& id) {
					if (!intersects)
					{
					    auto tmpPoints = linkInfos[id]._linkPoints2T;
						intersects = bg::intersects(tmpPoints, segment);
					}
				}));

		return intersects;
	}

}


