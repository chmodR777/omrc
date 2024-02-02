#pragma once
#include <vector>
#include "geometry/map_point3d64.h"
#include "cq_math.h"

namespace OMDB
{

    struct BoundingBox2d
	{
		MapPoint64 min;
		MapPoint64 max;

        static BoundingBox2d expand(const BoundingBox2d& box, int32 offset)
        {
            BoundingBox2d ex;
            ex.max.lon = box.max.lon + offset;
            ex.max.lat = box.max.lat + offset;
            ex.min.lon = box.min.lon - offset;
            ex.min.lat = box.min.lat - offset;
            return ex;
        }

        static NdsRect toNdsRect(const BoundingBox2d& input)
        {
			Point min84;
			Point min02 = input.min.toPoint();
			Math_marsToWgs(&min02, &min84);

			Point max84;
			Point max02 = input.max.toPoint();
			Math_marsToWgs(&max02, &max84);

			NdsPoint min = NdsPoint_makeWithPoint(min84);
			NdsPoint max = NdsPoint_makeWithPoint(max84);

			NdsRect ndsRect;
			NdsRect_set(&ndsRect, min.x, min.y, max.x, max.y);

            return ndsRect;
        }

        bool overlap(const BoundingBox2d& other, int tolerance = 0) const
        {
            return 
                (
                    (min.lon-tolerance >= other.min.lon-tolerance && min.lon-tolerance <= other.max.lon+tolerance) || 
                    (other.min.lon-tolerance >= min.lon-tolerance && other.min.lon-tolerance <= max.lon+tolerance)
                ) 
                &&
                (
                    (min.lat-tolerance >= other.min.lat-tolerance && min.lat-tolerance <= other.max.lat+tolerance) || 
                    (other.min.lat-tolerance >= min.lat-tolerance && other.min.lat-tolerance <= max.lat+tolerance)
                );
        }



        bool contain(const BoundingBox2d& other, int tolerance = 0) const
        {
            if (min.lat - tolerance <= other.min.lat &&
                min.lon - tolerance <= other.min.lon &&
                max.lat + tolerance >= other.max.lat &&
                max.lon + tolerance >= other.max.lon)
			{
				return true;
			}

			return false;
        }
	};


    struct BoundingBox3d
    {
        MapPoint3D64 min;
        MapPoint3D64 max;

        BoundingBox2d boundingbox2d() const
        {
            BoundingBox2d box;
            box.min = min.pos;
            box.max = max.pos;
            return box;
        }

        BoundingBox3d()
        {
			max = { {0,0,},0 };
            min = { {0,0,},0 };
        }
        bool overlap(const BoundingBox3d& other, int tolerance = 0) const
        {
            return
				(
					(min.pos.lon - tolerance >= other.min.pos.lon - tolerance && min.pos.lon - tolerance <= other.max.pos.lon + tolerance) ||
					(other.min.pos.lon - tolerance >= min.pos.lon - tolerance && other.min.pos.lon - tolerance <= max.pos.lon + tolerance)
					)
				&&
				(
					(min.pos.lat - tolerance >= other.min.pos.lat - tolerance && min.pos.lat - tolerance <= other.max.pos.lat + tolerance) ||
					(other.min.pos.lat - tolerance >= min.pos.lat - tolerance && other.min.pos.lat - tolerance <= max.pos.lat + tolerance)
					)
                &&
                ((min.z >= other.min.z && min.z <= other.max.z) || (other.min.z >= min.z && other.min.z <= max.z));
        }

        bool contain(const BoundingBox3d& other, int tolerance = 0) const
        {
            if (min.pos.lat - tolerance <= other.min.pos.lat &&
                min.pos.lon - tolerance <= other.min.pos.lon &&
                max.pos.lat + tolerance >= other.max.pos.lat &&
                max.pos.lon + tolerance >= other.max.pos.lon &&
                min.z <= other.min.z &&
                max.z >= other.max.z
                )
            {
                return true;
            }

            return false;
        }
    };


	struct MultiPoint3d
	{
		std::vector<MapPoint3D64> postions;
	};

	struct LineString3d
	{
		std::vector<MapPoint3D64> vertexes;
	};

	struct MultiLineString3d
	{
		std::vector<LineString3d> lines;
	};

	struct Polygon3d
	{
		std::vector<MapPoint3D64> vertexes;
	};

	struct MultiPolygon3d
	{
		std::vector<Polygon3d> polygons;
	};
}

