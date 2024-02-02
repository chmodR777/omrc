#include "stdafx.h"

#include "DataLoader.h"
#include "DbRecordIterator.h"
#include "spatialite.h"
#include "algorithm/polyline_simplifier.h"
#include "CompileSetting.h"

#define LINE_SIMPLIFY

namespace OMDB
{
    void DataLoader::Load(sqlite3* pDb,DbMesh* pDatabase)
    {
        m_pSqlite3 = pDb;
        m_pDatabase = pDatabase;
        load();
    }

    bool DataLoader::checkLaneBoundary(sqlite3* pDb)
    {
        auto recordIterator = DbRecordIterator(pDb);
        recordIterator.resetWithSqlStr("SELECT ROAD_BOUND_LINK_PID,GEOMETRY FROM HAD_RDBOUND_LINK");
        if (recordIterator.hasNextRecord())
            return true;
        recordIterator.resetWithSqlStr("SELECT LANE_MARK_LINK_PID,GEOMETRY FROM HAD_LANE_MARK_LINK");
        if (recordIterator.hasNextRecord())
            return true;
        return false;
    }

	MapPoint3D64 DataLoader::ToPoint3d(double x, double y, double z)
	{
		MapPoint3D64 pt;
		pt.pos.lon = (int64)(x * 1e8 + 0.5);
		pt.pos.lat = (int64)(y * 1e8 + 0.5);
		pt.z = int(z * 100 + 0.5);

        if (CompileSetting::instance()->isDaimlerShangHai)
        {
            pt.z -= 1200;
        }

		return pt;
	}

	DbStatus DataLoader::prasePoint3d(const unsigned char* pBlob, unsigned int size, MapPoint3D64& point)
    {
		gaiaGeomCollPtr geo = gaiaFromSpatiaLiteBlobWkb(pBlob, size);
		if (nullptr == geo)
			return DbStatus::GEOMETRY_INVALID;

		int geoType = gaiaGeometryType(geo);
		if (geoType != GAIA_POINTZ)
			return DbStatus::GEOMETRY_TYPE_ERROR;

        gaiaPointPtr pt = geo->FirstPoint;
        if (nullptr == pt)
            return DbStatus::GEOMETRY_INVALID;
		
        point = ToPoint3d(pt->X, pt->Y, pt->Z);

		gaiaFreeGeomColl(geo);
		return DbStatus::OK;
    }

    DbStatus DataLoader::praseLineString3d(const unsigned char* pBlob, unsigned int size, LineString3d& linestring)
    {
        gaiaGeomCollPtr geo = gaiaFromSpatiaLiteBlobWkb(pBlob, size);
        if (nullptr == geo)
            return DbStatus::GEOMETRY_INVALID;

        int geoType = gaiaGeometryType(geo);
        if (geoType != GAIA_LINESTRINGZ)
            return DbStatus::GEOMETRY_TYPE_ERROR;

        gaiaLinestringPtr line = geo->FirstLinestring;
        if (nullptr == line)
            return DbStatus::GEOMETRY_INVALID;

        linestring.vertexes.resize(line->Points);
        for (int i = 0; i < line->Points;i++)
        {
            double x, y, z;
            gaiaGetPointXYZ(line->Coords,i,&x,&y,&z);
            linestring.vertexes[i] = ToPoint3d(x, y, z);
        }

        gaiaFreeGeomColl(geo);
        // generator需要点序,此处不做稀释
//#ifdef LINE_SIMPLIFY
//		PolylineSimplifier::simplify(linestring.vertexes);
//#endif
        return DbStatus::OK;
    }

    DbStatus DataLoader::praseMultiPoint3d(const unsigned char* pBlob, unsigned int size, MultiPoint3d& multiPoint)
    {
        gaiaGeomCollPtr geo = gaiaFromSpatiaLiteBlobWkb(pBlob, size);
        if (nullptr == geo)
            return DbStatus::GEOMETRY_INVALID;

        int geoType = gaiaGeometryType(geo);
        if (geoType != GAIA_MULTIPOINTZ)
            return DbStatus::GEOMETRY_TYPE_ERROR;
        
        gaiaPointPtr pt = geo->FirstPoint;
        if (nullptr == pt)
            return DbStatus::GEOMETRY_INVALID;

        while (pt != nullptr)
        {
            multiPoint.postions.push_back(ToPoint3d(pt->X,pt->Y,pt->Z));
            pt = pt->Next;
        }

        gaiaFreeGeomColl(geo);
        return DbStatus::OK;
    }

    DbStatus DataLoader::praseMultiLineString3d(const unsigned char* pBlob, unsigned int size, MultiLineString3d& multiLineString)
    {
        gaiaGeomCollPtr geo = gaiaFromSpatiaLiteBlobWkb(pBlob, size);
        if (nullptr == geo)
            return DbStatus::GEOMETRY_INVALID;

        int geoType = gaiaGeometryType(geo);
        if (geoType != GAIA_MULTILINESTRINGZ)
            return DbStatus::GEOMETRY_TYPE_ERROR;
        
        gaiaLinestringPtr pt = geo->FirstLinestring;
        if (nullptr == pt)
            return DbStatus::GEOMETRY_INVALID;

        while (pt != nullptr)
        {
            LineString3d line;
            line.vertexes.resize(pt->Points);
            for (int i = 0; i < pt->Points; i++)
            {
                double x, y, z;
                gaiaGetPointXYZ(pt->Coords, i, &x,&y,&z);
                line.vertexes[i] = ToPoint3d(x, y, z);
            }
#ifdef LINE_SIMPLIFY
			PolylineSimplifier::simplify(line.vertexes);
#endif
            multiLineString.lines.push_back(line);
            pt = pt->Next;
        }

        gaiaFreeGeomColl(geo);
        return DbStatus::OK;
    }

    DbStatus DataLoader::prasePolygon3d(const unsigned char* pBlob, unsigned int size, Polygon3d& polygon)
    {
        gaiaGeomCollPtr geo = gaiaFromSpatiaLiteBlobWkb(pBlob, size);
        if (nullptr == geo)
            return DbStatus::GEOMETRY_INVALID;

        int geoType = gaiaGeometryType(geo);
        if (geoType != GAIA_POLYGONZ)
            return DbStatus::GEOMETRY_TYPE_ERROR;

        gaiaPolygonPtr pt = geo->FirstPolygon;

        if (nullptr == pt)
            return DbStatus::GEOMETRY_INVALID;
        if (pt != nullptr)
        {
            gaiaRingPtr rng = pt->Exterior;
            polygon.vertexes.resize(rng->Points);
            for (int i = 0; i < rng->Points; i++)
            {
                double x, y, z;
                gaiaGetPointXYZ(rng->Coords, i, &x,&y,&z);
                polygon.vertexes[i] = ToPoint3d(x, y, z);
            }
        }
        gaiaFreeGeomColl(geo);
        return DbStatus::OK;

    }

    DbStatus DataLoader::praseMultiPolygon3d(const unsigned char* pBlob, unsigned int size, MultiPolygon3d& multiPolygon)
    {
        gaiaGeomCollPtr geo = gaiaFromSpatiaLiteBlobWkb(pBlob, size);
        if (nullptr == geo)
            return DbStatus::GEOMETRY_INVALID;

        int geoType = gaiaGeometryType(geo);
        if (geoType != GAIA_MULTIPOLYGONZ)
            return DbStatus::GEOMETRY_TYPE_ERROR;

        gaiaPolygonPtr pt = geo->FirstPolygon;

        if (nullptr == pt)
            return DbStatus::GEOMETRY_INVALID;

        while (pt != nullptr)
        {
            gaiaRingPtr rng = pt->Exterior;
            Polygon3d polygon;
            polygon.vertexes.resize(rng->Points);
            for (int i = 0; i < rng->Points; i++)
            {
                double x, y, z;
                gaiaGetPointXYZ(rng->Coords, i, &x,&y,&z);
                polygon.vertexes[i] = ToPoint3d(x, y, z);
            }
            multiPolygon.polygons.push_back(polygon);
            pt = pt->Next;
        }
        gaiaFreeGeomColl(geo);
        return DbStatus::OK;
    }

}
