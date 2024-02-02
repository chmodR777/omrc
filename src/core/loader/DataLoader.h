#pragma once
#include <vector>
#include "../record/DbRecord.h"
#include "sqlite3/sqlite3.h"
#include "../record/DbMesh.h"
#include "../Geometry.h"
#define IN 
#define OUT
#define DB_HAD_APPLY_PA_REFERENCE	 -99

namespace OMDB
{

    enum class DbStatus
    {
        OK,
        GEOMETRY_TYPE_ERROR,
        GEOMETRY_NULL,
        GEOMETRY_INVALID
    };
    class DataLoader
    {
    public:
        void Load(sqlite3* pDb,DbMesh* pDatabase);
        bool checkLaneBoundary(sqlite3* pDb);
		static MapPoint3D64 ToPoint3d(double x, double y, double z);
    protected:
        virtual void load() = 0;

        DbStatus prasePoint3d(const unsigned char* pBlob,unsigned int size,MapPoint3D64& point);
        DbStatus praseLineString3d(const unsigned char* pBlob, unsigned int size, LineString3d& point);
        DbStatus praseMultiPoint3d(const unsigned char* pBlob, unsigned int size, MultiPoint3d& point);
        DbStatus praseMultiLineString3d(const unsigned char* pBlob, unsigned int size, MultiLineString3d& point);
        DbStatus prasePolygon3d(const unsigned char* pBlob, unsigned int size, Polygon3d& point);
        DbStatus praseMultiPolygon3d(const unsigned char* pBlob, unsigned int size, MultiPolygon3d& point);

        sqlite3*                m_pSqlite3;
        DbMesh*                 m_pDatabase;
    };
}

