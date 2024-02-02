#ifndef _SPATIALITEWRITER_H
#define _SPATIALITEWRITER_H

#include "Writer.h"
#include "sqlite3/sqlite3.h"
#include "spatialite.h"
class RdsGroup;
namespace RDS
{
    class SpatialiteWriter : public Writer
    {
    public:
        SpatialiteWriter();
    protected:
        void write(const char* path,RdsTile* tile) override;
    private:
        void writeHeader(sqlite3* pDb, const RdsTile* const tile);
        void writeRoad(sqlite3* pDb,const std::vector<RdsObject*>& roads);
        void writeText(sqlite3* pDb, const std::vector<RdsObject*>& texts);
        void writeToll(sqlite3* pDb, const std::vector<RdsObject*>& tolls);
        void writeTunnel(sqlite3* pDb, const std::vector<RdsObject*>& tunnels);
        void writeIntersection(sqlite3* pDb, const std::vector<RdsObject*>& intersections);
        void writeGuardrail(sqlite3* pDb, const std::vector<RdsObject*>& rails);
        void writeMarking(sqlite3* pDb, const std::vector<RdsObject*>& markings);
        void writeLine(sqlite3* pDb, const std::vector<RdsObject*>& lines);
        void writeDiversion(sqlite3* pDb, const std::vector<RdsObject*>& diversions);
        //void writeSign3d(sqlite3* pDb, const std::vector<RdsObject*>& sign3ds);
		void writeSpeedLimitBoard(sqlite3* pDb, const std::vector<RdsObject*>& sign3ds);
        void writePier(sqlite3* pDb, const std::vector<RdsObject*>& piers);
        void writeGreenbelt(sqlite3* pDb, const std::vector<RdsObject*>& greenbelts);
        void writeGroup(sqlite3* pDb, const std::vector<RdsObject*>& groups);
        void writeCrossWalk(sqlite3* pDb, const std::vector<RdsObject*>& crosswalks);
        void writeRelationship(sqlite3* pDb, const std::vector<RdsObject*>& relationships);
        void writeTrafficLights(sqlite3* pDb, const std::vector<RdsObject*>& trafficLights);
    private:
		void writeLineString(gaiaGeomCollPtr geo, const LineString3d& edge);
		void writeMultiLineString(gaiaGeomCollPtr geo, const MultiLineString3d& polygon);
		void writePoint(gaiaGeomCollPtr geo, const Point3d& pt);
		void writeMultiPoint(gaiaGeomCollPtr geo, const MultiPoint3d& pt);
        void writePolygon(gaiaGeomCollPtr geo, const Polygon3d& polygon);
        void writeMultiPolygon(gaiaGeomCollPtr geo, const MultiPolygon3d& multipolygon);
    };
}
#endif

