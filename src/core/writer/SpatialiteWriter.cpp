#include "stdafx.h"
#include "SpatialiteWriter.h"
#include <string.h>
#include <iostream>
#include <stdio.h>
#include "spatialite.h"
#include <fstream>
namespace RDS
{

	SpatialiteWriter::SpatialiteWriter()
	{
	}

	//#define USE_SPATIALITE_FLOAT 1
	void SpatialiteWriter::write(const char* path, RdsTile* tile)
	{
		std::ifstream f(path);
		if (!f.good())
		{
			remove(path);
		}
		f.open(path, std::ios::_Nocreate);

		sqlite3* pDb = nullptr;
		int status = sqlite3_open_v2(path, &pDb, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, NULL);
		if (status != SQLITE_OK)
		{
			sqlite3_close(pDb);
			return;
		}

        sqlite3_exec(pDb, "PRAGMA synchronous = OFF", 0, 0, 0);
		void* pConnection = spatialite_alloc_connection();
		spatialite_init_ex(pDb, pConnection, 0);

		std::string sql = "SELECT InitSpatialMetadata('WGS84')";
		char* msg = nullptr;
		status = sqlite3_exec(pDb, sql.c_str(), nullptr, nullptr, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_close(pDb);
			return;
		}

		writeHeader(pDb,tile);
		writeLine(pDb, tile->query(EntityType::RDS_LINE));
		writeRoad(pDb, tile->query(EntityType::RDS_ROAD));
		writeText(pDb, tile->query(EntityType::RDS_TEXT));
		writeToll(pDb, tile->query(EntityType::RDS_TOLL));
		writeTunnel(pDb, tile->query(EntityType::RDS_TUNNEL));
		writeIntersection(pDb, tile->query(EntityType::RDS_INTERSECTION));
		writeGuardrail(pDb, tile->query(EntityType::RDS_GUARDRAIL));
		writeMarking(pDb, tile->query(EntityType::RDS_MARKING));
		writeDiversion(pDb, tile->query(EntityType::RDS_DIVERSION));
		//writeSign3d(pDb, tile->query(EntityType::RDS_SIGN3D));
		writeSpeedLimitBoard(pDb, tile->query(EntityType::RDS_SPEEDLIMITBOARD));

		writePier(pDb, tile->query(EntityType::RDS_PIER));
		writeGreenbelt(pDb, tile->query(EntityType::RDS_GREENBELT));
		writeGroup(pDb, tile->query(EntityType::RDS_GROUP));
		writeRelationship(pDb, tile->query(EntityType::RDS_RELATIONSHIP));
		writeTrafficLights(pDb, tile->query(EntityType::RDS_TRAFFICLIGHT));

		status = sqlite3_close(pDb);
		if (status != SQLITE_OK)
		{
			return;
		}
		spatialite_cleanup_ex(pConnection);
	}

	void SpatialiteWriter::writeLineString(gaiaGeomCollPtr geo, const LineString3d& edge)
	{
		geo->Srid = 0;
		gaiaLinestringPtr pl = gaiaAddLinestringToGeomColl(geo, edge.vertexes.size());
		pl->DimensionModel = GAIA_XY_Z;
		for (int n = 0; n < edge.vertexes.size(); n++)
		{
			Point3d pt = edge.vertexes[n];
#if USE_SPATIALITE_FLOAT
			gaiaSetPointXYZ(pl->Coords, n, pt.x*1.0/1e8, pt.y*1.0/1e8, pt.z/100.0);
#else
			gaiaSetPointXYZ(pl->Coords, n, pt.x, pt.y, pt.z);
#endif
		}
	}

	void SpatialiteWriter::writeMultiLineString(gaiaGeomCollPtr geo, const MultiLineString3d& polygon)
	{
		geo->Srid = 0;
		for(auto line : polygon.lines)
		{
			writeLineString(geo, line);
		}
	}

	void SpatialiteWriter::writePoint(gaiaGeomCollPtr geo, const Point3d& pt)
	{
#if USE_SPATIALITE_FLOAT
		gaiaAddPointToGeomCollXYZ(geo, pt.x/1e8, pt.y/1e8, pt.z/100.0);
#else
		gaiaAddPointToGeomCollXYZ(geo, pt.x, pt.y, pt.z);
#endif
	}

	void SpatialiteWriter::writeMultiPoint(gaiaGeomCollPtr geo, const MultiPoint3d& multiPoint)
	{
		geo->Srid = 0;
		geo->DimensionModel = GAIA_XY_Z;
		for (auto p : multiPoint.postions)
		{
#if USE_SPATIALITE_FLOAT
			gaiaAddPointToGeomCollXYZ(geo, p.x/1e8, p.y/1e8, p.z/100.0);
#else
			gaiaAddPointToGeomCollXYZ(geo, p.x, p.y, p.z);

#endif
		}

	}

	void SpatialiteWriter::writePolygon(gaiaGeomCollPtr geo, const Polygon3d& polygon)
	{
		geo->Srid = 0;
		int size = polygon.vertexes.size();
		gaiaPolygonPtr gpoly = gaiaAddPolygonToGeomColl(geo, size, 0);
		gpoly->DimensionModel = GAIA_XY_Z;
		gaiaRingPtr pExterior = gpoly->Exterior;

		std::vector<Point3d> vertexes;
		for (int i = 0; i < polygon.vertexes.size(); i++)
		{
			Point3d pt = polygon.vertexes[i];
#if USE_SPATIALITE_FLOAT
			gaiaSetPointXYZ(pExterior->Coords, i, pt.x/1e8, pt.y/1e8, pt.z/100.0);
#else
			gaiaSetPointXYZ(pExterior->Coords, i, pt.x, pt.y, pt.z);
#endif
		}

	}

	void SpatialiteWriter::writeMultiPolygon(gaiaGeomCollPtr geo, const MultiPolygon3d& multipolygon)
	{
		geo->Srid = 0;
		for (auto pl : multipolygon.polygons)
		{
			writePolygon(geo, pl);
		}
	}

	void SpatialiteWriter::writeHeader(sqlite3* pDb, const RdsTile* const pTile)
	{
		const char* sql = "CREATE TABLE HEADER_INTO ("
			"MESH_ID				  INTEGER                 NOT NULL,"
			"DATE						  TEXT,"
			"VERSION				  TEXT, "
			"LONGITUDE             INTEGER, "
			"LATITUDE                INTEGER, "
			"EXTENT					BLOB  );";

		char* msg;
		int status = sqlite3_exec(pDb, sql, NULL, 0, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sqlite3_stmt* stmt;
		sql = "INSERT INTO HEADER_INTO (MESH_ID, DATE,VERSION,LONGITUDE,LATITUDE,EXTENT) VALUES (?,?,?,?,?,?)";
		status = sqlite3_prepare_v2(pDb, sql, strlen(sql), &stmt, NULL);
		if (status != SQLITE_OK)
			return;

		sqlite3_bind_int64(stmt, 1, pTile->meshId);

		std::string date;
		sqlite3_bind_text(stmt, 2, date.c_str(), date.length(), nullptr);
		sqlite3_bind_text(stmt, 3, pTile->version.c_str(), pTile->version.length(), nullptr);
		sqlite3_bind_int64(stmt, 4, pTile->longitude);
		sqlite3_bind_int64(stmt, 5, pTile->latitude);

		unsigned char* blob = (unsigned char*)(&pTile->extent);
		sqlite3_bind_blob(stmt, 6, blob, sizeof(pTile->extent), nullptr);
		sqlite3_finalize(stmt);
	}

	void SpatialiteWriter::writeRoad(sqlite3* pDb, const std::vector<RdsObject*>& roads)
	{
		const char* sql = "CREATE TABLE RDS_ROAD("
			"ID                INTEGER PRIMARY KEY      NOT NULL);";

		char* msg;
		int status = sqlite3_exec(pDb, sql, NULL, 0, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "SELECT AddGeometryColumn('RDS_ROAD', 'GEOMETRY',0, 'MULTILINESTRINGZ')";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "BEGIN";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}


		sqlite3_stmt* stmt;
		sql = "INSERT INTO RDS_ROAD (ID,GEOMETRY) VALUES (?,?)";
		status = sqlite3_prepare_v2(pDb, sql, strlen(sql), &stmt, NULL);
		if (status != SQLITE_OK)
			return;

		for (int i = 0; i < roads.size(); i++)
		{
			RdsRoad* pRoad = (RdsRoad*)roads[i];
			gaiaGeomCollPtr geo = gaiaAllocGeomCollXYZ();
			geo->Srid = 0;
			writeMultiLineString(geo, pRoad->contour);
			unsigned char* blob;
			int size;
			gaiaToSpatiaLiteBlobWkb(geo, &blob, &size);
			gaiaFreeGeomColl(geo);
			sqlite3_reset(stmt);
			sqlite3_clear_bindings(stmt);
			sqlite3_bind_int64(stmt, 1, pRoad->id);
			sqlite3_bind_blob(stmt, 2, blob, size, free);

			status = sqlite3_step(stmt);
			if (status != SQLITE_DONE && status != SQLITE_ROW)
			{
				break;
			}
		}
		sqlite3_finalize(stmt);
		sql = "COMMIT";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}
	}


	void SpatialiteWriter::writeText(sqlite3* pDb, const std::vector<RdsObject*>& texts)
	{
		const char* sql = "CREATE TABLE RDS_TEXT("
			"ID                INTEGER PRIMARY KEY      NOT NULL, "
			"CONTENT           TEXT, "
			"COLOR             INTEGER, "
			"TYPE			   INTEGER, "
			"SPEED_LIMIT_TYPE  INTEGER);";

		char* msg;
		int status = sqlite3_exec(pDb, sql, NULL, 0, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "SELECT AddGeometryColumn('RDS_TEXT', 'GEOMETRY', 0, 'POLYGONZ')";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "BEGIN";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}


		sqlite3_stmt* stmt;
		sql = "INSERT INTO RDS_TEXT (ID,COLOR,CONTENT,GEOMETRY,TYPE,SPEED_LIMIT_TYPE) VALUES (?,?,?,?,?,?)";
		status = sqlite3_prepare_v2(pDb, sql, strlen(sql), &stmt, NULL);
		if (status != SQLITE_OK)
			return;

		for (int i = 0; i < texts.size(); i++)
		{
			RdsText* pText = (RdsText*)texts[i];

			gaiaGeomCollPtr geo = gaiaAllocGeomCollXYZ();
			geo->Srid = 0;
			writePolygon(geo, pText->contour);
			unsigned char* blob;
			int size;
			gaiaToSpatiaLiteBlobWkb(geo, &blob, &size);
			gaiaFreeGeomColl(geo);
			sqlite3_reset(stmt);
			sqlite3_clear_bindings(stmt);

			sqlite3_bind_int64(stmt, 1, pText->id);
			sqlite3_bind_int(stmt, 2, (int32)pText->color);
			//sqlite3_bind_text(stmt, 3, pText->text.c_str(), pText->text.size(), nullptr);
			sqlite3_bind_text16(stmt, 3, pText->text.c_str(), sizeof(wchar_t)*pText->text.size(), nullptr);
			sqlite3_bind_blob(stmt, 4, blob, size, free);
			sqlite3_bind_int(stmt, 5, (int)pText->type);
			sqlite3_bind_int(stmt, 6, (int)pText->speedLimitType);

			status = sqlite3_step(stmt);
			if (status != SQLITE_DONE && status != SQLITE_ROW)
			{
				break;
			}
		}
		sqlite3_finalize(stmt);
		sql = "COMMIT";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}
	}

	void SpatialiteWriter::writeToll(sqlite3* pDb, const std::vector<RdsObject*>& tolls)
	{
		const char* sql = "CREATE TABLE RDS_TOLL("
			"ID                INTEGER PRIMARY KEY				, "
			"NAME              TEXT								, "
			"TYPELIST		   BLOB								, "
			"DIRECTION		   BLOB								);";

		char* msg;
		int status = sqlite3_exec(pDb, sql, NULL, 0, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "SELECT AddGeometryColumn('RDS_TOLL', 'GEOMETRY',0, 'MULTIPOINTZ')";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "BEGIN";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sqlite3_stmt* stmt;
		sql = "INSERT INTO RDS_TOLL (ID,NAME,TYPELIST,DIRECTION,GEOMETRY) VALUES (?,?,?,?,?)";
		status = sqlite3_prepare_v2(pDb, sql, strlen(sql), &stmt, NULL);
		if (status != SQLITE_OK)
			return;

		for (int i = 0; i < tolls.size(); i++)
		{
			RdsToll* pToll = (RdsToll*)tolls[i];

			gaiaGeomCollPtr geo = gaiaAllocGeomCollXYZ();
			geo->Srid = 0;
			writeMultiPoint(geo, pToll->positions);
			unsigned char* blob;
			int size;
			gaiaToSpatiaLiteBlobWkb(geo, &blob, &size);
			gaiaFreeGeomColl(geo);
			sqlite3_reset(stmt);
			sqlite3_clear_bindings(stmt);

			sqlite3_bind_int64(stmt, 1, pToll->id);
			//sqlite3_bind_text(stmt, 2, pToll->name.c_str(), pToll->name.size(), nullptr);
			sqlite3_bind_text16(stmt, 2, pToll->name.c_str(), sizeof(wchar_t)*pToll->name.size(), nullptr);
			sqlite3_bind_blob(stmt, 3, pToll->typeList.data(), sizeof(int)*pToll->typeList.size(), nullptr);
			sqlite3_bind_int(stmt,4,pToll->angle);
			sqlite3_bind_blob(stmt, 5, blob, size, free);

			status = sqlite3_step(stmt);
			if (status != SQLITE_DONE && status != SQLITE_ROW)
			{
				break;
			}
		}
		sqlite3_finalize(stmt);
		sql = "COMMIT";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}
	}

	void SpatialiteWriter::writeTunnel(sqlite3* pDb, const std::vector<RdsObject*>& tunnels)
	{
		const char* sql = "CREATE TABLE RDS_TUNNEL("
			"ID                INTEGER PRIMARY KEY      NOT NULL, "
			"TYPE			   INTERGER , "
			"THICKNESS		   REAL, "
			"HEIGHT         REAL                       );";

		char* msg;
		int status = sqlite3_exec(pDb, sql, NULL, 0, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "SELECT AddGeometryColumn('RDS_TUNNEL', 'GEOMETRY',0, 'MULTILINESTRINGZ')";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "BEGIN";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sqlite3_stmt* stmt;
		sql = "INSERT INTO RDS_TUNNEL (ID,TYPE,GEOMETRY,HEIGHT,THICKNESS) VALUES (?,?,?,?,?)";
		status = sqlite3_prepare_v2(pDb, sql, strlen(sql), &stmt, NULL);
		if (status != SQLITE_OK)
			return;

		for (int i = 0; i < tunnels.size(); i++)
		{
			RdsTunnel* pTunnel = (RdsTunnel*)tunnels[i];
			gaiaGeomCollPtr geo = gaiaAllocGeomCollXYZ();
			geo->Srid = 0;
			writeMultiLineString(geo, pTunnel->contour);
			unsigned char* blob;
			int size;
			gaiaToSpatiaLiteBlobWkb(geo, &blob, &size);
			gaiaFreeGeomColl(geo);
			sqlite3_reset(stmt);
			sqlite3_clear_bindings(stmt);
			sqlite3_bind_int64(stmt, 1, pTunnel->id);
			sqlite3_bind_int(stmt, 2, (int)pTunnel->tunnelType);
			sqlite3_bind_blob(stmt, 3, blob, size, free);
			sqlite3_bind_double(stmt, 4, pTunnel->height);
			sqlite3_bind_double(stmt, 5, pTunnel->thickness);

			status = sqlite3_step(stmt);
			if (status != SQLITE_DONE && status != SQLITE_ROW)
			{
				break;
			}
		}
		sqlite3_finalize(stmt);
		sql = "COMMIT";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}
	}

	void SpatialiteWriter::writeIntersection(sqlite3* pDb, const std::vector<RdsObject*>& intersections)
	{
		const char* sql = "CREATE TABLE RDS_INTERSECTION("
			"ID                INTEGER PRIMARY KEY      NOT NULL);";

		char* msg;
		int status = sqlite3_exec(pDb, sql, NULL, 0, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "SELECT AddGeometryColumn('RDS_INTERSECTION', 'GEOMETRY',0, 'POLYGONZ')";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "BEGIN";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}


		sqlite3_stmt* stmt;
		sql = "INSERT INTO RDS_INTERSECTION (ID,GEOMETRY) VALUES (?,?)";
		status = sqlite3_prepare_v2(pDb, sql, strlen(sql), &stmt, NULL);
		if (status != SQLITE_OK)
			return;

		for (int i = 0; i < intersections.size(); i++)
		{
			RdsIntersection* pIntersection = (RdsIntersection*)intersections[i];
			gaiaGeomCollPtr geo = gaiaAllocGeomCollXYZ();
			geo->Srid = 0;
			writePolygon(geo, pIntersection->contour);
			unsigned char* blob;
			int size;
			gaiaToSpatiaLiteBlobWkb(geo, &blob, &size);
			gaiaFreeGeomColl(geo);
			sqlite3_reset(stmt);
			sqlite3_clear_bindings(stmt);
			sqlite3_bind_int64(stmt, 1, pIntersection->id);
			sqlite3_bind_blob(stmt, 2, blob, size, free);

			status = sqlite3_step(stmt);
			if (status != SQLITE_DONE && status != SQLITE_ROW)
			{
				break;
			}
		}
		sqlite3_finalize(stmt);
		sql = "COMMIT";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}
	}

	void SpatialiteWriter::writeGuardrail(sqlite3* pDb, const std::vector<RdsObject*>& rails)
	{
		const char* sql = "CREATE TABLE RDS_GUARDRAIL("
			"ID                INTEGER PRIMARY KEY      NOT NULL, "
			"TYPE			   INTERGER                 );";

		char* msg;
		int status = sqlite3_exec(pDb, sql, NULL, 0, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "SELECT AddGeometryColumn('RDS_GUARDRAIL', 'GEOMETRY',0, 'LINESTRINGZ')";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "BEGIN";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}


		sqlite3_stmt* stmt;
		sql = "INSERT INTO RDS_GUARDRAIL (ID,TYPE,GEOMETRY) VALUES (?,?,?)";
		status = sqlite3_prepare_v2(pDb, sql, strlen(sql), &stmt, NULL);
		if (status != SQLITE_OK)
			return;

		for (int i = 0; i < rails.size(); i++)
		{
			RdsGuardrail* pGuardrail = (RdsGuardrail*)rails[i];
			gaiaGeomCollPtr geo = gaiaAllocGeomCollXYZ();
			geo->Srid = 0;
			writeLineString(geo, pGuardrail->location);
			unsigned char* blob;
			int size;
			gaiaToSpatiaLiteBlobWkb(geo, &blob, &size);
			gaiaFreeGeomColl(geo);
			sqlite3_reset(stmt);
			sqlite3_clear_bindings(stmt);
			sqlite3_bind_int64(stmt, 1, pGuardrail->id);
			sqlite3_bind_int(stmt, 2, (int)pGuardrail->railType);
			sqlite3_bind_blob(stmt, 3, blob, size, free);

			status = sqlite3_step(stmt);
			if (status != SQLITE_DONE && status != SQLITE_ROW)
			{
				break;
			}
		}
		sqlite3_finalize(stmt);
		sql = "COMMIT";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}
	}

	void SpatialiteWriter::writeMarking(sqlite3* pDb, const std::vector<RdsObject*>& markings)
	{
		const char* sql = "CREATE TABLE RDS_MARKING("
			"ID                INTEGER PRIMARY KEY      NOT NULL, "
			"TYPE			   INTERGER                 , "
			"COLOR			   INTERGER);";

		char* msg;
		int status = sqlite3_exec(pDb, sql, NULL, 0, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "SELECT AddGeometryColumn('RDS_MARKING', 'GEOMETRY',0, 'POLYGONZ')";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "BEGIN";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}


		sqlite3_stmt* stmt;
		sql = "INSERT INTO RDS_MARKING (ID,TYPE,COLOR,GEOMETRY) VALUES (?,?,?,?)";
		status = sqlite3_prepare_v2(pDb, sql, strlen(sql), &stmt, NULL);
		if (status != SQLITE_OK)
			return;

		for (int i = 0; i < markings.size(); i++)
		{
			RdsMarking* pMarking = (RdsMarking*)markings[i];
			gaiaGeomCollPtr geo = gaiaAllocGeomCollXYZ();
			geo->Srid = 0;
			writePolygon(geo, pMarking->contour);
			unsigned char* blob;
			int size;
			gaiaToSpatiaLiteBlobWkb(geo, &blob, &size);
			gaiaFreeGeomColl(geo);
			sqlite3_reset(stmt);
			sqlite3_clear_bindings(stmt);

			sqlite3_bind_int64(stmt, 1, pMarking->id);
			sqlite3_bind_int(stmt, 2, (int)pMarking->markingType);
			sqlite3_bind_int(stmt, 3, (int32)pMarking->color);
			sqlite3_bind_blob(stmt, 4, blob, size, free);

			status = sqlite3_step(stmt);
			if (status != SQLITE_DONE && status != SQLITE_ROW)
			{
				break;
			}
		}
		sqlite3_finalize(stmt);
		sql = "COMMIT";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}
	}

	void SpatialiteWriter::writeLine(sqlite3* pDb, const std::vector<RdsObject*>& lines)
	{
		const char* sql = "CREATE TABLE RDS_LINE("
			"ID                INTEGER PRIMARY KEY      NOT NULL, "
			"LINETYPE			   INTERGER                 , "
			"LINEWIDTH			   INTERGER                 , "
			"LINECOLOR			   INTERGER                 , "
			"SIDE			   INTERGER                 );";

		char* msg;
		int status = sqlite3_exec(pDb, sql, NULL, 0, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "SELECT AddGeometryColumn('RDS_LINE', 'GEOMETRY',0, 'LINESTRINGZ')";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "BEGIN";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}


		sqlite3_stmt* stmt;
		sql = "INSERT INTO RDS_LINE (ID,LINETYPE,LINEWIDTH,LINECOLOR,SIDE,GEOMETRY) VALUES (?,?,?,?,?,?)";
		status = sqlite3_prepare_v2(pDb, sql, strlen(sql), &stmt, NULL);
		if (status != SQLITE_OK)
			return;

		for (int i = 0; i < lines.size(); i++)
		{
			RdsLine* pLine = (RdsLine*)lines[i];
			gaiaGeomCollPtr geo = gaiaAllocGeomCollXYZ();
			geo->Srid = 0;
			writeLineString(geo, pLine->location);
			unsigned char* blob;
			int size;
			gaiaToSpatiaLiteBlobWkb(geo, &blob, &size);
			gaiaFreeGeomColl(geo);
			sqlite3_reset(stmt);
			sqlite3_clear_bindings(stmt);
			sqlite3_bind_int64(stmt, 1, pLine->id);

			sqlite3_bind_int(stmt, 2, (int)pLine->lineType);
			sqlite3_bind_int(stmt, 3, (int)pLine->width);
			sqlite3_bind_int(stmt, 4, (int)pLine->color);
			sqlite3_bind_int(stmt, 5, (int)pLine->side);
			sqlite3_bind_blob(stmt, 6, blob, size, free);

			status = sqlite3_step(stmt);
			if (status != SQLITE_DONE && status != SQLITE_ROW)
			{
				break;
			}
		}
		sqlite3_finalize(stmt);
		sql = "COMMIT";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}
	}

	void SpatialiteWriter::writeDiversion(sqlite3* pDb, const std::vector<RdsObject*>& diversions)
	{
		const char* sql = "CREATE TABLE RDS_DIVERSION("
			"ID                INTEGER PRIMARY KEY      NOT NULL, "
			"TYPE			   INTERGER                 , "
			"DIRECTION		   BLOB						);";

		char* msg;
		int status = sqlite3_exec(pDb, sql, NULL, 0, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "SELECT AddGeometryColumn('RDS_DIVERSION', 'CENTER',0, 'LINESTRINGZ')";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "SELECT AddGeometryColumn('RDS_DIVERSION', 'GEOMETRY',0, 'POLYGONZ')";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "BEGIN";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sqlite3_stmt* stmt;
		sql = "INSERT INTO RDS_DIVERSION (ID,TYPE,DIRECTION,CENTER,GEOMETRY) VALUES (?,?,?,?,?)";
		status = sqlite3_prepare_v2(pDb, sql, strlen(sql), &stmt, NULL);
		if (status != SQLITE_OK)
			return;

		for (int i = 0; i < diversions.size(); i++)
		{
			RdsDiversion* pDiversion = (RdsDiversion*)diversions[i];
			gaiaGeomCollPtr center = gaiaAllocGeomCollXYZ();
			center->Srid = 0;
			writeLineString(center, pDiversion->centerRefereceLine);
			unsigned char* centerBlob;
			int centerSize;
			gaiaToSpatiaLiteBlobWkb(center, &centerBlob, &centerSize);
			gaiaFreeGeomColl(center);

			gaiaGeomCollPtr geo = gaiaAllocGeomCollXYZ();
			geo->Srid = 0;
			writePolygon(geo, pDiversion->contour);
			unsigned char* geoBlob;
			int geoSize;
			gaiaToSpatiaLiteBlobWkb(geo, &geoBlob, &geoSize);
			gaiaFreeGeomColl(geo);

			sqlite3_reset(stmt);
			sqlite3_clear_bindings(stmt);
			sqlite3_bind_int64(stmt, 1, pDiversion->id);
			sqlite3_bind_int(stmt, 2, (int)pDiversion->diversionType);
			sqlite3_bind_int(stmt, 3, pDiversion->angle);

			sqlite3_bind_blob(stmt, 4, centerBlob, centerSize, free);
			sqlite3_bind_blob(stmt, 5, geoBlob, geoSize, free);

			status = sqlite3_step(stmt);
			if (status != SQLITE_DONE && status != SQLITE_ROW)
			{
				break;
			}
		}
		sqlite3_finalize(stmt);
		sql = "COMMIT";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}
	}

	/*
	void SpatialiteWriter::writeSign3d(sqlite3* pDb, const std::vector<RdsObject*>& sign3ds)
	{
		const char* sql = "CREATE TABLE RDS_SIGN3D("
			"ID                INTEGER PRIMARY KEY      NOT NULL, "
            "DIRECTION         BLOB                 NOT NULL,"
			"TYPE      INTEGER                 NOT NULL,"
            "CONTENT		   TEXT                 NOT NULL);";

		char* msg;
		int status = sqlite3_exec(pDb, sql, NULL, 0, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "SELECT AddGeometryColumn('RDS_SIGN3D', 'GEOMETRY',0, 'POINTZ')";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "BEGIN";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}
		sqlite3_stmt* stmt;
		sql = "INSERT INTO RDS_SIGN3D (ID,GEOMETRY,DIRECTION,CONTENT,TYPE) VALUES (?,?,?,?,?)";
		status = sqlite3_prepare_v2(pDb, sql, strlen(sql), &stmt, NULL);
		if (status != SQLITE_OK)
			return;

		for (int i = 0; i < sign3ds.size(); i++)
		{
			RdsSign3d* pSign3d = (RdsSign3d*)sign3ds[i];
			gaiaGeomCollPtr geo = gaiaAllocGeomCollXYZ();
			geo->Srid = 0;
			writePoint(geo, pSign3d->position);
			unsigned char* blob;
			int size;
			gaiaToSpatiaLiteBlobWkb(geo, &blob, &size);
			gaiaFreeGeomColl(geo);
			sqlite3_reset(stmt);
			sqlite3_clear_bindings(stmt);
			sqlite3_bind_int64(stmt, 1, pSign3d->id);
			sqlite3_bind_blob(stmt, 2, blob, size, free);
			sqlite3_bind_int(stmt, 3, pSign3d->angle);
			sqlite3_bind_text(stmt, 4, pSign3d->content.c_str(), pSign3d->content.size(), nullptr);
			sqlite3_bind_int(stmt, 5, (int)pSign3d->signType);

			status = sqlite3_step(stmt);
			if (status != SQLITE_DONE && status != SQLITE_ROW)
			{
				break;
			}
		}
		sqlite3_finalize(stmt);
		sql = "COMMIT";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}
	}
	*/

	void SpatialiteWriter::writeSpeedLimitBoard(sqlite3* pDb, const std::vector<RdsObject*>& sign3ds)
	{
		const char* sql = "CREATE TABLE RDS_SPEEDLIMITBOARD("
			"ID                INTEGER PRIMARY KEY      NOT NULL, "
			"TYPE         INTEGER                 NOT NULL,"
			"ABOVE         BLOB                 NOT NULL,"
			"BLOW      INTEGER                 NOT NULL);";

		char* msg;
		int status = sqlite3_exec(pDb, sql, NULL, 0, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "SELECT AddGeometryColumn('RDS_SPEEDLIMITBOARD', 'GEOMETRY',0, 'POINTZ')";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "BEGIN";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}
		sqlite3_stmt* stmt;
		sql = "INSERT INTO RDS_SPEEDLIMITBOARD (ID,GEOMETRY,TYPE,ABOVE,BLOW) VALUES (?,?,?,?,?)";
		status = sqlite3_prepare_v2(pDb, sql, strlen(sql), &stmt, NULL);
		if (status != SQLITE_OK)
			return;

		for (int i = 0; i < sign3ds.size(); i++)
		{
			RdsSpeedLimitBoard* pBoard = (RdsSpeedLimitBoard*)sign3ds[i];
			gaiaGeomCollPtr geo = gaiaAllocGeomCollXYZ();
			geo->Srid = 0;
			writePoint(geo, pBoard->position);
			unsigned char* blob;
			int size;
			gaiaToSpatiaLiteBlobWkb(geo, &blob, &size);
			gaiaFreeGeomColl(geo);
			sqlite3_reset(stmt);
			sqlite3_clear_bindings(stmt);
			sqlite3_bind_int64(stmt, 1, pBoard->id);
			sqlite3_bind_blob(stmt, 2, blob, size, free);
			sqlite3_bind_int(stmt, 3, (int)pBoard->type);
			sqlite3_bind_int(stmt, 4, pBoard->above);
			sqlite3_bind_int(stmt, 5, pBoard->blow);
			status = sqlite3_step(stmt);
			if (status != SQLITE_DONE && status != SQLITE_ROW)
			{
				break;
			}
		}
		sqlite3_finalize(stmt);
		sql = "COMMIT";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}
	}

	void SpatialiteWriter::writePier(sqlite3* pDb, const std::vector<RdsObject*>& piers)
	{
		const char* sql = "CREATE TABLE RDS_PIER("
			"ID                INTEGER PRIMARY KEY      NOT NULL, "
			"WIDTH         INTEGER							, "
			//"HEIGHT         REAL							,"
			//"DIRECTION         BLOB							);";
			"ANGLE         REAL							);";

		char* msg;
		int status = sqlite3_exec(pDb, sql, NULL, 0, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "SELECT AddGeometryColumn('RDS_PIER', 'GEOMETRY',0, 'POINTZ')";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "BEGIN";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}


		sqlite3_stmt* stmt;
		sql = "INSERT INTO RDS_PIER (ID,WIDTH,ANGLE,GEOMETRY) VALUES (?,?,?,?)";

		status = sqlite3_prepare_v2(pDb, sql, strlen(sql), &stmt, NULL);
		if (status != SQLITE_OK)
			return;

		for (int i = 0; i < piers.size(); i++)
		{
			RdsPier* pPier = (RdsPier*)piers[i];
			gaiaGeomCollPtr geo = gaiaAllocGeomCollXYZ();
			geo->Srid = 0;
			writePoint(geo, pPier->position);
			unsigned char* blob;
			int size;
			gaiaToSpatiaLiteBlobWkb(geo, &blob, &size);
			gaiaFreeGeomColl(geo);
			sqlite3_reset(stmt);
			sqlite3_clear_bindings(stmt);

			sqlite3_bind_int64(stmt, 1, pPier->id);
			sqlite3_bind_int(stmt, 2, pPier->width);
			sqlite3_bind_double(stmt, 3, pPier->angle);

			//sqlite3_bind_double(stmt, 4, pPier->height);
			//sqlite3_bind_blob(stmt, 5, &pPier->direction, sizeof(Vector3d), nullptr);
			sqlite3_bind_blob(stmt, 4, blob, size, free);

			status = sqlite3_step(stmt);
			if (status != SQLITE_DONE && status != SQLITE_ROW)
			{
				break;
			}
		}
		sqlite3_finalize(stmt);
		sql = "COMMIT";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}
	}

	void SpatialiteWriter::writeGreenbelt(sqlite3* pDb, const std::vector<RdsObject*>& greenbelts)
	{
		const char* sql = "CREATE TABLE RDS_GREENBELT("
			"ID                INTEGER PRIMARY KEY      NOT NULL);";

		char* msg;
		int status = sqlite3_exec(pDb, sql, NULL, 0, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "SELECT AddGeometryColumn('RDS_GREENBELT', 'GEOMETRY',0, 'MULTILINESTRINGZ')";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "BEGIN";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}


		sqlite3_stmt* stmt;
		sql = "INSERT INTO RDS_GREENBELT (ID,GEOMETRY) VALUES (?,?)";
		status = sqlite3_prepare_v2(pDb, sql, strlen(sql), &stmt, NULL);
		if (status != SQLITE_OK)
			return;

		for (int i = 0; i < greenbelts.size(); i++)
		{
			RdsGreenbelt* pGreenbelt = (RdsGreenbelt*)greenbelts[i];
			gaiaGeomCollPtr geo = gaiaAllocGeomCollXYZ();
			geo->Srid = 0;
			writeMultiLineString(geo, pGreenbelt->contour);
			unsigned char* blob;
			int size;
			gaiaToSpatiaLiteBlobWkb(geo, &blob, &size);
			gaiaFreeGeomColl(geo);
			sqlite3_reset(stmt);
			sqlite3_clear_bindings(stmt);
			sqlite3_bind_int64(stmt, 1, pGreenbelt->id);
			sqlite3_bind_blob(stmt, 2, blob, size, free);

			status = sqlite3_step(stmt);
			if (status != SQLITE_DONE && status != SQLITE_ROW)
			{
				break;
			}
		}
		sqlite3_finalize(stmt);
		sql = "COMMIT";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}
	}

	struct ObjectInfo
	{
		int type;
		int64 id;
	};

	void SpatialiteWriter::writeGroup(sqlite3* pDb, const std::vector<RdsObject*>& groups)
	{
		const char* sql = "CREATE TABLE RDS_GROUP("
			"ID                INTEGER PRIMARY KEY      NOT NULL, "
			"ORIGIN_ID         INTEGER                  NOT NULL, "
			"SEMI_LENGTH         INTEGER                  NOT NULL, "
			"OBJECTLIST		   BLOB                       );";

		char* msg;
		int status = sqlite3_exec(pDb, sql, NULL, 0, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "SELECT AddGeometryColumn('RDS_GROUP', 'GEOMETRY',0, 'MULTIPOINTZ')";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "BEGIN";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}


		sqlite3_stmt* stmt;
		sql = "INSERT INTO RDS_GROUP (ID, ORIGIN_ID,SEMI_LENGTH,OBJECTLIST,GEOMETRY) VALUES (?,?,?,?,?)";
		status = sqlite3_prepare_v2(pDb, sql, strlen(sql), &stmt, NULL);
		if (status != SQLITE_OK)
			return;

		auto createInfo = [](const RdsGroup* pGroup)-> std::vector<ObjectInfo> {

			std::vector<ObjectInfo> vs;
			vs.resize(pGroup->objects.size());
			for (int i = 0;i < pGroup->objects.size();i++)
			{
				RdsObject* pObj = pGroup->objects[i];
				ObjectInfo info;
				info.type = (int)pObj->getEntityType();
				info.id = pObj->id;
				vs[i] = info;
			}
			return vs;
		};

		for (int i = 0; i < groups.size(); i++)
		{
			RdsGroup* pGroup = (RdsGroup*)groups[i];

			gaiaGeomCollPtr geo = gaiaAllocGeomCollXYZ();
			geo->Srid = 0;
			writeMultiPoint(geo, pGroup->semiTransparentPoints);
			unsigned char* blob;
			int size;
			gaiaToSpatiaLiteBlobWkb(geo, &blob, &size);
			gaiaFreeGeomColl(geo);

			sqlite3_reset(stmt);
			sqlite3_clear_bindings(stmt);

			sqlite3_bind_int64(stmt, 1, pGroup->id);
			sqlite3_bind_int64(stmt, 2, pGroup->originId);
			sqlite3_bind_int(stmt, 3, pGroup->semiTransparentLength);

			std::vector<ObjectInfo> infos = createInfo(pGroup);
			sqlite3_bind_blob(stmt, 4, infos.data(), sizeof(ObjectInfo) * pGroup->objects.size(), nullptr);
			sqlite3_bind_blob(stmt, 5, blob, size, free);
			status = sqlite3_step(stmt);
			if (status != SQLITE_DONE && status != SQLITE_ROW)
			{
				break;
			}
		}
		sqlite3_finalize(stmt);
		sql = "COMMIT";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}
	}

	void SpatialiteWriter::writeCrossWalk(sqlite3* pDb, const std::vector<RdsObject*>& crosswalks)
	{
		const char* sql = "CREATE TABLE RDS_CROSSWALK("
			"ID                INTEGER PRIMARY KEY      NOT NULL);";

		char* msg;
		int status = sqlite3_exec(pDb, sql, NULL, 0, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "SELECT AddGeometryColumn('RDS_CROSSWALK',  'GEOMETRY',0, 'POLYGONZ')";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "BEGIN";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sqlite3_stmt* stmt;
		sql = "INSERT INTO RDS_CROSSWALK (ID,DIRECTION,GEOMETRY) VALUES (?,?,?)";
		status = sqlite3_prepare_v2(pDb, sql, strlen(sql), &stmt, NULL);
		if (status != SQLITE_OK)
			return;

		for (int i = 0; i < crosswalks.size(); i++)
		{
			RdsCrossWalk* pCrosswalk = (RdsCrossWalk*)crosswalks[i];
			gaiaGeomCollPtr geo = gaiaAllocGeomCollXYZ();
			geo->Srid = 0;
			writePolygon(geo, pCrosswalk->contour);
			unsigned char* blob;
			int size;
			gaiaToSpatiaLiteBlobWkb(geo, &blob, &size);
			gaiaFreeGeomColl(geo);
			sqlite3_reset(stmt);
			sqlite3_clear_bindings(stmt);
			sqlite3_bind_int64(stmt, 1, pCrosswalk->id);
			sqlite3_bind_int64(stmt, 2, pCrosswalk->zebraHeading);
			sqlite3_bind_blob(stmt, 3, blob, size, free);

			status = sqlite3_step(stmt);
			if (status != SQLITE_DONE && status != SQLITE_ROW)
			{
				break;
			}
		}
		sqlite3_finalize(stmt);
		sql = "COMMIT";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}
	}

	void SpatialiteWriter::writeRelationship(sqlite3* pDb, const std::vector<RdsObject*>& relationships)
	{
		const char* sql = "CREATE TABLE RDS_RELATIONSHIP("
			"HOST				INTEGER		NOT NULL, "
			"HOSTTYPE			INTEGER		NOT NULL, "
			"OBJECTLIST			BLOB, "
			"RELATIONSHIPTYPE	INTEGER);";

		char* msg;
		int status = sqlite3_exec(pDb, sql, NULL, 0, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "BEGIN";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sqlite3_stmt* stmt;
		sql = "INSERT INTO RDS_RELATIONSHIP (HOST, HOSTTYPE,OBJECTLIST,RELATIONSHIPTYPE) VALUES (?,?,?,?)";
		status = sqlite3_prepare_v2(pDb, sql, strlen(sql), &stmt, NULL);
		if (status != SQLITE_OK)
			return;

		for (int i = 0; i < relationships.size(); i++)
		{
			RdsRelationship* pRel = (RdsRelationship*)relationships[i];

			sqlite3_reset(stmt);
			sqlite3_clear_bindings(stmt);
			sqlite3_bind_int64(stmt, 1, pRel->first);
			sqlite3_bind_int64(stmt, 2, (int64)EntityType::RDS_GROUP);

			sqlite3_bind_blob(stmt, 3, pRel->second.data(), sizeof(RdsRelationship::ObjectInfo) * pRel->second.size(), nullptr);
			sqlite3_bind_int64(stmt, 4, (int64)pRel->type);
			status = sqlite3_step(stmt);

			if (status != SQLITE_DONE && status != SQLITE_ROW)
			{
				break;
			}
		}

		sqlite3_finalize(stmt);
		sql = "COMMIT";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}
	}

	void SpatialiteWriter::writeTrafficLights(sqlite3* pDb, const std::vector<RdsObject*>& trafficLights)
	{
		const char* sql = "CREATE TABLE RDS_TRAFFICLIGHTS("
			"ID				INTEGER PRIMARY KEY		NOT NULL, "
			"TYPE			INTEGER                 NOT NULL,"
			"BARHEADING		INTEGER                 NOT NULL,"
			"MAXBARLENGTH		INTEGER                 NOT NULL,"
			"OBJECTCOUNT	INTEGER                 NOT NULL);";

		char* msg;
		int status = sqlite3_exec(pDb, sql, NULL, 0, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "SELECT AddGeometryColumn('RDS_TRAFFICLIGHTS', 'GEOMETRY',0, 'POINTZ')";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "BEGIN";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}
		sqlite3_stmt* stmt;
		sql = "INSERT INTO RDS_TRAFFICLIGHTS (ID,TYPE,BARHEADING,MAXBARLENGTH,OBJECTCOUNT,GEOMETRY) VALUES (?,?,?,?,?,?)";
		status = sqlite3_prepare_v2(pDb, sql, strlen(sql), &stmt, NULL);
		if (status != SQLITE_OK)
			return;

		for (int i = 0; i < trafficLights.size(); i++)
		{
			RdsTrafficLight* pLights = (RdsTrafficLight*)trafficLights[i];
			gaiaGeomCollPtr geo = gaiaAllocGeomCollXYZ();
			geo->Srid = 0;
			writePoint(geo, pLights->position);
			unsigned char* blob;
			int size;
			gaiaToSpatiaLiteBlobWkb(geo, &blob, &size);
			gaiaFreeGeomColl(geo);
			sqlite3_reset(stmt);
			sqlite3_clear_bindings(stmt);
			sqlite3_bind_int64(stmt, 1, pLights->id);
			sqlite3_bind_int(stmt, 2, (int)pLights->type);
			sqlite3_bind_int(stmt, 3, (int)pLights->barHeading * MATH_RADIAN2DEGREE / 1e5);
			sqlite3_bind_int(stmt, 4, pLights->maxBarLenght);
			sqlite3_bind_int(stmt, 5, pLights->lightBoxs.size());
			sqlite3_bind_blob(stmt, 6, blob, size, free);

			status = sqlite3_step(stmt);
			if (status != SQLITE_DONE && status != SQLITE_ROW)
			{
				break;
			}
		}
		sqlite3_finalize(stmt);
		sql = "COMMIT";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}
	}

}
