#pragma once
#include "navi_rds/Entity.h"
#include "navi_rds/vector_tile.pb.h"
#include "Writer.h"
using namespace vector_tile;

namespace RDS
{
	class MapboxWriter : public Writer
	{
		enum class CommandType : unsigned char
		{
			MoveTo = 0x1,
			LineTo = 0x2,
			ClosePath = 0x7

		};

	protected:
		virtual void write(RdsTile* pTile, void*& pData, unsigned long long& size) override;
		virtual void write(const char* path, RdsTile* tile) override;
		void writeRoad(RdsTile* pTile,Tile* pMapboxTile);
		void writeText(RdsTile* pTile, Tile* pMapboxTile);
		void writeToll(RdsTile* pTile, Tile* pMapboxTile);
		void writeTunnel(RdsTile* pTile, Tile* pMapboxTile);
		void writeIntersection(RdsTile* pTile, Tile* pMapboxTile);
		void writeGuardrail(RdsTile* pTile, Tile* pMapboxTile);
		void writeMarking(RdsTile* pTile, Tile* pMapboxTile);
		void writeLine(RdsTile* pTile, Tile* pMapboxTile);
		void writeDiversion(RdsTile* pTile, Tile* pMapboxTile);
		void writeSign3d(RdsTile* pTile, Tile* pMapboxTile);
		void writeSpeedLimitBoard(RdsTile* pTile, Tile* pMapboxTile);
		void writePier(RdsTile* pTile, Tile* pMapboxTile);
		void writeGreenbelt(RdsTile* pTile, Tile* pMapboxTile);
		void writeCrossWalk(RdsTile* pTile, Tile* pMapboxTile);
		void writeGroup(RdsTile* pTile, Tile* pMapboxTile);
		void writeTrafficLight(RdsTile* pTile, Tile* pMapboxTile);
        void writeBridgeTransparency(RdsTile* pTile, Tile* pMapboxTile);

		static uint32 createCommandInteger(CommandType type, uint32 count);
		static uint64 createParameterInteger(int64 coordinate);

		static void offsetPoint(const RDS::Point3d& origin,RDS::Point3d& point);
		static void offsetPoints(const RDS::Point3d& origin,RDS::Point3d* points,unsigned int count);
		static void offsetLineString(const RDS::Point3d& origin, RDS::LineString3d& line);
		static void offsetMultiLineString(const RDS::Point3d& origin, RDS::LineString3d* lines, unsigned int count);
		
		static void createPoint(Tile_Feature* pFeature,const RDS::Point3d& point);
		static void createMultiPoint(Tile_Feature* pFeature,RDS::Point3d* points,uint32 count);
		static void createLineString(Tile_Feature* pFeature,const RDS::LineString3d& line);
		static void createMultiLineStrng(Tile_Feature* pFeature,RDS::LineString3d* lines,uint32 count);
		static void createPolygon(Tile_Feature* pFeature,const RDS::Polygon3d& polygon);
		static void createTriIndex(Tile_Feature* pFeature, const std::vector<uint16>& triangleIndex);
	};
}


