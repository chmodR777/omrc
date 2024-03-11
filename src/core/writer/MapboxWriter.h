#pragma once
#include "Writer.h"
#include "TileWriter.h"
namespace RDS
{
	class MapboxWriter : public Writer, public TileWriter
	{
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
		void writeGreenbeltUrban(RdsTile* pTile, Tile* pMapboxTile);
		void writeCrossWalk(RdsTile* pTile, Tile* pMapboxTile);
		void writeGroup(RdsTile* pTile, Tile* pMapboxTile);
		void writeTrafficLight(RdsTile* pTile, Tile* pMapboxTile);
        void writeBridgeTransparency(RdsTile* pTile, Tile* pMapboxTile);
        void writeWaitingZone(RdsTile* pTile, Tile* pMapboxTile);
        void writeStopLine(RdsTile* pTile, Tile* pMapboxTile);
        void writeCurb(RdsTile* pTile, Tile* pMapboxTile);
        void writeSpeedBump(RdsTile* pTile, Tile* pMapboxTile);

	};
}


