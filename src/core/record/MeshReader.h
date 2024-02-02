#pragma once
namespace OMDB
{
	class DbMesh;
	#include "util/byte_stream_reader.h"
	#include <vector>
#include "geometry/map_point3d64.h"
	class MeshReader
	{
	public:
		void read(void* pData, size_t size, DbMesh* const pMesh);

	protected:
		void parsePolyline3d(std::vector<MapPoint3D64>& points, ByteStreamReader* streamWriter);
		void parseLink(DbMesh* pMesh, ByteStreamReader* streamWriter);
		void parseLgLink(DbMesh* pMesh, ByteStreamReader* streamWriter);
		void parseLgAssociation(DbMesh* pMesh, ByteStreamReader* streamWriter);
		void parseNode(DbMesh* pMesh, ByteStreamReader* streamWriter);
		void parseLinkPA(DbMesh* pMesh, ByteStreamReader* streamWriter);
		void parseRoadRoundNode(DbMesh* pMesh, ByteStreamReader* streamWriter);
		void parseRoadRoundPA(DbMesh* pMesh, ByteStreamReader* streamWriter);
		void parseRoadBoundLink(DbMesh* pMesh, ByteStreamReader* streamWriter);
		void parseLaneLink(DbMesh* pMesh, ByteStreamReader* streamWriter);
		void parseLaneNode(DbMesh* pMesh, ByteStreamReader* streamWriter);
		void parseLaneMarkLink(DbMesh* pMesh, ByteStreamReader* streamWriter);
		void parseLaneMarkNode(DbMesh* pMesh, ByteStreamReader* streamWriter);
		void parseLaneMarkPA(DbMesh* pMesh, ByteStreamReader* streamWriter);
		void parseBarrier(DbMesh* pMesh, ByteStreamReader* streamWriter);
		void parseWall(DbMesh* pMesh, ByteStreamReader* streamWriter);
		void parseFillArea(DbMesh* pMesh, ByteStreamReader* streamWriter);
		void parseCrossWalk(DbMesh* pMesh, ByteStreamReader* streamWriter);
		void parseArrow(DbMesh* pMesh, ByteStreamReader* streamWriter);
		void parseTrafficSign(DbMesh* pMesh, ByteStreamReader* streamWriter);
		void parseStopLocation(DbMesh* pMesh, ByteStreamReader* streamWriter);
		void parseLgRel(DbMesh* pMesh, ByteStreamReader* streamWriter);
		void parseLaneLinkRel(DbMesh* pMesh, ByteStreamReader* streamWriter);
		void parseText(DbMesh* pMesh, ByteStreamReader* streamWriter);
		void parseFixedSpeedLimit(DbMesh* pMesh, ByteStreamReader* streamWriter);
		void parseLinkSpeedLimit(DbMesh* pMesh, ByteStreamReader* streamWriter);
		void parseIntersection(DbMesh* pMesh, ByteStreamReader* streamWriter);
		void parseTollGate(DbMesh* pMesh, ByteStreamReader* streamWriter);
		void parseTrafficLight(DbMesh* pMesh, ByteStreamReader* streamWriter);
		void parsePole(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseLaneTurnwaiting(DbMesh* pMesh, ByteStreamReader* streamReader);
	};

}

