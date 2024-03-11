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
		void parsePolyline3d(std::vector<MapPoint3D64>& points, ByteStreamReader* streamReader);
		void parseWString(std::wstring& w_str, ByteStreamReader* streamReader);
		void parseLink(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseLgLink(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseLgAssociation(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseNode(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseLinkPA(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseRoadRoundNode(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseRoadRoundPA(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseRoadBoundLink(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseLaneLink(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseLaneNode(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseLaneMarkLink(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseLaneMarkNode(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseLaneMarkPA(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseBarrier(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseWall(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseFillArea(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseCrossWalk(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseArrow(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseTrafficSign(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseStopLocation(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseLgRel(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseLaneLinkRel(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseText(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseFixedSpeedLimit(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseLinkSpeedLimit(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseIntersection(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseTollGate(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseTrafficLight(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parsePole(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseLaneTurnwaiting(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseLaneInfo(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseZLevel(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseLinkName(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseRoadName(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseRdLinkLanePa(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseRdLaneLinkCLM(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseRdLaneTopoDetail(DbMesh* pMesh, ByteStreamReader* streamReader);
		void parseSpeedBump(DbMesh* pMesh, ByteStreamReader* streamReader);
	};

}

