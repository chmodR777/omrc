#pragma once
#include "util/byte_stream_writer.h"
#include "geometry/map_point3d64.h"
namespace OMDB
{
	class DbMesh;
	class MeshWriter
	{
	public:
		void write(DbMesh* pMesh, unsigned char*& pData, size_t& size);
		void serializeLink(DbMesh* pMesh,ByteStreamWriter* streamWriter);
		void serializeLgLink(DbMesh* pMesh, ByteStreamWriter* streamWriter);
		void serializeLgAssociation(DbMesh* pMesh, ByteStreamWriter* streamWriter);
		void serializeNode(DbMesh* pMesh, ByteStreamWriter* streamWriter);
// 		void serializePAValue(DbMesh* pMesh, ByteStreamWriter* streamWriter);
		void serializeLinkPA(DbMesh* pMesh, ByteStreamWriter* streamWriter);
		void serializeRoadRoundNode(DbMesh* pMesh, ByteStreamWriter* streamWriter);
		void serializeRoadRoundPA(DbMesh* pMesh, ByteStreamWriter* streamWriter);
		void serializeRoadBoundLink(DbMesh* pMesh, ByteStreamWriter* streamWriter);
		void serializeLaneLink(DbMesh* pMesh, ByteStreamWriter* streamWriter);
		void serializeLaneNode(DbMesh* pMesh, ByteStreamWriter* streamWriter);
		void serializeLaneMarkLink(DbMesh* pMesh, ByteStreamWriter* streamWriter);
		void serializeLaneMarkNode(DbMesh* pMesh, ByteStreamWriter* streamWriter);
		void serializeLaneMarkPA(DbMesh* pMesh, ByteStreamWriter* streamWriter);
		void serializeBarrier(DbMesh* pMesh, ByteStreamWriter* streamWriter);
		void serializeWall(DbMesh* pMesh, ByteStreamWriter* streamWriter);
		void serializeFillArea(DbMesh* pMesh, ByteStreamWriter* streamWriter);
		void serializeCrossWalk(DbMesh* pMesh, ByteStreamWriter* streamWriter);
		void serializeArrow(DbMesh* pMesh, ByteStreamWriter* streamWriter);
		void serializeTrafficSign(DbMesh* pMesh, ByteStreamWriter* streamWriter);
		void serializeStopLocation(DbMesh* pMesh, ByteStreamWriter* streamWriter);
		void serializeLgRel(DbMesh* pMesh, ByteStreamWriter* streamWriter);
		void serializeLaneLinkRel(DbMesh* pMesh, ByteStreamWriter* streamWriter);
		void serializeText(DbMesh* pMesh, ByteStreamWriter* streamWriter);
		void serializeFixedSpeedLimit(DbMesh* pMesh, ByteStreamWriter* streamWriter);
		void serializeLinkSpeedLimit(DbMesh* pMesh, ByteStreamWriter* streamWriter);
		void serializeIntersection(DbMesh* pMesh, ByteStreamWriter* streamWriter);
// 		void serializeTollLane(DbMesh* pMesh, ByteStreamWriter* streamWriter);
		void serializeTollGate(DbMesh* pMesh, ByteStreamWriter* streamWriter);
		void serializeTrafficLights(DbMesh* pMesh, ByteStreamWriter* streamWriter);
		void serializePole(DbMesh* pMesh, ByteStreamWriter* streamWriter);
		void serializeLaneTurnwaiting(DbMesh* pMesh, ByteStreamWriter* streamWriter);
		void serializePolyline3d(std::vector<MapPoint3D64>& points, ByteStreamWriter* ow);



	};

}

