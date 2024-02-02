#include "stdafx.h"
#include "MeshWriter.h"
#include "DbRecord.h"
#include "DbMesh.h"
#pragma warning(disable:4267)
#pragma warning(disable:4244)
namespace OMDB
{
	std::string WString2String(const std::wstring& wstr)
	{
		int len = WideCharToMultiByte(CP_ACP, 0, wstr.c_str(), wstr.size(), NULL, 0, NULL, NULL);
		char* buffer = new char[len + 1];
		memset(buffer, '\0', sizeof(char) * (len + 1));
		WideCharToMultiByte(CP_ACP, 0, wstr.c_str(), wstr.size(), buffer, len, NULL, NULL);
		std::string result(buffer);
		delete[] buffer;
		return result;
	}

	void MeshWriter::write(DbMesh* pMesh, unsigned char*& pData, size_t& size)
	{
		ByteStreamWriter sw;
		ByteStreamWriter* streamWriter = &sw;
		serializeLink(pMesh, streamWriter);
		serializeLgLink(pMesh, streamWriter);
		serializeLgAssociation(pMesh, streamWriter);
		serializeNode(pMesh, streamWriter);
		serializeLinkPA(pMesh, streamWriter);
		serializeRoadRoundNode(pMesh, streamWriter);
		serializeRoadRoundPA(pMesh, streamWriter);
		serializeRoadBoundLink(pMesh, streamWriter);
		serializeLaneLink(pMesh, streamWriter);
		serializeLaneNode(pMesh, streamWriter);
		serializeLaneMarkLink(pMesh, streamWriter);
		serializeLaneMarkNode(pMesh, streamWriter);
		serializeLaneMarkPA(pMesh, streamWriter);

		serializeBarrier(pMesh, streamWriter);
		serializeWall(pMesh, streamWriter);
		serializeFillArea(pMesh, streamWriter);
		serializeCrossWalk(pMesh, streamWriter);
		serializeArrow(pMesh, streamWriter);
		serializeTrafficSign(pMesh, streamWriter);
		serializeStopLocation(pMesh, streamWriter);
		serializeLgRel(pMesh, streamWriter);
		serializeLaneLinkRel(pMesh, streamWriter);
		serializeText(pMesh, streamWriter);
		serializeFixedSpeedLimit(pMesh, streamWriter);
		serializeLinkSpeedLimit(pMesh, streamWriter);
		serializeIntersection(pMesh, streamWriter);
		serializeTollGate(pMesh, streamWriter);
		serializeTrafficLights(pMesh, streamWriter);
		serializePole(pMesh, streamWriter);
		serializeLaneTurnwaiting(pMesh, streamWriter);
		
		pData = (unsigned char*)malloc(sw.lengthInBytes());
		memcpy(pData,sw.bytes(),sw.lengthInBytes());
		size = sw.lengthInBytes();
	}
	void MeshWriter::serializePolyline3d(std::vector<MapPoint3D64>& points,ByteStreamWriter* ow)
	{
// 		ow->writeVarUint32((uint32)points.size());
// 		if (points.empty())
// 			return;
// 
// 		ow->writeInt64(points[0].pos.lon);
// 		ow->writeInt64(points[0].pos.lat);
// 		ow->writeVarInt32(points[0].z);
// 		if (points.size() == 1)
// 			return;
// 
// 		for (size_t i = 0; i + 1 < points.size(); ++i)
// 		{
// 			ow->writeVarInt64((int32)(points[i + 1].pos.lon - points[i].pos.lon));
// 			ow->writeVarInt64((int32)(points[i + 1].pos.lat - points[i].pos.lat));
// 			ow->writeVarInt32((int32)(points[i + 1].z - points[i].z));
// 		}
		size_t xBitWidth = 0;
		size_t yBitWidth = 0;
		size_t zBitWidth = 0;

		for (size_t idx = 0; idx + 1 < points.size(); ++idx)
		{
			size_t bitWidth;

			bitWidth = Util_getBitWidthOfInt((int32)(points[idx + 1].pos.lon - points[idx].pos.lon));
			xBitWidth = cq_max(xBitWidth, bitWidth);
			bitWidth = Util_getBitWidthOfInt((int32)(points[idx + 1].pos.lat - points[idx].pos.lat));
			yBitWidth = cq_max(yBitWidth, bitWidth);
			bitWidth = Util_getBitWidthOfInt((int32)(points[idx + 1].z - points[idx].z));
			zBitWidth = cq_max(zBitWidth, bitWidth);
		}

		ow->writeVarUint32((uint32)points.size());
		if (points.empty())
			return;

		ow->writeUintN64(points[0].pos.lon, 36);
		ow->writeUintN64(points[0].pos.lat, 36);
		ow->writeVarInt32(points[0].z);
		if (points.size() == 1)
			return;

		ow->writeUintN((uint32)(xBitWidth - 1), 6);
		ow->writeUintN((uint32)(yBitWidth - 1), 6);
		ow->writeUintN((uint32)(zBitWidth - 1), 5);
		for (size_t idx = 0; idx + 1 < points.size(); ++idx)
		{
			ow->writeIntN((int32)(points[idx + 1].pos.lon - points[idx].pos.lon), xBitWidth);
			ow->writeIntN((int32)(points[idx + 1].pos.lat - points[idx].pos.lat), yBitWidth);
			ow->writeIntN((int32)(points[idx + 1].z - points[idx].z), zBitWidth);
		}

	}
	void MeshWriter::serializeLink(DbMesh* pMesh, ByteStreamWriter* streamWriter)
	{
		ByteStreamWriter writer;
		writer.writeUint8((uint8)RecordType::DB_HAD_LINK);
		auto rds = pMesh->query(RecordType::DB_HAD_LINK);
		writer.writeVarUint32(rds.size());
		for (auto& rd : rds )
		{
			DbLink* pLink = (DbLink*)rd;
			writer.writeInt64(pLink->uuid);
			writer.writeInt64(pLink->startNode);
			writer.writeInt64(pLink->endNode);
			writer.writeVarInt32(pLink->kind);
			writer.writeVarInt32(pLink->multi_digitized);
			writer.writeVarInt32(pLink->direct);
			writer.writeVarInt32(pLink->separation_left);
			writer.writeVarInt32(pLink->separation_right);
			writer.writeVarInt32(pLink->median_left);
			writer.writeVarInt32(pLink->median_right);
			writer.writeVarInt32(pLink->overhead_obstruction);
			writer.writeVarUint32(pLink->wayTypes.size());
			for (auto wayType : pLink->wayTypes) {
				writer.writeVarInt32(wayType);
			}
			writer.writeVarUint32(pLink->dataLevels.size());
			for (auto dataLevel : pLink->dataLevels) {
				writer.writeInt64(dataLevel.uuid);
				writer.writeVarInt32(dataLevel.dataLevel);
				writer.writeVarInt32(dataLevel.featureType);
				writer.writeInt64(dataLevel.startOffset * 1e6);
				writer.writeInt64(dataLevel.endOffset * 1e6);

				serializePolyline3d(dataLevel.geometry.postions, &writer);
			}
			writer.writeVarUint32(pLink->separations.size());
			for (auto separation : pLink->separations) {
				writer.writeInt64(separation.uuid);
				writer.writeVarInt32(separation.side);
				writer.writeVarInt32(separation.separation);
				writer.writeVarInt32(separation.featureType);
				writer.writeInt64(separation.startOffset * 1e6);
				writer.writeInt64(separation.endOffset * 1e6);

				serializePolyline3d(separation.geometry.postions, &writer);
			}
			writer.writeVarUint32(pLink->medians.size());
			for (auto median : pLink->medians) {
				writer.writeInt64(median.uuid);
				writer.writeVarInt32(median.side);
				writer.writeVarInt32(median.median);
				writer.writeVarInt32(median.featureType);
				writer.writeInt64(median.startOffset * 1e6);
				writer.writeInt64(median.endOffset * 1e6);

				serializePolyline3d(median.geometry.postions, &writer);
			}
			writer.writeVarUint32(pLink->overheadObstructions.size());
			for (auto overheadObstruction : pLink->overheadObstructions) {
				writer.writeInt64(overheadObstruction.uuid);
				writer.writeVarInt32(overheadObstruction.featureType);
				writer.writeInt64(overheadObstruction.startOffset * 1e6);
				writer.writeInt64(overheadObstruction.endOffset * 1e6);

				serializePolyline3d(overheadObstruction.geometry.postions, &writer);
			}
			writer.writeVarUint32(pLink->tollAreas.size());
			for (auto tollArea : pLink->tollAreas) {
				writer.writeInt64(tollArea.uuid);
				writer.writeVarInt32(tollArea.featureType);
				writer.writeInt64(tollArea.startOffset * 1e6);
				writer.writeInt64(tollArea.endOffset * 1e6);

				serializePolyline3d(tollArea.geometry.postions, &writer);
			}
			serializePolyline3d(pLink->geometry.vertexes, &writer);
		}
		writer.alignToBits(8);
		streamWriter->writeVarUint64(writer.lengthInBytes());
		streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
	}

	void MeshWriter::serializeLgLink(DbMesh* pMesh, ByteStreamWriter* streamWriter)
	{
		ByteStreamWriter writer;
		writer.writeUint8((uint8)RecordType::DB_HAD_LG_LINK);
		auto rds = pMesh->query(RecordType::DB_HAD_LG_LINK);
		writer.writeVarUint32(rds.size());
		for (auto& rd : rds)
		{
			DbLgLink* pLgLink = (DbLgLink*)rd;
			writer.writeInt64(pLgLink->uuid);
			writer.writeVarUint32(pLgLink->relLinks.size());
			for (auto& pair : pLgLink->relLinks)
			{
				DbLgLink::DbRelLink rel = pair.second;
				writer.writeInt64(pair.first);
				writer.writeInt64(rel.relLinkid);
				writer.writeInt64(rel.startOffset * 1e6);
				writer.writeInt64(rel.endOffset * 1e6);
				writer.writeVarInt32(rel.type);
				writer.writeVarInt32(rel.directType);
			}
		}
		writer.alignToBits(8);
		streamWriter->writeVarUint64(writer.lengthInBytes());
		streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
	}

	void MeshWriter::serializeLgAssociation(DbMesh* pMesh, ByteStreamWriter* streamWriter)
	{
		ByteStreamWriter writer;
		writer.writeUint8((uint8)RecordType::DB_HAD_LG_ASSOCIATION);
		auto rds = pMesh->query(RecordType::DB_HAD_LG_ASSOCIATION);
		writer.writeVarUint32(rds.size());
		for (auto& rd : rds)
		{
			DbLgAssociation* pLgLink = (DbLgAssociation*)rd;
			writer.writeInt64(pLgLink->uuid);
			writer.writeVarUint32(pLgLink->relLgAssociations.size());
			for (auto& pair : pLgLink->relLgAssociations)
			{
				DbLgAssociation::DbRelLgAssociation rel = pair;
				writer.writeVarInt32(rel.directType);
				writer.writeInt64(rel.firstLgLinkId);
				writer.writeInt64(rel.secondLgLinkId);
			}
		}
		writer.alignToBits(8);
		streamWriter->writeVarUint64(writer.lengthInBytes());
		streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
	}

	void MeshWriter::serializeNode(DbMesh* pMesh, ByteStreamWriter* streamWriter)
	{
		ByteStreamWriter writer;
		writer.writeUint8((uint8)RecordType::DB_HAD_NODE);
		auto rds = pMesh->query(RecordType::DB_HAD_NODE);
		writer.writeVarUint32(rds.size());
		for (auto& rd : rds)
		{
			DbNode* pNode = (DbNode*)rd;
			writer.writeInt64(pNode->uuid);
			writer.writeInt64(pNode->intersectionId);
			writer.writeVarInt32(pNode->wayType);

			writer.writeVarUint32(pNode->meshIds.size());
			for (auto id : pNode->meshIds)
			{
				writer.writeVarInt32(id);
			}
			writer.writeInt64(pNode->geometry.pos.lon);
			writer.writeInt64(pNode->geometry.pos.lat);
			writer.writeInt32(pNode->geometry.z);
		}
		writer.alignToBits(8);
		streamWriter->writeVarUint64(writer.lengthInBytes());
		streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
	}
	void MeshWriter::serializeLinkPA(DbMesh* pMesh, ByteStreamWriter* streamWriter)
	{
		ByteStreamWriter writer;
		writer.writeUint8((uint8)RecordType::DB_HAD_LINK_PA);
		auto rds = pMesh->query(RecordType::DB_HAD_LINK_PA);
		writer.writeVarUint32(rds.size());
		for (auto& rd : rds)
		{
			DbLinkPA* pLinkPa = (DbLinkPA*)rd;

			writer.writeInt64(pLinkPa->uuid);
			writer.writeInt64(pLinkPa->relLinkId);
			writer.writeVarInt32(pLinkPa->featureType);
			writer.writeInt64(pLinkPa->startOffset * 1e6);
			writer.writeInt64(pLinkPa->endOffset * 1e6);

			serializePolyline3d(pLinkPa->geometry.postions, &writer);

			writer.writeVarUint32(pLinkPa->paValues.size());
			for (auto p : pLinkPa->paValues)
			{
				writer.writeVarInt32(p->seqNum);
				writer.writeVarInt32(p->attributeType);
				writer.writeVarInt32(p->attributeValue);
			}
		}
		writer.alignToBits(8);
		streamWriter->writeVarUint64(writer.lengthInBytes());
		streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
	}

	void MeshWriter::serializeRoadRoundNode(DbMesh* pMesh, ByteStreamWriter* streamWriter)
	{
		ByteStreamWriter writer;
		writer.writeUint8((uint8)RecordType::DB_HAD_ROAD_BOUNDARY_NODE);
		auto rds = pMesh->query(RecordType::DB_HAD_ROAD_BOUNDARY_NODE);
		writer.writeVarUint32(rds.size());
		for (auto& rd : rds)
		{
			DbRoadBoundNode* pNode = (DbRoadBoundNode*)rd;
			writer.writeInt64(pNode->uuid);
			writer.writeInt64(pNode->geometry.pos.lon);
			writer.writeInt64(pNode->geometry.pos.lat);
			writer.writeInt32(pNode->geometry.z);
		}
		writer.alignToBits(8);
		streamWriter->writeVarUint64(writer.lengthInBytes());
		streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
	}

	void MeshWriter::serializeRoadRoundPA(DbMesh* pMesh, ByteStreamWriter* streamWriter)
	{
		ByteStreamWriter writer;
		writer.writeUint8((uint8)RecordType::DB_HAD_ROAD_BOUNDARY_PA);
		auto rds = pMesh->query(RecordType::DB_HAD_ROAD_BOUNDARY_PA);
		writer.writeVarUint32(rds.size());
		for (auto& rd : rds)
		{
			DbRoadBoundPA* pRoadBoundPA = (DbRoadBoundPA*)rd;
			writer.writeInt64(pRoadBoundPA->uuid);
			writer.writeInt64(pRoadBoundPA->relRoadBoundLinkId);
			writer.writeVarInt32(pRoadBoundPA->featureType);
			writer.writeInt64(pRoadBoundPA->startOffset * 1e6);
			writer.writeInt64(pRoadBoundPA->endOffset * 1e6);
			writer.writeVarUint32(pRoadBoundPA->paValues.size());
			for (auto p : pRoadBoundPA->paValues)
			{
				writer.writeVarInt32(p->seqNum);
				writer.writeVarInt32(p->attributeType);
				writer.writeVarInt32(p->attributeValue);
			}
			serializePolyline3d(pRoadBoundPA->geometry.postions, &writer);
		}

		writer.alignToBits(8);
		streamWriter->writeVarUint64(writer.lengthInBytes());
		streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
	}

	void MeshWriter::serializeRoadBoundLink(DbMesh* pMesh, ByteStreamWriter* streamWriter)
	{
		ByteStreamWriter writer;
		writer.writeUint8((uint8)RecordType::DB_HAD_ROAD_BOUNDARY_LINK);
		auto rds = pMesh->query(RecordType::DB_HAD_ROAD_BOUNDARY_LINK);
		writer.writeVarUint32(rds.size());
		for (auto& rd : rds)
		{
			DbRoadBoundLink* pLink = (DbRoadBoundLink*)rd;
			writer.writeInt64(pLink->uuid);
			writer.writeInt64(pLink->starRoadBoundNodeId);
			writer.writeInt64(pLink->endRoadBoundNodeId);
			writer.writeVarInt32(pLink->boundaryType);
			writer.writeVarUint32(pLink->relLgs.size());

			for (auto& pair : pLink->relLgs)
			{
				writer.writeInt64(pair.first);
				writer.writeVarInt32(pair.second.side);
				writer.writeVarInt32(pair.second.direction);
			}
			serializePolyline3d(pLink->geometry.vertexes, &writer);
		}
		writer.alignToBits(8);
		streamWriter->writeVarUint64(writer.lengthInBytes());
		streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
	}

	void MeshWriter::serializeLaneLink(DbMesh* pMesh, ByteStreamWriter* streamWriter)
	{
		ByteStreamWriter writer;
		writer.writeUint8((uint8)RecordType::DB_HAD_LANE_LINK);
		auto rds = pMesh->query(RecordType::DB_HAD_LANE_LINK);
		writer.writeUint32(rds.size());
		for (auto& rd : rds)
		{
			DbLaneLink* pLink = (DbLaneLink*)rd;
			writer.writeInt64(pLink->uuid);
			writer.writeUint32(pLink->laneType);
			writer.writeVarInt32(pLink->arrowDir);
			writer.writeVarInt32(pLink->width);
			writer.writeInt64(pLink->relLgId);
			writer.writeUint32(pLink->laneNum);
			writer.writeInt64(pLink->startLaneNodeId);
			writer.writeInt64(pLink->endLaneNodeId);
			writer.writeUint32(pLink->seqNumber);
			writer.writeVarInt32(pLink->laneDir);
			writer.writeVarInt32(pLink->conditionType);
			serializePolyline3d(pLink->geometry.vertexes, &writer);
		}
		writer.alignToBits(8);
		streamWriter->writeVarUint64(writer.lengthInBytes());
		streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
	}

	void MeshWriter::serializeLaneNode(DbMesh* pMesh, ByteStreamWriter* streamWriter)
	{
		ByteStreamWriter writer;
		writer.writeUint8((uint8)RecordType::DB_HAD_LANE_NODE);
		auto rds = pMesh->query(RecordType::DB_HAD_LANE_NODE);
		writer.writeVarUint32(rds.size());
		for (auto& rd : rds)
		{
			DbLaneNode* pNode = (DbLaneNode*)rd;
			writer.writeInt64(pNode->uuid);
			writer.writeInt64(pNode->geometry.pos.lon);
			writer.writeInt64(pNode->geometry.pos.lat);
			writer.writeInt32(pNode->geometry.z);
		}
		writer.alignToBits(8);
		streamWriter->writeVarUint64(writer.lengthInBytes());
		streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
	}

	void MeshWriter::serializeLaneMarkLink(DbMesh* pMesh, ByteStreamWriter* streamWriter)
	{
		ByteStreamWriter writer;
		writer.writeUint8((uint8)RecordType::DB_HAD_LANE_MARK_LINK);
		auto rds = pMesh->query(RecordType::DB_HAD_LANE_MARK_LINK);
		writer.writeVarUint32(rds.size());
		for (auto& rd : rds)
		{
			DbLaneMarkLink* pLink = (DbLaneMarkLink*)rd;
			writer.writeInt64(pLink->uuid);
			writer.writeInt64(pLink->startLaneMarkNodeId);
			writer.writeInt64(pLink->endLaneMarkNodeId);
			writer.writeVarInt32(pLink->boundaryType);
			writer.writeVarInt32(pLink->markingCount);
			writer.writeVarInt32(pLink->traversal);

			//writer.writeInt64(pLink->relLaneLinkId);
			//writer.writeVarInt32(pLink->side);
			//writer.writeVarInt32(pLink->direct);
			writer.writeVarUint32(pLink->laneMarkRels.size());
			for (size_t i = 0; i < pLink->laneMarkRels.size(); i++)
			{
				writer.writeInt64(pLink->laneMarkRels[i].relLaneLinkId);
				writer.writeVarInt32(pLink->laneMarkRels[i].side);
				writer.writeVarInt32(pLink->laneMarkRels[i].direct);
			}

			writer.writeVarUint32(pLink->markings.size());
			for (size_t i = 0; i < pLink->markings.size(); i++)
			{
				DbLaneMarkLink::DbLaneMarkLinkMarking& marking = pLink->markings[i];
				writer.writeInt32(marking.markSeqNum);
				writer.writeInt32(marking.markType);
				writer.writeInt32(marking.markColor);
				writer.writeInt32(marking.markWidth);
				writer.writeInt32(marking.markMaterial);
				writer.writeInt32(marking.lateralOffset);
			}

			writer.writeVarUint32(pLink->relLgs.size());
			for (auto& pair : pLink->relLgs)
			{
				DbLaneMarkLink::DbLgMarkRel rel = pair.second;
				writer.writeInt64(pair.first);
				writer.writeVarInt32(rel.lgMarkDirect);
				writer.writeVarInt32(rel.lgMarkSeqNum);
			}
			serializePolyline3d(pLink->geometry.vertexes, &writer);
		}
		writer.alignToBits(8);
		streamWriter->writeVarUint64(writer.lengthInBytes());
		streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
	}

	void MeshWriter::serializeLaneMarkNode(DbMesh* pMesh, ByteStreamWriter* streamWriter)
	{
		ByteStreamWriter writer;
		writer.writeUint8((uint8)RecordType::DB_HAD_LANE_MARK_NODE);
		auto rds = pMesh->query(RecordType::DB_HAD_LANE_MARK_NODE);
		writer.writeVarUint32(rds.size());
		for (auto& rd : rds)
		{
			DbLaneMarkNode* pNode = (DbLaneMarkNode*)rd;
			writer.writeInt64(pNode->uuid);
			writer.writeInt64(pNode->geometry.pos.lon);
			writer.writeInt64(pNode->geometry.pos.lat);
			writer.writeInt32(pNode->geometry.z);
		}

		writer.alignToBits(8);
		streamWriter->writeVarUint64(writer.lengthInBytes());
		streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
	}

	void MeshWriter::serializeLaneMarkPA(DbMesh* pMesh, ByteStreamWriter* streamWriter)
	{
		ByteStreamWriter writer;
		writer.writeUint8((uint8)RecordType::DB_HAD_LANE_MARK_PA);
		auto rds = pMesh->query(RecordType::DB_HAD_LANE_MARK_PA);
		writer.writeVarUint32(rds.size());
		for (auto& rd : rds)
		{
			DbLaneMarkPA* pLaneMarkPA = (DbLaneMarkPA*)rd;

			writer.writeInt64(pLaneMarkPA->uuid);
			writer.writeInt64(pLaneMarkPA->relLaneMarkLinkId);
			writer.writeVarInt32(pLaneMarkPA->featureType);
			writer.writeInt64(pLaneMarkPA->startOffset * 1e6);
			writer.writeInt64(pLaneMarkPA->endOffset * 1e6);
			writer.writeVarUint32(pLaneMarkPA->paValues.size());
			for (auto p : pLaneMarkPA->paValues)
			{
				writer.writeVarInt32(p->seqNum);
				writer.writeVarInt32(p->attributeType);
				writer.writeVarInt32(p->attributeValue);
			}
			serializePolyline3d(pLaneMarkPA->geometry.postions, &writer);

		}
		writer.alignToBits(8);
		streamWriter->writeVarUint64(writer.lengthInBytes());
		streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
	}

	void MeshWriter::serializeBarrier(DbMesh* pMesh, ByteStreamWriter* streamWriter)
	{
		ByteStreamWriter writer;
		writer.writeUint8((uint8)RecordType::DB_HAD_OBJECT_BARRIER);
		auto rds = pMesh->query(RecordType::DB_HAD_OBJECT_BARRIER);
		writer.writeUint32(rds.size());
		for (auto& rd : rds)
		{
			DbBarrier* pBarrier = (DbBarrier*)rd;
			writer.writeInt64(pBarrier->uuid);
			writer.writeVarInt32(pBarrier->barrierType);
			writer.writeVarUint32(pBarrier->geometry.lines.size());
			for (auto& line : pBarrier->geometry.lines)
			{
				serializePolyline3d(line.vertexes, &writer);
			}
		}
		writer.alignToBits(8);
		streamWriter->writeVarUint64(writer.lengthInBytes());
		streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
	}

	void MeshWriter::serializeWall(DbMesh* pMesh, ByteStreamWriter* streamWriter)
	{
		ByteStreamWriter writer;
		writer.writeUint8((uint8)RecordType::DB_HAD_OBJECT_WALL);
		auto rds = pMesh->query(RecordType::DB_HAD_OBJECT_WALL);
		writer.writeVarUint32(rds.size());
		for (auto& rd : rds)
		{
			DbWall* pWall = (DbWall*)rd;
			writer.writeInt64(pWall->uuid);
			writer.writeVarInt32(pWall->wallType);
			writer.writeVarUint32(pWall->geometry.lines.size());
			for (auto& line : pWall->geometry.lines)
			{
				serializePolyline3d(line.vertexes, &writer);
			}
		}
		writer.alignToBits(8);
		streamWriter->writeVarUint64(writer.lengthInBytes());
		streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
	}

	void MeshWriter::serializeFillArea(DbMesh* pMesh, ByteStreamWriter* streamWriter)
	{
		ByteStreamWriter writer;
		writer.writeUint8((uint8)RecordType::DB_HAD_OBJECT_FILL_AREA);
		auto rds = pMesh->query(RecordType::DB_HAD_OBJECT_FILL_AREA);
		writer.writeVarUint32(rds.size());
		for (auto& rd : rds)
		{
			DbFillArea* pFilledArea = (DbFillArea*)rd;
			writer.writeInt64(pFilledArea->uuid);
			serializePolyline3d(pFilledArea->geometry.vertexes, &writer);
		}
		writer.alignToBits(8);
		streamWriter->writeVarUint64(writer.lengthInBytes());
		streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
	}

	void MeshWriter::serializeCrossWalk(DbMesh* pMesh, ByteStreamWriter* streamWriter)
	{
		ByteStreamWriter writer;
		writer.writeUint8((uint8)RecordType::DB_HAD_OBJECT_CROSS_WALK);
		auto rds = pMesh->query(RecordType::DB_HAD_OBJECT_CROSS_WALK);
		writer.writeVarUint32(rds.size());
		for (auto& rd : rds)
		{
			DbCrossWalk* pCrossWalk = (DbCrossWalk*)rd;
			writer.writeInt64(pCrossWalk->uuid);
			writer.writeVarInt32(pCrossWalk->color);

			serializePolyline3d(pCrossWalk->geometry.vertexes, &writer);
		}
		writer.alignToBits(8);
		streamWriter->writeVarUint64(writer.lengthInBytes());
		streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
	}

	void MeshWriter::serializeArrow(DbMesh* pMesh, ByteStreamWriter* streamWriter)
	{
		ByteStreamWriter writer;
		writer.writeUint8((uint8)RecordType::DB_HAD_OBJECT_ARROW);
		auto rds = pMesh->query(RecordType::DB_HAD_OBJECT_ARROW);
		writer.writeVarUint32(rds.size());
		for (auto& rd : rds)
		{
			DbArrow* pArrow = (DbArrow*)rd;
			writer.writeInt64(pArrow->uuid);
			writer.writeVarInt32(pArrow->color);
			writer.writeVarInt32(pArrow->width);
			writer.writeVarInt32(pArrow->length);
			writer.writeVarInt32(pArrow->arrowClass);
			writer.writeInt64(pArrow->center.pos.lon);
			writer.writeInt64(pArrow->center.pos.lat);
			writer.writeInt32(pArrow->center.z);
			serializePolyline3d(pArrow->geometry.vertexes, &writer);
		}
		writer.alignToBits(8);
		streamWriter->writeVarUint64(writer.lengthInBytes());
		streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
	}

	void MeshWriter::serializeTrafficSign(DbMesh* pMesh, ByteStreamWriter* streamWriter)
	{
		ByteStreamWriter writer;
		writer.writeUint8((uint8)RecordType::DB_HAD_OBJECT_TRAFFIC_SIGN);
		auto rds = pMesh->query(RecordType::DB_HAD_OBJECT_TRAFFIC_SIGN);
		writer.writeVarUint32(rds.size());
		for (auto& rd : rds)
		{
			DbTrafficSign* pTrafficSign = (DbTrafficSign*)rd;
			writer.writeInt64(pTrafficSign->uuid);
			writer.writeVarInt32(pTrafficSign->trafSignShape);
			writer.writeVarInt32(pTrafficSign->signType);
			writer.writeVarInt32(pTrafficSign->color);
			writer.writeInt64(pTrafficSign->heading * 1e6);
			serializePolyline3d(pTrafficSign->geometry.vertexes, &writer);
		}
		writer.alignToBits(8);
		streamWriter->writeVarUint64(writer.lengthInBytes());
		streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
	}

	void MeshWriter::serializeStopLocation(DbMesh* pMesh, ByteStreamWriter* streamWriter)
	{
		ByteStreamWriter writer;
		writer.writeUint8((uint8)RecordType::DB_HAD_OBJECT_STOPLOCATION);
		auto rds = pMesh->query(RecordType::DB_HAD_OBJECT_STOPLOCATION);
		writer.writeVarUint32(rds.size());
		for (auto& rd : rds)
		{
			DbStopLocation* pStopLocation = (DbStopLocation*)rd;
			writer.writeInt64(pStopLocation->uuid);
			writer.writeVarInt32(pStopLocation->color);
			writer.writeVarInt32(pStopLocation->width);
			writer.writeVarInt32(pStopLocation->locationType);
			serializePolyline3d(pStopLocation->geometry.vertexes, &writer);
		}
		writer.alignToBits(8);
		streamWriter->writeVarUint64(writer.lengthInBytes());
		streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
	}

	void MeshWriter::serializeLgRel(DbMesh* pMesh, ByteStreamWriter* streamWriter)
	{
		ByteStreamWriter writer;
		writer.writeUint8((uint8)RecordType::DB_HAD_OBJECT_LG_REL);
		auto rds = pMesh->query(RecordType::DB_HAD_OBJECT_LG_REL);
		writer.writeUint32(rds.size());
		for (auto& rd : rds)
		{
			DbLgRel* pRel = (DbLgRel*)rd;
			writer.writeInt64(pRel->uuid);
			writer.writeVarUint32(pRel->relLgs.size());
			for (auto& pair : pRel->relLgs)
			{
				writer.writeInt64(pair.first);
				writer.writeInt64(pair.second.relLgId);
				writer.writeVarInt32(pair.second.objectType);
			}
		}
		writer.alignToBits(8);
		streamWriter->writeVarUint64(writer.lengthInBytes());
		streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
	}
	void MeshWriter::serializeLaneLinkRel(DbMesh* pMesh, ByteStreamWriter* streamWriter)
	{
		ByteStreamWriter writer;
		writer.writeUint8((uint8)RecordType::DB_HAD_OBJECT_LANE_LINK_REL);
		auto rds = pMesh->query(RecordType::DB_HAD_OBJECT_LANE_LINK_REL);
		writer.writeVarUint32(rds.size());
		for (auto& rd : rds)
		{
			DbLaneLinkRel* pRel = (DbLaneLinkRel*)rd;
			writer.writeInt64(pRel->uuid);
			writer.writeVarUint32(pRel->relLaneLinkIds.size());
			for (auto& pair : pRel->relLaneLinkIds)
			{
				writer.writeInt64(pair.first);
				writer.writeInt64(pair.second.relLaneLinkId);
				writer.writeVarInt32(pair.second.objectType);
			}
		}
		writer.alignToBits(8);
		streamWriter->writeVarUint64(writer.lengthInBytes());
		streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
	}

	void MeshWriter::serializeText(DbMesh* pMesh, ByteStreamWriter* streamWriter)
	{
		ByteStreamWriter writer;
		writer.writeUint8((uint8)RecordType::DB_HAD_OBJECT_TEXT);
		auto rds = pMesh->query(RecordType::DB_HAD_OBJECT_TEXT);
		writer.writeVarUint32(rds.size());
		for (auto& rd : rds)
		{
			DbText* pText = (DbText*)rd;
			writer.writeInt64(pText->uuid);
			writer.writeVarInt32(pText->color);
			writer.writeVarInt32(pText->width);
			writer.writeVarInt32(pText->length);

			//writer.writeVarUint32(pText->textContent.size());
			std::string textContent = WString2String(pText->textContent);
			writer.writeVarUint32(textContent.size());
			if (!pText->textContent.empty())
			{
				// 需要多输出一个结尾符号'\0'
				//writer.writeBytes(pText->textContent.c_str(), pText->textContent.size()+1);
				writer.writeBytes(textContent.c_str(), textContent.size() + 1);
			}

			writer.writeInt64(pText->center.pos.lon);
			writer.writeInt64(pText->center.pos.lat);
			writer.writeInt32(pText->center.z);
			serializePolyline3d(pText->geometry.vertexes, &writer);
		}
		writer.alignToBits(8);
		streamWriter->writeVarUint64(writer.lengthInBytes());
		streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
	}

	void MeshWriter::serializeFixedSpeedLimit(DbMesh* pMesh, ByteStreamWriter* streamWriter)
	{
		ByteStreamWriter writer;
		writer.writeUint8((uint8)RecordType::DB_HAD_LANE_FIXED_SPEEDLIMIT);
		auto rds = pMesh->query(RecordType::DB_HAD_LANE_FIXED_SPEEDLIMIT);
		writer.writeVarUint32(rds.size());
		for (auto& rd : rds)
		{
			DbFixedSpeedLimit* pLimit = (DbFixedSpeedLimit*)rd;
			writer.writeInt64(pLimit->uuid);
			writer.writeVarInt32(pLimit->maxSpeedLimit);
			writer.writeVarInt32((int)pLimit->maxSpeedLimitSource);
			writer.writeVarInt32(pLimit->minSpeedLimit);
			writer.writeVarInt32((int)pLimit->minSpeedLimitSource);
		}
		writer.alignToBits(8);
		streamWriter->writeVarUint64(writer.lengthInBytes());
		streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
	}

	void MeshWriter::serializeLinkSpeedLimit(DbMesh* pMesh, ByteStreamWriter* streamWriter)
	{
		ByteStreamWriter writer;
		writer.writeUint8((uint8)RecordType::DB_HAD_LINK_FIXED_SPEEDLIMIT);
		auto rds = pMesh->query(RecordType::DB_HAD_LINK_FIXED_SPEEDLIMIT);
		writer.writeVarUint32(rds.size());
		for (auto& rd : rds)
		{
			DbLinkSpeedLimit* pLimit = (DbLinkSpeedLimit*)rd;
			writer.writeInt64(pLimit->uuid);
			writer.writeInt64(pLimit->relLinkId);
			writer.writeVarInt32(pLimit->featureType);
			writer.writeVarInt32(pLimit->direction);
			writer.writeBool(pLimit->isLaneDependent);
			writer.writeVarInt32((int)pLimit->maxSpeedLimitClass);
			writer.writeInt64(pLimit->startOffset * 1e6);
			writer.writeInt64(pLimit->endOffset * 1e6);
			serializePolyline3d(pLimit->geometry.postions, &writer);
		}
		writer.alignToBits(8);
		streamWriter->writeVarUint64(writer.lengthInBytes());
		streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
	}

	void MeshWriter::serializeIntersection(DbMesh* pMesh, ByteStreamWriter* streamWriter)
	{
		ByteStreamWriter writer;
		writer.writeUint8((uint8)RecordType::DB_HAD_INTERSECTION);
		auto rds = pMesh->query(RecordType::DB_HAD_INTERSECTION);
		writer.writeVarUint32(rds.size());
		for (auto& rd : rds)
		{
			DbIntersection* pIntersection = (DbIntersection*)rd;
			writer.writeInt64(pIntersection->uuid);
			writer.writeVarInt32(pIntersection->intersectionType);
			writer.writeVarUint32(pIntersection->refLaneGroups.size());
			for (auto id : pIntersection->refLaneGroups)
			{
				writer.writeInt64(id);
			}
			writer.writeVarUint32(pIntersection->refLanePAs.size());
			for (auto id : pIntersection->refLanePAs)
			{
				writer.writeInt64(id);
			}
			writer.writeVarUint32(pIntersection->refMeshs.size());
			for (auto id : pIntersection->refMeshs)
			{
				writer.writeInt64(id);
			}
			writer.writeVarUint32(pIntersection->refNodes.size());
			for (auto id : pIntersection->refNodes)
			{
				writer.writeInt64(id);
			}
			writer.writeVarUint32(pIntersection->refPoints.size());
			for (auto id : pIntersection->refPoints)
			{
				writer.writeInt64(id);
			}
			writer.writeVarUint32(pIntersection->outLinks.size());
			for (auto link : pIntersection->outLinks)
			{
				writer.writeInt64(link.linkId);
				writer.writeVarInt32(link.featureType);
				writer.writeInt64(link.startOffset * 1e6);
				writer.writeInt64(link.endOffset * 1e6);
				writer.writeVarInt32(link.direction);
				writer.writeVarInt32(link.isCenterLink);
				serializePolyline3d(link.geometry.postions, &writer);

			}
			writer.writeVarUint32(pIntersection->outLines.size());
			for (auto link : pIntersection->outLines)
			{
				writer.writeInt64(link.linkId);
				writer.writeInt64(link.offset * 1e6);
				writer.writeInt64(link.position.pos.lon);
				writer.writeInt64(link.position.pos.lat);
				writer.writeInt32(link.position.z);
			}
		}
		writer.alignToBits(8);
		streamWriter->writeVarUint64(writer.lengthInBytes());
		streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
	}

	void MeshWriter::serializeTollGate(DbMesh* pMesh, ByteStreamWriter* streamWriter)
	{
		ByteStreamWriter writer;
		writer.writeUint8((uint8)RecordType::DB_HAD_TOLLGATE);
		auto rds = pMesh->query(RecordType::DB_HAD_TOLLGATE);
		writer.writeVarUint32(rds.size());
		for (auto& rd : rds)
		{
			DbTollGate* pTollGate = (DbTollGate*)rd;
			writer.writeInt64(pTollGate->uuid);
			writer.writeInt64(pTollGate->paPid);
			writer.writeVarInt32(pTollGate->tollType);
			writer.writeInt64(pTollGate->linkPid);
			writer.writeInt64(pTollGate->offset * 1e6);
			writer.writeInt64(pTollGate->geometry.pos.lon);
			writer.writeInt64(pTollGate->geometry.pos.lat);
			writer.writeInt32(pTollGate->geometry.z);

            std::string tollName = WString2String(pTollGate->tollName);
            writer.writeVarUint32(tollName.size());
            if (!pTollGate->tollName.empty())
            {
                // 需要多输出一个结尾符号'\0'
                writer.writeBytes(tollName.c_str(), tollName.size() + 1);
            }

			writer.writeVarUint32(pTollGate->tollLanes.size());
			for (int i = 0; i < pTollGate->tollLanes.size(); i++)
			{
				DbTollLane* pTollLane = pTollGate->tollLanes[i];
				writer.writeVarInt32(pTollLane->seqNum);
				writer.writeVarInt32(pTollLane->cardType);
				writer.writeVarInt32(pTollLane->payMethod);
				writer.writeInt64(pTollLane->laneLinkPid);
			}
		}
		writer.alignToBits(8);
		streamWriter->writeVarUint64(writer.lengthInBytes());
		streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
	}


	void MeshWriter::serializeTrafficLights(DbMesh* pMesh, ByteStreamWriter* streamWriter)
	{
		ByteStreamWriter writer;
		writer.writeUint8((uint8)RecordType::DB_HAD_OBJECT_TRAFFIC_LIGHTS);
		auto rds = pMesh->query(RecordType::DB_HAD_OBJECT_TRAFFIC_LIGHTS);
		writer.writeVarUint32(rds.size());
		for (auto& rd : rds)
		{
			DbTrafficLights* pTrafficLights = (DbTrafficLights*)rd;
			writer.writeInt64(pTrafficLights->uuid);
			writer.writeVarInt32(pTrafficLights->type);
			writer.writeVarInt32(pTrafficLights->orientation);
			writer.writeVarInt32(pTrafficLights->rowNumber);
			writer.writeVarInt32(pTrafficLights->columnNumber);
			writer.writeVarInt32(pTrafficLights->heading*1000);
			serializePolyline3d(pTrafficLights->geometry.vertexes, &writer);
		}
		writer.alignToBits(8);
		streamWriter->writeVarUint64(writer.lengthInBytes());
		streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
	}

	void MeshWriter::serializePole(DbMesh* pMesh, ByteStreamWriter* streamWriter)
	{
		ByteStreamWriter writer;
		writer.writeUint8((uint8)RecordType::DB_HAD_OBJECT_POLE);
		auto rds = pMesh->query(RecordType::DB_HAD_OBJECT_POLE);
		writer.writeVarUint32(rds.size());
		for (auto& rd : rds)
		{
			DbPole* pPole = (DbPole*)rd;
			writer.writeInt64(pPole->uuid);
			writer.writeVarInt32(pPole->type);
			writer.writeVarInt32(pPole->diameterTop);
			writer.writeVarInt32(pPole->diameterBottom);
			serializePolyline3d(pPole->geometry.vertexes, &writer);
		}
		writer.alignToBits(8);
		streamWriter->writeVarUint64(writer.lengthInBytes());
		streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
	}


    void MeshWriter::serializeLaneTurnwaiting(DbMesh* pMesh, ByteStreamWriter* streamWriter)
    {
        ByteStreamWriter writer;
        writer.writeUint8((uint8)RecordType::DB_HAD_LANE_LINK_TURNWAITING);
        auto rds = pMesh->query(RecordType::DB_HAD_LANE_LINK_TURNWAITING);
        writer.writeVarUint32(rds.size());
        for (auto& rd : rds)
        {
			DbLaneLinkTurnwaiting* laneLinkTurnwaiting = (DbLaneLinkTurnwaiting*)rd;
            writer.writeInt64(laneLinkTurnwaiting->uuid);
			writer.writeVarInt32(laneLinkTurnwaiting->featureType);
            writer.writeInt64(laneLinkTurnwaiting->startOffset * 1e6);
            writer.writeInt64(laneLinkTurnwaiting->endOffset * 1e6);

            serializePolyline3d(laneLinkTurnwaiting->geometry.postions, &writer);
        }
        writer.alignToBits(8);
        streamWriter->writeVarUint64(writer.lengthInBytes());
        streamWriter->writeBytes(writer.bytes(), writer.lengthInBytes());
    }
}