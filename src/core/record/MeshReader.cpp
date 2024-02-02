#include "stdafx.h"
#include "MeshReader.h"
#include "DbMesh.h"
#include "DbRecord.h"
#pragma warning(disable:4700)
#pragma warning(disable:4244)
#pragma warning(disable:4018)
namespace OMDB
{
	std::wstring String2WString(const std::string& str)
	{
		int len = MultiByteToWideChar(CP_ACP, 0, str.c_str(), -1, NULL, 0);
		wchar_t* wide = new wchar_t[len + 1];
		memset(wide, '\0', sizeof(wchar_t) * (len + 1));
		MultiByteToWideChar(CP_ACP, 0, str.c_str(), -1, wide, len);
		std::wstring w_str(wide);
		delete[] wide;
		return w_str;
	}

	void MeshReader::read(void* pData, size_t size, DbMesh* pMesh)
	{
		if (size == 0)
			return;
		ByteStreamReader streamReader(pData,size);
		parseLink( pMesh,&streamReader);
		parseLgLink( pMesh,&streamReader);
		parseLgAssociation( pMesh,&streamReader);
		parseNode( pMesh,&streamReader);
		parseLinkPA( pMesh,&streamReader);
		parseRoadRoundNode( pMesh,&streamReader);
		parseRoadRoundPA( pMesh,&streamReader);
		parseRoadBoundLink( pMesh,&streamReader);
		parseLaneLink( pMesh,&streamReader);
		parseLaneNode( pMesh,&streamReader);
		parseLaneMarkLink( pMesh,&streamReader);
		parseLaneMarkNode( pMesh,&streamReader);
		parseLaneMarkPA( pMesh,&streamReader);

		parseBarrier(pMesh, &streamReader);
		parseWall(pMesh, &streamReader);
		parseFillArea(pMesh, &streamReader);
		parseCrossWalk(pMesh, &streamReader);
		parseArrow(pMesh, &streamReader);
		parseTrafficSign(pMesh, &streamReader);
		parseStopLocation(pMesh, &streamReader);
		parseLgRel(pMesh, &streamReader);
		parseLaneLinkRel(pMesh, &streamReader);

		parseText(pMesh, &streamReader);
		parseFixedSpeedLimit( pMesh,&streamReader);
		parseLinkSpeedLimit( pMesh,&streamReader);
		parseIntersection( pMesh,&streamReader);
		parseTollGate( pMesh,&streamReader);
		parseTrafficLight(pMesh, &streamReader);
		parsePole(pMesh, &streamReader);
		parseLaneTurnwaiting(pMesh, &streamReader);
	}
	void MeshReader::parsePolyline3d(std::vector<MapPoint3D64>& points, ByteStreamReader* streamReader)
	{
// 		uint32 size;
// 		if (!streamReader->readVarUint32(&size))
// 			return;
// 
// 		if (size == 0)
// 			return;
// 
// 		MapPoint3D64 previous{ {0,0},0 };
// 		streamReader->readInt64(&previous.pos.lon);
// 		streamReader->readInt64(&previous.pos.lat);
// 		streamReader->readVarInt32(&previous.z);
// 
// 		points.resize(size);
// 		points[0] = previous;
// 
// 		for (int i = 1;i < size;i++)
// 		{
// 			int64 x, y;
// 			int32 z;
// 			streamReader->readVarInt64(&x);
// 			streamReader->readVarInt64(&y);
// 			streamReader->readVarInt32(&z);
// 			points[i].pos.lat = previous.pos.lat + y;
// 			points[i].pos.lon = previous.pos.lon + x;
// 			points[i].z = previous.z + z;
// 			previous = points[i];
// 		}
		uint32 size;
		if (!streamReader->readVarUint32(&size))
			return;

		if (size > 0)
		{
			points.resize(size);
			MapPoint3D64 point{ {0,0},0 };
			if (!streamReader->readIntN64(&point.pos.lon, 36) || !streamReader->readIntN64(&point.pos.lat, 36) || !streamReader->readVarInt32(&point.z))
				return;

			points[0] = point;
			if (size > 1)
			{
				uint32 xBitNum;
				uint32 yBitNum;
				uint32 zBitNum;
				int64 dx;
				int64 dy;
				int32 dz;
				if (!streamReader->readUintN(&xBitNum, 6) || !streamReader->readUintN(&yBitNum, 6) || !streamReader->readUintN(&zBitNum, 5))
					return;

				for (uint32 i = 1; i < size; i++)
				{
					if (!streamReader->readIntN64(&dx, xBitNum + 1) || !streamReader->readIntN64(&dy, yBitNum + 1) || !streamReader->readIntN(&dz, zBitNum + 1))
						return;

					point.pos.lon += dx;
					point.pos.lat += dy;
					point.z += dz;
					points[i] = point;
				}
			}
		}
	}
	void MeshReader::parseLink(DbMesh* pMesh, ByteStreamReader* streamReader)
	{
		uint64 bytesize;
		streamReader->readVarUint64(&bytesize);
		uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;

		uint8 type;
		streamReader->readUint8(&type);
		if (type == (uint8)RecordType::DB_HAD_LINK)
		{
			uint32 size;
			streamReader->readVarUint32(&size);
			for (int i = 0; i < size; i++)
			{
				DbLink* pLink = (DbLink*)pMesh->alloc(RecordType::DB_HAD_LINK);
				streamReader->readInt64(&pLink->uuid);
				streamReader->readInt64(&pLink->startNode);
				streamReader->readInt64(&pLink->endNode);
				streamReader->readVarInt32(&pLink->kind);
				streamReader->readVarInt32(&pLink->multi_digitized);
				streamReader->readVarInt32(&pLink->direct);
				streamReader->readVarInt32(&pLink->separation_left);
				streamReader->readVarInt32(&pLink->separation_right);
				streamReader->readVarInt32(&pLink->median_left);
				streamReader->readVarInt32(&pLink->median_right);
				streamReader->readVarInt32(&pLink->overhead_obstruction);
				uint32 wayTypeSize;
				streamReader->readVarUint32(&wayTypeSize);
				for (int j = 0; j < wayTypeSize; j++) {
					int wayType;
					streamReader->readVarInt32(&wayType);
					pLink->wayTypes.push_back(wayType);
				}
				uint32 dataLevelSize;
				streamReader->readVarUint32(&dataLevelSize);
				for (int j = 0; j < dataLevelSize; j++)
				{
					DbLinkDataLevel dataLevel;
					streamReader->readInt64(&dataLevel.uuid);
					streamReader->readVarInt32(&dataLevel.dataLevel);
					streamReader->readVarInt32(&dataLevel.featureType);

					int64 s, e;
					streamReader->readInt64(&s);
					streamReader->readInt64(&e);
					dataLevel.startOffset = s * 1.0 / 1e6;
					dataLevel.endOffset = e * 1.0 / 1e6;
					parsePolyline3d(dataLevel.geometry.postions, streamReader);
					pLink->dataLevels.push_back(dataLevel);
				}
				uint32 separationSize;
				streamReader->readVarUint32(&separationSize);
				for (int j = 0; j < separationSize; j++)
				{
					DbLinkSeparation separation;
					streamReader->readInt64(&separation.uuid);
					streamReader->readVarInt32(&separation.side);
					streamReader->readVarInt32(&separation.separation);
					streamReader->readVarInt32(&separation.featureType);

					int64 s, e;
					streamReader->readInt64(&s);
					streamReader->readInt64(&e);
					separation.startOffset = s * 1.0 / 1e6;
					separation.endOffset = e * 1.0 / 1e6;
					parsePolyline3d(separation.geometry.postions, streamReader);
					pLink->separations.push_back(separation);
				}
				uint32 medianSize;
				streamReader->readVarUint32(&medianSize);
				for (int j = 0; j < medianSize; j++)
				{
					DbLinkMedian median;
					streamReader->readInt64(&median.uuid);
					streamReader->readVarInt32(&median.side);
					streamReader->readVarInt32(&median.median);
					streamReader->readVarInt32(&median.featureType);

					int64 s, e;
					streamReader->readInt64(&s);
					streamReader->readInt64(&e);
					median.startOffset = s * 1.0 / 1e6;
					median.endOffset = e * 1.0 / 1e6;
					parsePolyline3d(median.geometry.postions, streamReader);
					pLink->medians.push_back(median);
				}
				uint32 overheadObstructionSize;
				streamReader->readVarUint32(&overheadObstructionSize);
				for (int j = 0; j < overheadObstructionSize; j++)
				{
					DbLinkOverheadObstruction overheadObstruction;
					streamReader->readInt64(&overheadObstruction.uuid);
					streamReader->readVarInt32(&overheadObstruction.featureType);

					int64 s, e;
					streamReader->readInt64(&s);
					streamReader->readInt64(&e);
					overheadObstruction.startOffset = s * 1.0 / 1e6;
					overheadObstruction.endOffset = e * 1.0 / 1e6;
					parsePolyline3d(overheadObstruction.geometry.postions, streamReader);
					pLink->overheadObstructions.push_back(overheadObstruction);
				}
				uint32 tollAreaSize;
				streamReader->readVarUint32(&tollAreaSize);
				for (int j = 0; j < tollAreaSize; j++)
				{
					DbLinkTollArea tollArea;
					streamReader->readInt64(&tollArea.uuid);
					streamReader->readVarInt32(&tollArea.featureType);

					int64 s, e;
					streamReader->readInt64(&s);
					streamReader->readInt64(&e);
					tollArea.startOffset = s * 1.0 / 1e6;
					tollArea.endOffset = e * 1.0 / 1e6;
					parsePolyline3d(tollArea.geometry.postions, streamReader);
					pLink->tollAreas.push_back(tollArea);
				}
				parsePolyline3d(pLink->geometry.vertexes, streamReader);
				pMesh->insert(pLink->uuid, pLink);
			}
		}
		uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
		if (s > 1)
		{
			printInfo("-----------------parseLink----------------------");
			printInfo("nextpos = %d", s);
		}
		streamReader->seek(nextPos);
	}

	void MeshReader::parseLgLink(DbMesh* pMesh, ByteStreamReader* streamReader)
	{

		uint64 bytesize;
		streamReader->readVarUint64(&bytesize);
		uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;

		uint8 type;
		streamReader->readUint8(&type);
		if (type == (uint8)RecordType::DB_HAD_LG_LINK)
		{
			uint32 size;
			streamReader->readVarUint32(&size);
			for (int i = 0; i < size; i++)
			{
				DbLgLink* pLink = (DbLgLink*)pMesh->alloc(RecordType::DB_HAD_LG_LINK);

				streamReader->readInt64(&pLink->uuid);
				uint32 relSize;
				streamReader->readVarUint32(&relSize);
				for (int j = 0; j < relSize; j++)
				{
					DbLgLink::DbRelLink rel;
					int64 id;
					streamReader->readInt64(&id);
					streamReader->readInt64(&rel.relLinkid);

					int64 s, e;
					streamReader->readInt64(&s);
					streamReader->readInt64(&e);
					rel.startOffset = s * 1.0 / 1e6;
					rel.endOffset = e * 1.0 / 1e6;
					streamReader->readVarInt32(&rel.type);
					streamReader->readVarInt32(&rel.directType);
					pLink->relLinks.emplace(id, rel);
				}
				pMesh->insert(pLink->uuid, pLink);
			}
		}
		uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
		if (s > 1)
		{
			printInfo("-----------------parseLgLink----------------------");
			printInfo("nextpos = %d", s);
		}
		streamReader->seek(nextPos);

	}

	void MeshReader::parseLgAssociation(DbMesh* pMesh, ByteStreamReader* streamReader)
	{
		uint64 bytesize;
		streamReader->readVarUint64(&bytesize);
		uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;

		uint8 type;
		streamReader->readUint8(&type);
		if (type == (uint8)RecordType::DB_HAD_LG_ASSOCIATION)
		{
			uint32 size;
			streamReader->readVarUint32(&size);
			for (int i = 0; i < size; i++)
			{
				DbLgAssociation* pAssociation = (DbLgAssociation*)pMesh->alloc(RecordType::DB_HAD_LG_ASSOCIATION);
				streamReader->readInt64(&pAssociation->uuid);
				uint32 relSize;
				streamReader->readVarUint32(&relSize);
				for (int j = 0; j < relSize; j++)
				{
					DbLgAssociation::DbRelLgAssociation rel;
					streamReader->readVarInt32(&rel.directType);
					streamReader->readInt64(&rel.firstLgLinkId);
					streamReader->readInt64(&rel.secondLgLinkId);
					pAssociation->relLgAssociations.push_back(rel);
				}
				pMesh->insert(pAssociation->uuid, pAssociation);
			}
		}

		uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
		if (s > 1)
		{
			printInfo("-----------------parseLgAssociation----------------------");
			printInfo("nextpos = %d", s);
		}
		streamReader->seek(nextPos);
	}

	void MeshReader::parseNode(DbMesh* pMesh, ByteStreamReader* streamReader)
	{
		uint64 bytesize;
		streamReader->readVarUint64(&bytesize);
		uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;
		uint8 type;
		streamReader->readUint8(&type);
		if (type == (uint8)RecordType::DB_HAD_NODE)
		{
			uint32 size;
			streamReader->readVarUint32(&size);
			for (int i = 0; i < size; i++)
			{
				DbNode* pNode = (DbNode*)pMesh->alloc(RecordType::DB_HAD_NODE);
				streamReader->readInt64(&pNode->uuid);
				streamReader->readInt64(&pNode->intersectionId);
				streamReader->readVarInt32(&pNode->wayType);

				uint32 mz;
				streamReader->readVarUint32(&mz);
				pNode->meshIds.resize(mz);
				for (int j = 0; j < mz; j++)
				{
					streamReader->readVarInt32(&pNode->meshIds[j]);
				}
				streamReader->readInt64(&pNode->geometry.pos.lon);
				streamReader->readInt64(&pNode->geometry.pos.lat);
				streamReader->readInt32(&pNode->geometry.z);
				pMesh->insert(pNode->uuid, pNode);
			}
		}
		uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
		if (s > 1)
		{
			printInfo("-----------------parseNode----------------------");
			printInfo("nextpos = %d", s);
		}
		streamReader->seek(nextPos);
	}
	void MeshReader::parseLinkPA(DbMesh* pMesh, ByteStreamReader* streamReader)
	{
		uint64 bytesize;
		streamReader->readVarUint64(&bytesize);
		uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;
		uint8 type;
		streamReader->readUint8(&type);
		if (type == (uint8)RecordType::DB_HAD_LINK_PA)
		{
			uint32 size;
			streamReader->readVarUint32(&size);
			for (int i = 0; i < size; i++)
			{
				DbLinkPA* pLinkPa = (DbLinkPA*)pMesh->alloc(RecordType::DB_HAD_LINK_PA);

				streamReader->readInt64(&pLinkPa->uuid);
				streamReader->readInt64(&pLinkPa->relLinkId);
				streamReader->readVarInt32(&pLinkPa->featureType);
				int64 s, e;
				streamReader->readInt64(&s);
				streamReader->readInt64(&e);
				pLinkPa->startOffset = s * 1.0 / 1e6;
				pLinkPa->endOffset = e * 1.0 / 1e6;
				parsePolyline3d(pLinkPa->geometry.postions, streamReader);

				uint32 ps;
				streamReader->readVarUint32(&ps);
				for (int j = 0; j < ps; j++)
				{
					DbPAValue* pValue = (DbPAValue*)pMesh->alloc(RecordType::DB_HAD_PA_VALUE);
					streamReader->readVarInt32(&pValue->seqNum);
					streamReader->readVarInt32(&pValue->attributeType);
					streamReader->readVarInt32(&pValue->attributeValue);
					pLinkPa->paValues.push_back(pValue);
				}
				pMesh->insert(pLinkPa->uuid, pLinkPa);
			}
		}
		uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
		if (s > 1)
		{
			printInfo("-----------------parseLinkPA----------------------");
			printInfo("nextpos = %d", s);
		}
		streamReader->seek(nextPos);
	}

	void MeshReader::parseRoadRoundNode(DbMesh* pMesh, ByteStreamReader* streamReader)
	{
		uint64 bytesize;
		streamReader->readVarUint64(&bytesize);
		uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;

		uint8 type;
		streamReader->readUint8(&type);
		if (type == (uint8)RecordType::DB_HAD_ROAD_BOUNDARY_NODE)
		{
			uint32 size;
			streamReader->readVarUint32(&size);
			for (int i = 0; i < size; i++)
			{
				DbRoadBoundNode* pNode = (DbRoadBoundNode*)pMesh->alloc(RecordType::DB_HAD_ROAD_BOUNDARY_NODE);
				streamReader->readInt64(&pNode->uuid);
				streamReader->readInt64(&pNode->geometry.pos.lon);
				streamReader->readInt64(&pNode->geometry.pos.lat);
				streamReader->readInt32(&pNode->geometry.z);
				pMesh->insert(pNode->uuid, pNode);
			}
		}
		uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
		if (s > 1)
		{
			printInfo("-----------------parseRoadRoundNode----------------------");
			printInfo("nextpos = %d", s);
		}
		streamReader->seek(nextPos);
	}

	void MeshReader::parseRoadRoundPA(DbMesh* pMesh, ByteStreamReader* streamReader)
	{
		uint64 bytesize;
		streamReader->readVarUint64(&bytesize);
		uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;

		uint8 type;
		streamReader->readUint8(&type);
		if (type == (uint8)RecordType::DB_HAD_ROAD_BOUNDARY_PA)
		{
			uint32 size;
			streamReader->readVarUint32(&size);
			for (int i = 0; i < size; i++)
			{
				DbRoadBoundPA* pLinkPa = (DbRoadBoundPA*)pMesh->alloc(RecordType::DB_HAD_ROAD_BOUNDARY_PA);
				streamReader->readInt64(&pLinkPa->uuid);
				streamReader->readInt64(&pLinkPa->relRoadBoundLinkId);
				streamReader->readVarInt32(&pLinkPa->featureType);
				int64 s, e;
				streamReader->readInt64(&s);
				streamReader->readInt64(&e);
				pLinkPa->startOffset = s * 1.0 / 1e6;
				pLinkPa->endOffset = e * 1.0 / 1e6;

				uint32 ps;
				streamReader->readVarUint32(&ps);
				for (int j = 0; j < ps; j++)
				{
					DbPAValue* pValue = (DbPAValue*)pMesh->alloc(RecordType::DB_HAD_PA_VALUE);
					streamReader->readVarInt32(&pValue->seqNum);
					streamReader->readVarInt32(&pValue->attributeType);
					streamReader->readVarInt32(&pValue->attributeValue);
					pLinkPa->paValues.push_back(pValue);
				}
				parsePolyline3d(pLinkPa->geometry.postions, streamReader);

				pMesh->insert(pLinkPa->uuid, pLinkPa);
			}
		}
		uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
		if (s > 1)
		{
			printInfo("-----------------parseRoadRoundPA----------------------");
			printInfo("nextpos = %d", s);
		}
		streamReader->seek(nextPos);
	}

	void MeshReader::parseRoadBoundLink(DbMesh* pMesh, ByteStreamReader* streamReader)
	{
		uint64 bytesize;
		streamReader->readVarUint64(&bytesize);
		uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;

		uint8 type;
		streamReader->readUint8(&type);
		if (type == (uint8)RecordType::DB_HAD_ROAD_BOUNDARY_LINK)
		{
			uint32 size;
			streamReader->readVarUint32(&size);
			for (int i = 0; i < size; i++)
			{
				DbRoadBoundLink* pLink = (DbRoadBoundLink*)pMesh->alloc(RecordType::DB_HAD_ROAD_BOUNDARY_LINK);
				streamReader->readInt64(&pLink->uuid);
				streamReader->readInt64(&pLink->starRoadBoundNodeId);
				streamReader->readInt64(&pLink->endRoadBoundNodeId);
				streamReader->readVarInt32(&pLink->boundaryType);

				uint32 lgSize;
				streamReader->readVarUint32(&lgSize);
				for (int j = 0; j < lgSize; j++)
				{
					DbRoadBoundLink::DbLgRoadBoundREL rel;
					int64 id;
					streamReader->readInt64(&id);
					streamReader->readVarInt32(&rel.side);
					streamReader->readVarInt32(&rel.direction);
					pLink->relLgs.emplace(id, rel);
				}

				parsePolyline3d(pLink->geometry.vertexes, streamReader);
				pMesh->insert(pLink->uuid, pLink);
			}
		}
		uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
		if (s > 1)
		{
			printInfo("-----------------parseRoadBoundLink----------------------");
			printInfo("nextpos = %d", s);
		}
		streamReader->seek(nextPos);

	}

	void MeshReader::parseLaneLink(DbMesh* pMesh, ByteStreamReader* streamReader)
	{
		uint64 bytesize;
		streamReader->readVarUint64(&bytesize);
		uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;
		uint8 type;
		streamReader->readUint8(&type);
		if (type == (uint8)RecordType::DB_HAD_LANE_LINK)
		{
			uint32 size;
			streamReader->readUint32(&size);
			for (int i = 0; i < size; i++)
			{
				DbLaneLink* pLink = (DbLaneLink*)pMesh->alloc(RecordType::DB_HAD_LANE_LINK);
				streamReader->readInt64(&pLink->uuid);
				streamReader->readUint32(&pLink->laneType);
				streamReader->readVarInt32(&pLink->arrowDir);
				streamReader->readVarInt32(&pLink->width);
				streamReader->readInt64(&pLink->relLgId);
				streamReader->readUint32(&pLink->laneNum);
				streamReader->readInt64(&pLink->startLaneNodeId);
				streamReader->readInt64(&pLink->endLaneNodeId);
				streamReader->readUint32(&pLink->seqNumber);
				streamReader->readVarInt32(&pLink->laneDir);
				streamReader->readVarInt32(&pLink->conditionType);
				parsePolyline3d(pLink->geometry.vertexes, streamReader);

				pMesh->insert(pLink->uuid, pLink);
			}
		}
		uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
		if (s > 1)
		{
			printInfo("-----------------parseLaneLink----------------------");
			printInfo("nextpos = %d", s);
		}
		streamReader->seek(nextPos);
	}

	void MeshReader::parseLaneNode(DbMesh* pMesh, ByteStreamReader* streamReader)
	{
		uint64 bytesize;
		streamReader->readVarUint64(&bytesize);
		uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;

		uint8 type;
		streamReader->readUint8(&type);
		if (type == (uint8)RecordType::DB_HAD_LANE_NODE)
		{
			uint32 size;
			streamReader->readVarUint32(&size);
			for (int i = 0; i < size; i++)
			{
				DbLaneNode* pNode = (DbLaneNode*)pMesh->alloc(RecordType::DB_HAD_LANE_NODE);
				streamReader->readInt64(&pNode->uuid);
				streamReader->readInt64(&pNode->geometry.pos.lon);
				streamReader->readInt64(&pNode->geometry.pos.lat);
				streamReader->readInt32(&pNode->geometry.z);
				pMesh->insert(pNode->uuid, pNode);
			}
		}
		uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
		if (s > 1)
		{
			printInfo("-----------------parseLaneNode----------------------");
			printInfo("nextpos = %d", s);
		}
		streamReader->seek(nextPos);
	}

	void MeshReader::parseLaneMarkLink(DbMesh* pMesh, ByteStreamReader* streamReader)
	{
		uint64 bytesize;
		streamReader->readVarUint64(&bytesize);
		uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;

		uint8 type;
		streamReader->readUint8(&type);
		if (type == (uint8)RecordType::DB_HAD_LANE_MARK_LINK)
		{
			uint32 size;
			streamReader->readVarUint32(&size);
			for (int i = 0; i < size; i++)
			{
				DbLaneMarkLink* pLink = (DbLaneMarkLink*)pMesh->alloc(RecordType::DB_HAD_LANE_MARK_LINK);
				streamReader->readInt64(&pLink->uuid);
				streamReader->readInt64(&pLink->startLaneMarkNodeId);
				streamReader->readInt64(&pLink->endLaneMarkNodeId);
				streamReader->readVarInt32(&pLink->boundaryType);
				streamReader->readVarInt32(&pLink->markingCount);
				streamReader->readVarInt32(&pLink->traversal);

				//streamReader->readInt64(&pLink->relLaneLinkId);
				//streamReader->readVarInt32(&pLink->side);
				//streamReader->readVarInt32(&pLink->direct);
				uint32 laneMarkRelSize = 0;
				streamReader->readVarUint32(&laneMarkRelSize);
				pLink->laneMarkRels.resize(laneMarkRelSize);
				for( int j=0; j< laneMarkRelSize; j++)
				{
					streamReader->readInt64(&pLink->laneMarkRels[j].relLaneLinkId);
					streamReader->readVarInt32(&pLink->laneMarkRels[j].side);
					streamReader->readVarInt32(&pLink->laneMarkRels[j].direct);
				}

				uint32 mz;
				streamReader->readVarUint32(&mz);
				pLink->markings.resize(mz);
				for (int j = 0; j < mz; j++)
				{
					DbLaneMarkLink::DbLaneMarkLinkMarking& marking = pLink->markings[j];
					streamReader->readInt32(&marking.markSeqNum);
					streamReader->readInt32(&marking.markType);
					streamReader->readInt32(&marking.markColor);
					streamReader->readInt32(&marking.markWidth);
					streamReader->readInt32(&marking.markMaterial);
					streamReader->readInt32(&marking.lateralOffset);
				}

				uint32 rz;
				streamReader->readVarUint32(&rz);
				for (int j = 0; j < rz; j++)
				{
					DbLaneMarkLink::DbLgMarkRel rel;
					int64 id;
					streamReader->readInt64(&id);
					streamReader->readVarInt32(&rel.lgMarkDirect);
					streamReader->readVarInt32(&rel.lgMarkSeqNum);
					pLink->relLgs.emplace(id, rel);
				}
				parsePolyline3d(pLink->geometry.vertexes, streamReader);
				pMesh->insert(pLink->uuid, pLink);
			}
		}
		uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
		//if (s > 1)
		//{
		//	printInfo("-----------------parseLaneMarkLink----------------------");
		//	printInfo("nextpos = %d", s);
		//}
		streamReader->seek(nextPos);
	}

	void MeshReader::parseLaneMarkNode(DbMesh* pMesh, ByteStreamReader* streamReader)
	{
		uint64 bytesize;
		streamReader->readVarUint64(&bytesize);
		uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;

		uint8 type;
		streamReader->readUint8(&type);
		if (type == (uint8)RecordType::DB_HAD_LANE_MARK_NODE)
		{
			uint32 size;
			streamReader->readVarUint32(&size);
			for (int i = 0; i < size; i++)
			{
				DbLaneMarkNode* pNode = (DbLaneMarkNode*)pMesh->alloc(RecordType::DB_HAD_LANE_MARK_NODE);
				streamReader->readInt64(&pNode->uuid);
				streamReader->readInt64(&pNode->geometry.pos.lon);
				streamReader->readInt64(&pNode->geometry.pos.lat);
				streamReader->readInt32(&pNode->geometry.z);
				pMesh->insert(pNode->uuid, pNode);
			}
		}
		uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
		if (s > 1)
		{
			printInfo("-----------------parseLaneMarkNode----------------------");
			printInfo("nextpos = %d", s);
		}
		streamReader->seek(nextPos);
	}

	void MeshReader::parseLaneMarkPA(DbMesh* pMesh, ByteStreamReader* streamReader)
	{
		uint64 bytesize;
		streamReader->readVarUint64(&bytesize);
		uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;
		uint8 type;
		streamReader->readUint8(&type);
		if (type == (uint8)RecordType::DB_HAD_LANE_MARK_PA)
		{
			uint32 size;
			streamReader->readVarUint32(&size);
			for (int i = 0; i < size; i++)
			{
				DbLaneMarkPA* pLinkPa = (DbLaneMarkPA*)pMesh->alloc(RecordType::DB_HAD_LANE_MARK_PA);
				streamReader->readInt64(&pLinkPa->uuid);
				streamReader->readInt64(&pLinkPa->relLaneMarkLinkId);
				streamReader->readVarInt32(&pLinkPa->featureType);
				int64 s, e;
				streamReader->readInt64(&s);
				streamReader->readInt64(&e);
				pLinkPa->startOffset = s * 1.0 / 1e6;
				pLinkPa->endOffset = e * 1.0 / 1e6;

				uint32 ps;
				streamReader->readVarUint32(&ps);
				for (int j = 0; j < ps; j++)
				{
					DbPAValue* pValue = (DbPAValue*)pMesh->alloc(RecordType::DB_HAD_PA_VALUE);
					streamReader->readVarInt32(&pValue->seqNum);
					streamReader->readVarInt32(&pValue->attributeType);
					streamReader->readVarInt32(&pValue->attributeValue);
					pLinkPa->paValues.push_back(pValue);
				}
				parsePolyline3d(pLinkPa->geometry.postions, streamReader);

				pMesh->insert(pLinkPa->uuid, pLinkPa);
			}
		}
		uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
		if (s > 1)
		{
			printInfo("-----------------parseLaneMarkPA----------------------");
			printInfo("nextpos = %d", s);
		}
		streamReader->seek(nextPos);

	}

	void MeshReader::parseBarrier(DbMesh* pMesh, ByteStreamReader* streamReader)
	{
		uint64 bytesize;
		streamReader->readVarUint64(&bytesize);
		uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;
		uint8 type;
		streamReader->readUint8(&type);
		if (type == (uint8)RecordType::DB_HAD_OBJECT_BARRIER)
		{

			uint32 size;
			streamReader->readUint32(&size);
			for (int i = 0; i < size; i++)
			{
				DbBarrier* pBarrier = (DbBarrier*)pMesh->alloc(RecordType::DB_HAD_OBJECT_BARRIER);
				streamReader->readInt64(&pBarrier->uuid);
				streamReader->readVarInt32(&pBarrier->barrierType);
				uint32 gz;
				streamReader->readVarUint32(&gz);
				pBarrier->geometry.lines.resize(gz);
				for (int j = 0; j < gz; j++)
				{
					parsePolyline3d(pBarrier->geometry.lines[j].vertexes, streamReader);
				}
				pMesh->insert(pBarrier->uuid, pBarrier);
			}
		}
		uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
		if (s > 1)
		{
			printInfo("-----------------parseBarrier----------------------");
			printInfo("nextpos = %d", s);
		}
		streamReader->seek(nextPos);
	}


	void MeshReader::parseWall(DbMesh* pMesh, ByteStreamReader* streamReader)
	{
		uint64 bytesize;
		streamReader->readVarUint64(&bytesize);
		uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;

		uint8 type;
		streamReader->readUint8(&type);
		if (type == (uint8)RecordType::DB_HAD_OBJECT_WALL)
		{
			uint32 size;
			streamReader->readVarUint32(&size);
			for (int i = 0; i < size; i++)
			{
				DbWall* pWall = (DbWall*)pMesh->alloc(RecordType::DB_HAD_OBJECT_WALL);
				streamReader->readInt64(&pWall->uuid);
				streamReader->readVarInt32(&pWall->wallType);
				uint32 gz;
				streamReader->readVarUint32(&gz);
				pWall->geometry.lines.resize(gz);
				for (int j = 0; j < gz; j++)
				{
					parsePolyline3d(pWall->geometry.lines[j].vertexes, streamReader);
				}
				pMesh->insert(pWall->uuid, pWall);
			}
		}
		uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
		if (s > 1)
		{
			printInfo("-----------------parseWall----------------------");
			printInfo("nextpos = %d", s);
		}
		streamReader->seek(nextPos);
	}

	void MeshReader::parseFillArea(DbMesh* pMesh, ByteStreamReader* streamReader)
	{
		uint64 bytesize;
		streamReader->readVarUint64(&bytesize);
		uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;
		uint8 type;
		streamReader->readUint8(&type);
		if (type == (uint8)RecordType::DB_HAD_OBJECT_FILL_AREA)
		{
			uint32 size;
			streamReader->readVarUint32(&size);
			for (int i = 0; i < size; i++)
			{
				DbFillArea* pFillArea = (DbFillArea*)pMesh->alloc(RecordType::DB_HAD_OBJECT_FILL_AREA);
				streamReader->readInt64(&pFillArea->uuid);
				parsePolyline3d(pFillArea->geometry.vertexes, streamReader);
				pMesh->insert(pFillArea->uuid, pFillArea);
			}
		}
		uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
		if (s > 1)
		{
			printInfo("-----------------parseFillArea----------------------");
			printInfo("nextpos = %d", s);
		}
		streamReader->seek(nextPos);

	}

	void MeshReader::parseCrossWalk(DbMesh* pMesh, ByteStreamReader* streamReader)
	{
		uint64 bytesize;
		streamReader->readVarUint64(&bytesize);
		uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;

		uint8 type;
		streamReader->readUint8(&type);
		if (type == (uint8)RecordType::DB_HAD_OBJECT_CROSS_WALK)
		{
			uint32 size;
			streamReader->readVarUint32(&size);
			for (int i = 0; i < size; i++)
			{
				DbCrossWalk* pCrossWalk = (DbCrossWalk*)pMesh->alloc(RecordType::DB_HAD_OBJECT_CROSS_WALK);
				streamReader->readInt64(&pCrossWalk->uuid);
				streamReader->readVarInt32(&pCrossWalk->color);
				parsePolyline3d(pCrossWalk->geometry.vertexes, streamReader);
				pMesh->insert(pCrossWalk->uuid, pCrossWalk);
			}
		}
		uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
		if (s > 1)
		{
			printInfo("-----------------parseCrossWalk----------------------");
			printInfo("nextpos = %d", s);
		}
		streamReader->seek(nextPos);

	}

	void MeshReader::parseArrow(DbMesh* pMesh, ByteStreamReader* streamReader)
	{
		uint64 bytesize;
		streamReader->readVarUint64(&bytesize);
		uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;

		uint8 type;
		streamReader->readUint8(&type);
		if (type == (uint8)RecordType::DB_HAD_OBJECT_ARROW)
		{
			uint32 size;
			streamReader->readVarUint32(&size);
			for (int i = 0; i < size; i++)
			{
				DbArrow* pArrow = (DbArrow*)pMesh->alloc(RecordType::DB_HAD_OBJECT_ARROW);
				streamReader->readInt64(&pArrow->uuid);
				streamReader->readVarInt32(&pArrow->color);
				streamReader->readVarInt32(&pArrow->width);
				streamReader->readVarInt32(&pArrow->length);
				streamReader->readVarInt32(&pArrow->arrowClass);
				streamReader->readInt64(&pArrow->center.pos.lon);
				streamReader->readInt64(&pArrow->center.pos.lat);
				streamReader->readInt32(&pArrow->center.z);
				parsePolyline3d(pArrow->geometry.vertexes, streamReader);
				pMesh->insert(pArrow->uuid, pArrow);
			}
		}
		uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
		if (s > 1)
		{
			printInfo("-----------------parseArrow----------------------");
			printInfo("nextpos = %d", s);
		}
		streamReader->seek(nextPos);

	}

	void MeshReader::parseTrafficSign(DbMesh* pMesh, ByteStreamReader* streamReader)
	{
		uint64 bytesize;
		streamReader->readVarUint64(&bytesize);
		uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;
		uint8 type;
		streamReader->readUint8(&type);
		if (type == (uint8)RecordType::DB_HAD_OBJECT_TRAFFIC_SIGN)
		{
			uint32 size;
			streamReader->readVarUint32(&size);
			for (int i = 0; i < size; i++)
			{
				DbTrafficSign* pTrafficSign = (DbTrafficSign*)pMesh->alloc(RecordType::DB_HAD_OBJECT_TRAFFIC_SIGN);
				streamReader->readInt64(&pTrafficSign->uuid);
				streamReader->readVarInt32(&pTrafficSign->trafSignShape);
				streamReader->readVarInt32(&pTrafficSign->signType);
				streamReader->readVarInt32(&pTrafficSign->color);

				int64 heading;
				streamReader->readInt64(&heading);
				pTrafficSign->heading = heading * 1.0 / 1e6;
				parsePolyline3d(pTrafficSign->geometry.vertexes, streamReader);
				pMesh->insert(pTrafficSign->uuid, pTrafficSign);
			}
		}
		uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
		if (s > 1)
		{
			printInfo("-----------------parseTrafficSign----------------------");
			printInfo("nextpos = %d", s);
		}
		streamReader->seek(nextPos);

	}

	void MeshReader::parseStopLocation(DbMesh* pMesh, ByteStreamReader* streamReader)
	{
		uint64 bytesize;
		streamReader->readVarUint64(&bytesize);
		uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;
		uint8 type;
		streamReader->readUint8(&type);
		if (type == (uint8)RecordType::DB_HAD_OBJECT_STOPLOCATION)
		{
			uint32 size;
			streamReader->readVarUint32(&size);
			for (int i = 0; i < size; i++)
			{
				DbStopLocation* pStopLocation = (DbStopLocation*)pMesh->alloc(RecordType::DB_HAD_OBJECT_STOPLOCATION);
				streamReader->readInt64(&pStopLocation->uuid);
				streamReader->readVarInt32(&pStopLocation->color);
				streamReader->readVarInt32(&pStopLocation->width);
				streamReader->readVarInt32(&pStopLocation->locationType);
				parsePolyline3d(pStopLocation->geometry.vertexes, streamReader);
				pMesh->insert(pStopLocation->uuid, pStopLocation);
			}
		}
		uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
		if (s > 1)
		{
			printInfo("-----------------parseStopLocation----------------------");
			printInfo("nextpos = %d", s);
		}

		streamReader->seek(nextPos);

	}

	void MeshReader::parseLgRel(DbMesh* pMesh, ByteStreamReader* streamReader)
	{
		uint64 bytesize;
		streamReader->readVarUint64(&bytesize);
		uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;
		uint8 type;
		streamReader->readUint8(&type);
		if (type == (uint8)RecordType::DB_HAD_OBJECT_LG_REL)
		{
			uint32 size;
			streamReader->readUint32(&size);
			for (int i = 0; i < size; i++)
			{
				DbLgRel* pLgRel = (DbLgRel*)pMesh->alloc(RecordType::DB_HAD_OBJECT_LG_REL);
				streamReader->readInt64(&pLgRel->uuid);

				uint32 rz;
				streamReader->readVarUint32(&rz);
				for (int j = 0; j < rz; j++)
				{
					DbLgRel::DbRelLg lg;
					int64 id;
					streamReader->readInt64(&id);
					streamReader->readInt64(&lg.relLgId);
					streamReader->readVarInt32(&lg.objectType);
					pLgRel->relLgs.emplace(id, lg);
				}
				pMesh->insert(pLgRel->uuid, pLgRel);
			}
		}
		uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
		if (s > 1)
		{
			printInfo("-----------------parseStopLocation----------------------");
			printInfo("nextpos = %d", s);
		}
		streamReader->seek(nextPos);

	}

	void MeshReader::parseLaneLinkRel(DbMesh* pMesh, ByteStreamReader* streamReader)
	{
		uint64 bytesize;
		streamReader->readVarUint64(&bytesize);
		uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;
		uint8 type;
		streamReader->readUint8(&type);
		if (type == (uint8)RecordType::DB_HAD_OBJECT_LANE_LINK_REL)
		{
			uint32 size;
			streamReader->readVarUint32(&size);
			for (int i = 0; i < size; i++)
			{
				DbLaneLinkRel* pLgRel = (DbLaneLinkRel*)pMesh->alloc(RecordType::DB_HAD_OBJECT_LANE_LINK_REL);
				streamReader->readInt64(&pLgRel->uuid);

				uint32 rz;
				streamReader->readVarUint32(&rz);
				for (int j = 0; j < rz; j++)
				{
					DbLaneLinkRel::DbRelLaneLInk link;
					int64 id;
					streamReader->readInt64(&id);
					streamReader->readInt64(&link.relLaneLinkId);
					streamReader->readVarInt32(&link.objectType);
					pLgRel->relLaneLinkIds.emplace(id, link);
				}
				pMesh->insert(pLgRel->uuid, pLgRel);
			}
		}
		uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
		if (s > 1)
		{
			printInfo("-----------------parseLaneLinkRel----------------------");
			printInfo("nextpos = %d", s);
		}
		streamReader->seek(nextPos);

	}

	void MeshReader::parseText(DbMesh* pMesh, ByteStreamReader* streamReader)
	{
		uint64 bytesize;
		streamReader->readVarUint64(&bytesize);
		uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;
		uint8 type;
		streamReader->readUint8(&type);
		if (type == (uint8)RecordType::DB_HAD_OBJECT_TEXT)
		{
			uint32 size;
			streamReader->readVarUint32(&size);
			for (int i = 0; i < size; i++)
			{
				DbText* pText = (DbText*)pMesh->alloc(RecordType::DB_HAD_OBJECT_TEXT);
				streamReader->readInt64(&pText->uuid);
				streamReader->readVarInt32(&pText->color);
				streamReader->readVarInt32(&pText->width);
				streamReader->readVarInt32(&pText->length);

				uint32 sz{0};
				streamReader->readVarUint32(&sz);
				if (sz != 0)
				{
					void* ss = malloc(sz+1 );
					streamReader->readBytes(ss, sz+1);
					//pText->textContent = (const char*)ss;
					std::string textContent = (const char*)ss;
					pText->textContent = String2WString(textContent);
					free(ss);
				}
				streamReader->readInt64(&pText->center.pos.lon);
				streamReader->readInt64(&pText->center.pos.lat);
				streamReader->readInt32(&pText->center.z);
				parsePolyline3d(pText->geometry.vertexes, streamReader);
				pMesh->insert(pText->uuid, pText);
			}
		}
		uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
		if (s > 1)
		{
			printInfo("-----------------parseText----------------------");
			printInfo("nextpos = %d", s);
		}
		streamReader->seek(nextPos);

	}

	void MeshReader::parseFixedSpeedLimit(DbMesh* pMesh, ByteStreamReader* streamReader)
	{
		uint64 bytesize;
		streamReader->readVarUint64(&bytesize);
		uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;

		uint8 type;
		streamReader->readUint8(&type);
		if (type == (uint8)RecordType::DB_HAD_LANE_FIXED_SPEEDLIMIT)
		{
			uint32 size;
			streamReader->readVarUint32(&size);
			for (int i = 0; i < size; i++)
			{
				DbFixedSpeedLimit* pLimit = (DbFixedSpeedLimit*)pMesh->alloc(RecordType::DB_HAD_LANE_FIXED_SPEEDLIMIT);
				streamReader->readInt64(&pLimit->uuid);
				int32 max, min;

				streamReader->readVarInt32(&pLimit->maxSpeedLimit);
				streamReader->readVarInt32(&max);
				pLimit->maxSpeedLimitSource = (DbSpeedLimitSource)max;

				streamReader->readVarInt32(&pLimit->minSpeedLimit);
				streamReader->readVarInt32(&min);
				pLimit->minSpeedLimitSource = (DbSpeedLimitSource)min;
				pMesh->insert(pLimit->uuid, pLimit);
			}
		}
		uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
		if (s > 1)
		{
			printInfo("-----------------parseFixedSpeedLimit----------------------");
			printInfo("nextpos = %d", s);
		}

		streamReader->seek(nextPos);
	}

	void MeshReader::parseLinkSpeedLimit(DbMesh* pMesh, ByteStreamReader* streamReader)
	{
		uint64 bytesize;
		streamReader->readVarUint64(&bytesize);
		uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;

		uint8 type;
		streamReader->readUint8(&type);
		if (type == (uint8)RecordType::DB_HAD_LINK_FIXED_SPEEDLIMIT)
		{
			uint32 size;
			streamReader->readVarUint32(&size);
			for (int i = 0; i < size; i++)
			{
				DbLinkSpeedLimit* pLimit = (DbLinkSpeedLimit*)pMesh->alloc(RecordType::DB_HAD_LINK_FIXED_SPEEDLIMIT);
				streamReader->readInt64(&pLimit->uuid);
				streamReader->readInt64(&pLimit->relLinkId);
				streamReader->readVarInt32(&pLimit->featureType);
				streamReader->readVarInt32(&pLimit->direction);
				streamReader->readBool(&pLimit->isLaneDependent);

				int sc;
				streamReader->readVarInt32(&sc);
				pLimit->maxSpeedLimitClass = (MaxSpeedLimitClass)sc;

				int64 s, e;
				streamReader->readInt64(&s);
				streamReader->readInt64(&e);
				pLimit->startOffset = s * 1.0 / 1e6;
				pLimit->endOffset = e * 1.0 / 1e6;

				parsePolyline3d(pLimit->geometry.postions, streamReader);
				pMesh->insert(pLimit->uuid, pLimit);
			}
		}
		uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
		if (s > 1)
		{
			printInfo("-----------------parseLinkSpeedLimit----------------------");
			printInfo("nextpos = %d", s);
		}
		streamReader->seek(nextPos);

	}

	void MeshReader::parseIntersection(DbMesh* pMesh, ByteStreamReader* streamReader)
	{
		uint64 bytesize;
		streamReader->readVarUint64(&bytesize);
		uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;
		uint8 type;
		streamReader->readUint8(&type);
		if (type == (uint8)RecordType::DB_HAD_INTERSECTION)
		{
			uint32 size;
			streamReader->readVarUint32(&size);
			for (int i = 0; i < size; i++)
			{
				DbIntersection* pIntersection = (DbIntersection*)pMesh->alloc(RecordType::DB_HAD_INTERSECTION);
				streamReader->readInt64(&pIntersection->uuid);
				streamReader->readVarInt32(&pIntersection->intersectionType);

				uint32 gz;
				streamReader->readVarUint32(&gz);
				for (int j = 0; j < gz; j++)
				{
					int64 id;
					streamReader->readInt64(&id);
					pIntersection->refLaneGroups.push_back(id);
				}

				uint32 pz;
				streamReader->readVarUint32(&pz);
				for (int j = 0; j < pz; j++)
				{
					int64 id;
					streamReader->readInt64(&id);
					pIntersection->refLanePAs.push_back(id);
				}

				uint32 mz;
				streamReader->readVarUint32(&mz);
				for (int j = 0; j < mz; j++)
				{
					int64 id;
					streamReader->readInt64(&id);
					pIntersection->refMeshs.push_back(id);
				}

				uint32 dz;
				streamReader->readVarUint32(&dz);
				for (int j = 0; j < dz; j++)
				{
					int64 id;
					streamReader->readInt64(&id);
					pIntersection->refNodes.push_back(id);
				}

				uint32 rpz;
				streamReader->readVarUint32(&rpz);
				for (int j = 0; j < rpz; j++)
				{
					int64 id;
					streamReader->readInt64(&id);
					pIntersection->refPoints.push_back(id);
				}

				uint32 oz;
				streamReader->readVarUint32(&oz);
				for (int j = 0; j < oz; j++)
				{
					DbIntersection::OutLink link;
					streamReader->readInt64(&link.linkId);
					streamReader->readVarInt32(&link.featureType);

					int64 s, e;
					streamReader->readInt64(&s);
					link.startOffset = s * 1.0 / 1e6;
					streamReader->readInt64(&e);
					link.endOffset = e * 1.0 / 1e6;

					streamReader->readVarInt32(&link.direction);
					streamReader->readVarInt32(&link.isCenterLink);

					parsePolyline3d(link.geometry.postions, streamReader);

					pIntersection->outLinks.push_back(link);
				}

				uint32 lz;
				streamReader->readVarUint32(&lz);
				for (int j = 0; j < lz; j++)
				{
					int64 offset;
					DbIntersection::OutlinePA link;
					streamReader->readInt64(&link.linkId);
					streamReader->readInt64(&offset);
					link.offset = offset * 1.0 / 1e6;
					streamReader->readInt64(&link.position.pos.lon);
					streamReader->readInt64(&link.position.pos.lat);
					streamReader->readInt32(&link.position.z);
				}
				pMesh->insert(pIntersection->uuid, pIntersection);
			}
		}
		uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
		if (s > 1)
		{
			printInfo("-----------------parseIntersection----------------------");
			printInfo("nextpos = %d", s);
		}

		streamReader->seek(nextPos);
	}

	void MeshReader::parseTollGate(DbMesh* pMesh, ByteStreamReader* streamReader)
	{
		uint64 bytesize;
		streamReader->readVarUint64(&bytesize);
		uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;
		uint8 type;
		streamReader->readUint8(&type);
		if (type == (uint8)RecordType::DB_HAD_TOLLGATE)
		{
			uint32 size;
			streamReader->readVarUint32(&size);
			for (int i = 0; i < size; i++)
			{
				int64 offset;
				DbTollGate* pGate = (DbTollGate*)pMesh->alloc(RecordType::DB_HAD_TOLLGATE);
				streamReader->readInt64(&pGate->uuid);
				streamReader->readInt64(&pGate->paPid);
				streamReader->readVarInt32(&pGate->tollType);
				streamReader->readInt64(&pGate->linkPid);
				streamReader->readInt64(&offset);
				pGate->offset = offset * 1.0 / 1e6;
				streamReader->readInt64(&pGate->geometry.pos.lon);
				streamReader->readInt64(&pGate->geometry.pos.lat);
				streamReader->readInt32(&pGate->geometry.z);


                uint32 sz{ 0 };
                streamReader->readVarUint32(&sz);
                if (sz != 0)
                {
                    void* ss = malloc(sz + 1);
                    streamReader->readBytes(ss, sz + 1);
                    //pText->textContent = (const char*)ss;
                    std::string  tollName = (const char*)ss;
					pGate->tollName = String2WString(tollName);
                    free(ss);
                }


				uint32 lz;
				streamReader->readVarUint32(&lz);

				for (int j = 0; j < lz; j++)
				{
					DbTollLane* pTollLane = (DbTollLane*)pMesh->alloc(RecordType::DB_HAD_TOLL_LANE);
					streamReader->readVarInt32(&pTollLane->seqNum);
					streamReader->readVarInt32(&pTollLane->cardType);
					streamReader->readVarInt32(&pTollLane->payMethod);
					streamReader->readInt64(&pTollLane->laneLinkPid);
					pGate->tollLanes.push_back(pTollLane);
				}
				pMesh->insert(pGate->uuid, pGate);
			}
		}
		uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
		if (s > 1)
		{
			printInfo("-----------------parseTollGate----------------------");
			printInfo("nextpos = %d", s);
		}

		streamReader->seek(nextPos);
	}

	void MeshReader::parseTrafficLight(DbMesh* pMesh, ByteStreamReader* streamReader)
	{
		uint64 bytesize;
		streamReader->readVarUint64(&bytesize);
		uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;
		uint8 type;
		streamReader->readUint8(&type);
		if (type == (uint8)RecordType::DB_HAD_OBJECT_TRAFFIC_LIGHTS)
		{
			uint32 size;
			streamReader->readVarUint32(&size);
			for (int i = 0; i < size; i++)
			{
				DbTrafficLights* pTrafficLights = (DbTrafficLights*)pMesh->alloc(RecordType::DB_HAD_OBJECT_TRAFFIC_LIGHTS);
				streamReader->readInt64(&pTrafficLights->uuid);
				streamReader->readVarInt32(&pTrafficLights->type);
				streamReader->readVarInt32(&pTrafficLights->orientation);
				streamReader->readVarInt32(&pTrafficLights->rowNumber);
				streamReader->readVarInt32(&pTrafficLights->columnNumber);
				int32 heading;
				streamReader->readVarInt32(&heading);
				pTrafficLights->heading = heading / 1000;
				parsePolyline3d(pTrafficLights->geometry.vertexes, streamReader);
				pMesh->insert(pTrafficLights->uuid, pTrafficLights);
			}
		}
		uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
		if (s > 1)
		{
			printInfo("-----------------parseTrafficLights----------------------");
			printInfo("nextpos = %d", s);
		}

		streamReader->seek(nextPos);
	}

	void MeshReader::parsePole(DbMesh* pMesh, ByteStreamReader* streamReader)
	{
		uint64 bytesize;
		streamReader->readVarUint64(&bytesize);
		uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;
		uint8 type;
		streamReader->readUint8(&type);
		if (type == (uint8)RecordType::DB_HAD_OBJECT_POLE)
		{
			uint32 size;
			streamReader->readVarUint32(&size);
			for (int i = 0; i < size; i++)
			{
				DbPole* pPole = (DbPole*)pMesh->alloc(RecordType::DB_HAD_OBJECT_POLE);
				streamReader->readInt64(&pPole->uuid);
				streamReader->readVarInt32(&pPole->type);
				streamReader->readVarInt32(&pPole->diameterTop);
				streamReader->readVarInt32(&pPole->diameterBottom);
				parsePolyline3d(pPole->geometry.vertexes, streamReader);
				pMesh->insert(pPole->uuid, pPole);
			}
		}
		uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
		if (s > 1)
		{
			printInfo("-----------------parsePole----------------------");
			printInfo("nextpos = %d", s);
		}

		streamReader->seek(nextPos);
	}


    void MeshReader::parseLaneTurnwaiting(DbMesh* pMesh, ByteStreamReader* streamReader)
    {
        uint64 bytesize;
        streamReader->readVarUint64(&bytesize);
        uint32 nextPos = (uint32)streamReader->positionInBytes() + bytesize;
        uint8 type;
        streamReader->readUint8(&type);
        if (type == (uint8)RecordType::DB_HAD_LANE_LINK_TURNWAITING)
        {
            uint32 size;
            streamReader->readVarUint32(&size);
            for (int i = 0; i < size; i++)
            {
				DbLaneLinkTurnwaiting* laneLinkTurnwaiting = (DbLaneLinkTurnwaiting*)pMesh->alloc(RecordType::DB_HAD_LANE_LINK_TURNWAITING);
                streamReader->readInt64(&laneLinkTurnwaiting->uuid);
                streamReader->readVarInt32(&laneLinkTurnwaiting->featureType);

                int64 s, e;
                streamReader->readInt64(&s);
                streamReader->readInt64(&e);
				laneLinkTurnwaiting->startOffset = s * 1.0 /1e6;
                laneLinkTurnwaiting->endOffset = e * 1.0 /1e6;

				parsePolyline3d(laneLinkTurnwaiting->geometry.postions, streamReader);
                pMesh->insert(laneLinkTurnwaiting->uuid, laneLinkTurnwaiting);
            }
        }
        uint32 s = std::abs((int)streamReader->positionInBytes() - (int)nextPos);
        if (s > 1)
        {
            printInfo("-----------------parseLaneTurnwaiting----------------------");
            printInfo("nextpos = %d", s);
        }

        streamReader->seek(nextPos);
    }



}