#include "stdafx.h"
#include "RoutingCompiler.h"
#include "../record/DbRecord.h"
#include <iostream>
#include <fstream>
#include "boost/filesystem.hpp"
#include "boost/filesystem/path.hpp"
#include "boost/filesystem/operations.hpp"
#include "writer/had_stream_writer.h"

namespace fs = boost::filesystem;

namespace OMDB
{
	/// <summary>
	/// 需要输出给算路的要素，其对象要素高度进行了调整，只支持车道线(lane_mark_link)与车道(lane)
	/// </summary>
	struct RoutingObject
	{
		uint64 meshId;
		uint64 uuid;
		// 1 - lane
		// 2 - lane_mark_link
		unsigned char type;
		std::vector<MapPoint3D64> geometry;
	};

	void RoutingCompiler::compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile)
	{
		UNREFERENCED_PARAMETER(nearby);

		// 路口三角化结果
		std::vector<Triangle> surfaceTriangles{ compilerData.m_intersectionTriangles.begin(), compilerData.m_intersectionTriangles.end() };

		// 路面三角化结果
		surfaceTriangles.insert(surfaceTriangles.end(), compilerData.m_roadTriangles.begin(), compilerData.m_roadTriangles.end());

		// rtree
		parameters surfaceParam;
		std::vector<box_2t> surfaceTriangleBoxes;
		LinearInterpolationTriangleSurface::getTriangleBoxes(surfaceTriangles, surfaceTriangleBoxes);
		index_getter_2box surfaceOriginInd(surfaceTriangleBoxes);
		rtree_type_2box surfaceRTree2T(boost::irange<std::size_t>(0lu, surfaceTriangleBoxes.size()), surfaceParam, surfaceOriginInd);

		//rp file
		fs::path exePath = boost::filesystem::initial_path<fs::path>();
		fs::path outDir = exePath / fs::path("routing");
		if (!outDir.empty())
			fs::create_directories(outDir);

		std::vector<RoutingObject> routingLaneBoundaryObjects;
		std::vector<RoutingObject> routingLaneLinkObjects;
		std::string rpName = std::to_string(pTile->meshId) + ".rp";
		fs::path rpFile = outDir / fs::path(rpName);
		if (boost::filesystem::exists(rpFile))
			boost::filesystem::remove(rpFile);

		//push rp
		for (auto obj : pGrid->query(ElementType::HAD_LANE_BOUNDARY))
		{
			HadLaneBoundary* pLaneBoundary = (HadLaneBoundary*)obj;
			std::vector<MapPoint3D64> location = pLaneBoundary->location.vertexes;
			std::vector<MapPoint3D64> lineOnRoadSurface;
			LinearInterpolationTriangleSurface::interpolationLine(coordinatesTransform, surfaceTriangles, surfaceRTree2T, location, lineOnRoadSurface);
			routingLaneBoundaryObjects.push_back(RoutingObject{ uint64(pGrid->getId()), uint64(pLaneBoundary->originId), 2, lineOnRoadSurface });
		}
		for (auto obj : pGrid->query(ElementType::HAD_LANE))
		{
			HadLane* laneLink = (HadLane*)obj;
			std::vector<MapPoint3D64> location = laneLink->location.vertexes;
			std::vector<MapPoint3D64> lineOnRoadSurface;
			LinearInterpolationTriangleSurface::interpolationLine(coordinatesTransform, surfaceTriangles, surfaceRTree2T, location, lineOnRoadSurface);
			routingLaneLinkObjects.push_back(RoutingObject{ uint64(pGrid->getId()), uint64(laneLink->originId), 1, lineOnRoadSurface });
		}

		auto pushMapId = [&](
			uint64 id, 
			const RdsGroup* pRdsGroup,
			std::unordered_map<uint64, unsigned char>& tmpMap) {
			auto it = tmpMap.find(id);
			if (it == tmpMap.end())
			{
				unsigned char tmpValue{ 0 };
				if (pRdsGroup->isCreatedTunnel)
					tmpValue = 1;
				tmpMap.emplace(id, tmpValue);
			}
		};

		//隧道属性判断
		for (auto obj : pGrid->query(ElementType::HAD_LANE_GROUP))
		{
			HadLaneGroup* pLinkGroup = (HadLaneGroup*)obj;
			RdsGroup* pRdsGroup = queryGroup(pLinkGroup->originId, pTile);
			if (pRdsGroup)
			{
				for (auto& relLink : pLinkGroup->relLinks)
				{
					for (auto& wayType : relLink.first->wayTypes)
					{
						bool inTunnelArea = false;
						if (wayType == LINK_IS_IN_TUNNEL)
						{
							pushMapId(relLink.first->originId, pRdsGroup, isTunnelLinkMap);
						}
						else if (wayType == DB_HAD_APPLY_PA_REFERENCE)
						{
							std::vector<std::pair<double, double>> intervals;
							for (auto paValue : relLink.first->attributes)
							{
								if (paValue->name == LINK_WAY_TYPE_PA_NAME && paValue->value == LINK_IS_IN_TUNNEL)
								{
									intervals.emplace_back(std::make_pair(paValue->start, paValue->end));
								}
							}
							if (!intervals.empty())
							{
								// 排序区间
								std::sort(intervals.begin(), intervals.end(), [](const std::pair<double, double>& s1, const std::pair<double, double>& s2)
									{
										if (s1.first != s2.first)
											return s1.first < s2.first;
										else
											return s1.second < s2.second;
									});

								// 合并重叠的区间
								std::vector<std::pair<double, double>> merged;
								std::pair<double, double> curRange = intervals.front();
								for (int i = 1; i < intervals.size(); i++)
								{
									if (curRange.second + 0.01 >= intervals[i].first)
									{
										curRange.second = max(curRange.second + 0.01, intervals[i].second);
									}
									else
									{
										merged.push_back(curRange);
										curRange = intervals[i];
									}
								}
								merged.push_back(curRange);

								for (auto& range : merged)
								{
									if (range.first <= relLink.second.start + 0.01 && relLink.second.end <= range.second + 0.01) {
										inTunnelArea = true;
										break;
									}
								}

							}

							for (auto paValue : relLink.first->attributes)
							{
								if (paValue->name == LINK_WAY_TYPE_PA_NAME && paValue->value == LINK_IS_IN_TUNNEL)
								{
									if (inTunnelArea)
									{
										pushMapId(paValue->originId, pRdsGroup, isTunnelLinkPAMap);
									}
								}
							}
						}
					}
				}
			}
		}

		//高架桥上的压低相机属性，只有非高速才输出压盖属性
		for (auto obj : pGrid->query(ElementType::HAD_LINK))
		{
			HadLink* pLink = (HadLink*)obj;
			if (!isProDataLevel(pLink->groups))
			{
				if (checkOverlayOnLink(pLink))
				{
					auto it = isLowerCameraLinkMap.find(pLink->originId);
					if (it == isLowerCameraLinkMap.end())
					{
						isLowerCameraLinkMap.emplace(pLink->originId, 1);
					}
				}
			}
		}

		//写lane到StreamWriter
		HadStreamWriter rootWriter;
		//lane mark
		rootWriter.writeUint8(2);
		rootWriter.writeVarInt64(routingLaneBoundaryObjects.size());
		for (auto& routing : routingLaneBoundaryObjects)
		{
			rootWriter.writeUint64(routing.uuid);
			std::vector<MapPoint3D64> tmpPoints;
			tmpPoints.resize(routing.geometry.size());
			memcpy(tmpPoints.data(), routing.geometry.data(), sizeof(Point3d) * routing.geometry.size());
			rootWriter.writePolyline3D(tmpPoints.data(), tmpPoints.size());
		}

		//lane link
		rootWriter.writeUint8(1);
		rootWriter.writeVarInt64(routingLaneLinkObjects.size());
		for (auto& routing : routingLaneLinkObjects)
		{
			rootWriter.writeUint64(routing.uuid);
			std::vector<MapPoint3D64> tmpPoints;
			tmpPoints.resize(routing.geometry.size());
			memcpy(tmpPoints.data(), routing.geometry.data(), sizeof(Point3d) * routing.geometry.size());
			rootWriter.writePolyline3D(tmpPoints.data(), tmpPoints.size());
		}

		////link id
		rootWriter.writeUint8(3);
		rootWriter.writeVarInt64(isTunnelLinkMap.size());
		for (auto& tunnelInfo : isTunnelLinkMap)
		{
			rootWriter.writeUint64(tunnelInfo.first);
			rootWriter.writeUint8(tunnelInfo.second);
		}

		////link pa id
		rootWriter.writeUint8(4);
		rootWriter.writeVarInt64(isTunnelLinkPAMap.size());
		for (auto& tunnelInfo : isTunnelLinkPAMap)
		{
			rootWriter.writeUint64(tunnelInfo.first);
			rootWriter.writeUint8(tunnelInfo.second);
		}

		//// lower camera link id
		rootWriter.writeUint8(5);
		rootWriter.writeVarInt64(isLowerCameraLinkMap.size());
		for (auto& tmpInfo : isLowerCameraLinkMap)
		{
			rootWriter.writeUint64(tmpInfo.first);
			rootWriter.writeUint8(tmpInfo.second);
		}

		//写入rp文件并保存
		std::ofstream rpOutStream(rpFile.string(), std::ios::trunc | std::ios::binary);
		if (rpOutStream.fail())
			return;
		unsigned char* pData = const_cast<unsigned char*>(rootWriter.buffer());
		unsigned long long size = rootWriter.lengthInBytes();
		rpOutStream.write(reinterpret_cast<char*>(pData), size);
		rpOutStream.close();

#ifdef RP_FILE_READ_DEBUG
		//打开文件， 获取文件大小，读取文件
		std::ifstream rpInStream(rpFile.string(), std::ios::binary);
		if (!rpInStream.is_open())
			return;
		rpInStream.seekg(0, std::ios::end);
		unsigned long long rpFileSize = rpInStream.tellg();
		rpInStream.seekg(0, std::ios::beg);
		char* rpBufferData = new char[rpFileSize];
		rpInStream.read(rpBufferData, rpFileSize);
		rpInStream.close();
		void* rpData = reinterpret_cast<void*>(rpBufferData);

		//将二进制格式转化为车道信息数据
		ByteStreamReader rootReader(rpData, rpFileSize);
		uint8 laneBoundaryEntityType;
		rootReader.readUint8(&laneBoundaryEntityType);
		if (laneBoundaryEntityType != 2)
			return;
		int64 laneBoundarySize;
		rootReader.readVarInt64(&laneBoundarySize);
		for (size_t i = 0; i < laneBoundarySize; i++)
		{
			uint64 lineId;
			rootReader.readUint64(&lineId);			//id
			uint32 pointCount;
			rootReader.readVarUint32(&pointCount);		//第一个点
			if (pointCount == 0)
				return;
			std::vector<MapPoint3D64> points(pointCount);
			MapPoint3D64 firstPoint;
			uint64 x;
			uint64 y;
			int32 z;
			rootReader.readUintN64(&x, 36);
			rootReader.readUintN64(&y, 36);
			rootReader.readVarInt32(&z);
			firstPoint.pos.lon = int64(x);
			firstPoint.pos.lat = int64(y);
			firstPoint.z = z;
			points[0] = firstPoint;

			if (pointCount == 1)
				return;
			uint32 bitWidth[3];
			rootReader.readUintN(&bitWidth[0], 6);		 //x的bit最大宽度
			rootReader.readUintN(&bitWidth[1], 6);		 //y的bit最大宽度
			rootReader.readUintN(&bitWidth[2], 5);		 //z的bit最大宽度
			for (size_t j = 0; j < 3; j++)
				bitWidth[j] += 1;

			int32 d[3];
			for (size_t j = 1; j < pointCount; j++)		 //其他点
			{
				MapPoint3D64 point;
				for (size_t k = 0; k < 3; k++)
					rootReader.readIntN(&d[k], bitWidth[k]);
				point.pos.lon = points[j - 1].pos.lon + int64(d[0]);
				point.pos.lat = points[j - 1].pos.lat + int64(d[1]);
				point.z = points[j - 1].z + d[2];
				points[j] = point;
			}
		}

		uint8 laneLinkEntityType;
		rootReader.readUint8(&laneLinkEntityType);
		if (laneLinkEntityType != 1)
			return;
		int64 laneLinkSize;
		rootReader.readVarInt64(&laneLinkSize);
		for (size_t i = 0; i < laneLinkSize; i++)
		{
			uint64 lineId;
			rootReader.readUint64(&lineId);			//id
			uint32 pointCount;
			rootReader.readVarUint32(&pointCount);		//第一个点
			if (pointCount == 0)
				return;
			std::vector<MapPoint3D64> points(pointCount);
			MapPoint3D64 firstPoint;
			uint64 x;
			uint64 y;
			int32 z;
			rootReader.readUintN64(&x, 36);
			rootReader.readUintN64(&y, 36);
			rootReader.readVarInt32(&z);
			firstPoint.pos.lon = int64(x);
			firstPoint.pos.lat = int64(y);
			firstPoint.z = z;
			points[0] = firstPoint;

			if (pointCount == 1)
				return;
			uint32 bitWidth[3];
			rootReader.readUintN(&bitWidth[0], 6);		 //x的bit最大宽度
			rootReader.readUintN(&bitWidth[1], 6);		 //y的bit最大宽度
			rootReader.readUintN(&bitWidth[2], 5);		 //z的bit最大宽度
			for (size_t j = 0; j < 3; j++)
				bitWidth[j] += 1;

			int32 d[3];
			for (size_t j = 1; j < pointCount; j++)		 //其他点
			{
				MapPoint3D64 point;
				for (size_t k = 0; k < 3; k++)
					rootReader.readIntN(&d[k], bitWidth[k]);
				point.pos.lon = points[j - 1].pos.lon + int64(d[0]);
				point.pos.lat = points[j - 1].pos.lat + int64(d[1]);
				point.z = points[j - 1].z + d[2];
				points[j] = point;
			}
		}

		uint8 tunnelLinkEntityType;
		rootReader.readUint8(&tunnelLinkEntityType);
		if (tunnelLinkEntityType != 3)
			return;
		int64 tunnelLinkSize;
		rootReader.readVarInt64(&tunnelLinkSize);
		for (size_t i = 0; i < tunnelLinkSize; i++)
		{
			uint64 linkId;
			rootReader.readUint64(&linkId);
			uint8 isTunnelObject;
			rootReader.readUint8(&isTunnelObject);
		}

		uint8 tunnelLinkPaEntityType;
		rootReader.readUint8(&tunnelLinkPaEntityType);
		if (tunnelLinkPaEntityType != 4)
			return;
		int64 tunnelLinkPaSize;
		rootReader.readVarInt64(&tunnelLinkPaSize);
		for (size_t i = 0; i < tunnelLinkPaSize; i++)
		{
			uint64 linkPaId;
			rootReader.readUint64(&linkPaId);
			uint8 isTunnelObject;
			rootReader.readUint8(&isTunnelObject);
		}

		delete rpBufferData;
#endif
	}
}

