#include "stdafx.h"
#include "ZGenerator.h"
#include "algorithm/map_point_math.h"
#include "algorithm/geometry_utils.h"
#include "core/framework/RTreeSeacher.h"

#include <algorithm>
#include <utility>

namespace OMDB
{
	static const double ADJUST_ANGLE = tan(2 * MATH_DEGREE2RADIAN_D);// 调节的角度，表示前进单位水平方向距离带来的竖直方向距离变化
	static const double ADJUST_ANGLE_PEDESTRIAN = tan(10 * MATH_DEGREE2RADIAN_D);// 调节的角度
	static const double MIN_HEIGHT_DIFFERENCE = 100.0;// 厘米，与OMDB中常用的MapPoint3D64相同
	static const double ADJUST_HEIGHT_SPACING = 450.0;
	static const double ADJUST_HEIGHT_SPACING_PEDESTRIAN = 250.0;
	static const int CHECK_COUNT_LIMIT = 10;

	/**
	 * @brief 判断从角1转向角2是否需要左转或者右转。角1 & 角2 均为弧度。
	 * @return true -> 左转, false -> 右转
	*/
	static bool angleTurnLeft(double angle1, double angle2)
	{
		std::array<double, 2> target{ std::cos(angle2), std::sin(angle2) };
		std::array<double, 2> heading{ std::cos(angle1), std::sin(angle1) };

		double cross = target[0] * heading[1] - target[1] * heading[0];

		return cross < 0.0;
	}

	/**
	 * @brief 判断两个角度是否同向。角1 & 角2 均为弧度。
	 * @param angle1 radians
	 * @param angle2 radians
	 * @return
	*/
	static bool angleIsTheSameDirection(double angle1, double angle2)
	{
		double x1 = std::cos(angle1), y1 = std::sin(angle1);
		double x2 = std::cos(angle2), y2 = std::sin(angle2);

		double dot = x1 * x2 + y1 * y2;
		double  angle = std::acos(dot);
		return angle < (90.0 * MATH_DEGREE2RADIAN_D);
	}

	struct MeshIDTriple
	{
		int level;
		int row;
		int col;

		bool operator==(const MeshIDTriple& other) const
		{
			return this->level == other.level
				&& this->row == other.row
				&& this->col == other.col;
		}

		bool operator!=(const MeshIDTriple& other) const
		{
			return this->level != other.level
				|| this->row != other.row
				|| this->col != other.col;
		}

		bool operator<(const MeshIDTriple& other) const
		{
			if (this->level != other.level)
				return this->level < other.level;
			if (this->row != other.row)
				return this->row < other.row;
			return this->col < other.col;
		}

		static MeshIDTriple fromMeshID(int32 meshID)
		{
			// 瓦片行列号在wgs平面坐标系下从下到上、从左到右递增。
			NdsGridId ndsID = MeshId_toNdsGridId(meshID);
			MeshIDTriple t;
			NdsGridId_toRowCol(ndsID, &t.level, &t.row, &t.col);
			return t;
		}

	};

	/**
	 * @brief 对link对象上的所有z level点按照其shpSeqNum排序
	 */
	static void sortLinkZLevelByShpSeqNum(DbLink* pLink)
	{
		auto linkid = pLink->uuid;
		std::sort(pLink->zLevels.begin(), pLink->zLevels.end(),
			[linkid](auto& first, auto& second)->bool {
				return first.shpSeqNum < second.shpSeqNum;
			});
	}

	/**
	 * @brief 检查同一位置的Z Level记录的level随索引递增，不允许出现相同或者减少的情况。
	 *        检查 同一位置的Z Level记录数量大于2。
	 * @param relLinks
	 * @return
	*/
	static bool checkZLevelAscending(const std::vector<DbZLevel::DbRelLink>& relLinks)
	{
		if (relLinks.size() < 2)
			return false;

		for (std::size_t i = 1; i < relLinks.size(); ++i)
		{
			if (relLinks[i].zLevel <= relLinks[i - 1].zLevel)
			{
				return false;
			}
		}

		return true;
	}


	/**
	 * @brief 无效点定义：link对象上的所有z level对象中shpSeqNum相同的点（不应出现这些点）
	 * @returns link对象上的无效z level对象的索引，注意索引保证从大到小顺序，所以后面可以安全移除vector中元素。
	 */
	static std::set<DbZLevel*> findInvalidZLevelOnLink(DbLink* pLink)
	{
		std::set<DbZLevel*>linkInvalidZLevels;
		for (int i = 0, maxI = pLink->zLevels.size(); i < maxI; ++i)
		{
			DbZLevel::DbRelLink& curlinkZLevel = pLink->zLevels[i];

			if (curlinkZLevel.shpSeqNum >= pLink->geometry.vertexes.size())
				linkInvalidZLevels.insert(curlinkZLevel.dbZLevel);

			if (i > 0)
			{
				DbZLevel::DbRelLink& prevLinkZLevel = pLink->zLevels[i - 1];
				if (prevLinkZLevel.shpSeqNum == curlinkZLevel.shpSeqNum)
				{
					linkInvalidZLevels.insert(prevLinkZLevel.dbZLevel);
					linkInvalidZLevels.insert(curlinkZLevel.dbZLevel);
				}
			}
		}
		return linkInvalidZLevels;
	}

	std::vector<ZLevelLinks> ZGenerator::readZLevelLinks(DbMesh* pMesh)
	{
		// ZLEVEL
		std::vector<ZLevelLinks> zLevelLinks;
		for (auto& zl : pMesh->query(RecordType::DB_HAD_ZLEVEL)) {
			DbZLevel* pZLevel = (DbZLevel*)zl;

			if (pZLevel->relLinks.empty()) continue;

			std::vector<DirLink> links;
			for (auto& relLink : pZLevel->relLinks) {
				DbLink* pLink = (DbLink*)pMesh->query(relLink.relLinkid, RecordType::DB_HAD_LINK);
				links.emplace_back(pLink, true, getOrAddLinkGeomCopy(*pLink));
			}
			zLevelLinks.push_back(ZLevelLinks{ pZLevel, std::move(links) });
		}

		return zLevelLinks;
	}

	void ZGenerator::generate(DbMesh* const pMesh)
	{
		// 相同Z Level ID的记录按照z level从小到大排序
		for (auto& zl : pMesh->query(RecordType::DB_HAD_ZLEVEL)) {
			DbZLevel* pZLevel = (DbZLevel*)zl;
			std::sort(pZLevel->relLinks.begin(), pZLevel->relLinks.end(),
				[](const auto& first, const auto& second)->bool {
					return first.zLevel < second.zLevel;
				});
		}

		// 填充DbLink的Z Level数组
		for (auto& zl : pMesh->query(RecordType::DB_HAD_ZLEVEL))
		{
			DbZLevel* pZLevel = (DbZLevel*)zl;
			for (auto& relLink : pZLevel->relLinks)
			{
				DbLink* pLink = (DbLink*)pMesh->query(relLink.relLinkid, RecordType::DB_HAD_LINK);
				pLink->zLevels.push_back(relLink);
			}
		}

		// link上的z level按照shpSeqNum排序
		for (auto& hl : pMesh->query(RecordType::DB_HAD_LINK))
		{
			DbLink* pLink = (DbLink*)hl;
			sortLinkZLevelByShpSeqNum(pLink);
		}

		std::set<DbZLevel*> deprecatedZLevels;

		// 检查项一：检查同一位置的Z Level记录的level随索引递增，不允许出现相同或者减少的情况。同一位置的Z Level记录数量需大于2，否则没意义。
		for (auto& zl : pMesh->query(RecordType::DB_HAD_ZLEVEL))
		{
			DbZLevel* zlevel = (DbZLevel*)zl;
			if (!checkZLevelAscending(zlevel->relLinks))
				deprecatedZLevels.insert(zlevel);
		}
		// 检查项二：检查Link上的Z Level记录的shpSeqNum合法，且link上不存在两个seqNum相同的Z Level记录
		for (auto& hl : pMesh->query(RecordType::DB_HAD_LINK)) {
			DbLink* pLink = (DbLink*)hl;
			std::set<DbZLevel*> invalidZLevels = findInvalidZLevelOnLink(pLink);
			if (!invalidZLevels.empty())
				deprecatedZLevels.insert(invalidZLevels.begin(), invalidZLevels.end());
		}
		// 检查项三：若Link的direct属性为PA(-99)，则link上的所有ZLevel记录都要删除
		for (auto& hl : pMesh->query(RecordType::DB_HAD_LINK)) {
			DbLink* pLink = (DbLink*)hl;
			if (pLink->direct == -99)
			{
				for (auto& relLink : pLink->zLevels)
				{
					deprecatedZLevels.insert(relLink.dbZLevel);
				}
			}
		}
		// 检查项四：若Zlevel中出现不会编译输出的道路（如步行路），则移除该ZLevel
		for (auto& hl : pMesh->query(RecordType::DB_HAD_LINK))
		{
			DbLink* pLink = (DbLink*)hl;
			if (pLink->lanePas.empty())
			{
				for (auto& relLink : pLink->zLevels)
				{
					deprecatedZLevels.insert(relLink.dbZLevel);
				}
			}
		}

		// 移除ZLevel
		for (DbZLevel* deprecatedZLevel : deprecatedZLevels)
		{
			for (auto& relLink : deprecatedZLevel->relLinks)
			{
				DbLink* pLink = (DbLink*)pMesh->query(relLink.relLinkid, RecordType::DB_HAD_LINK);
				while (true)
				{
					auto iterator = std::find_if(pLink->zLevels.begin(), pLink->zLevels.end(), [&deprecatedZLevel](DbZLevel::DbRelLink& i) {return i.dbZLevel->uuid == deprecatedZLevel->uuid; });
					if (iterator != pLink->zLevels.end())
						pLink->zLevels.erase(iterator);
					else
						break;
				}
			}
			deprecatedZLevel->relLinks.clear();
		}
	}

	void ZGenerator::generateRelation(DbMesh* const pMesh, std::vector<DbMesh*>* nearby)
	{
		m_currentMeshID = pMesh->getId();

		// 全部网格，网格顺序不保证
		std::vector<DbMesh*> totalMesh{ pMesh };

		std::vector<ZLevelLinks> zLevelLinks;
		{
			zLevelLinks = readZLevelLinks(pMesh);
			if (nearby && !nearby->empty())
			{
				totalMesh.insert(totalMesh.end(), nearby->begin(), nearby->end());

				for (auto nearbyMesh : *nearby)
				{
					auto nearbyZLevelLinks = readZLevelLinks(nearbyMesh);
					zLevelLinks.insert(zLevelLinks.end(), nearbyZLevelLinks.begin(), nearbyZLevelLinks.end());
				}
			}
		}

		{
			struct MeshInfo { MeshIDTriple id; MapPoint64 center; };
			std::map<DbMesh*, MeshInfo> meshInfoMap;
			for (std::size_t i = 0; i < totalMesh.size(); ++i)
			{
				DbMesh* mesh = totalMesh[i];
				BoundingBox2d bbox = mesh->getBoundingbox2d();
				MeshInfo info{
					MeshIDTriple::fromMeshID(mesh->getId()),
					MapPoint64{static_cast<int64>((bbox.min.lon + bbox.max.lon) * 0.5), static_cast<int64>((bbox.min.lat + bbox.max.lat) * 0.5)}
				};
				meshInfoMap[mesh] = info;
			}

			// zLevelLinks得每一项按照瓦片下上到上，从左到右排序(nearby顺序就是如此)。瓦片内Z Level按照与瓦片中心点距离按照从小到大排序。以保证ZLevel处理顺序一样。
			std::sort(zLevelLinks.begin(), zLevelLinks.end(), [&meshInfoMap](ZLevelLinks& l, ZLevelLinks& r)
				{
					auto leftInfo = meshInfoMap[l.zlevel->owner];
					auto rightInfo = meshInfoMap[r.zlevel->owner];
					if (leftInfo.id != rightInfo.id)
						return leftInfo.id < rightInfo.id;

					// 同网格内的ZLevel
					MapPoint64 leftZlevelPos = l.zlevel->relLinks.front().geometry.pos;
					MapPoint64 rightZlevelPos = r.zlevel->relLinks.front().geometry.pos;
					return MapPoint64::geodistance(leftZlevelPos, leftInfo.center) < MapPoint64::geodistance(rightZlevelPos, leftInfo.center);
				}
			);
		}


		std::vector<DividedRoad> dividedRoads = DividedRoadSearcher{ pMesh, nearby ? *nearby : std::vector<DbMesh*>{} }.searchZGen();

		this->generateHeight(zLevelLinks, dividedRoads);

		// ZGen运行完成后恢复当前瓦片内link高程
		for (auto& hl : pMesh->query(RecordType::DB_HAD_LINK)) {
			DbLink* pLink = (DbLink*)hl;
			LineString3d* linestring = getLinkGeomCopyOrNull(pLink->uuid);
			if (linestring)
				pLink->geometry = *linestring;
		}

		// IMPORTANT
		m_linkGeometryOwner.clear();

		// 给当前Mesh赋值上下行道路
		{
			std::vector<std::array<std::vector<std::pair<DbLink*, bool>>, 2>> meshDividedRoads;
			for (DividedRoad& dividedRoad : dividedRoads)
			{
				std::array<std::vector<std::pair<DbLink*, bool>>, 2> meshDividedRoad;
				for (auto& portion : dividedRoad.leftPath.portions())
				{
					auto dirLink = std::make_pair(portion.dirLink.getLinkDangerous(), portion.dirLink.forward());
					meshDividedRoad[0].push_back(dirLink);
				}
				for (auto& portion : dividedRoad.rightPath.portions())
				{
					auto dirLink = std::make_pair(portion.dirLink.getLinkDangerous(), portion.dirLink.forward());
					meshDividedRoad[1].push_back(dirLink);
				}

				meshDividedRoads.push_back(meshDividedRoad);
			}

			pMesh->setDividedRoads(std::move(meshDividedRoads));
		}
	}

	LineString3d& ZGenerator::getOrAddLinkGeomCopy(DbLink& link)
	{
		auto it = m_linkGeometryOwner.find(link.uuid);
		if (it == m_linkGeometryOwner.end())
		{
			auto geom = std::make_unique<LineString3d>(link.geometry);
			LineString3d& lineString = *geom;
			for (auto& vertex : lineString.vertexes)
				vertex.z = 0;
			m_linkGeometryOwner.emplace(link.uuid, std::move(geom));
			return lineString;
		}
		else
		{
			return *it->second;
		}
	}

	LineString3d* ZGenerator::getLinkGeomCopyOrNull(int64 linkId)
	{
		auto it = m_linkGeometryOwner.find(linkId);
		if (it == m_linkGeometryOwner.end())
		{
			return nullptr;
		}
		else
		{
			return it->second.get();
		}
	}

	std::vector<DirLink> ZGenerator::getEndDirectOutlinksExceptSelf(DirLink& link)
	{
		std::vector<DirLink> outlinks = link.getEndDirectOutlinksExceptSelf();
		for (DirLink& outlink : outlinks)
		{
			outlink.setGeoemtry(&this->getOrAddLinkGeomCopy(*outlink.getLinkDangerous()));
		}
		return outlinks;
	}

	void ZGenerator::generateHeight(std::vector<ZLevelLinks>& zLevelLinksArray, std::vector<DividedRoad>& dividedRoads)
	{
		int checkCount = 0;
		for (int i = 0; i < CHECK_COUNT_LIMIT; i++)
			if (!checkAndAdjustZLevel(zLevelLinksArray)) {
				checkCount = i;
				break;
			}

		// 调整两个ZLevel点之间的道路的高程
		processRoadBetweenZLevels(zLevelLinksArray);

		// 使得上下行道路高程平齐
		for (DividedRoad& dividedRoad : dividedRoads)
		{
			adjustDividedRoadHeight(dividedRoad);
		}
	}

	bool ZGenerator::checkAndAdjustZLevel(std::vector<ZLevelLinks>& zLevelLinksArray)
	{
		bool hasAdjusted = false;
		for (auto& zLevelLinks : zLevelLinksArray) {
			bool hasIntersectingAdjusted = checkIntersectingInfo(zLevelLinks);
			hasAdjusted = (hasAdjusted || hasIntersectingAdjusted);
		}

		return hasAdjusted;
	}

	bool ZGenerator::checkIntersectingInfo(ZLevelLinks& zLevelLinks)
	{
		bool hasAdjusted = false;
		for (size_t i = 1; i < zLevelLinks.links.size(); i++) {
			DirLink& lowerLink = zLevelLinks.links[i - 1];
			DbZLevel::DbRelLink& lowerLinkZLevel = zLevelLinks.zlevel->relLinks[i - 1];
			double lowerLinkHeight = lowerLink.getPointRefNoDir(lowerLinkZLevel.shpSeqNum).z;

			DirLink& upperLink = zLevelLinks.links[i];
			DbZLevel::DbRelLink& upperLinkZLevel = zLevelLinks.zlevel->relLinks[i];
			double upperLinkHeight = upperLink.getPointRefNoDir(upperLinkZLevel.shpSeqNum).z;


			bool havePedestrian = isPedestrian(upperLink) || isPedestrian(lowerLink);
			double adjustHeight = havePedestrian ? ADJUST_HEIGHT_SPACING_PEDESTRIAN : ADJUST_HEIGHT_SPACING;
			double slope = havePedestrian ? ADJUST_ANGLE_PEDESTRIAN : ADJUST_ANGLE;

			if (upperLinkHeight <= lowerLinkHeight + MIN_HEIGHT_DIFFERENCE)
			{
				adjustPointHeight(upperLink, upperLinkZLevel.shpSeqNum, upperLinkHeight + adjustHeight, slope);
				hasAdjusted = true;
			}
		}

		return hasAdjusted;
	}

	void ZGenerator::adjustViaductRoad(const std::vector<DirLink>& links)
	{
		for (auto link : links)
		{
			// TODO: 判断link是否具有高架属性
			// adjustViaductRoad(link);
		}
	}

	// TODO: 抬高高架路逻辑有待商榷
	void ZGenerator::adjustViaductRoad(DirLink& link)
	{
		int32 minHeight = INT_MAX;
		std::size_t minHeightVertexIndex = SIZE_MAX;
		for (std::size_t i = 0; i < link.geometry().vertexes.size(); ++i)
		{
			const MapPoint3D64& vertex = link.geometry().vertexes[i];
			if (vertex.z < ADJUST_HEIGHT_SPACING && vertex.z < minHeight)
			{
				minHeight = vertex.z;
				minHeightVertexIndex = i;
			}
		}

		if (minHeightVertexIndex != SIZE_MAX)
		{
			adjustPointHeight(link, static_cast<int>(minHeightVertexIndex), ADJUST_HEIGHT_SPACING, ADJUST_ANGLE);
		}
	}

	void ZGenerator::adjustPointHeight(DirLink& link, int idx, double targetZ, double slope)
	{
		double influenceLength = 0;
		influenceLength = abs(link.getPointRefNoDir(idx).z - targetZ) / slope;
		influenceLength /= 100.0; // 转换成米，因为adjustZ中需要influenceLength单位为米

		AdjustZVertexAccessor accessor{ targetZ, targetZ - link.getPointRefNoDir(idx).z,influenceLength };

		// 反向直到末端
		DirLink revLink = link.getReverseDirLink();
		revLink.accessVerticesToEnd(idx, accessor);
		double remainingBackward = accessor.restLength();

		// 正向直到末端
		link.accessVerticesToEnd(idx, accessor);
		double remainingForward = accessor.restLength();

		// construct queue and visit
		AdjustHeightQueue adjustQueue;

		// the end of link
		if (remainingForward > 0.0)
		{
			std::vector<DirLink> outlinks = getEndDirectOutlinksExceptSelf(link);
			for (DirLink& outlink : outlinks)
			{
				AdjustHeightInfo info;
				info.dlink = outlink;
				info.lastHeight = link.getEndHeight();
				info.length = remainingForward;
				info.priority = outlink.calLength();
				adjustQueue.push(info);
			}
		}

		// the start of link
		if (remainingBackward > 0.0)
		{
			std::vector<DirLink> outlinks = getEndDirectOutlinksExceptSelf(revLink);
			for (DirLink& outlink : outlinks)
			{
				AdjustHeightInfo info;
				info.dlink = outlink;
				info.lastHeight = link.getStartHeight();
				info.length = remainingBackward;
				info.priority = outlink.calLength();
				adjustQueue.push(info);
			}
		}

		std::set<int64> adjustedLinks{ link.uuid() };

		bfs(adjustQueue, adjustedLinks);
	}

	bool ZGenerator::checkDividedRoadNotModified(DividedRoad& dividedRoad)
	{
		// 警告，此时dividedRoad中的DirLink持有DbLink原始geometry。
		// 判断DividedRoad是否有一部分的DbLink是否存在本地geometry备份，如果没有，则表明该上下线高程压根没有修改过，所以简单忽略。
		for (auto& portion : dividedRoad.leftPath.portions())
			if (getLinkGeomCopyOrNull(portion.dirLink.uuid()))
				return false;

		for (auto& portion : dividedRoad.rightPath.portions())
			if (getLinkGeomCopyOrNull(portion.dirLink.uuid()))
				return false;

		return true;
	}

	void ZGenerator::adjustDividedRoadHeight(DividedRoad& dividedRoad)
	{
		if (checkDividedRoadNotModified(dividedRoad))
			return;

		// 更新DividedRoad中的DirLink持有的geometry为复制的，检查左右上下线累计高程是否都为0，若为0，则表明上下线高程也没修改过
		for (auto& portion : dividedRoad.leftPath.portions())
		{
			LineString3d* linestring = &getOrAddLinkGeomCopy(*portion.dirLink.getLinkDangerous());
			portion.dirLink.setGeoemtry(linestring);
		}
		for (auto& portion : dividedRoad.rightPath.portions())
		{
			LineString3d* linestring = &getOrAddLinkGeomCopy(*portion.dirLink.getLinkDangerous());
			portion.dirLink.setGeoemtry(linestring);
		}

		int32 leftHeightSum{ 0 };
		std::vector<MapPoint3D64Ref> leftVerteices;
		{
			DirLinkPathVertexRefCollector collector;
			dividedRoad.leftPath.accessVertices(collector);
			leftHeightSum = collector.heightSum();
			leftVerteices = collector.verticesRef();
		}
		int32 rightHeightSum{ 0 };
		std::vector<MapPoint3D64Ref> rightVerteices;
		{
			DirLinkPathVertexRefCollector collector;
			dividedRoad.rightPath.accessVertices(collector);
			rightHeightSum = collector.heightSum();
			rightVerteices = collector.verticesRef();
		}

		// 两个上下行道路高程都为0，没必要做后续的调整
		if (leftHeightSum + rightHeightSum == 0)
			return;

		// 左右道路边线需要开始处理顶点的范围，[From, To]
		std::size_t rightRangeFrom = -1, rightRangeTo = -1;
		std::size_t leftRangeFrom = -1, leftRangeTo = -1;

		// 末端对齐程度参数
		const double maxAngle = 30.0 * MATH_DEGREE2RADIAN_D;
		const double ratioBelowOne = std::cos(maxAngle);
		const double ratioAboveOne = 1.0 / ratioBelowOne;

		// 起点端
		{
			const MapPoint3D64Ref leftEndPoint = leftVerteices.front();
			const MapPoint3D64Ref rightEndPoint = rightVerteices.front();

			if (leftEndPoint.get() == rightEndPoint.get())
			{
				// 特殊情况：上下行道路起点相同
				rightRangeFrom = leftRangeFrom = 0;
			}
			else
			{
				auto leftEndPointNearestPoint = findDividedRoadInsertPosition(leftEndPoint.get(), rightVerteices);
				auto rightEndPointNearestPoint = findDividedRoadInsertPosition(rightEndPoint.get(), leftVerteices);

				double distanceLeftToRight = MapPoint64::geodistance(leftEndPoint.get().pos, leftEndPointNearestPoint.nearestPoint.pos);
				double distanceRightToLeft = MapPoint64::geodistance(rightEndPoint.get().pos, rightEndPointNearestPoint.nearestPoint.pos);
				double r = distanceLeftToRight / distanceRightToLeft;

				if (r < ratioBelowOne)
				{
					leftRangeFrom = 0;
					rightRangeFrom = leftEndPointNearestPoint.insertPos;
				}
				else if (r > ratioAboveOne)
				{
					rightRangeFrom = 0;
					leftRangeFrom = rightEndPointNearestPoint.insertPos;
				}
				else
				{
					rightRangeFrom = leftRangeFrom = 0;
				}
			}
		}
		// 终点端
		{
			const MapPoint3D64Ref leftEndPoint = leftVerteices.back();
			const MapPoint3D64Ref rightEndPoint = rightVerteices.back();

			if (leftEndPoint.get() == rightEndPoint.get())
			{
				// 特殊情况：上下行道路起点相同
				leftRangeTo = leftVerteices.size() - 1;
				rightRangeTo = rightVerteices.size() - 1;
			}
			else
			{
				auto leftEndPointNearestPoint = findDividedRoadInsertPosition(leftEndPoint.get(), rightVerteices, false, 0);
				auto rightEndPointNearestPoint = findDividedRoadInsertPosition(rightEndPoint.get(), leftVerteices, false, 0);

				double distanceLeftToRight = MapPoint64::geodistance(leftEndPoint.get().pos, leftEndPointNearestPoint.nearestPoint.pos);
				double distanceRightToLeft = MapPoint64::geodistance(rightEndPoint.get().pos, rightEndPointNearestPoint.nearestPoint.pos);
				double r = distanceLeftToRight / distanceRightToLeft;

				if (r < ratioBelowOne)
				{
					leftRangeTo = leftVerteices.size() - 1;
					rightRangeTo = leftEndPointNearestPoint.insertPos > 0 ? leftEndPointNearestPoint.insertPos - 1 : 0;
				}
				else if (r > ratioAboveOne)
				{
					rightRangeTo = rightVerteices.size() - 1;
					leftRangeTo = rightEndPointNearestPoint.insertPos > 0 ? rightEndPointNearestPoint.insertPos - 1 : 0;
				}
				else
				{
					leftRangeTo = leftVerteices.size() - 1;
					rightRangeTo = rightVerteices.size() - 1;
				}
			}
		}

		// 左右侧需要调整高程的部分分别向对侧计算最邻近点，选取当前点和对侧最邻近点最高高程作为当前点高程
		{
			std::vector<MapPoint3D64> leftVerticesCopy;
			for (auto& pt : leftVerteices) { leftVerticesCopy.push_back(pt.get()); }
			std::vector<MapPoint3D64> rightVerticesCopy;
			for (auto& pt : rightVerteices) { rightVerticesCopy.push_back(pt.get()); }
		{
			std::size_t lastSegmentIndex = 0;
			for (std::size_t i = rightRangeFrom; i <= rightRangeTo; ++i)
			{
				MapPoint3D64Ref& rightPt = rightVerteices[i];
				auto leftNearestPoint = findDividedRoadInsertPosition(rightPt.get(), leftVerteices, true, lastSegmentIndex);
				lastSegmentIndex = leftNearestPoint.lastSegmentIndex;
					if (leftNearestPoint.nearestPoint.z > rightPt.get().z)
						rightVerticesCopy[i].z = leftNearestPoint.nearestPoint.z; // Note: 点高程写到VertexCopy
			}
			// TODO: 调整右侧剩余部分道路
		}
		{
			std::size_t lastSegmentIndex = 0;
			for (std::size_t i = leftRangeFrom; i <= leftRangeTo; ++i)
			{
				MapPoint3D64Ref& leftPt = leftVerteices[i];
				auto rightNearestPoint = findDividedRoadInsertPosition(leftPt.get(), rightVerteices, true, lastSegmentIndex);
				lastSegmentIndex = rightNearestPoint.lastSegmentIndex;
					if (rightNearestPoint.nearestPoint.z > leftPt.get().z)
						leftVerticesCopy[i].z = rightNearestPoint.nearestPoint.z; // Note: 点高程写到VertexCopy
			}
			// TODO: 调整左侧剩余部分道路
		}
			// 最后高程回写到vertex引用
			for (std::size_t i = 0; i < leftVerteices.size(); ++i)
			{
				leftVerteices[i].get().z = leftVerticesCopy[i].z;
			}
			for (std::size_t i = 0; i < rightVerticesCopy.size(); ++i)
			{
				rightVerteices[i].get().z = rightVerticesCopy[i].z;
			}
		}

		// 上述调整过程中两相邻link衔接处的点高程可能不一致，此处将其修正为完全相同。
		// TODO: 相邻的Link可能是其它上下行道路的一部分，此时又该如何处理？如果存在该情况，则原本高程对齐调整后的上下行道路又变得不对齐
		auto adjustConnectedLinks = [this](DirLinkPath &path) 
			{
				// 第一个dirLink逆向出边
				{
					DirLink& firstDirLink = path.portions().front().dirLink;
					const double firstDirLinkStartHeight = firstDirLink.getStartHeight();
					std::vector<DirLink> outlinks = firstDirLink.getReverseDirLink().getEndDirectOutlinksExceptSelf();
					for (DirLink& outlink : outlinks)
					{
						// oulink可能不存在geometry对象
						LineString3d* linestring = &this->getOrAddLinkGeomCopy(*outlink.getLinkDangerous());
						outlink.setGeoemtry(linestring);

						AdjustZVertexAccessor heightModifier{ firstDirLinkStartHeight, firstDirLinkStartHeight - outlink.getStartHeight(), outlink.calLength() };
						outlink.accessVerticesToEnd(heightModifier);
					}
				}
				// path中所有非最后一个dirLink顺向出边
				for (std::size_t i = 0; i + 1 < path.portions().size(); ++i)
				{
					auto& portion = path.portions()[i];
					auto& nextPortion = path.portions()[i + 1];
					DirLink& dirLink = portion.dirLink;
					const double dirLinkEndHeight = dirLink.getEndHeight();
					std::vector<DirLink> outlinks = dirLink.getEndDirectOutlinksExceptSelf();
					for (DirLink& outlink : outlinks)
					{
						// 忽略处于path中的出边
						if (outlink.uuid() == nextPortion.dirLink.uuid())
							continue;

						// oulink可能不存在geometry对象
						LineString3d* linestring = &this->getOrAddLinkGeomCopy(*outlink.getLinkDangerous());
						outlink.setGeoemtry(linestring);

						AdjustZVertexAccessor heightModifier{ dirLinkEndHeight, dirLinkEndHeight - outlink.getStartHeight(), outlink.calLength() };
						outlink.accessVerticesToEnd(heightModifier);
					}
				}
				// path中最后一个dirLink顺向出边
				{
					auto& portion = path.portions().back();
					DirLink& dirLink = portion.dirLink;
					const double dirLinkEndHeight = dirLink.getEndHeight();
					std::vector<DirLink> outlinks = dirLink.getEndDirectOutlinksExceptSelf();
					for (DirLink& outlink : outlinks)
					{
						// oulink可能不存在geometry对象
						LineString3d* linestring = &this->getOrAddLinkGeomCopy(*outlink.getLinkDangerous());
						outlink.setGeoemtry(linestring);

						AdjustZVertexAccessor heightModifier{ dirLinkEndHeight, dirLinkEndHeight - outlink.getStartHeight(), outlink.calLength() };
						outlink.accessVerticesToEnd(heightModifier);
					}
				}
			};

		adjustConnectedLinks(dividedRoad.leftPath);
		adjustConnectedLinks(dividedRoad.rightPath);
	}

	/**
	 * @brief
	 * @param adjustQueue
	 * @param adjustedSegment DbLinkd->uuid的数组, uuid能唯一标识该link对象
	*/
	void ZGenerator::bfs(AdjustHeightQueue& adjustQueue, std::set<int64>& adjustedLinks)
	{
		while (!adjustQueue.empty())
		{
			AdjustHeightInfo curInfo = adjustQueue.top();
			adjustQueue.pop();

			DirLink curDirLink = curInfo.dlink;
			if (adjustedLinks.count(curDirLink.uuid()) > 0)
				continue;
			adjustedLinks.insert(curDirLink.uuid());

			// TODO: 思考bfs逻辑，当前面的道路被访问过但是没有修改过startHeight，后面其它地方link设置endHeight为0.

			double adjustedHeight = NAN;
			std::vector<DirLink> outlinks = getEndDirectOutlinksExceptSelf(curDirLink);
			for (DirLink& outlink : outlinks)
			{
				if (adjustedLinks.count(outlink.uuid()))
					adjustedHeight = outlink.getStartHeight();
			}

			if (isnan(adjustedHeight)) // 头一次搜索到
			{
				AdjustZVertexAccessor accessor{ curInfo.lastHeight, curInfo.lastHeight - curDirLink.getStartHeight(),curInfo.length };
				curDirLink.accessVerticesToEnd(accessor);
				double remainingLength = accessor.restLength();
				if (remainingLength > 0.0)
				{
					for (DirLink& outlink : outlinks)
					{
						AdjustHeightInfo nextInfo;
						nextInfo.dlink = outlink;
						nextInfo.lastHeight = curDirLink.getEndHeight();
						nextInfo.length = remainingLength;
						nextInfo.priority = curInfo.priority + outlink.calLength();
						adjustQueue.push(nextInfo);
					}
				}
			}
			else
			{
				AdjustZ2VertexAccessor accessor{ curDirLink, curInfo.lastHeight, adjustedHeight };
				curDirLink.accessVerticesToEnd(accessor);
			}
		}
	}

	void ZGenerator::processRoadBetweenZLevels(std::vector<ZLevelLinks>& zLevelLinks)
	{
		SingleDirRoadPortionSet processedRoads;

		for (auto zLevelLinkPair : zLevelLinks) {
			DbZLevel* pZLevel = zLevelLinkPair.zlevel;
			for (size_t i = 1; i < zLevelLinkPair.links.size(); i++)
			{
				DbZLevel::DbRelLink& relLink = pZLevel->relLinks[i];
				DirLink& pLink = zLevelLinkPair.links[i];
				processHeightByZLevelPoint(relLink, pLink, processedRoads);
			}
		}
	}

	void ZGenerator::processHeightByZLevelPoint(DbZLevel::DbRelLink& relLink, DirLink& link, SingleDirRoadPortionSet& processedRoadPortion)
	{
		auto ProcessInOneDirection = [&](DirLink& link)
			{
				SingleDirRoad road = extendRoadFromZLevelPoint(link, relLink.shpSeqNum);

				if (road.valid())
				{
					// 倘若该有向路段的任何一部分已经在其它SingleDirRoad处理过（一般情况下不应该出现该种情况！！否则程序查找SingleDirRoad策略有点不好），则不进行任何处理，且把当前道路的所有portion标记为已处理。
					bool containsProcessedPortion = ([&]() -> bool
						{
							for (auto& portion : road.portions())
							{
								if (processedRoadPortion.find(portion) != processedRoadPortion.end())
									return true;
							}
							return false;
						})();

						if (!containsProcessedPortion)
						{
							adjustSingleDirRoadHeight(road);
						}

						for (auto& portion : road.portions())
						{
							processedRoadPortion.insert(portion);
						}
						if (road.portions().back().stopReason == SingleDirRoad::StopReason::MEET_Z_LEVEL_POINT)
						{
							// 最后一部分的反方向也加入已访问portion。
							processedRoadPortion.insert(road.portions().back().asBeginPortion());
						}
				}
			};

		// process in forward direction
		ProcessInOneDirection(link);

		// process in backward direction
		ProcessInOneDirection(link.getReverseDirLink());
	}

	SingleDirRoad ZGenerator::extendRoadFromZLevelPoint(DirLink& beginLink, const int beginLinkZLevelPointIndex)
	{
		// some helper functions 
		auto NextZLevelPointIndex = [](std::vector<DbZLevel::DbRelLink>& zPoints, int thisZLevelPointSeqNum, bool forward) -> DbZLevel::DbRelLink*
			{
				if (zPoints.empty()) return nullptr;

				if (forward)
				{
					int thisZLevelPointIndex{ INVALID_INDEX };
					for (int i = 0; i + 1 < int(zPoints.size()); ++i)
					{
						if (zPoints[i].shpSeqNum == thisZLevelPointSeqNum)
						{
							thisZLevelPointIndex = i;
							break;
						}
					}
					return thisZLevelPointIndex != INVALID_INDEX ? &zPoints[thisZLevelPointIndex + 1] : nullptr;
				}
				else
				{
					int thisZLevelPointIndex{ INVALID_INDEX };
					for (int i = int(zPoints.size()) - 1; i > 0; --i)
					{
						if (zPoints[i].shpSeqNum == thisZLevelPointSeqNum)
						{
							thisZLevelPointIndex = i;
							break;
						}
					}
					return thisZLevelPointIndex != INVALID_INDEX ? &zPoints[thisZLevelPointIndex - 1] : nullptr;
				}
			};
		auto NextZLevelPointIndexNoSearch = [](std::vector<DbZLevel::DbRelLink>& zPoints, bool forward) -> DbZLevel::DbRelLink*
			{
				if (zPoints.empty()) return nullptr;

				return forward ? &zPoints.front() : &zPoints.back();
			};
		auto FindNextLink = [this](DirLink thisLink, std::size_t& totalOutLinkNum, DirLink& nextLink, std::vector<DirLink>& connectedLinks) {
			std::vector<DirLink> outlinks = getEndDirectOutlinksExceptSelf(thisLink);

			totalOutLinkNum = outlinks.size();

			if (totalOutLinkNum == 1)
				nextLink = outlinks[0];
			else if (totalOutLinkNum > 1)
			{
				DirLink suitableOtherRoad;
				int suitableOtherRoadNum = 0;
				auto thisLinkName = thisLink.getLinkName();
				if (thisLinkName != nullptr)
				{
					for (auto& otherLink : outlinks)
					{
						// 道路名称相同、道路等级相同、用途相同
						// TODO 此处需要优化,通过坐标点找link的名称
						if (otherLink.containsLinkName(thisLinkName)
							&& thisLink.kind() == otherLink.kind()
							&& !angleIsTheSameDirection(thisLink.endHeadingAngle(), otherLink.startHeadingAngle()) // 有向路段连接处方向相同，此处由于startHeadingAngle性质取非同向
							// WARNING: 两个link的wayTypes(用途)都只有一个且都相同，不然就不检查
							&& ((thisLink.wayTypes().size() == 1 && otherLink.wayTypes().size() == 1) ? (thisLink.wayTypes().front() == otherLink.wayTypes().front()) : true)
							) {
							++suitableOtherRoadNum;
							suitableOtherRoad = otherLink;
						}
						else
							connectedLinks.push_back(otherLink);
					}
				}
				if (suitableOtherRoadNum == 1)
					nextLink = suitableOtherRoad;
				else
					connectedLinks.clear();
			}
			};

		SingleDirRoad road;
		{
			SingleDirRoad::Portion portion;
			portion.dirLink = beginLink;
			portion.zLevelPointIndex = beginLinkZLevelPointIndex;
			portion.stopReason = SingleDirRoad::StopReason::NONE;
			portion.accLength = 0.0;
			road.addPortion(portion);
		}

		DirLink nextLink = beginLink;
		int nextLinkZLevelPointIndex = beginLinkZLevelPointIndex;
		while (true)
		{
			// check if the remain portion of road has a z level point
			DbZLevel::DbRelLink* nextZLevelPoint = nullptr;

			if (nextLink.valid() && !nextLink.zLevels().empty())
			{
				if (nextLinkZLevelPointIndex != INVALID_INDEX)
					nextZLevelPoint = NextZLevelPointIndex(nextLink.zLevels(), nextLinkZLevelPointIndex, nextLink.forward());
				else
					nextZLevelPoint = NextZLevelPointIndexNoSearch(nextLink.zLevels(), nextLink.forward());
			}

			if (nextZLevelPoint != nullptr)
			{
				if (nextZLevelPoint->zLevel == 0)
				{
					// Z Level点延伸搜索过程中，如果遇到道路上Z Level点 level为0的情况，可继续延伸搜索，但只能使用LeastHeightVertexModifier
					road.setHasPassedZeroLevelPoint(true);
					nextLinkZLevelPointIndex = nextZLevelPoint->shpSeqNum;
					continue;
				}
				else
				{
					SingleDirRoad::Portion portion;
					portion.dirLink = nextLink;
					portion.stopReason = SingleDirRoad::StopReason::MEET_Z_LEVEL_POINT;
					portion.zLevelPointIndex = nextZLevelPoint->shpSeqNum;
					double length;
					{
						if (nextLinkZLevelPointIndex != INVALID_INDEX)
							length = portion.dirLink.calPortionLength(nextLinkZLevelPointIndex, nextZLevelPoint->shpSeqNum);
						else
							length = portion.dirLink.calStartToMidLength(nextZLevelPoint->shpSeqNum);
					}
					portion.accLength = road.length() + length;
					road.addPortion(portion);
					break;
				}
			}

			// get end outlinks 
			std::size_t nextLinkOutlinksNum{ 0 };
			DirLink nextLinkNextLink;
			std::vector<DirLink> connectedLinks;
			FindNextLink(nextLink, nextLinkOutlinksNum, nextLinkNextLink, connectedLinks);
			if (!nextLinkNextLink.valid())
			{
				SingleDirRoad::Portion portion;
				portion.dirLink = nextLink;
				portion.stopReason = nextLinkOutlinksNum == 0 ? SingleDirRoad::StopReason::NO_OUTLINKS : SingleDirRoad::StopReason::MULTI_OUT_LINKS;
				double length;
				{
					if (nextLinkZLevelPointIndex == INVALID_INDEX)
						length = nextLink.calLength();
					else
						length = nextLink.calMidToEndLength(nextLinkZLevelPointIndex);
				}
				portion.accLength = road.length() + length;
				road.addPortion(portion);
				break;
			}
			else
			{
				SingleDirRoad::Portion portion;
				portion.dirLink = nextLink;
				portion.stopReason = SingleDirRoad::StopReason::NONE;
				double length;
				{
					if (nextLinkZLevelPointIndex == INVALID_INDEX)
						length = nextLink.calLength();
					else
						length = nextLink.calMidToEndLength(nextLinkZLevelPointIndex);
				}
				portion.accLength = road.length() + length;
				for (auto& link : connectedLinks)
				{
					SingleDirRoad road = extendRoadFromDSegment(link);
					if (road.valid() && road.length() > 0.0)
						portion.connectedRoads.push_back(road);
					}
				road.addPortion(portion);

				nextLink = nextLinkNextLink;
				nextLinkZLevelPointIndex = INVALID_INDEX;
			}
		}

		return road;
	}

	SingleDirRoad ZGenerator::extendRoadFromDSegment(DirLink& beginLink)
	{
		auto NextZLevelPointIndexNoSearch = [](std::vector<DbZLevel::DbRelLink>& zPoints, bool forward) -> DbZLevel::DbRelLink*
			{
				if (zPoints.empty()) return nullptr;

				return forward ? &zPoints.front() : &zPoints.back();
			};

		SingleDirRoad road;
		{
			SingleDirRoad::Portion portion;
			portion.dirLink = beginLink;
			portion.zLevelPointIndex = beginLink.forward() ? 0 : int(beginLink.geometry().vertexes.size()) - 1;
			portion.stopReason = SingleDirRoad::StopReason::NONE;
			portion.accLength = 0.0;
			road.addPortion(portion);
		}

		DirLink nextLink = beginLink;
		int nextLinkZLevelPointIndex = road.back().zLevelPointIndex;
		while (true)
		{
			// check if the remain portion of road has a z level point
			DbZLevel::DbRelLink* nextZLevelPoint = nullptr;

			if (nextLink.valid() && !nextLink.zLevels().empty())
			{
				nextZLevelPoint = NextZLevelPointIndexNoSearch(nextLink.zLevels(), nextLink.forward());
			}

			if (nextZLevelPoint != nullptr)
			{
				SingleDirRoad::Portion portion;
				portion.dirLink = nextLink;
				portion.stopReason = SingleDirRoad::StopReason::MEET_Z_LEVEL_POINT;
				portion.zLevelPointIndex = nextZLevelPoint->shpSeqNum;
				double length;
				{
					if (nextLinkZLevelPointIndex != INVALID_INDEX)
						length = portion.dirLink.calPortionLength(nextLinkZLevelPointIndex, nextZLevelPoint->shpSeqNum);
					else
						length = portion.dirLink.calStartToMidLength(nextZLevelPoint->shpSeqNum);
				}
				portion.accLength = road.length() + length;
				road.addPortion(portion);
				break;
			}

			// get end outlinks 
			std::vector<DirLink> outlinks = getEndDirectOutlinksExceptSelf(nextLink);
			std::size_t nextLinkOutlinksNum{ outlinks.size() };
			DirLink nextLinkNextLink;
			if (nextLinkOutlinksNum == 1)
			{
				nextLinkNextLink = outlinks.front();
			}

			if (!nextLinkNextLink.valid())
			{
				SingleDirRoad::Portion portion;
				portion.dirLink = nextLink;
				portion.stopReason = nextLinkOutlinksNum == 0 ? SingleDirRoad::StopReason::NO_OUTLINKS : SingleDirRoad::StopReason::MULTI_OUT_LINKS;
				double length;
				{
					if (nextLinkZLevelPointIndex == INVALID_INDEX)
						length = nextLink.calLength();
					else
						length = nextLink.calMidToEndLength(nextLinkZLevelPointIndex);
				}
				portion.accLength = road.length() + length;
				road.addPortion(portion);
				break;
			}
			else
			{
				SingleDirRoad::Portion portion;
				portion.dirLink = nextLink;
				portion.stopReason = SingleDirRoad::StopReason::NONE;
				double length;
				{
					if (nextLinkZLevelPointIndex == INVALID_INDEX)
						length = nextLink.calLength();
					else
						length = nextLink.calMidToEndLength(nextLinkZLevelPointIndex);
				}
				portion.accLength = road.length() + length;
				road.addPortion(portion);

				nextLink = nextLinkNextLink;
				nextLinkZLevelPointIndex = INVALID_INDEX;
			}
		}

		return road;
	}

	void ZGenerator::adjustSingleDirRoadHeight(SingleDirRoad& road)
	{
		assert(road.front().zLevelPointIndex != INVALID_INDEX);

		// 处理当前道路： road
		{
			std::unique_ptr<PathVerticesAccessor> modifier;

			if (road.hasPassedZeroLevelPoint())
				modifier = std::make_unique<LeastHeightVertexModifier>(road.getStartHeight(), road.getEndHeight());
		else
		{
			if (road.back().stopReason == SingleDirRoad::StopReason::MEET_Z_LEVEL_POINT && road.back().accLength < 1500.0)
					modifier = std::make_unique<GradientVertexModifier>(road.getStartHeight(), road.getEndHeight(), road.length());
				else
					modifier = std::make_unique<LeastHeightVertexModifier>(road.getStartHeight(), road.getEndHeight());
			}

			road.accessVertices(*modifier);
		}

		// 处理与road连接的其它道路
		for (SingleDirRoad::Portion& portion : road.portions())
			{
			for (SingleDirRoad& connectedRoad : portion.connectedRoads)
			{
				ConnectedRoadVertexModifier modifier{ portion.dirLink.getEndPointRef() , connectedRoad.length() };
				connectedRoad.accessVertices(modifier);
			}
		}
	}

	double DirLink::startHeadingAngle()
	{
		MapPoint64 origin = getPointRef(1).pos;
		MapPoint64 point = getPointRef(0).pos;

		return map_pt_math::twoPointAngle(origin, point);
	}

	double DirLink::endHeadingAngle()
	{
		MapPoint64 origin = getPointRef(m_linkGeometry->vertexes.size() - 2).pos;
		MapPoint64 point = getPointRef(m_linkGeometry->vertexes.size() - 1).pos;

		return map_pt_math::twoPointAngle(origin, point);
	}

	MapPoint3D64 DirLink::calPointAt(double perc) const
	{
		perc = nc_clamp(perc, 0.0, 1.0);
		if (!m_forward)
			perc = 1.0 - perc;

		// TODO：需要确保计算link长度的流程和下面一样才能确保避免浮点数运算带来的微小误差
		const double totalLen = calLength();
		double len = 0.0;
		for (std::size_t i = 0; i + 1 < m_linkGeometry->vertexes.size(); ++i)
		{
			auto& point = m_linkGeometry->vertexes[i];
			auto& nextPoint = m_linkGeometry->vertexes[i + 1];
			double segmentLen = MapPoint64::geodistance(point.pos, nextPoint.pos);

			if ((len + segmentLen) / totalLen >= perc)
			{
				double restPerc = ((perc * totalLen) - len) / segmentLen;
				restPerc = nc_clamp(restPerc, 0.0, 1.0);
				return mapPoint3D64_lerp(point, nextPoint, restPerc);
			}
			len += segmentLen;
		}

		// 不应到达此处，否则程序错误，也可能是浮点数计算误差
		return getEndPoint();
	}

	double DirLink::calLength() const
	{
		double len = 0.0;
		for (std::size_t i = 0; i + 1 < m_linkGeometry->vertexes.size(); ++i)
		{
			auto& point = m_linkGeometry->vertexes[i];
			auto& nextPoint = m_linkGeometry->vertexes[i + 1];
			len += MapPoint64::geodistance(point.pos, nextPoint.pos);
		}
		return len;
	}

	double DirLink::calPortionLength(int indexFrom, int indexTo) const
	{
		int linkVertexNum = static_cast<int>(m_linkGeometry->vertexes.size());
		if (indexFrom < 0
			|| indexFrom >= linkVertexNum
			|| indexTo < 0 || indexFrom >= linkVertexNum)
		{
			assert(false); // TODO: 使用其它提示方式来表明这是一个错误。
			return -1.0;
		}

		double len = 0.0;
		for (int i = indexFrom; i < indexTo; ++i)
		{
			auto& point = m_linkGeometry->vertexes.at(i);
			auto& nextPoint = m_linkGeometry->vertexes.at(i + 1);
			len += MapPoint64::geodistance(point.pos, nextPoint.pos);
		}

		for (int i = indexFrom; i > indexTo; --i)
		{
			auto& point = m_linkGeometry->vertexes.at(i);
			auto& nextPoint = m_linkGeometry->vertexes.at(i - 1);
			len += MapPoint64::geodistance(point.pos, nextPoint.pos);
		}

		return len;
	}

	void DirLink::accessVerticesOrdered(int indexFrom, int indexTo, VertexAccessor& accessor)
	{
		accessor.accessFirst(m_linkGeometry->vertexes[indexFrom]);

		double distToPrevPoint{ 0.0 };
		MapPoint3D64* prevPoint{ &m_linkGeometry->vertexes[indexFrom] };

		if (indexFrom < indexTo)
		{
			for (int i = indexFrom + 1; i < indexTo; ++i)
			{
				MapPoint3D64& point = m_linkGeometry->vertexes[i];
				distToPrevPoint = MapPoint64::geodistance(point.pos, prevPoint->pos);
				accessor.accessMiddle(point, *prevPoint, distToPrevPoint);
				prevPoint = &point;
			}
		}
		else
		{
			for (int i = indexFrom - 1; i > indexTo; --i)
			{
				MapPoint3D64& point = m_linkGeometry->vertexes[i];
				distToPrevPoint = MapPoint64::geodistance(point.pos, prevPoint->pos);
				accessor.accessMiddle(point, *prevPoint, distToPrevPoint);
				prevPoint = &point;
			}
		}

		MapPoint3D64& lastPoint = m_linkGeometry->vertexes[indexTo];
		distToPrevPoint = MapPoint64::geodistance(lastPoint.pos, prevPoint->pos);
		accessor.accessLast(lastPoint, *prevPoint, distToPrevPoint);
	}

	template <bool withLinkGeometry>
	std::vector<DirLink> DirLink::getEndDirectOutlinks()
	{
		DbNode* endNode = m_forward ? m_link->endNodePtr : m_link->startNodePtr;

		// 生成OutDirLink
		std::vector<DirLink> outDirLinks;
		outDirLinks.reserve(endNode->links.size());
		for (DbLink* outlink : endNode->links)
		{
			bool oulinkIsForward = outlink->startNodePtr->uuid == endNode->uuid;
			if constexpr (withLinkGeometry)
				outDirLinks.emplace_back(outlink, oulinkIsForward, outlink->geometry);
			else
				outDirLinks.emplace_back(outlink, oulinkIsForward);
		}
		return outDirLinks;
	}

	std::vector<int64> DirLink::getEndDirectOutlinksUUID()
	{
		std::vector<DirLink> outlinks = this->getEndDirectOutlinks();
		std::vector<int64> outlinksUUID(outlinks.size());
		for (std::size_t i = 0; i < outlinks.size(); ++i)
		{
			outlinksUUID[i] = outlinks[i].m_link->uuid;
		}
		return outlinksUUID;
	}

	std::vector<DirLink> DirLink::getEndDirectOutlinksExceptSelf()
	{
		// 从所有oultinks中移除当前link
		std::vector<DirLink> outlinks = this->getEndDirectOutlinks();
		auto selfIter = std::find_if(outlinks.begin(), outlinks.end(), [this](DirLink& link) { return this->rawLinkEqual(link); });
		if (selfIter != outlinks.end())
		{
			outlinks.erase(selfIter);
		}
		return outlinks;
	}

	std::vector<int64> DirLink::getEndDirectOutlinksExceptSelfUUID()
	{
		std::vector<DirLink> outlinks = getEndDirectOutlinksExceptSelf();
		std::vector<int64> outlinksUUID(outlinks.size());
		for (std::size_t i = 0; i < outlinks.size(); ++i)
		{
			outlinksUUID[i] = outlinks[i].m_link->uuid;
		}
		return outlinksUUID;
	}

	void DirLinkPath::Portion::reverse(double pathLength)
	{
		dirLink = dirLink.getReverseDirLink();
		accLength = pathLength - accLength;
	}

	void DirLinkPath::addPortion(DirLink& dirLink)
	{
		Portion portion;
		portion.dirLink = dirLink;
		portion.accLength = m_portions.empty() ? dirLink.calLength() : m_portions.back().accLength + dirLink.calLength();
		addPortion(portion);
	}

	DirLinkPath::Portion DirLinkPath::popBack()
	{
		Portion portion = m_portions.back();
		m_portions.erase(m_portions.end() - 1);
		return portion;
	}

	void DirLinkPath::reverse()
	{
		double pathLength = m_portions.back().accLength;
		std::reverse(m_portions.begin(), m_portions.end());
		for (auto& portion : m_portions)
			portion.reverse(pathLength);
	}

	void DirLinkPath::appendPath(DirLinkPath& path)
	{
		double originPathLength = m_portions.empty() ? 0.0 : m_portions.back().accLength;
		for (Portion portion : path.m_portions)
		{
			portion.accLength += originPathLength;
			m_portions.push_back(portion);
		}
	}

	void DirLinkPath::accessVertices(PathVerticesAccessor& accessor)
	{
		PathLinkVerticesAccessor linkAccessor{ accessor, calTotalVertexNum() };
		for (auto& portion : m_portions)
			portion.dirLink.accessVerticesToEnd(linkAccessor);
	}

	std::size_t DirLinkPath::calTotalVertexNum()
	{
		std::size_t totalVertexNum{ 0 };

		for (auto& portion : m_portions)
		{
			totalVertexNum += portion.countVertexNum();
		}

		return totalVertexNum;
	}

	void DirLinkPathVertexRefCollector::access(MapPoint3D64& vertex, const PathVerticesAccessor::AccessInfo& accessInfo)
	{
		UNREFERENCED_PARAMETER(accessInfo);

		m_verticesRef.push_back(vertex);
		m_heightSum += vertex.z;
	}

	DividedRoadSearcher::DividedRoadSearcher(DbMesh* currentMesh, std::vector<DbMesh*> nearbyMesh) : m_mesh(currentMesh), m_nearbyMesh(nearbyMesh)
	{
		m_totalMeshes = nearbyMesh;
		m_totalMeshes.push_back(currentMesh);

		updateCenterLinks(currentMesh);
		for (auto nearby : nearbyMesh)
			updateCenterLinks(nearby);
			}

	std::vector<DividedRoad> DividedRoadSearcher::searchZGen()
	{
		std::vector<DirLink> dividedLinks;
		auto appendLinks = [&dividedLinks, this](DbMesh* mesh)
			{
				for (auto& hl : mesh->query(RecordType::DB_HAD_LINK)) {
					DbLink* link = (DbLink*)hl;
					DirLink dirLink{ link, true, link->geometry };
					if (isDividedRoadZGen(dirLink))
						dividedLinks.emplace_back(dirLink);
				}
			};

		// 取所有网格内的link而不是仅是中心网格
		appendLinks(m_mesh);
		for (auto nearbyMesh : m_nearbyMesh)
			appendLinks(nearbyMesh);

		// 先对道路排序后搜索会得到更好的结果
		sortLinksByStraightness(dividedLinks);

		return searchZGen(dividedLinks);
	}

	std::vector<DividedRoad> DividedRoadSearcher::searchZGen(std::vector<DirLink>& dividedLinks)
	{
		std::set<int64> visitedLinks;

		std::vector<DividedRoad> dividedRoads;

		for (auto& link : dividedLinks) {

			if (visitedLinks.count(link.uuid()) > 0)
				continue;

			// 抓取对侧道路
			DirLink oneSideLink = link.getForwardDirLink();
			DividedRoadSearcher::GrabRoadResult anotherSideResult = grabAnotherRoadZGenMultiTimes(oneSideLink);

			if (!anotherSideResult.valid()
				|| !isDividedRoadZGen(anotherSideResult.link)
				|| visitedLinks.count(anotherSideResult.link.uuid()) > 0)
				continue;

			visitedLinks.insert(oneSideLink.uuid());
			visitedLinks.insert(anotherSideResult.link.uuid());

			// 得到左边和右边的link。因为两个link同向，所以一定会有左右之分。
			DirLink leftSideLink = anotherSideResult.link;
			DirLink rightSideLink = oneSideLink;
			if (!anotherSideResult.isLeft)
				std::swap(leftSideLink, rightSideLink);

			// 检查左右侧初始link行驶方向，方向相同则忽略
			{
				static const bool diffTrafficDirLookupTable[][4] =
				{
					{false, true, true, false},
					{true, false, false, true},
					{true, false, false, true},
					{false, true, true, false}
				};
				int forwardCode = int(leftSideLink.forward()) << 1 | int(rightSideLink.forward());
				int directionCode = int(leftSideLink.direct() - 2) << 1 | int(rightSideLink.direct() - 2);
				if (!diffTrafficDirLookupTable[forwardCode][directionCode])
					continue;
			}

			DirLinkPath leftPath = extendRoad(leftSideLink, true, visitedLinks);
			DirLinkPath rightPath = extendRoad(rightSideLink, false, visitedLinks);
			dividedRoads.push_back(DividedRoad{ leftPath, rightPath });
		}

		// 处理上下行道路使得两端对齐
		for (DividedRoad& dividedRoad : dividedRoads)
		{
			std::array<std::vector<DirLink>, 2> removedLinks = trimEndRoadsPerDividedRoad(dividedRoad, true, true);
		}

		return dividedRoads;
	}

	std::array<std::vector<DirLink>, 2> DividedRoadSearcher::trimEndRoadsPerDividedRoad(DividedRoad& dividedRoad, bool trimBegin, bool trimEnd)
	{
		using Searcher = NearestPointOnLineSegmentsSearcher;
		using SearcherPoint = NearestPointOnLineSegmentsSearcher::Point;

		// 读取两侧道路shape points
		std::vector<Searcher> leftSearchers, rightSearchers;
		{
			DirLinkVertexReader vertexReader;
			for (auto& portion : dividedRoad.leftPath)
			{
				portion.dirLink.accessVerticesToEnd(vertexReader);
				leftSearchers.push_back(NearestPointOnLineSegmentsSearcher_make(vertexReader.vertices()));
			}
			for (auto& portion : dividedRoad.rightPath)
			{
				portion.dirLink.accessVerticesToEnd(vertexReader);
				rightSearchers.push_back(NearestPointOnLineSegmentsSearcher_make(vertexReader.vertices()));
			}
		}

		// 在一侧道路上搜索距离某点最近的点
		auto SearchNearestPointFromOneSide = [](const std::vector<Searcher>& searchers, const SearcherPoint& point)
			{
				double minDist = std::numeric_limits<double>::infinity();
				SearcherPoint nearestPoint{ std::numeric_limits<double>::quiet_NaN(),
					std::numeric_limits<double>::quiet_NaN(),
					std::numeric_limits<double>::quiet_NaN() };
				for (auto& searcher : searchers)
				{
					Searcher::SearchResult result = searcher.searchNearest(point);
					if (result.sqDist < minDist)
					{
						minDist = result.sqDist;
						nearestPoint = result.point;
					}
				}

				return nearestPoint;
			};

		// 计算道路两端对齐指数
		auto CalculateFlatIndex = [](SearcherPoint sideJEndPoint, SearcherPoint sideJNearestPoint, SearcherPoint sideKEndPoint, SearcherPoint sideKNearestPoint)
			{
				double endPointDist = std::sqrt(std::pow(sideJEndPoint[0] - sideKEndPoint[0], 2.0) + std::pow(sideJEndPoint[1] - sideKEndPoint[1], 2.0));
				double nearestPointDist1 = std::sqrt(std::pow(sideJEndPoint[0] - sideKNearestPoint[0], 2.0) + std::pow(sideJEndPoint[1] - sideKNearestPoint[1], 2.0));
				double nearestPointDist2 = std::sqrt(std::pow(sideKEndPoint[0] - sideJNearestPoint[0], 2.0) + std::pow(sideKEndPoint[1] - sideJNearestPoint[1], 2.0));
				double minNearestPointDist = min(nearestPointDist1, nearestPointDist2);

				return std::abs(endPointDist - minNearestPointDist);
			};

		// 处理一侧道路(mainSide)
		auto TrimBeginRoads = [&SearchNearestPointFromOneSide, &CalculateFlatIndex](DirLinkPath &mainSidePath, std::vector<Searcher>& mainSideSearchers, const std::vector<Searcher>& anotherSideSearchers, std::vector<DirLink>& removedLinks)
			{
				auto CalculateCurrentFlatIndex = [&mainSideSearchers, &anotherSideSearchers, &SearchNearestPointFromOneSide, &CalculateFlatIndex]() {
					SearcherPoint mainSideEndPoint = mainSideSearchers.front().segments().front()[0];
					SearcherPoint anotherSideEndPoint = anotherSideSearchers.front().segments().front()[0];
					SearcherPoint mainSideNearestPoint = SearchNearestPointFromOneSide(mainSideSearchers, anotherSideEndPoint);
					SearcherPoint anotherSideNearestPoint = SearchNearestPointFromOneSide(anotherSideSearchers, mainSideEndPoint);
					return CalculateFlatIndex(mainSideEndPoint, mainSideNearestPoint, anotherSideEndPoint, anotherSideNearestPoint);
					};

				mainSidePath.reverse();

				double prevFlatIndex = CalculateCurrentFlatIndex();
				for (int i = 1, maxI = mainSideSearchers.size(); i < maxI; ++i)
				{
					// 尝试去除mainSide前一段道路，然后计算对齐指数，
					// 如果指数减小说明去除该道路有助于对齐，则接受去除该道路，否则恢复不去除
					Searcher prevSearcher = mainSideSearchers.front();
					mainSideSearchers.erase(mainSideSearchers.begin());
					double newFlatIndex = CalculateCurrentFlatIndex();
					if (newFlatIndex < prevFlatIndex)
					{
						prevFlatIndex = newFlatIndex;

						auto removedPortion = mainSidePath.popBack();
						removedLinks.push_back(removedPortion.dirLink);
					}
					else
					{
						mainSideSearchers.insert(mainSideSearchers.begin(), prevSearcher);
						break;
					}
				}

				mainSidePath.reverse();
			};
		auto TrimEndRoads = [&SearchNearestPointFromOneSide, &CalculateFlatIndex](DirLinkPath& mainSidePath, std::vector<Searcher>& mainSideSearchers, const std::vector<Searcher>& anotherSideSearchers, std::vector<DirLink>& removedLinks)
			{
				auto CalculateCurrentFlatIndex = [&mainSideSearchers, &anotherSideSearchers, &SearchNearestPointFromOneSide, &CalculateFlatIndex]() {
					SearcherPoint mainSideEndPoint = mainSideSearchers.back().segments().back()[1];
					SearcherPoint anotherSideEndPoint = anotherSideSearchers.back().segments().back()[1];
					SearcherPoint mainSideNearestPoint = SearchNearestPointFromOneSide(mainSideSearchers, anotherSideEndPoint);
					SearcherPoint anotherSideNearestPoint = SearchNearestPointFromOneSide(anotherSideSearchers, mainSideEndPoint);
					return CalculateFlatIndex(mainSideEndPoint, mainSideNearestPoint, anotherSideEndPoint, anotherSideNearestPoint);
					};

				double prevFlatIndex = CalculateCurrentFlatIndex();
				for (int i = int(mainSideSearchers.size()) - 2; i >= 0; --i)
				{
					// 尝试去除mainSide前一段道路，然后计算对齐指数，
					// 如果指数减小说明去除该道路有助于对齐，则接受去除该道路，否则恢复不去除
					Searcher prevSearcher = mainSideSearchers.back();
					mainSideSearchers.erase(mainSideSearchers.end() - 1);
					double newFlatIndex = CalculateCurrentFlatIndex();
					if (newFlatIndex < prevFlatIndex)
					{
						prevFlatIndex = newFlatIndex;

						auto removedPortion = mainSidePath.popBack();
						removedLinks.push_back(removedPortion.dirLink);
					}
					else
					{
						mainSideSearchers.push_back(prevSearcher);
						break;
					}
				}
			};

		std::array<std::vector<DirLink>, 2> removedLinks; // [left links, right links]

		if (trimBegin) {
			TrimBeginRoads(dividedRoad.leftPath, leftSearchers, rightSearchers, removedLinks[0]);
			TrimBeginRoads(dividedRoad.rightPath, rightSearchers, leftSearchers, removedLinks[1]);
		}
		if (trimEnd) {
			TrimEndRoads(dividedRoad.leftPath, leftSearchers, rightSearchers, removedLinks[0]);
			TrimEndRoads(dividedRoad.rightPath, rightSearchers, leftSearchers, removedLinks[1]);
		}

		return removedLinks;
	}

	bool DividedRoadSearcher::isDividedRoadZGen(DirLink& link, bool ignoreCneterLink)
	{
		return link.multi_digitized() == 1  // 上下线分离
			&& !link.containsWayType(LINK_IS_IN_TUNNEL) // 非隧道路
			&& (ignoreCneterLink || m_centerLinks.count(link.uuid()) == 0) // link非路口内路段
			&& link.direct() != 1;  // 依据OMDB产品规格书，上下行Link应当只为单向行驶
	}

	void DividedRoadSearcher::updateCenterLinks(DbMesh* mesh)
	{
		for (auto& record : mesh->query(RecordType::DB_HAD_INTERSECTION))
		{
			DbIntersection* intersection = (DbIntersection*)record;
			for (DbIntersection::OutLink& outlink : intersection->outLinks)
			{
				if (outlink.isCenterLink == 1)
					m_centerLinks.insert(outlink.linkId);
			}
		}

	}

	void DividedRoadSearcher::sortLinksByStraightness(std::vector<DirLink>& dividedLinks)
	{
		// 计算表征link平直程度的量。
		// 原理：计算线上的点到link首尾连线的最大距离，该距离越小说明线越平直
		auto findLinkStraightness = [](DirLink& link)  -> double
			{
				using Point = NearestPointOnLineSegmentsSearcher::Point;
				using Segment = NearestPointOnLineSegmentsSearcher::Segment;
				using SearchResult = NearestPointOnLineSegmentsSearcher::SearchResult;

				const MapPoint3D64& firstVertex = link.geometry().vertexes.front();
				const MapPoint3D64& lastVertex = link.geometry().vertexes.back();
				Point linkFirstPoint{ double(firstVertex.pos.lon), double(firstVertex.pos.lat) };
				Point linkLastPoint{ double(lastVertex.pos.lon), double(lastVertex.pos.lat) };
				Segment linkSegment{ linkFirstPoint, linkLastPoint };

				double maxDist{ 0.0 };
				for (int i = 1, maxI = int(link.geometry().vertexes.size()) - 1; i < maxI; ++i)
				{
					const MapPoint3D64& vertex = link.geometry().vertexes[i];
					SearchResult searchResult = NearestPointOnLineSegmentsSearcher::findNearestPointToSegment(linkSegment, { double(vertex.pos.lon), double(vertex.pos.lat) });
					double dist = std::sqrt(searchResult.sqDist);
					if (dist > maxDist)
						maxDist = dist;
				}

				return maxDist;
			};

		struct LinkProperties
		{
			DirLink link;
			double length; // 长度
			double straightness; // 笔直程度
			double sortScore; // 排序分，越大该道路越靠前
		};


		std::vector<LinkProperties> linksWithProperties(dividedLinks.size());
		for (std::size_t i = 0; i < dividedLinks.size(); ++i)
		{
			DirLink& link = dividedLinks[i];
			LinkProperties& properties = linksWithProperties[i];

			properties.link = link;
			properties.length = link.calLength();
			properties.straightness = findLinkStraightness(link);
			properties.sortScore = properties.length - 5 * properties.straightness; // 经验值
		}

		// 按照得分从大到小排序，期望得分越高，其为高速路
		std::sort(linksWithProperties.begin(), linksWithProperties.end(), [](auto& l, auto& r) { return l.sortScore > r.sortScore; });

		// 更新输入的links
		for (std::size_t i = 0; i < dividedLinks.size(); ++i)
		{
			dividedLinks[i] = linksWithProperties[i].link;
		}
	}

	DirLink DividedRoadSearcher::getNextDirLink(std::vector<DirLink>& outlinks, DirLink& curLink, bool nextLinkInCCW)
	{
		// outlinks按照startHeadingAngle从小到大排序
		std::sort(outlinks.begin(), outlinks.end(), [](DirLink& l, DirLink& r) {return l.startHeadingAngle() < r.startHeadingAngle(); });
		// 找到curLink索引
		auto curLinkIt = std::find_if(outlinks.begin(), outlinks.end(), [&curLink](DirLink& dirLink) {return dirLink.uuid() == curLink.uuid(); });
		if (curLinkIt == outlinks.end())
			return DirLink{};
		else
		{
			std::size_t curLinkIndex = curLinkIt - outlinks.begin();
			if (nextLinkInCCW)
				return outlinks[(curLinkIndex + 1) % outlinks.size()];
			else
				return outlinks[(curLinkIndex - 1 + outlinks.size()) % outlinks.size()];
		}
	}

	double DividedRoadSearcher::findLineDirectionWithPoint(const LineString3d& linestring, const MapPoint3D64& point, MapPoint64& nearestPointOnLineString)
	{
		auto MapPoint64OnLine = [&linestring](std::size_t index) {return linestring.vertexes[index].pos; };

		using Searcher = NearestPointOnLineSegmentsSearcher;
		auto searcher = NearestPointOnLineSegmentsSearcher_make(linestring);
		auto nearestResult = searcher.searchNearest(Searcher::Point_make(point));

		nearestPointOnLineString = Searcher::Point_toMapPoint(nearestResult.point).pos;

		switch (nearestResult.pointPosition)
		{
		case Searcher::NearestPointPosition::SEGMENT_START:
			// 最邻近点在为线段起点，使用该点前后线段方向角之平均
			// 注意临界情况，不存在前线段
			if (nearestResult.nearestSegmentIndex > 0)
			{
				double prevAngle = map_pt_math::twoPointAngle(MapPoint64OnLine(nearestResult.nearestSegmentIndex - 1), MapPoint64OnLine(nearestResult.nearestSegmentIndex));
				double curAngle = map_pt_math::twoPointAngle(MapPoint64OnLine(nearestResult.nearestSegmentIndex), MapPoint64OnLine(nearestResult.nearestSegmentIndex + 1));
				return (prevAngle + curAngle) * 0.5;
			}
			else
				return map_pt_math::twoPointAngle(MapPoint64OnLine(0), MapPoint64OnLine(1));
		case Searcher::NearestPointPosition::SEGMENT_MIDDLE:
			// 最邻近点在线段中间，使用该线段计算方向角
			return map_pt_math::twoPointAngle(MapPoint64OnLine(nearestResult.nearestSegmentIndex), MapPoint64OnLine(nearestResult.nearestSegmentIndex + 1));
		case Searcher::NearestPointPosition::SEGMENT_END:
		default:
			// 最邻近点在为线段终点，使用该点前后线段方向角之平均
			// 注意临界情况，不存在后线段
			if (nearestResult.nearestSegmentIndex + 1 < searcher.segments().size())
			{
				double prevAngle = map_pt_math::twoPointAngle(MapPoint64OnLine(nearestResult.nearestSegmentIndex - 1), MapPoint64OnLine(nearestResult.nearestSegmentIndex));
				double curAngle = map_pt_math::twoPointAngle(MapPoint64OnLine(nearestResult.nearestSegmentIndex), MapPoint64OnLine(nearestResult.nearestSegmentIndex + 1));
				return (prevAngle + curAngle) * 0.5;
			}
			else
				return map_pt_math::twoPointAngle(MapPoint64OnLine(nearestResult.nearestSegmentIndex), MapPoint64OnLine(nearestResult.nearestSegmentIndex + 1));
		}
	}

	std::vector<DividedRoadSearcher::NeighborLink> DividedRoadSearcher::searchParallelNeighborLinks(DirLink& link, const MapPoint3D64& point, const double maxFindDist, const double direction)
	{
		// 判断两个夹角(单位radians, 范围[0, 2PI) )之间平行程度，返回两夹角之间的夹角大小，正数，无方向，值越小表明两个夹角越接近。
		auto angleParallelDiff = [](double angle1, double angle2) -> double
			{
				double maxAngle = max(angle1, angle2);
				double minAngle = min(angle1, angle2);

				double diff = maxAngle - minAngle;
				// diff分别和0度、180度、360度的差异绝对值的最小值
				return min(min(diff, std::abs(diff - MATH_PI_D)), std::abs(2.0 * MATH_PI_D - diff));
			};

		// 以point为圆心，maxFindDist为半径的圆内的其它所有Link的数组
		std::vector<DbLink*> neighborLinks;
		{
			std::vector<DbMesh*> allMesh = m_nearbyMesh;
			allMesh.push_back(m_mesh);
			neighborLinks = RTreeSeacher::seachNearby2d(allMesh, link.getLinkDangerous(), point, maxFindDist);;
		}
		// 将上述搜索结果按照道路方向相似程度从大到小排序
		std::vector<NeighborLink> neighborLinksParallel; // 其中pair.second为角度差异
		neighborLinksParallel.reserve(neighborLinks.size());
		for (DbLink* neighborLink : neighborLinks)
		{
			MapPoint64 nearestPoint;
			double lineDirection = findLineDirectionWithPoint(neighborLink->geometry, point, nearestPoint);
			double diff = angleParallelDiff(lineDirection, direction);

			NeighborLink neighborLinkObj;
			neighborLinkObj.link = neighborLink;
			neighborLinkObj.direction = lineDirection;
			neighborLinkObj.directionDiff = diff;
			neighborLinkObj.nearestPointOnLink = nearestPoint;

			neighborLinksParallel.push_back(neighborLinkObj);
		}
		// 排序
		std::sort(neighborLinksParallel.begin(), neighborLinksParallel.end(), [](auto& l, auto& r) {return l.directionDiff < r.directionDiff; });

		return neighborLinksParallel;
	}

	DividedRoadSearcher::GrabRoadResult DividedRoadSearcher::grabAnotherRoadZGen(DirLink& link, const MapPoint3D64& pointOnLink, const double pointOnLinkDirection)
	{
		std::set<int64> linkOutlinksUUID;
		{
			std::vector<int64> forwardOutlinks = link.getEndDirectOutlinksExceptSelfUUID();
			std::vector<int64> backwardOutlinks = link.getReverseDirLink().getEndDirectOutlinksExceptSelfUUID();
			linkOutlinksUUID.insert(forwardOutlinks.begin(), forwardOutlinks.end());
			linkOutlinksUUID.insert(backwardOutlinks.begin(), backwardOutlinks.end());
		}

		std::set<std::wstring*> linkNames = link.getLinkNames();

		auto intersectsWithOtherLink = [](std::vector<DbMesh*> meshes, const MapPoint64& segStart, const MapPoint64& segEnd)
			{

				segment_2t bgSegment{ POINT_2T(segStart), POINT_2T(segEnd) };

				// segment缩短约10cm，足矣
				std::array<MapPoint64, 2> segment = map_pt_math::shrinkSegment({ segStart, segEnd }, 100);


				segment_2t bgSegmentNew{ POINT_2T(segment[0]), POINT_2T(segment[1]) };

				// 
				return RTreeSeacher::intersectsWithLinks2d(meshes, segment[0], segment[1]);
			};


		auto isLeftRightParalel = [](const LineString3d& line1, const LineString3d& line2)
			{
				linestring_2t bgLine1 = LINESTRING_2T(line1.vertexes);
				linestring_2t bgLine2 = LINESTRING_2T(line2.vertexes);

				// 取line2的起终点到Line1的最邻近点，如果这两个最邻近点都相同则认为这两条线不是左右平行
				segment_2t seg1;
				bg::closest_points(bgLine2.front(), bgLine1, seg1);

				segment_2t seg2;
				bg::closest_points(bgLine2.back(), bgLine1, seg2);

				// 再加策略，解决两最邻近点是否接近的情况，此时line1和line2还是不足以达到左右平行
				double nearestPointDist = bg::distance(seg1.second, seg2.second);
				double line1Dist = bg::distance(POINT_2T(line1.vertexes.front()), POINT_2T(line1.vertexes.back()));
				double line2Dist = bg::distance(POINT_2T(line2.vertexes.front()), POINT_2T(line2.vertexes.back()));

				bool isLeftRightParalel = (nearestPointDist / line1Dist) > (1.0 / 3.0) || (nearestPointDist / line2Dist) > (1.0 / 3.0);

				return isLeftRightParalel;
			};

		// Note: MapPoint64在以半径为6378137的球体上的精度为0.0011131949079327358米，约为0.111cm
		// 故而此处30000单位最远可找约33.3米
		int maxFindDist = 30000;
		std::vector<NeighborLink> paralelLinks = searchParallelNeighborLinks(link, pointOnLink, maxFindDist, pointOnLinkDirection);
		// 从最平行的道路开始找，最有可能是上下行的另一对
		for (auto& parallelLink : paralelLinks)
		{
			if (Generator::containsLinkNames(parallelLink.link, linkNames) // 名称相同
				&& !intersectsWithOtherLink(m_totalMeshes, pointOnLink.pos, parallelLink.nearestPointOnLink) // 不能横跨其它道路
				&& isLeftRightParalel(link.geometry(), parallelLink.link->geometry) // 确保两条道路是左右平行（类似于 ||）而不是前后平行（类似于 --）
				)
			{
				// 从当前道路指向对侧道路最邻近点的向量转向道路方向是否往左
				double angleToOpposite = map_pt_math::twoPointAngle(pointOnLink.pos, parallelLink.nearestPointOnLink);
				bool linkIsLeft = angleTurnLeft(angleToOpposite, pointOnLinkDirection);

				// 判断当前link与找到的link是否共向，然后生成共向的DirLink
				if (angleIsTheSameDirection(pointOnLinkDirection, parallelLink.direction))
					return GrabRoadResult{ DirLink{ parallelLink.link, true, parallelLink.link->geometry } , !linkIsLeft };
				else
					return GrabRoadResult{ DirLink{ parallelLink.link, false , parallelLink.link->geometry } , !linkIsLeft };
			}
		}

		return GrabRoadResult{};
	}

	DividedRoadSearcher::GrabRoadResult DividedRoadSearcher::grabAnotherRoadZGenMultiTimes(DirLink& link)
	{
		// 从link的三等分点处和终点分别抓路，可能出现多个抓路结果，优先出现次数最多的道路，如果道路出现次数同样多则取最长的那条道路。
		const std::vector<double> pointsPercOnLink{
			1.0 / 2.0,
			1.0 / 3.0,
			2.0 / 3.0,
		};

		std::vector<GrabRoadResult> results;
		for (double perc : pointsPercOnLink)
		{
			MapPoint3D64 pointOnLink = link.calPointAt(perc);
			double pointOnLinkDirection = map_pt_math::twoPointAngle(link.calPointAt(perc - 0.05).pos, link.calPointAt(perc + 0.05).pos);
			GrabRoadResult result = grabAnotherRoadZGen(link, pointOnLink, pointOnLinkDirection);
			results.push_back(result);
		}

		// 统计在不同抓取配置中每个道路被抓取的次数
		std::map<int64, std::size_t> validResultCount;
		std::map<int64, GrabRoadResult*> validResultMap;
		for (auto& result : results)
		{
			if (result.valid())
			{
				validResultCount[result.link.uuid()] += 1;
				validResultMap[result.link.uuid()] = &result;
			}
		}

		// 统计被抓取出现次数最多的道路
		std::vector<GrabRoadResult*> maxValidResults;
		std::size_t maxCount = 0;
		for (auto& entry : validResultCount)
		{
			if (entry.second > maxCount)
			{
				maxValidResults.clear();

				maxCount = entry.second;
				maxValidResults.push_back(validResultMap[entry.first]);
			}
			else if (entry.second == maxCount)
			{
				maxValidResults.push_back(validResultMap[entry.first]);
			}
		}

		// 选择在所有抓路结果中出现次数最多的道路
		// 如果有多个道路抓取次数相同，选择最长的道路

		if (maxValidResults.empty())
		{
			return GrabRoadResult{};
		}
		else if (maxValidResults.size() == 1)
		{
			return *maxValidResults.front();
		}
				else
		{
			double maxLength{ 0.0 };
			GrabRoadResult* finalResult;
			for (GrabRoadResult* result : maxValidResults)
			{
				double len = result->link.calLength();
				if (len > maxLength)
				{
					maxLength = len;
					finalResult = result;
			}
		}

			return *finalResult;
		}
	}

	DirLinkPath DividedRoadSearcher::extendRoad(DirLink& beginLink, bool beginLinkOnLeft, std::set<int64>& visitedLinks)
	{
		auto extendInOneDirection = [&](DirLink curLink, double lastLinkEndAngle, bool nextRoadInCCW) -> DirLinkPath
			{
				DirLinkPath path;

				while (true)
				{
					std::set<std::wstring*> curLinkNames = curLink.getLinkNames();

					int highwayOutlinkNum;
					std::vector<DirLink> outlinks = curLink.getEndDirectOutlinks<true>();

					DirLink nextLink = getNextDirLink(outlinks, curLink, nextRoadInCCW);

					if (!nextLink.valid()
						|| !isDividedRoadZGen(nextLink, true))
						break;

					// 当前道路和下一道路名称相同
					if (!nextLink.containsLinkNames(curLinkNames))
						break;

					// 下一道路已经纳入其它上下行道路
					if (visitedLinks.count(nextLink.uuid()) > 0)
						break;

					double nextLinkStartAngle = nextLink.startHeadingAngle();
					float turnAngle = Math_getTurnAngleF(lastLinkEndAngle * MATH_RADIAN2DEGREE_D, nextLinkStartAngle * MATH_RADIAN2DEGREE_D);
					if (std::abs(turnAngle) < 90.0f)
					{
						break;
					}

					// 路口内道路对转折角要求更高
					if (m_centerLinks.count(nextLink.uuid())
						&& std::abs(turnAngle) < 135.0f)
					{
						break;
					}

					lastLinkEndAngle = nextLink.endHeadingAngle();

					visitedLinks.insert(nextLink.uuid());
					curLink = nextLink;

					path.addPortion(nextLink);
				}

				return path;
			};

		// 往beginLink反方向搜索
		DirLink beginLinkRev = beginLink.getReverseDirLink();
		DirLinkPath pathBackward = extendInOneDirection(beginLinkRev, beginLinkRev.endHeadingAngle(), !beginLinkOnLeft);

		if (pathBackward.valid())
		{
		pathBackward.reverse(); // 此时pathBackward方向已经不是backward了，而是forward
		}
		pathBackward.addPortion(beginLink);

		DirLinkPath pathForward = extendInOneDirection(beginLink, beginLink.endHeadingAngle(), beginLinkOnLeft);

		pathBackward.appendPath(pathForward);

		return pathBackward;
	}

	std::size_t SingleDirRoad::Portion::countVertexNum()
	{
		if (stopReason == StopReason::NONE
			|| stopReason == StopReason::MULTI_OUT_LINKS
			|| stopReason == StopReason::NO_OUTLINKS)
		{
			if (zLevelPointIndex != INVALID_INDEX)
			{
				if (dirLink.forward())
					return dirLink.geometry().vertexes.size() - zLevelPointIndex;
				else
					return zLevelPointIndex + 1;
			}
			else
			{
				return dirLink.geometry().vertexes.size();
			}
		}
		else if (stopReason == StopReason::MEET_Z_LEVEL_POINT)
		{
			if (dirLink.forward())
				return zLevelPointIndex + 1;
			else
				return dirLink.geometry().vertexes.size() - zLevelPointIndex;
		}
		else
		{
			assert(!"Unknown StopReason!");
			return static_cast<std::size_t>(-1);
		}
	}

	SingleDirRoad::Portion SingleDirRoad::Portion::asBeginPortion()
	{
		return Portion{
			dirLink.getReverseDirLink(),
			0.0,
			StopReason::NONE,
			zLevelPointIndex
		};
	}

	bool SingleDirRoad::PortionLessComp::operator()(const Portion& l, const Portion& r) const
	{
		if (l.dirLink.uuid() != r.dirLink.uuid())
			return l.dirLink.uuid() < r.dirLink.uuid();
		if (l.stopReason != r.stopReason)
			return int(l.stopReason) < int(r.stopReason);
		if (l.zLevelPointIndex != r.zLevelPointIndex)
			return l.zLevelPointIndex < r.zLevelPointIndex;

		return static_cast<int>(l.dirLink.forward()) < static_cast<int>(r.dirLink.forward());
	}

	MapPoint3D64* SingleDirRoad::frontPoint()
	{
		assert(!m_portions.empty());

		return &(front().dirLink.geometry().vertexes[m_portions.front().zLevelPointIndex]);
	}

	MapPoint3D64* SingleDirRoad::backPoint()
	{
		assert(!m_portions.empty());

		Portion& lastPortion = back();

		if (lastPortion.stopReason == StopReason::MULTI_OUT_LINKS
			|| lastPortion.stopReason == StopReason::NO_OUTLINKS
			|| lastPortion.stopReason == StopReason::NONE)
		{
			return &lastPortion.dirLink.getEndPointRef();
		}
		else if (lastPortion.stopReason == StopReason::MEET_Z_LEVEL_POINT)
		{
			return &(lastPortion.dirLink.geometry().vertexes[lastPortion.zLevelPointIndex]);
		}
		else
		{
			assert(!"Unknown StopReason!");
			return nullptr;
		}
	}

	void SingleDirRoad::accessVertices(PathVerticesAccessor& vertexModifier)
	{
		if (m_portions.size() < 2)
			return;

		PathLinkVerticesAccessor linkAccessor{ vertexModifier, _calculateTotoalVertexNum() };

		Portion& theFirstPortion = _portionAt(0);
		Portion& theSecondPortion = _portionAt(1);

		if (theSecondPortion.stopReason == StopReason::MULTI_OUT_LINKS
			|| theSecondPortion.stopReason == StopReason::NO_OUTLINKS
			|| theSecondPortion.stopReason == StopReason::NONE)
			theFirstPortion.dirLink.accessVerticesToEnd(theFirstPortion.zLevelPointIndex, linkAccessor);
		else if (theSecondPortion.stopReason == StopReason::MEET_Z_LEVEL_POINT)
			theFirstPortion.dirLink.accessVerticesOrdered(theFirstPortion.zLevelPointIndex, theSecondPortion.zLevelPointIndex, linkAccessor);
		else
			assert(!"Unknown StopReason!");

		for (std::size_t i = 2; i < m_portions.size(); ++i)
		{
			Portion& portion = _portionAt(i);

			if (portion.stopReason == StopReason::NONE
				|| portion.stopReason == StopReason::MULTI_OUT_LINKS
				|| portion.stopReason == StopReason::NO_OUTLINKS)
			{
				if (portion.zLevelPointIndex != INVALID_INDEX)
					portion.dirLink.accessVerticesToEnd(portion.zLevelPointIndex, linkAccessor);
				else
					portion.dirLink.accessVerticesToEnd(linkAccessor);
			}
			else if (portion.stopReason == StopReason::MEET_Z_LEVEL_POINT)
			{
				portion.dirLink.accessVerticesToMid(portion.zLevelPointIndex, linkAccessor);
			}
			else
				assert(!"Unknown StopReason!");
		}
	}

	std::size_t SingleDirRoad::_calculateTotoalVertexNum()
	{
		if (m_portions.size() == 2)
		{
			Portion& theFirstPortion = _portionAt(0);
			Portion& theSecondPortion = _portionAt(1);

			std::size_t totalVertexNum{ 0 };

			if (theSecondPortion.stopReason == StopReason::MULTI_OUT_LINKS
				|| theSecondPortion.stopReason == StopReason::NO_OUTLINKS
				|| theSecondPortion.stopReason == StopReason::NONE)
			{
				if (theFirstPortion.dirLink.forward())
					totalVertexNum = theFirstPortion.dirLink.geometry().vertexes.size() - theFirstPortion.zLevelPointIndex;
				else
					totalVertexNum = theFirstPortion.zLevelPointIndex + 1;
			}
			else if (theSecondPortion.stopReason == StopReason::MEET_Z_LEVEL_POINT)
			{
				return std::abs(theFirstPortion.zLevelPointIndex - theSecondPortion.zLevelPointIndex) + 1;
			}
			else
				assert(!"Unknown StopReason!");

			return totalVertexNum;
		}
		else
		{
			std::size_t totalVertexNum{ 0 };

			totalVertexNum += _portionAt(0).countVertexNum();
			for (std::size_t i = 2; i < m_portions.size(); ++i)
			{
				Portion& portion = _portionAt(i);
				totalVertexNum += portion.countVertexNum();
			}

			return totalVertexNum;
		}
	}


	void GradientVertexModifier::access(MapPoint3D64& vertex, const PathVerticesAccessor::AccessInfo& accessInfo)
	{
		// ignore the first and the last vertex
		if (accessInfo.index != 0 && accessInfo.index + 1 != accessInfo.totalVertexNum)
		{
			double vertexNewHeight = m_heightFrom + (accessInfo.accLength / m_length) * (m_heightTo - m_heightFrom);
			vertex.z = vertexNewHeight;
		}
	}

	LeastHeightVertexModifier::LeastHeightVertexModifier(double heightFrom, double heightTo)
	{
		double maxHeight = std::fmax(heightFrom, heightTo);
		m_leastHeight = maxHeight * 0.16;
	}

	void LeastHeightVertexModifier::access(MapPoint3D64& vertex, const PathVerticesAccessor::AccessInfo& accessInfo)
	{
		// ignore the first and the last vertex
		if (accessInfo.index != 0 && accessInfo.index + 1 != accessInfo.totalVertexNum)
		{
			if (vertex.z < m_leastHeight) vertex.z = m_leastHeight;
		}
	}

	void ConnectedRoadVertexModifier::access(MapPoint3D64& vertex, const PathVerticesAccessor::AccessInfo& accessInfo)
	{
		auto updateVertexHeight = [&accessInfo, &vertex, this]()
			{
				const double MAX_ROAD_LENGTH = 200.0;

				if (m_roadLength < MAX_ROAD_LENGTH) {
					double heightAdded = m_heightDiff * nc_clamp(1.0 - accessInfo.accLength / m_roadLength, 0.0, 1.0);
					vertex.z += heightAdded;
					if (vertex.z < 0) { vertex.z = 0.0; }
				}
				else
				{
					double heightAdded = m_heightDiff * nc_clamp(1.0 - accessInfo.accLength / MAX_ROAD_LENGTH, 0.0, 1.0);
					vertex.z += heightAdded;
					if (vertex.z < 0) { vertex.z = 0.0; }
				}
			};

		if (accessInfo.index < 2)
		{
			m_theFirstTwoPoints[accessInfo.index] = &vertex;

			if (accessInfo.index == 1)
			{
				auto& h = m_mainRoadPoint.z;
				auto& h1 = m_theFirstTwoPoints[0]->z;
				auto& h2 = m_theFirstTwoPoints[1]->z;

				// true -> h处于h1 和 h2 之间
				m_skipTheRestPoints = (h <= h1 && h >= h2) || (h >= h1 && h <= h2);
				if (!m_skipTheRestPoints)
				{
					m_heightDiff = h - h1;
				}
				h1 = h;
			}
		}

		if (!m_skipTheRestPoints)
		{
			updateVertexHeight();
		}
	}

	AdjustZVertexAccessor::AdjustZVertexAccessor(double targetHeight, double firstDeltaHeight, double influenceLength) :
		m_targetHeight(targetHeight), m_influenceLength(influenceLength), m_accLength(0.0), m_firstDeltaHeight(firstDeltaHeight)
	{
	}
	void AdjustZVertexAccessor::accessFirst(MapPoint3D64& point)
	{
		point.z = m_targetHeight;
		m_accLength = 0.0;
	}
	void AdjustZVertexAccessor::accessMiddle(MapPoint3D64& point, MapPoint3D64& prevPoint, double distToPrevPoint)
	{
		UNREFERENCED_PARAMETER(prevPoint);

		m_accLength += distToPrevPoint;
		if (m_accLength < m_influenceLength)
		{
			double percent = 1.0 - m_accLength / m_influenceLength;
			point.z = point.z + m_firstDeltaHeight * percent;
		}
	}

	AdjustZ2VertexAccessor::AdjustZ2VertexAccessor(DirLink& link, double startHeight, double endHeight)
		: m_accessTotalLength(link.length()),
		m_startHeight(startHeight), m_endHeight(endHeight),
		m_startDeltaHeight(startHeight - link.getStartHeight()), m_endDeltaHeight(endHeight - link.getEndHeight())
	{
	}

	void AdjustZ2VertexAccessor::accessFirst(MapPoint3D64& point)
	{
		point.z = m_startHeight;
		m_accLength = 0;
	}

	void AdjustZ2VertexAccessor::accessMiddle(MapPoint3D64& point, MapPoint3D64& prevPoint, double distToPrevPoint)
	{
		UNREFERENCED_PARAMETER(prevPoint);

		m_accLength += distToPrevPoint;
		double percent = m_accLength / m_accessTotalLength;
		point.z = point.z + m_startDeltaHeight + (m_endDeltaHeight - m_startDeltaHeight) * percent;
	}

	void AdjustZ2VertexAccessor::accessLast(MapPoint3D64& point, MapPoint3D64& prevPoint, double distToPrevPoint)
	{
		UNREFERENCED_PARAMETER(prevPoint);
		UNREFERENCED_PARAMETER(distToPrevPoint);

		point.z = m_endHeight;
	}


	PathLinkVerticesAccessor::PathLinkVerticesAccessor(PathVerticesAccessor& vertexAccessor, std::size_t totalVertexNum)
		: m_vertexAccessor(vertexAccessor), m_totalVertexNum(totalVertexNum), m_lastPortionLastPoint(nullptr), m_index(-1), m_accLength(0.0)
	{
	}

	void PathLinkVerticesAccessor::accessFirst(MapPoint3D64& point)
	{
		++m_index;
		if (m_lastPortionLastPoint && point.pos != m_lastPortionLastPoint->pos) // Note: 一般path中上一部分和当前部分首尾点相同，所以距离为0。该情况不大可能出现。
			m_accLength += MapPoint64::geodistance(point.pos, m_lastPortionLastPoint->pos);

		PathVerticesAccessor::AccessInfo accessInfo;
		accessInfo.index = m_index;
		accessInfo.totalVertexNum = m_totalVertexNum;
		accessInfo.accLength = m_accLength;
		m_vertexAccessor.access(point, accessInfo);
	}

	void PathLinkVerticesAccessor::accessMiddle(MapPoint3D64& point, MapPoint3D64& prevPoint, double distToPrevPoint)
	{
		UNREFERENCED_PARAMETER(prevPoint);

		++m_index;
		m_accLength += distToPrevPoint;

		PathVerticesAccessor::AccessInfo accessInfo;
		accessInfo.index = m_index;
		accessInfo.totalVertexNum = m_totalVertexNum;
		accessInfo.accLength = m_accLength;
		m_vertexAccessor.access(point, accessInfo);
	}

	void PathLinkVerticesAccessor::accessLast(MapPoint3D64& point, MapPoint3D64& prevPoint, double distToPrevPoint)
	{
		accessMiddle(point, prevPoint, distToPrevPoint);
		m_lastPortionLastPoint = &point;
	}
}
