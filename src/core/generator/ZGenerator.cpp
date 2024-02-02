#include "stdafx.h"
#include "ZGenerator.h"
#include "algorithm/grap_point_algorithm.h"

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

	//////////////////////////////////////////////////////////////////////////

	#define AdjustHeightInfo_less(o1, o2) ((o1)->priority > (o2)->priority) 
	algorithm_declare(AdjustHeightInfo);
	algorithm_define(AdjustHeightInfo);

	void AdjustHeightQueue::push(const AdjustHeightInfo& data)
	{
		m_data.push_back(data);
		AdjustHeightInfo_push_heap(m_data.begin(), m_data.end());
	}

	void AdjustHeightQueue::pop(AdjustHeightInfo* data)
	{
		AdjustHeightInfo_pop_heap(m_data.begin(), m_data.end());
		*data = m_data.back();
		m_data.pop_back();
	}

	GradientVertexModifier::GradientVertexModifier(double heightFrom, double heightTo, double length) :
		m_heightFrom(heightFrom), m_heightTo(heightTo), m_length(length)
	{
	}

	void GradientVertexModifier::access(MapPoint3D64& vertex, const sd::DirLinkPath::VertexAccessor::AccessInfo& accessInfo)
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

	void LeastHeightVertexModifier::access(MapPoint3D64& vertex, const sd::DirLinkPath::VertexAccessor::AccessInfo& accessInfo)
	{
		// ignore the first and the last vertex
		if (accessInfo.index != 0 && accessInfo.index + 1 != accessInfo.totalVertexNum)
		{
			if (vertex.z < m_leastHeight) vertex.z = m_leastHeight;
		}
	}

	ConnectedRoadVertexModifier::ConnectedRoadVertexModifier(MapPoint3D64& mainRoadPoint, double roadLength) : m_mainRoadPoint(mainRoadPoint)
	{
		m_roadLength = roadLength;
	}

	void ConnectedRoadVertexModifier::access(MapPoint3D64& vertex, const sd::DirLinkPath::VertexAccessor::AccessInfo& accessInfo)
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

	MapPoint3D64* SingleDirRoad::frontPoint()
	{
		assert(!m_portions.empty());

		return &(front().dirLink.link()->geometry.vertexes[m_portions.front().zLevelPointIndex]);
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
			return &(lastPortion.dirLink.link()->geometry.vertexes[lastPortion.zLevelPointIndex]);
		}
		else
		{
			assert(!"Unknown StopReason!");
			return nullptr;
		}
	}

	double SingleDirRoad::getStartHeight()
	{
		return frontPoint()->z;
	}

	double SingleDirRoad::getEndHeight()
	{
		return backPoint()->z;
	}

	double SingleDirRoad::length()
	{
		if (m_portions.empty())
			return 0.0;
		else
			return back().accLength;
	}

	void SingleDirRoad::accessVertices(VertexAccessor& vertexModifier)
	{
		if (m_portions.size() < 2)
			return;

		std::size_t index{ 0 };
		double accLength{ 0.0 };
		const std::size_t totalVertexNum = _calculateTotoalVertexNum();

		_accessVerticesSameLinkPortion(_portionAt(0), _portionAt(1), index, accLength, totalVertexNum, vertexModifier);
		for (std::size_t i = 2; i < m_portions.size(); ++i)
		{
			Portion& portion = _portionAt(i);
			_accessVerticesDifferentLinkPortion(portion, index, accLength, totalVertexNum, vertexModifier);
		}
	}

	void SingleDirRoad::_accessVerticesSameLinkPortion(Portion& portion, Portion& nextPortion, std::size_t& vertexIndex, double& accLength, const std::size_t totalVertexNum, VertexAccessor& vertexModifier)
	{
		assert(portion.zLevelPointIndex != INVALID_INDEX);

		DbLinkVertexAccessor vertexAccessor{ *portion.dirLink.link() };

		if (nextPortion.stopReason == StopReason::MULTI_OUT_LINKS
			|| nextPortion.stopReason == StopReason::NO_OUTLINKS
			|| nextPortion.stopReason == StopReason::NONE)
		{

			if (portion.forward())
				vertexAccessor.accessVerticesToEndForward(portion.zLevelPointIndex, vertexIndex, accLength, totalVertexNum, vertexModifier);
			else
				vertexAccessor.accessVerticesToEndBackward(portion.zLevelPointIndex, vertexIndex, accLength, totalVertexNum, vertexModifier);
		}
		else if (nextPortion.stopReason == StopReason::MEET_Z_LEVEL_POINT)
		{
			vertexAccessor.accessVerticesOrdered(portion.zLevelPointIndex, nextPortion.zLevelPointIndex, vertexIndex, accLength, totalVertexNum, vertexModifier);
		}
		else
			assert(!"Unknown StopReason!");
	}

	void SingleDirRoad::_accessVerticesDifferentLinkPortion(SingleDirRoad::Portion& portion, std::size_t& vertexIndex, double& accLength, const std::size_t totalVertexNum, VertexAccessor& vertexModifier)
	{
		DbLinkVertexAccessor vertexAccessor{ *portion.dirLink.link() };

		if (portion.stopReason == StopReason::NONE
			|| portion.stopReason == StopReason::MULTI_OUT_LINKS
			|| portion.stopReason == StopReason::NO_OUTLINKS)
		{
			if (portion.zLevelPointIndex != INVALID_INDEX)
				if (portion.forward())
					vertexAccessor.accessVerticesToEndForward(portion.zLevelPointIndex, vertexIndex, accLength, totalVertexNum, vertexModifier);
				else
					vertexAccessor.accessVerticesToEndBackward(portion.zLevelPointIndex, vertexIndex, accLength, totalVertexNum, vertexModifier);
			else
				if (portion.forward())
					vertexAccessor.accessVerticesForward(vertexIndex, accLength, totalVertexNum, vertexModifier);
				else
					vertexAccessor.accessVerticesBackward(vertexIndex, accLength, totalVertexNum, vertexModifier);
		}
		else if (portion.stopReason == StopReason::MEET_Z_LEVEL_POINT)
		{
			if (portion.forward())
				vertexAccessor.accessVerticesToMidForward(portion.zLevelPointIndex, vertexIndex, accLength, totalVertexNum, vertexModifier);
			else
				vertexAccessor.accessVerticesToMidBackward(portion.zLevelPointIndex, vertexIndex, accLength, totalVertexNum, vertexModifier);
		}
		else
			assert(!"Unknown StopReason!");
	}

	std::size_t SingleDirRoad::_calculateTotoalVertexNum()
	{
		assert(m_portions.size() >= 2);

		if (m_portions.size() == 2)
			return _calculateSameLinkPortionVertexNum(front(), back());

		std::size_t totalVertexNum{ 0 };

		totalVertexNum += _portionAt(0).countVertexNum();
		for (std::size_t i = 2; i < m_portions.size(); ++i)
		{
			Portion& portion = _portionAt(i);
			totalVertexNum += portion.countVertexNum();
		}

		return totalVertexNum;
	}

	std::size_t SingleDirRoad::_calculateSameLinkPortionVertexNum(Portion& portion, Portion& nextPortion)
	{
		assert(portion.zLevelPointIndex != INVALID_INDEX);

		std::size_t totalVertexNum{ 0 };

		if (nextPortion.stopReason == StopReason::MULTI_OUT_LINKS
			|| nextPortion.stopReason == StopReason::NO_OUTLINKS
			|| nextPortion.stopReason == StopReason::NONE)
		{
			if (portion.forward())
				totalVertexNum = (portion.dirLink.link()->geometry.vertexes.size() - portion.zLevelPointIndex);
			else
				totalVertexNum = portion.zLevelPointIndex + 1;
		}
		else if (nextPortion.stopReason == StopReason::MEET_Z_LEVEL_POINT)
		{
			if (portion.forward())
				totalVertexNum = (nextPortion.zLevelPointIndex - portion.zLevelPointIndex) + 1;
			else
				totalVertexNum = (portion.zLevelPointIndex - nextPortion.zLevelPointIndex) + 1;
		}
		else
			assert(!"Unknown StopReason!");

		return totalVertexNum;
	}

	void SingleDirRoad::addPortion(Portion portion)
	{
		m_portions.push_back(portion);
		DirLinkPath::addPortion(portion);
	}


	std::size_t SingleDirRoad::Portion::countVertexNum()
	{
		if (stopReason == StopReason::NONE
			|| stopReason == StopReason::MULTI_OUT_LINKS
			|| stopReason == StopReason::NO_OUTLINKS)
		{
			if (zLevelPointIndex != INVALID_INDEX)
			{
				if (forward())
					return dirLink.link()->geometry.vertexes.size() - zLevelPointIndex;
				else
					return zLevelPointIndex + 1;
			}
			else
			{
				return dirLink.link()->geometry.vertexes.size();
			}
		}
		else if (stopReason == StopReason::MEET_Z_LEVEL_POINT)
		{
			if (forward())
				return zLevelPointIndex + 1;
			else
				return dirLink.link()->geometry.vertexes.size() - zLevelPointIndex;
		}
		else
		{
			assert(!"Unknown StopReason!");
			return static_cast<std::size_t>(-1);
		}
	}

	/**
	 * @brief 对link对象上的所有z level点按照其shpSeqNum排序
	 */
	static void sortLinkZLevelByShpSeqNum(DbLink* pLink)
	{
		auto linkid = pLink->uuid;
		std::sort(pLink->zLevels.begin(), pLink->zLevels.end(),
			[linkid](auto& first, auto& second)->bool {
				return first->relLink(linkid)->shpSeqNum < second->relLink(linkid)->shpSeqNum;
			});
	}

	/**
	 * @brief 检查z level记录和link对象是否有效。
	 *        无效的情况：一、DBLink对象形状点数量小于2 二、形状点号（索引）不在DbLink对象形状点范围内
	*/
	static bool checkBothZLevelLinkValid(DbZLevel::DbRelLink& relLink, DbLink* pLink)
	{
		size_t count = pLink->geometry.vertexes.size();
		if (count < 2) {
			return false;
		}
		if (relLink.shpSeqNum >= 0 && relLink.shpSeqNum < count) {
			return true;
		}
		return false;
	}

	/**
	 * @brief 检查同一位置的Z Level记录的level随索引递增，不允许出现相同或者减少的情况。
	 * @param relLinks 
	 * @return 
	*/
	static bool checkZLevelAscending(const std::vector<DbZLevel::DbRelLink>& relLinks)
	{
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
	static std::vector<int> findInvalidLinkZLevel(DbLink* pLink)
	{
		std::vector<int> linkInvalidZLevels;
		for (int i = pLink->zLevels.size() - 1; i > 0; --i)
		{
			DbZLevel::DbRelLink* cur = pLink->zLevels[i]->relLink(pLink->uuid);
			DbZLevel::DbRelLink* prev = pLink->zLevels[i - 1]->relLink(pLink->uuid);
			if (prev->shpSeqNum == cur->shpSeqNum)
			{
				linkInvalidZLevels.push_back(i);
				linkInvalidZLevels.push_back(i - 1);
			}
		}
		return linkInvalidZLevels;
	}

	/**
	 * @brief 移除Link上无效的ZLevel对象，同时Z Level对象到link对象数组的Pair也要移除
	 * @param linkInvalidZLevels link对象上的无效z level对象的索引，注意索引保证从大到小顺序，所以后面可以安全移除vector中元素。
	 */
	static void removeLinkInvalidZLevels(DbLink* pLink, 
		const std::vector<int>& linkInvalidZLevels,
		std::vector<std::pair<DbZLevel*, std::vector<DbLink*>>>& zLevelLinks)
	{
		for (int i : linkInvalidZLevels)
		{
			DbZLevel* zLevel = pLink->zLevels[i];
			std::remove_if(zLevelLinks.begin(), zLevelLinks.end(), [&zLevel](auto& item) {return item.first == zLevel; });
			pLink->zLevels.erase(pLink->zLevels.begin() + i);
		}
	}

	MapPoint3D64& SingleDirRoad::Portion::getVertexInDir(std::size_t index)
	{
		OMDB::LineString3d &linkLineString = this->dirLink.link()->geometry;
		return forward() ? linkLineString.vertexes[index] : linkLineString.vertexes[linkLineString.vertexes.size() - 1 - index];
	}

    void ZGenerator::generate(DbMesh* const pMesh)
    {
		// ZLEVEL
		std::vector<std::pair<DbZLevel*, std::vector<DbLink*>>> zLevelLinks;
		for (auto& zl : pMesh->query(RecordType::DB_HAD_ZLEVEL)) {
			DbZLevel* pZLevel = (DbZLevel*)zl;

			if (!checkZLevelAscending(pZLevel->relLinks))
			{
				// TODO: 反馈数据错误
				continue;
			}

			bool hasInvalidLinkZLevelPair = false;
			std::vector<DbLink*> links;
			for (auto& relLink : pZLevel->relLinks) {
				DbLink* pLink = (DbLink*)pMesh->query(relLink.relLinkid, RecordType::DB_HAD_LINK);

				if (!checkBothZLevelLinkValid(relLink, pLink)) {
					hasInvalidLinkZLevelPair = true;
					break;
				} else {
					links.push_back(pLink);
					pLink->zLevels.push_back(pZLevel);
				}
			}

			// 如果存在无效的zlevel&link组合则该出的z level都不处理
			if (!hasInvalidLinkZLevelPair) {
				auto zLevelLinkPair = std::make_pair(pZLevel, links);
				zLevelLinks.push_back(zLevelLinkPair);
			} else {
				// TODO: 反馈数据错误
			}
		}

		for (auto& hl : pMesh->query(RecordType::DB_HAD_LINK)) {
			DbLink* pLink = (DbLink*)hl;
			sortLinkZLevelByShpSeqNum(pLink);

			// 检查是否出现link的z level数组中出现异常情况
			std::vector<int> invalidZLevel = findInvalidLinkZLevel(pLink);
			if (!invalidZLevel.empty())
			{
				// TODO: 反馈数据错误

				removeLinkInvalidZLevels(pLink, invalidZLevel, zLevelLinks);
			}
		}

		// 搜索上下行道路
		std::vector<sd::DividedRoad> dividedRoads = sd::DivicedRoadSearcher(pMesh).searchZGen();

		// 生成高程。高程值将会更新到mesh的link对象里。
		this->generateHeight(zLevelLinks, dividedRoads);
    }

    void ZGenerator::generateRelation(DbMesh* const pMesh, std::vector<DbMesh*>* nearby)
    {
		UNREFERENCED_PARAMETER(pMesh);
		UNREFERENCED_PARAMETER(nearby);

		// TODO: 搜索上下行道路得到所有范围网格的然后再拿去处理。
		
		// TODO: 调用generateHeight方法时，瓦片内的DBLink使用原始对象，相邻瓦片内的DBLink使用复制的对象。
    }

    void ZGenerator::generateHeight(std::vector<std::pair<DbZLevel*, std::vector<DbLink*>>> &zLevelLinks,
		std::vector<sd::DividedRoad>& dividedRoads)
	{
		int checkCount = 0;
		for (int i = 0; i < CHECK_COUNT_LIMIT; i++)
			if (!checkAndAdjustZLevel(zLevelLinks)) {
				checkCount = i;
				break;
			}

		// 使用Z Level点调整道路高程，同时收集与受到调整的道路相连接的道路
		m_roadsModifier.clear();
		m_roadsNeedAdjust.clear();
		processHeightFromZLevelPoints(zLevelLinks);

		// 使得上下行道路高程平齐
		for (sd::DividedRoad& dividedRoad : dividedRoads)
		{
			adjustDividedRoadHeight(dividedRoad);
		}

		// 处理那些与上面受到调整的道路连接的道路的高程，使其连接处高程过度平滑
		CQ_ASSERT(m_roadsNeedAdjust.size() == m_roadsModifier.size());
		for (std::size_t i = 0; i < m_roadsNeedAdjust.size(); ++i)
		{
			m_roadsNeedAdjust[i].accessVertices(m_roadsModifier[i]);
		}
		m_roadsModifier.clear();
		m_roadsNeedAdjust.clear();
	}

	bool ZGenerator::checkAndAdjustZLevel(std::vector<std::pair<DbZLevel*, std::vector<DbLink*>>>& zLevelLinks)
	{
		bool hasAdjusted = false;
		for (auto zLevelLink : zLevelLinks) {
			bool hasIntersectingAdjusted = checkIntersectingInfo(zLevelLink);
			hasAdjusted = (hasAdjusted || hasIntersectingAdjusted);
		}

		return hasAdjusted;
	}

	bool ZGenerator::checkIntersectingInfo(std::pair<DbZLevel*, std::vector<DbLink*>>& zLevelLinkPair)
	{
		bool hasAdjusted = false;
		DbZLevel* pZLevel = zLevelLinkPair.first;
		for (size_t i = 1; i < zLevelLinkPair.second.size(); i++) {
			auto pre = zLevelLinkPair.second[i - 1];
			auto cur = zLevelLinkPair.second[i];
			if (cur->lanePas.empty() || pre->lanePas.empty())
				continue;
			if (!cur->groups.empty())
				continue;

			auto& preRelLink = pZLevel->relLinks[i - 1];
			auto& curRelLink = pZLevel->relLinks[i];

			bool havePedestrian = isPedestrian(cur) || isPedestrian(pre);
			double iz = getZByPoint(curRelLink, cur);
			double jz = getZByPoint(preRelLink, pre);

			double adjustHeight = havePedestrian ?
				ADJUST_HEIGHT_SPACING_PEDESTRIAN : ADJUST_HEIGHT_SPACING;
			double slope = havePedestrian ? ADJUST_ANGLE_PEDESTRIAN : ADJUST_ANGLE;

			if (iz <= jz + MIN_HEIGHT_DIFFERENCE)
			{
				adjustPointHeight(cur, curRelLink.shpSeqNum, iz + adjustHeight, slope);
				hasAdjusted = true;
			}
		}

		return hasAdjusted;
	}

	void ZGenerator::adjustPointHeight(DbLink* pLink, int idx, double targetZ, double slope)
	{
		double influenceLength = 0;
		influenceLength = abs(pLink->geometry.vertexes[idx].z - targetZ) / slope;
		influenceLength /= 100.0; // 转换成米，因为adjustZ中需要influenceLength单位为米
		std::vector<MapPoint3D64> firstHalf;
		std::vector<MapPoint3D64> secondHalf;
		firstHalf.insert(firstHalf.end(), pLink->geometry.vertexes.begin(), pLink->geometry.vertexes.begin() + idx + 1);
		secondHalf.insert(secondHalf.end(), pLink->geometry.vertexes.begin() + idx, pLink->geometry.vertexes.end());
		double remainingFirst = adjustZ(firstHalf, false, targetZ, influenceLength);
		double remainingSecond = adjustZ(secondHalf, true, targetZ, influenceLength);
		pLink->geometry.vertexes.clear();
		pLink->geometry.vertexes.insert(pLink->geometry.vertexes.end(), firstHalf.begin(), firstHalf.end());
		pLink->geometry.vertexes.pop_back();
		pLink->geometry.vertexes.insert(pLink->geometry.vertexes.end(), secondHalf.begin(), secondHalf.end());

		// construct queue and visit
		AdjustHeightQueue adjustQueue;

		// the end of link
		if (remainingSecond > 0)
		{
			sd::DirLink forwardLink{ pLink, true };
			std::vector<sd::DirLink> outlinks = forwardLink.getEndDirectOutlinksExceptSelf();
			for (sd::DirLink& outlink : outlinks)
			{
				AdjustHeightInfo info;
				info.dlink = outlink;
				info.lastHeight = pLink->geometry.vertexes.back().z;
				info.length = remainingSecond;
				info.priority = outlink.calLength();
				adjustQueue.push(info);
			}
		}

		// the start of link
		if (remainingFirst)
		{
			sd::DirLink backwardLink{ pLink, false };
			std::vector<sd::DirLink> outlinks = backwardLink.getEndDirectOutlinksExceptSelf();
			for (sd::DirLink& outlink : outlinks)
			{
				AdjustHeightInfo info;
				info.dlink = outlink;
				info.lastHeight = pLink->geometry.vertexes.front().z;
				info.length = remainingFirst;
				info.priority = outlink.calLength();
				adjustQueue.push(info);
			}
		}

		std::set<int64> adjustedSegment{ pLink->uuid };

		bfs(adjustQueue, adjustedSegment);
	}

	void ZGenerator::adjustDividedRoadHeight(sd::DividedRoad& dividedRoad)
	{
		int32 leftMaxHeight{ 0 };
		std::vector<MapPoint3D64Ref> leftVerteices;
		{
			DirLinkPathVertexRefCollector collector;
			dividedRoad.leftPath.accessVertices(collector);
			leftMaxHeight = collector.maxHeight();
			leftVerteices = collector.verticesRef();
		}
		int32 rightMaxHeight{ 0 };
		std::vector<MapPoint3D64Ref> rightVerteices;
		{
			DirLinkPathVertexRefCollector collector;
			dividedRoad.rightPath.accessVertices(collector);
			rightMaxHeight = collector.maxHeight();
			rightVerteices = collector.verticesRef();
		}

		// 两个上下行道路高程都为0，没必要做后续的调整
		if (leftMaxHeight + rightMaxHeight == 0)
			return;

		auto findNearestPoint = [](const MapPoint3D64& point, std::vector<MapPoint3D64Ref> &linestring,
			MapPoint3D64& nearestPoint, double& distToNearestPoint) // distToNearestPoint单位为米
			{
				// 使用非常原始朴素的方法搜索与指定点最近的linestring上的点
				distToNearestPoint = DBL_MAX;
				for (std::size_t i = 0; i < linestring.size(); ++i)
				{
					double dist = MapPoint64::geodistance(point.pos, linestring[i].get().pos);
					if (dist < distToNearestPoint)
					{
						distToNearestPoint = dist;
						nearestPoint = linestring[i];
					}
				}
			};

		auto adjustHeightBeseOneSide = [&findNearestPoint](std::vector<MapPoint3D64Ref>& baseSide, std::vector<MapPoint3D64Ref>& anotherSide, bool isSecond)
			{
				// 不要修改上下行道路的起终点，且如果下述搜索到的最邻近点是对侧道路的最邻近点，也不做高程处，结合之前的上下行道路搜索逻辑可确保其不会修改出现高程突变情况。
				for (std::size_t i = 1; i < baseSide.size() - 1; ++i)
				{
					auto basePoint = baseSide[i];
					double distToNearestPoint;
					MapPoint3D64 nearestPoint;
					findNearestPoint(basePoint.get(), anotherSide, nearestPoint, distToNearestPoint);

					if (distToNearestPoint < 30.0
						&& nearestPoint != anotherSide.front()
						&& nearestPoint != anotherSide.back())
					{
						if (isSecond)
							basePoint.get().z = nearestPoint.z;
						else
							basePoint.get().z = int32((basePoint.get().z + nearestPoint.z) / 2.0);
					}
				}
			};

		adjustHeightBeseOneSide(rightVerteices, leftVerteices, false);
		adjustHeightBeseOneSide(leftVerteices, rightVerteices, true);

		// 上述调整过程中两相邻link衔接处的点高程可能不一致，此处将其修正为完全相同。
		auto processConnectedPoint = [](sd::DirLinkPath& path)
			{
				for (std::size_t i = 1; i < path.portions().size(); ++i)
				{
					sd::DirLinkPath::Portion& prevPortion = path.portions()[i - 1];
					sd::DirLinkPath::Portion& thisPortion = path.portions()[i];

					int32 maxHeight = max(prevPortion.dirLink.getEndHeight(), thisPortion.dirLink.getStartHeight());
					prevPortion.dirLink.getEndPointRef().z = maxHeight;
					thisPortion.dirLink.getStartPointRef().z = maxHeight;
				}
			};

		processConnectedPoint(dividedRoad.leftPath);
		processConnectedPoint(dividedRoad.rightPath);
	}

	/**
	 * @brief 
	 * @param adjustQueue 
	 * @param adjustedSegment DbLinkd->uuid的数组, uuid能唯一标识该link对象
	*/
	void ZGenerator::bfs(AdjustHeightQueue& adjustQueue, std::set<int64>& adjustedSegment)
	{
		while (adjustQueue.size() > 0)
		{
			AdjustHeightInfo curInfo;
			adjustQueue.pop(&curInfo);
			sd::DirLink curDirLink = curInfo.dlink;
			if (adjustedSegment.count(curDirLink.link()->uuid))
				continue;
			adjustedSegment.insert(curDirLink.link()->uuid);

			double adjustedHeight = NAN;
			std::vector<sd::DirLink> outlinks = curDirLink.getEndDirectOutlinksExceptSelf();
			for (sd::DirLink& outlink : outlinks)
			{
				if (adjustedSegment.count(outlink.link()->uuid))
					adjustedHeight = outlink.getStartHeight();
			}

			double lastEndHeight = curInfo.lastHeight;
			if (isnan(adjustedHeight)) // 头一次搜索到
			{
				double remainingLength = adjustZ(curDirLink.link()->geometry.vertexes, curDirLink.forward(), lastEndHeight, curInfo.length);
				assert(lastEndHeight == curDirLink.getStartHeight());

				if (remainingLength > 0)
				{
					for (sd::DirLink& outlink : outlinks)
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
				adjustZ2(curDirLink.link()->geometry.vertexes, curDirLink.forward(), lastEndHeight, adjustedHeight);
				assert(curDirLink.getStartHeight() == lastEndHeight);
				assert(curDirLink.getEndHeight() == adjustedHeight);
			}
		}
	}

	double ZGenerator::adjustZ(std::vector<MapPoint3D64>& points, bool isSameDir, double targetHeight, double length)
	{
		double sum = 0;
		if (isSameDir)
		{
			double delta = targetHeight - points.front().z;
			for (size_t i = 0; i < points.size(); i++)
			{
				if (i != 0)
				{
					Point pre = Point_make((int)points[i - 1].pos.lon / 1000, (int)points[i - 1].pos.lat / 1000);
					Point now = Point_make((int)points[i].pos.lon / 1000, (int)points[i].pos.lat / 1000);
					sum += Math_segGeoLength(pre, now);
				}

				double percent = 1.f - sum / length;
				if (percent <= 0)
					return 0;
				points[i].z = (i == 0 ? targetHeight : points[i].z + delta * percent);
			}
		}
		else
		{
			double delta = targetHeight - points.back().z;
			for (int i = (int)points.size() - 1; i >= 0; i--)
			{
				if (i != (int)points.size() - 1)
				{
					Point pre = Point_make((int)points[i + 1].pos.lon / 1000, (int)points[i + 1].pos.lat / 1000);
					Point now = Point_make((int)points[i].pos.lon / 1000, (int)points[i].pos.lat / 1000);
					sum += Math_segGeoLength(pre, now);
				}

				double percent = 1.f - sum / length;
				if (percent <= 0)
					return 0;
				points[i].z = (i == (int)points.size() - 1 ? targetHeight : points[i].z + delta * percent);
			}
			points.back().z = targetHeight;
		}

		assert(length > sum);
		return length - sum;
	}

	void ZGenerator::adjustZ2(std::vector<MapPoint3D64>& points, bool isSameDir, double startHeight, double endHeight)
	{
		assert(points.size() >= 2);

		double length = 0;
		double sum = 0;
		for (size_t i = 1; i < points.size(); i++)
		{
			Point pre = Point_make((int)points[i - 1].pos.lon / 1000, (int)points[i - 1].pos.lat / 1000);
			Point now = Point_make((int)points[i].pos.lon / 1000, (int)points[i].pos.lat / 1000);
			length += Math_segGeoLength(pre, now);
		}

		if (isSameDir)
		{
			double startDelta = startHeight - points.front().z;
			double endDealta = endHeight - points.back().z;
			for (size_t i = 0; i < points.size(); i++)
			{
				if (i != 0)
				{
					Point pre = Point_make((int)points[i - 1].pos.lon / 1000, (int)points[i - 1].pos.lat / 1000);
					Point now = Point_make((int)points[i].pos.lon / 1000, (int)points[i].pos.lat / 1000);
					sum += Math_segGeoLength(pre, now);
				}
				double percent = sum / length;
				points[i].z = points[i].z + startDelta + (endDealta - startDelta) * percent;
			}
			points.front().z = startHeight;
			points.back().z = endHeight;
		}
		else
		{
			double startDelta = startHeight - points.back().z;
			double endDealta = endHeight - points.front().z;
			for (int i = (int)points.size() - 1; i >= 0; i--)
			{
				if (i != (int)points.size() - 1)
				{
					Point pre = Point_make((int)points[i + 1].pos.lon / 1000, (int)points[i + 1].pos.lat / 1000);
					Point now = Point_make((int)points[i].pos.lon / 1000, (int)points[i].pos.lat / 1000);
					sum += Math_segGeoLength(pre, now);
				}
				double percent = sum / length;
				points[i].z = points[i].z + startDelta + (endDealta - startDelta) * percent;
			}
			points.back().z = startHeight;
			points.front().z = endHeight;
		}
	}

	void ZGenerator::processHeightFromZLevelPoints(std::vector<std::pair<DbZLevel*, std::vector<DbLink*>>>& zLevelLinks)
	{
		std::unordered_set<int64> processedRoads;

		for (auto zLevelLinkPair : zLevelLinks) {
			DbZLevel* pZLevel = zLevelLinkPair.first;
			for (size_t i = 1; i < zLevelLinkPair.second.size(); i++)
			{
				auto& relLink = pZLevel->relLinks[i];
				auto& pLink = zLevelLinkPair.second[i];
				processHeightByZLevelPoint(pZLevel, relLink, pLink, processedRoads);
			}
		}
	}

	void ZGenerator::processHeightByZLevelPoint(DbZLevel* pZLevel, DbZLevel::DbRelLink& relLink, DbLink*& pLink, std::unordered_set<int64>& processedRoadPortion)
	{
		UNREFERENCED_PARAMETER(pZLevel);

		auto ProcessInOneDirection = [&](bool forward)
		{
			auto DSegmentPointIndexHash = [](int64 uuidWithDir, uint16 index) { return uuidWithDir | (static_cast<uint64>(index) << 48); };
			auto dsegPointIndexHash = DSegmentPointIndexHash(sd::DirLink::linkUUIDWithDirStatic(pLink->uuid, forward), relLink.shpSeqNum);
			if (processedRoadPortion.count(dsegPointIndexHash) == 0)
			{
				processedRoadPortion.insert(dsegPointIndexHash);

				sd::DirLink dirLink{ pLink, forward };
				SingleDirRoad road = extendRoadFromZLevelPoint(dirLink, relLink.shpSeqNum);

				if (road.valid())
				{
					adjustSingleDirRoadHeight(road);
					if (road.back().stopReason == SingleDirRoad::StopReason::MEET_Z_LEVEL_POINT)
					{
						processedRoadPortion.insert(DSegmentPointIndexHash(road.back().dirLink.linkUUIDWithReversedDir(), uint16(road.back().zLevelPointIndex)));
					}
				}
			}
		};

		// process in forward direction
		DSegmentId forwardSeg = DSegmentId_getDSegmentId(pLink->uuid);
		ProcessInOneDirection(forwardSeg);

		// process in backward direction
		DSegmentId backwardSeg = DSegmentId_getReversed(forwardSeg);
		ProcessInOneDirection(backwardSeg);
	}

	SingleDirRoad ZGenerator::extendRoadFromZLevelPoint(sd::DirLink& beginLink, const int beginLinkZLevelPointIndex)
	{
		// some helper functions 
		auto NextZLevelPointIndex = [](DbLink* link, const std::vector<DbZLevel*>& zPoints, int thisZLevelPointSeqNum, bool forward) -> DbZLevel::DbRelLink*
		{
			if (zPoints.empty()) return nullptr;

			if (forward)
			{
				int thisZLevelPointIndex{ INVALID_INDEX };
				for (int i = 0; i + 1 < int(zPoints.size()); ++i)
				{
					if (zPoints[i]->relLink(link->uuid)->shpSeqNum == thisZLevelPointSeqNum)
					{
						thisZLevelPointIndex = i;
						break;
					}
				}
				return thisZLevelPointIndex != INVALID_INDEX ? zPoints[thisZLevelPointIndex + 1]->relLink(link->uuid) : nullptr;
			}
			else
			{
				int thisZLevelPointIndex{ INVALID_INDEX };
				for (int i = int(zPoints.size()) - 1; i > 0; --i)
				{
					if (zPoints[i]->relLink(link->uuid)->shpSeqNum == thisZLevelPointSeqNum)
					{
						thisZLevelPointIndex = i;
						break;
					}
				}
				return thisZLevelPointIndex != INVALID_INDEX ? zPoints[thisZLevelPointIndex - 1]->relLink(link->uuid) : nullptr;
			}
		};
		auto NextZLevelPointIndexNoSearch = [](DbLink* link, const std::vector<DbZLevel*>& zPoints, bool forward) -> DbZLevel::DbRelLink*
		{
			if (zPoints.empty()) return nullptr;

			return forward ? zPoints.front()->relLink(link->uuid) : zPoints.back()->relLink(link->uuid);
		};
		auto FindNextLink = [](sd::DirLink thisLink, std::size_t& totalOutLinkNum, sd::DirLink& nextLink, std::vector<sd::DirLink>& connectedLinks) {
			std::vector<sd::DirLink> outlinks = thisLink.getEndDirectOutlinksExceptSelf();

			totalOutLinkNum = outlinks.size();

			if (totalOutLinkNum == 1)
				nextLink = outlinks[0];
			else if (totalOutLinkNum > 1)
			{
				sd::DirLink suitableOtherRoad;
				int suitableOtherRoadNum = 0;
				auto thisLinkName = Generator::getLinkName(thisLink.link());
				if (thisLinkName != nullptr)
				{
					for (auto& otherLink : outlinks)
					{
						// 道路名称相同、道路等级相同、用途相同
						// TODO 此处需要优化,通过坐标点找link的名称
						if (containsLinkName(otherLink.link(), thisLinkName)
							&& thisLink.link()->kind == otherLink.link()->kind
							//TODO wayTypes字段
							//&& thisRoadAttrs.usage == otherRoadAttrs.usage
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

		sd::DirLink nextLink = beginLink;
		int nextLinkZLevelPointIndex = beginLinkZLevelPointIndex;
		while (true)
		{
			// check if the remain portion of road has a z level point
			DbZLevel::DbRelLink* nextZLevelPoint = nullptr;

			if (nextLink.valid() && nextLink.link()->zLevels.size() != 0)
			{
				if (nextLinkZLevelPointIndex != INVALID_INDEX)
					nextZLevelPoint = NextZLevelPointIndex(nextLink.link(), nextLink.link()->zLevels, nextLinkZLevelPointIndex, nextLink.forward());
				else
					nextZLevelPoint = NextZLevelPointIndexNoSearch(nextLink.link(), nextLink.link()->zLevels, nextLink.forward());
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
			sd::DirLink nextLinkNextLink;
			std::vector<sd::DirLink> connectedLinks;
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
					{
						m_roadsNeedAdjust.push_back(road);
						m_roadsModifier.emplace_back(nextLink.getEndPointRef(), road.length());
					}
				}
				road.addPortion(portion);

				nextLink = nextLinkNextLink;
				nextLinkZLevelPointIndex = INVALID_INDEX;
			}
		}

		return road;
	}

	SingleDirRoad ZGenerator::extendRoadFromDSegment(sd::DirLink& beginLink)
	{
		auto NextZLevelPointIndexNoSearch = [](DbLink* link, const std::vector<DbZLevel*>& zPoints, bool forward) -> DbZLevel::DbRelLink*
		{
			if (zPoints.empty()) return nullptr;

			return forward ? zPoints.front()->relLink(link->uuid) : zPoints.back()->relLink(link->uuid);
		};

		SingleDirRoad road;
		{
			SingleDirRoad::Portion portion;
			portion.dirLink = beginLink;
			portion.zLevelPointIndex = beginLink.forward() ? 0 : int(portion.dirLink.link()->geometry.vertexes.size()) - 1;
			portion.stopReason = SingleDirRoad::StopReason::NONE;
			portion.accLength = 0.0;
			road.addPortion(portion);
		}

		sd::DirLink nextLink = beginLink;
		int nextLinkZLevelPointIndex = road.back().zLevelPointIndex;
		while (true)
		{
			// check if the remain portion of road has a z level point
			DbZLevel::DbRelLink* nextZLevelPoint = nullptr;

			if (nextLink.valid() && nextLink.link()->zLevels.size() != 0)
			{
				nextZLevelPoint = NextZLevelPointIndexNoSearch(nextLink.link(), nextLink.link()->zLevels, nextLink.forward());
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
			std::vector<sd::DirLink> outlinks = nextLink.getEndDirectOutlinksExceptSelf();
			std::size_t nextLinkOutlinksNum{ outlinks.size() };
			sd::DirLink nextLinkNextLink;
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

		std::unique_ptr<SingleDirRoad::VertexAccessor> vertexModifier;

		if (road.hasPassedZeroLevelPoint()) {
			vertexModifier = std::make_unique<LeastHeightVertexModifier>(road.getStartHeight(), road.getEndHeight());
		}
		else
		{
			if (road.back().stopReason == SingleDirRoad::StopReason::MEET_Z_LEVEL_POINT && road.back().accLength < 1500.0)
			{
				vertexModifier = std::make_unique<GradientVertexModifier>(road.getStartHeight(), road.getEndHeight(), road.length());
			}
			else
			{
				vertexModifier = std::make_unique<LeastHeightVertexModifier>(road.getStartHeight(), road.getEndHeight());
			}
		}

		road.accessVertices(*vertexModifier);
	}

	double ZGenerator::getZByPoint(DbZLevel::DbRelLink& relLink, DbLink* pLink)
	{
		return pLink->geometry.vertexes[relLink.shpSeqNum].z;
	}

}
