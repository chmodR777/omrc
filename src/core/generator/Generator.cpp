#include "stdafx.h"
#include "Generator.h"
#include "math3d/vector_math.h"
#include "algorithm/geometry_utils.h"
#include "algorithm/polyline_intersector.h"

namespace OMDB
{

	void Generator::Generate(DbMesh* const pMesh)
	{
		if (!CompileSetting::instance()->isGenerateHdData)
			return;

		generateCoordinatesTransform(pMesh);
		if (!generatorData.coordinatesTransformGenerated)
			return;
		return generate(pMesh);
	}

	void Generator::GenerateRelation(DbMesh* pMesh, std::vector<DbMesh*>* nearby)
	{
		if (!CompileSetting::instance()->isGenerateHdData)
			return;

		generateCoordinatesTransform(pMesh);
		if (!generatorData.coordinatesTransformGenerated)
			return;
		return generateRelation(pMesh, nearby);
	}

	void Generator::generateCoordinatesTransform(DbMesh* const pMesh)
	{
		auto& coordinatesTransform = generatorData.coordinatesTransform;
		if (!generatorData.coordinatesTransformGenerated)
		{
			for (auto obj : pMesh->query(RecordType::DB_HAD_ROAD_BOUNDARY_LINK))
			{
				DbRoadBoundLink* pBoundary = (DbRoadBoundLink*)obj;
				if (!pBoundary->geometry.vertexes.empty())
				{
					coordinatesTransform.setBasePoint(pBoundary->geometry.vertexes.front());
					generatorData.coordinatesTransformGenerated = true;
				}
				if (generatorData.coordinatesTransformGenerated)
					break;
			}
		}
		if (!generatorData.coordinatesTransformGenerated)
		{
			for (auto obj : pMesh->query(RecordType::DB_HAD_LANE_MARK_LINK))
			{
				DbLaneMarkLink* pBoundary = (DbLaneMarkLink*)obj;
				if (!pBoundary->geometry.vertexes.empty())
				{
					coordinatesTransform.setBasePoint(pBoundary->geometry.vertexes.front());
					generatorData.coordinatesTransformGenerated = true;
				}
				if (generatorData.coordinatesTransformGenerated)
					break;
			}
		}
		if (!generatorData.coordinatesTransformGenerated)
		{
			for (auto obj : pMesh->query(RecordType::DB_HAD_LINK))
			{
				DbLink* pLink = (DbLink*)obj;
				if (!pLink->geometry.vertexes.empty())
				{
					coordinatesTransform.setBasePoint(pLink->geometry.vertexes.front());
					generatorData.coordinatesTransformGenerated = true;
				}
				if (generatorData.coordinatesTransformGenerated)
					break;
			}
		}

		this->coordinatesTransform = coordinatesTransform;
	}

	bool Generator::getExpandPolyByOffset(const ring_2t& ring, const int64& offsetSize, ring_2t& resultRing)
	{
		multi_polygon_2t bufferResults;
		bg::strategy::buffer::distance_symmetric<int64> distance_strategy(offsetSize);
		bg::buffer(ring, bufferResults, distance_strategy, 
			generatorData.side_strategy, generatorData.join_strategy, generatorData.end_strategy, generatorData.circle_strategy);
		if (bufferResults.empty())
			return false;
		resultRing = bufferResults[0].outer();
		return true;
	}

	DbRecord* Generator::queryNearby(DbMesh* pMesh, std::vector<DbMesh*>* nearby, int64 id, RecordType objectType)
	{
		UNREFERENCED_PARAMETER(pMesh);
		DbRecord* pRecord = nullptr;
		//DbRecord* pRecord = pMesh->query(id, objectType);
		if (pRecord == nullptr && nearby != nullptr) {
			for (auto grid : *nearby) {
				pRecord = grid->query(id, objectType);
				if (pRecord != nullptr) {
					return pRecord;
				}
			}
		}

		return nullptr;
	}

	DbRecord* Generator::queryOrCreate(DbMesh* pMesh, std::vector<DbMesh*>* nearby, int64 id, RecordType objectType)
	{
		DbRecord* pRecord = pMesh->query(id, objectType);
		if (pRecord == nullptr && nearby != nullptr) {
			pRecord = queryNearby(pMesh, nearby, id, objectType);
			if (pRecord != nullptr) {
				DbRecord* pCreateRecord = pMesh->alloc(objectType);
				*pCreateRecord = *pRecord;
				pMesh->insert(pRecord->uuid, pCreateRecord);
			}
		}
		return pRecord;
	}

	bool Generator::generateHdData(const DbLink* pLink)
	{
		bool generateHdData = false;
		if (CompileSetting::instance()->isNotCompileUrbanData) {
			for (auto& tmpDataLevel : pLink->dataLevels) {
				if (tmpDataLevel.dataLevel != 1)
					return generateHdData;
			}
		}

		for (auto lanePa : pLink->lanePas) {
			if (lanePa->generateHdData) {
				generateHdData = true;
				break;
			}
		}
		return generateHdData;
	}

	float Generator::getLinkLength(DbLink* pLink)
	{
		if (pLink->length > 0)
			return pLink->length;

		double length = 0;
		for (int idx = 0; idx < pLink->geometry.vertexes.size() - 1; idx++) {
			auto& currPt = pLink->geometry.vertexes[idx];
			auto& nextPt = pLink->geometry.vertexes[idx + 1];
			Point curr = Point_make((int)currPt.pos.lon / 1000, (int)currPt.pos.lat / 1000);
			Point next = Point_make((int)nextPt.pos.lon / 1000, (int)nextPt.pos.lat / 1000);
			length += Math_segGeoLength(curr, next);
		}
		pLink->length = length;
		return length;
	}

	std::wstring* Generator::getLinkName(const DbLink* pLink)
	{
		if (pLink->linkNames.empty())
			return nullptr;

		std::set<std::wstring*> names = getLinkNames(pLink);
		return names.size() == 1 ? *names.begin() : nullptr;
	}

	std::set<int> Generator::getLinkNameGroups(const DbLink* pLink)
	{
		std::set<int> nameGroups;
		if (pLink->linkNames.empty())
			return nameGroups;

		for (auto linkName : pLink->linkNames)
			for (auto& linkNamePa : linkName->linkNamePas)
				nameGroups.emplace(linkNamePa.nameGroup);

		return nameGroups;
	}

	std::set<std::wstring*> Generator::getLinkNames(const DbLink* pLink)
	{
		std::set<std::wstring*> names;
		if (pLink->linkNames.empty())
			return names;

		for (auto linkName : pLink->linkNames)
		{
			for (auto& linkNamePa : linkName->linkNamePas)
			{
				for (auto roadName : linkNamePa.roadNames)
				{
					if (!roadName->nameGroups.empty())
					{
						names.emplace(&roadName->nameGroups[0].name);
					}
				}
			}
		}

		return names;
	}

	bool Generator::containsLinkName(const DbLink* pLink, std::wstring* name)
	{
		if (pLink->linkNames.empty() || name == nullptr)
			return false;

		std::set<std::wstring*> names = getLinkNames(pLink);
		for (auto tmpName : names)
		{
			if (*tmpName == *name)
				return true;
		}

		return false;
	}

	bool Generator::containsLinkNames(const DbLink* pLink, std::set<std::wstring*>& names)
	{
		std::set<std::wstring*> linkNames = getLinkNames(pLink);
		for (auto name : names)
		{
			for (auto linkName : linkNames)
			{
				if (*name == *linkName)
					return true;
			}
		}
		return false;
	}

	void Generator::getDSegmentTopoLinks(DbLink* pLink, DSegmentId dsegId,
		std::vector<DbSkeleton*>& previousLinks, std::vector<DbSkeleton*>& nextLinks)
	{
		previousLinks.clear();
		nextLinks.clear();
		if (pLink->direct != 3) 
		{
			previousLinks = getDSegmentPreviousLink(pLink, dsegId);
			nextLinks = getDSegmentNextLink(pLink, dsegId);
		}
		else
		{
			auto reversedDSegId = DSegmentId_getReversed(dsegId);
			previousLinks = getDSegmentPreviousLink(pLink, reversedDSegId);
			nextLinks = getDSegmentNextLink(pLink, reversedDSegId);
		}
	}

	std::vector<DbSkeleton*> Generator::getDSegmentPreviousLink(DbLink* pLink, DSegmentId dSegmenId)
	{
		std::vector<DbSkeleton*> ret;
		DSegmentId tSegmentId = DSegmentId_getDSegmentId(pLink->uuid);
		if (pLink->direct != 1) {
			if (dSegmenId == tSegmentId) { // 正向
				if (pLink->direct == 2) {
					return pLink->previous;
				}
				if (pLink->direct == 3) {
					return pLink->next;
				}
			}
			if (dSegmenId == DSegmentId_getReversed(tSegmentId)) { // 反向
				if (pLink->direct == 2) {
					return pLink->next;
				}
				if (pLink->direct == 3) {
					return pLink->previous;
				}
			}
			return ret;
		}

		// 假定1的默认方向为正向
		if (dSegmenId == tSegmentId) { // 正向
			for (auto sk : pLink->previous) {
				auto skDSegid = getPreviousDSegId(pLink, dSegmenId, (DbLink*)sk);
				if (skDSegid != INVALID_DSEGMENT_ID) {
					ret.push_back(sk);
				}
			}
			return ret;
		}

		if (dSegmenId == DSegmentId_getReversed(tSegmentId)) { // 反向
			for (auto sk : pLink->previous) {
				auto skDSegid = getPreviousDSegId(pLink, dSegmenId, (DbLink*)sk);
				if (skDSegid != INVALID_DSEGMENT_ID) {
					ret.push_back(sk);
				}
			}
			return ret;
		}

		return ret;
	}

	std::vector<DbSkeleton*> Generator::getDSegmentNextLink(DbLink* pLink, DSegmentId dSegmenId)
	{
		std::vector<DbSkeleton*> ret;
		DSegmentId tSegmentId = DSegmentId_getDSegmentId(pLink->uuid);
		if (pLink->direct != 1) {
			if (dSegmenId == tSegmentId) { // 正向
				if (pLink->direct == 2) {
					return pLink->next;
				}
				if (pLink->direct == 3) {
					return pLink->previous;
				}
			}
			if (dSegmenId == DSegmentId_getReversed(tSegmentId)) { // 反向
				if (pLink->direct == 2) {
					return pLink->previous;
				}
				if (pLink->direct == 3) {
					return pLink->next;
				}
			}
			return ret;
		}

		// 假定1的默认方向为正向
		if (dSegmenId == tSegmentId) { // 正向
			for (auto sk : pLink->next) {
				auto skDSegid = getNextDSegId(pLink, dSegmenId, (DbLink*)sk);
				if (skDSegid != INVALID_DSEGMENT_ID) {
					ret.push_back(sk);
				}
			}
			return ret;
		}

		if (dSegmenId == DSegmentId_getReversed(tSegmentId)) { // 反向
			for (auto sk : pLink->next) {
				auto skDSegid = getNextDSegId(pLink, dSegmenId, (DbLink*)sk);
				if (skDSegid != INVALID_DSEGMENT_ID) {
					ret.push_back(sk);
				}
			}
			return ret;
		}

		return ret;
	}

	DSegmentId Generator::getPreviousDSegId(DbLink* thisLink, DSegmentId thisDSegid, DbLink* previousLink)
	{
		auto previousDSegid = DSegmentId_getDSegmentId(previousLink->uuid);
		auto thisStartNode = getDSegmentStartNode(thisLink, thisDSegid);
		auto previousEndNode = getDSegmentEndNode(previousLink, previousDSegid);
		if (thisStartNode == previousEndNode)
		{
			return previousDSegid;
		}

		auto reversedPreviousDSegid = DSegmentId_getReversed(previousDSegid);
		auto reversedPreviousEndNode = getDSegmentEndNode(previousLink, reversedPreviousDSegid);
		if (thisStartNode == reversedPreviousEndNode)
		{
			return reversedPreviousDSegid;
		}

		return INVALID_DSEGMENT_ID;
	}

	DSegmentId Generator::getNextDSegId(DbLink* thisLink, DSegmentId thisDSegid, DbLink* nextLink)
	{
		auto nextDSegid = DSegmentId_getDSegmentId(nextLink->uuid);
		auto thisEndNode = getDSegmentEndNode(thisLink, thisDSegid);
		auto nextStartNode = getDSegmentStartNode(nextLink, nextDSegid);
		if (thisEndNode == nextStartNode)
		{
			return nextDSegid;
		}

		auto reversedNextDSegid = DSegmentId_getReversed(nextDSegid);
		auto reversedNextStartNode = getDSegmentStartNode(nextLink, reversedNextDSegid);
		if (thisEndNode == reversedNextStartNode)
		{
			return reversedNextDSegid;
		}

		return INVALID_DSEGMENT_ID;
	}

	int64 Generator::getDSegmentStartNode(DbLink* pLink, DSegmentId dSegmenId)
	{
		DSegmentId tSegmentId = DSegmentId_getDSegmentId(pLink->uuid);
		if (pLink->direct != 1) {
			if (dSegmenId == tSegmentId) { // 正向
				if (pLink->direct == 2) {
					return pLink->startNode;
				}
				if (pLink->direct == 3) {
					return pLink->endNode;
				}
			}
			if (dSegmenId == DSegmentId_getReversed(tSegmentId)) { // 反向
				if (pLink->direct == 2) {
					return pLink->endNode;
				}
				if (pLink->direct == 3) {
					return pLink->startNode;
				}
			}
			return false;
		}

		// 假定1的默认方向为正向
		if (dSegmenId == tSegmentId) { // 正向
			return pLink->startNode;
		}

		if (dSegmenId == DSegmentId_getReversed(tSegmentId)) { // 反向
			return pLink->endNode;
		}

		// should never reach here
		return INVALID_INDEX;
	}

	int64 Generator::getDSegmentEndNode(DbLink* pLink, DSegmentId dSegmenId)
	{
		DSegmentId tSegmentId = DSegmentId_getDSegmentId(pLink->uuid);
		if (pLink->direct != 1) {
			if (dSegmenId == tSegmentId) { // 正向
				if (pLink->direct == 2) {
					return pLink->endNode;
				}
				if (pLink->direct == 3) {
					return pLink->startNode;
				}
			}
			if (dSegmenId == DSegmentId_getReversed(tSegmentId)) { // 反向
				if (pLink->direct == 2) {
					return pLink->startNode;
				}
				if (pLink->direct == 3) {
					return pLink->endNode;
				}
			}
			return false;
		}

		// 假定1的默认方向为正向
		if (dSegmenId == tSegmentId) { // 正向
			return pLink->endNode;
		}

		if (dSegmenId == DSegmentId_getReversed(tSegmentId)) { // 反向
			return pLink->startNode;
		}

		// should never reach here
		return INVALID_INDEX;
	}

	bool Generator::getDSegmentDir(DbLink* pLink, DSegmentId dSegmenId)
	{
		DSegmentId tSegmentId = DSegmentId_getDSegmentId(pLink->uuid);
		if (pLink->direct != 1) {
			if (dSegmenId == tSegmentId) { // 正向
				if (pLink->direct == 2) {
					return true;
				}
				if (pLink->direct == 3) {
					return false;
				}
			}
			if (dSegmenId == DSegmentId_getReversed(tSegmentId)) { // 反向
				if (pLink->direct == 2) {
					return false;
				}
				if (pLink->direct == 3) {
					return true;
				}
			}
			return false;
		}

		// 假定1的默认方向为正向
		if (dSegmenId == tSegmentId) { // 正向
			return true;
		}

		if (dSegmenId == DSegmentId_getReversed(tSegmentId)) { // 反向
			return false;
		}

		// should never reach here
		return false;
	}

	int Generator::getDSegmentDirect(DbLink* pLink, DSegmentId dSegmenId)
	{
		DSegmentId tSegmentId = DSegmentId_getDSegmentId(pLink->uuid);
		if (pLink->direct != 1) {
			return pLink->direct;
		}

		// 假定1的默认方向为正向
		if (dSegmenId == tSegmentId) { // 正向
			return 2;
		}

		if (dSegmenId == DSegmentId_getReversed(tSegmentId)) { // 反向
			return 3;
		}

		// should never reach here
		return INVALID_INDEX;
	}

	DSegmentId Generator::getDirectDSegment(DbLink* pLink, int direct)
	{
		DSegmentId tSegmentId = DSegmentId_getDSegmentId(pLink->uuid);
		if (direct == getDSegmentDirect(pLink, tSegmentId)) { // 正向
			return tSegmentId;
		}

		DSegmentId reversedDSegId = DSegmentId_getReversed(tSegmentId);
		if (direct == getDSegmentDirect(pLink, reversedDSegId)) { // 反向
			return reversedDSegId;
		}

		// should never reach here
		return INVALID_DSEGMENT_ID;
	}

	bool Generator::sideEqual(DbRoadBoundLink* const pBoundary, DbRdLinkLanePa* const lanePa, int side)
	{
		if (pBoundary == nullptr || lanePa == nullptr)
			return false;
		if (pBoundary->relLgs.count(lanePa->uuid) && pBoundary->relLgs[lanePa->uuid].side == side)
			return true;
		return false;
	}

	bool Generator::directionEqual(DbLaneMarkLink* const pBoundary, DbRdLinkLanePa* const lanePa, int direction)
	{
		if (pBoundary == nullptr || lanePa == nullptr)
			return false;
		if (pBoundary->relLgs.count(lanePa->uuid) && pBoundary->relLgs[lanePa->uuid].lgMarkDirect == direction)
			return true;
		return false;
	}

	bool Generator::directionEqual(DbRoadBoundLink* const pBoundary, DbRdLinkLanePa* const lanePa, int direction)
	{
		if (pBoundary == nullptr || lanePa == nullptr)
			return false;
		if (pBoundary->relLgs.count(lanePa->uuid) && pBoundary->relLgs[lanePa->uuid].direction == direction)
			return true;
		return false;
	}

	std::pair<int64, DbRdLinkLanePa::DbRelLink> Generator::getRelLinkPair(const DbLink* pLink, const DbRdLinkLanePa* lanePa)
	{
		std::pair<int64, DbHadLinkLanePa::DbRelLink> relLinkPair;
		for (auto& pair : lanePa->relLinks) {
			if (pair.second.relLinkid == pLink->uuid) {
				relLinkPair = pair;
				break;
			}
		}
		return relLinkPair;
	}

	std::vector<DbRdLinkLanePa*> Generator::getLanePas(const DbLink* pLink, const std::vector<DbRdLinkLanePa*>& lanePas, int direct)
	{
		std::vector<DbRdLinkLanePa*> grps;
		for (auto lanePa : lanePas) {
			for (auto& relLinkPair : lanePa->relLinks) {
				if (relLinkPair.second.relLinkid == pLink->uuid) {
					if (relLinkPair.second.directType == direct) {
						if (std::find(grps.begin(), grps.end(), lanePa) == grps.end()) {
							grps.push_back(lanePa);
							break;
						}
					}
				}
			}
		}

		std::sort(grps.begin(), grps.end(),
			[&](const DbRdLinkLanePa* first, const DbRdLinkLanePa* second)->bool {
				auto relLinkFirst = getRelLinkPair(pLink, first);
				auto relLInkSecond = getRelLinkPair(pLink, second);
				if (relLinkFirst.second.startOffset != relLInkSecond.second.startOffset)
					return relLinkFirst.second.startOffset < relLInkSecond.second.startOffset;
				return relLinkFirst.second.endOffset < relLInkSecond.second.endOffset;
			}
		);
		return grps;
	}

	void Generator::getLanePaBoundary(DbRdLinkLanePa* lanePa, LineString3d& leftSide, LineString3d& rightSide)
	{
		// 使用道路边界和车道边界
		if (lanePa->roadBoundaries.size() == 2)
		{
			leftSide = lanePa->roadBoundaries[0]->geometry;
			if (directionEqual(lanePa->roadBoundaries[0], lanePa, 3))
			{
				std::reverse(leftSide.vertexes.begin(), leftSide.vertexes.end());
			}
			rightSide = lanePa->roadBoundaries[1]->geometry;
			if (directionEqual(lanePa->roadBoundaries[1], lanePa, 3))
			{
				// 反向，进行点的反转
				std::reverse(rightSide.vertexes.begin(), rightSide.vertexes.end());
			}
		}
		else if (lanePa->laneBoundaries.size() >= 2)
		{
			leftSide = lanePa->laneBoundaries[0]->geometry;
			if (directionEqual(lanePa->laneBoundaries[0], lanePa, 3))
			{
				std::reverse(leftSide.vertexes.begin(), leftSide.vertexes.end());
			}
			auto rightIdx = lanePa->laneBoundaries.size() - 1;
			rightSide = lanePa->laneBoundaries[rightIdx]->geometry;
			if (directionEqual(lanePa->laneBoundaries[rightIdx], lanePa, 3))
			{
				// 反向，进行点的反转
				std::reverse(rightSide.vertexes.begin(), rightSide.vertexes.end());
			}
		}
		auto leftSideIp = std::unique(leftSide.vertexes.begin(), leftSide.vertexes.end(), mapPoint3D64_compare);
		leftSide.vertexes.resize(std::distance(leftSide.vertexes.begin(), leftSideIp));
		auto rightSideIp = std::unique(rightSide.vertexes.begin(), rightSide.vertexes.end(), mapPoint3D64_compare);
		rightSide.vertexes.resize(std::distance(rightSide.vertexes.begin(), rightSideIp));
	}

	void Generator::makeLanePaPolygon(const std::vector<MapPoint3D64>& leftSide, const std::vector<MapPoint3D64>& rightSide, Polygon3d& polygon)
	{
		polygon.vertexes.clear();
		MapPoint3D64 prevVertex = {};
		for_each(leftSide.begin(), leftSide.end(), [&](auto& vertex) {
			if (!floatEqual(vertex.pos.distanceSquare(prevVertex.pos), 0)) {
				polygon.vertexes.push_back(vertex);
				prevVertex = vertex;
			}
			});
		for_each(rightSide.rbegin(), rightSide.rend(), [&](auto& vertex) {
			if (!floatEqual(vertex.pos.distanceSquare(prevVertex.pos), 0)) {
				polygon.vertexes.push_back(vertex);
				prevVertex = vertex;
			}
			});

		// 闭环
		MapPoint3D64& startPt = polygon.vertexes.front();
		MapPoint3D64& endPt = polygon.vertexes.back();
		if (!floatEqual(startPt.pos.distanceSquare(endPt.pos), 0)) {
			polygon.vertexes.push_back(startPt);
		}
	}

	MapPoint3D64 Generator::getBoundaryVector(std::vector<MapPoint3D64>& vertexes)
	{
		MapPoint3D64 vec = {};
		for (int idx = 0; idx < vertexes.size() - 1; idx++) {
			auto& currPt = vertexes[idx];
			auto& nextPt = vertexes[idx + 1];
			MapPoint3D64 tmpVec = nextPt - currPt;
			vec = vec + tmpVec;
		}
		return vec;
	}

	double Generator::getLanePaWidth(const DbLink* pLink, const DbRdLinkLanePa* lanePa)
	{
		//return getLanePaWidthByLaneNum(pLink, lanePa);
		return getLanePaWidthByKind(pLink, lanePa);
	}

	double Generator::getLanePaWidthByLaneNum(const DbLink* pLink, const DbRdLinkLanePa* lanePa)
	{
		UNREFERENCED_PARAMETER(pLink);
		return 3500.0 * lanePa->relLinkLanes.size();
	}

	double Generator::getLanePaWidthByKind(const DbLink* pLink, const DbRdLinkLanePa* lanePa)
	{
		UNREFERENCED_PARAMETER(lanePa);
		float roadWidth = 3500.0f;
		switch (pLink->kind)
		{
		case 1:
		case 2:
		case 3:
		case 4:
		{
			if (pLink->lane_num == 3)
			{
				roadWidth = 10000.0f;
			}
			else if (pLink->lane_num == 2)
			{
				roadWidth = 6000.0f;
			}
			else
			{
				roadWidth = 3500.0f;
			}
		}
		break;
		case 5:
		case 6:
		case 7:
		{
			if (pLink->lane_num == 3)
			{
				roadWidth = 9000.0f;
			}
			else if (pLink->lane_num == 2)
			{
				roadWidth = 6000.0f;
			}
			else
			{
				roadWidth = 3500.0f;
			}
		}
		break;
		case 8:
		case 9:
		{
			if (pLink->lane_num >= 2)
			{
				roadWidth = 5000.0f;
			}
			else
			{
				roadWidth = 3500.0f;
			}
		}
		break;
		}
		return roadWidth;
	}

	void Generator::getLanePasBoundary(std::vector<DbRdLinkLanePa*>& lanePas, int direct, LineString3d& leftSide, LineString3d& rightSide)
	{
		auto mergeBoundary = [](LineString3d& src, LineString3d& dst) {
			MapPoint3D64 prevVertex = {};
			if (!dst.vertexes.empty())
				prevVertex = dst.vertexes.back();
			for_each(src.vertexes.begin(), src.vertexes.end(), [&](MapPoint3D64& vertex) {
				const float CONNECT_POINT_IN_EPSILON = 200.f;  // ≈20cm
				double distance = vertex.pos.distance(prevVertex.pos);
				if (!floatEqualWithEpsilon(distance, 0, CONNECT_POINT_IN_EPSILON)) {
					dst.vertexes.push_back(vertex);
					prevVertex = vertex;
				}
			});
		};

		if (direct != 3)
		{
			if (!lanePas.empty()) {
				auto lanePa = lanePas.front();
				getLanePaBoundary(lanePa, leftSide, rightSide);
			}
			for (int idx = 1; idx < (int)lanePas.size(); idx++) {
				LineString3d left, right;
				auto lanePa = lanePas[idx];
				getLanePaBoundary(lanePa, left, right);

				mergeBoundary(left, leftSide);
				mergeBoundary(right, rightSide);
			}
		}
		else
		{
			if (!lanePas.empty()) {
				auto lanePa = lanePas.back();
				getLanePaBoundary(lanePa, leftSide, rightSide);
			}
			for (int idx = (int)lanePas.size() - 2; idx >= 0; idx--) {
				LineString3d left, right;
				auto lanePa = lanePas[idx];
				getLanePaBoundary(lanePa, left, right);

				mergeBoundary(left, leftSide);
				mergeBoundary(right, rightSide);
			}
		}
	}

	void Generator::generateLanePaBoundary(
		const double lanePaAngle,
		const std::vector<MapPoint3D64>& firstLine, 
		const std::vector<MapPoint3D64>& secondLine, 
		std::vector<MapPoint3D64>& boundaryVertexes)
	{
		if (firstLine.size() < 2 || secondLine.size() < 2)
			return;

		auto& startMapPoint = *firstLine.rbegin();
		auto& prevStartPoint = *(firstLine.rbegin() + 1);

		auto& endMapPoint = *secondLine.begin();
		auto& nextEndPoint = *(secondLine.begin() + 1);
		generateLanePaBoundary(lanePaAngle, startMapPoint, endMapPoint, prevStartPoint, nextEndPoint, boundaryVertexes);
	}

	void Generator::generateLanePaBoundary(double lanePaAngle, MapPoint3D64 startMapPoint, MapPoint3D64 endMapPoint,
		MapPoint3D64 prevStartPoint, MapPoint3D64 nextEndPoint, std::vector<MapPoint3D64>& boundaryVertexes)
	{
		boundaryVertexes.push_back(startMapPoint);
		const auto& startPt = POINT_2T(startMapPoint);
		const auto& endPt = POINT_2T(endMapPoint);
		const auto& prevPt = POINT_2T(prevStartPoint);
		const auto& nextPt = POINT_2T(nextEndPoint);
		point_2t startPtDir = { startPt.get<1>() - prevPt.get<1>() , startPt.get<0>() - prevPt.get<0>() };
		point_2t endPtDir = { nextPt.get<1>() - endPt.get<1>() , nextPt.get<0>() - endPt.get<0>() };
		segment_2t startSeg = segment_2t(startPt, prevPt);
		segment_2t endSeg = segment_2t(endPt, nextPt);
		double disP = bg::distance(startPt, endPt);
		double disPaEx = bg::distance(startPt, SEGMENT_2T_EX(endSeg, 1.0e6));
		double disPbEx = bg::distance(endPt, SEGMENT_2T_EX(startSeg, 1.0e6));
		double disPa = bg::distance(startPt, endSeg);
		double disPb = bg::distance(endPt, startSeg);
		double kPa = disPaEx / disP;
		double kPb = disPbEx / disP;
		double startEndPtDirDegree = minimalDegree(startPtDir, endPtDir);
		if (startEndPtDirDegree > 30 && kPa > 0.2 && kPb > 0.2)
		{
			int numSegments = 10;
			auto p0 = POINT_T(startMapPoint);
			auto p2 = POINT_T(endMapPoint);
			segment_2t tmpSeg = segment_2t(P3_P2(p0), P3_P2(p2));
			tmpSeg = SEGMENT_2T_IN(tmpSeg, 0.01);
			// 使用外面计算的两条道路边界左右关系
			bool isOut = lanePaAngle > 0;

			auto vd = S3_V3(p0, p2);
			vector_t vn = isOut ? bg::cross_product(vector_t(0.0, 0.0, 1.0), vd) : bg::cross_product(vector_t(0.0, 0.0, -1.0), vd);
			double var = isOut ? 0.3 : 0.2;
			auto vnUnit = V3_N(vn);
			double factor = std::abs(bg::distance(p0, p2)) * var;
			point_t p1 = point_t(
				(p0.get<0>() + p2.get<0>()) / 2 + vnUnit.get<0>() * factor,
				(p0.get<1>() + p2.get<1>()) / 2 + vnUnit.get<1>() * factor,
				(p0.get<2>() + p2.get<2>()) / 2);
			for (int j = 1; j < numSegments; j++)
			{
				double t = static_cast<double>(j) / numSegments;
				point_t curvePoint = calculateBezierPoint(p0, p1, p2, t);
				auto curvePointMap = MapPoint3D64_make(curvePoint.get<0>(), curvePoint.get<1>(), curvePoint.get<2>() / 10);
				// coordinatesTransform.invert(&curvePointMap, 1);
				boundaryVertexes.push_back(curvePointMap);
			}
		}
		boundaryVertexes.push_back(endMapPoint);
	}

	std::vector<boundaryPoint> Generator::buildBoundaryPoints(const std::vector<MapPoint3D64>& originPoints)
	{
		std::vector<boundaryPoint> tmpBoundaryPoints;
		if (originPoints.empty())
			return tmpBoundaryPoints;
		double tmpLength = 0;
		tmpBoundaryPoints.emplace_back(originPoints.front(), 0.0, 0);
		for (size_t i = 1; i < originPoints.size(); ++i)
		{
			double twoPointsDis = bg::distance(POINT_T(originPoints[i - 1]), POINT_T(originPoints[i]));
			tmpLength += twoPointsDis;
			tmpBoundaryPoints.emplace_back(originPoints[i], tmpLength, i);
		}
		return tmpBoundaryPoints;
	}

	LineString3d Generator::getBoundaryByOffset(std::vector<MapPoint3D64>& originPoints, std::vector<boundaryPoint> boundaryPoints, double startOffset, double endOffset)
	{
		LineString3d tmpLine;
		std::vector<MapPoint3D64> tmpPoints;
		if (originPoints.empty())
			return tmpLine;
		point_t sp;
		auto originLine = LINESTRING_T(originPoints);
		bg::line_interpolate(originLine, startOffset, sp);
		boundaryPoints.emplace_back(MapPoint3D64_make(sp.get<0>(), sp.get<1>(), sp.get<2>() / 10), startOffset, -1);
		point_t ep;
		bg::line_interpolate(originLine, endOffset, ep);
		boundaryPoints.emplace_back(MapPoint3D64_make(ep.get<0>(), ep.get<1>(), ep.get<2>() / 10), endOffset, -2);
		std::sort(boundaryPoints.begin(), boundaryPoints.end(), [&](boundaryPoint& a, boundaryPoint& b)->bool { return a._fractionDistance < b._fractionDistance; });
		auto isOffsetPt = [](boundaryPoint& bPt) { return bPt._id == -1 || bPt._id == -2; };
		for (auto it = std::begin(boundaryPoints); it != std::end(boundaryPoints);) 
		{
			auto& currPt = *it;
			if ((it + 1) == std::end(boundaryPoints)) { // end
				break;
			}

			auto& nextPt = *(it + 1);
			if ((!isOffsetPt(currPt) && !isOffsetPt(nextPt)) ||
				(isOffsetPt(currPt) && isOffsetPt(nextPt))) {
				++it;
				continue;
			}

			if (currPt._originPoint.pos.distance(nextPt._originPoint.pos) < 10) {
				if (isOffsetPt(currPt)) {
					currPt._originPoint = nextPt._originPoint;
					it = boundaryPoints.erase(it + 1);
					it--;
					continue;
				}
				if (isOffsetPt(nextPt)) {
					nextPt._originPoint = currPt._originPoint;
					it = boundaryPoints.erase(it);
					continue;
				}
			}
			++it;
		}

		for (auto it = std::begin(boundaryPoints); it != std::end(boundaryPoints); ++it)
		{
			if (it->_id == -1)
			{
				tmpPoints.push_back(it->_originPoint);
				while (it != std::end(boundaryPoints))
				{
					it++;
					tmpPoints.push_back(it->_originPoint);
					if (it->_id == -2)
						break;
				};
				break;
			}
		}

		tmpLine.vertexes = tmpPoints;
		return tmpLine;
	}

	LineString3d Generator::adjustBoundaryOffset(LineString3d& location, int offset)
	{
		static const double tolerance = 10;
		auto ip = std::unique(location.vertexes.begin(), location.vertexes.end(), mapPoint3D64_compare);
		location.vertexes.resize(std::distance(location.vertexes.begin(), ip));
		if (std::abs(offset) == 0)
			return location;

		auto getOffsetXY = [offset](MapPoint3D64& fromPt, MapPoint3D64& toPt, double& offsetX, double& offsetY) {
			auto r = std::atan2(toPt.pos.lon - fromPt.pos.lon, toPt.pos.lat - fromPt.pos.lat) + MATH_PI_D / 2;
			offsetX = std::sin(r) * offset;
			offsetY = std::cos(r) * offset;
		};

		LineString3d newLoc;
		int endIdx = location.vertexes.size() - 1;
		for (int idx = 0; idx < endIdx; idx++) {
			auto currPt = location.vertexes[idx];
			auto nextPt = location.vertexes[idx + 1];
			double currX, currY;
			getOffsetXY(currPt, nextPt, currX, currY);
			if (idx == 0) {
				currPt.pos.lon += currX;
				currPt.pos.lat += currY;
				newLoc.vertexes.push_back(currPt);
			}
			if (idx != 0 && idx != endIdx) {
				// 获取前一段segment
				auto& prevPt = newLoc.vertexes.back();
				auto& origPrevPt = location.vertexes[idx - 1];
				double prevX, prevY;
				getOffsetXY(origPrevPt, currPt, prevX, prevY);
				auto firstCurrPt = currPt;
				firstCurrPt.pos.lon += prevX;
				firstCurrPt.pos.lat += prevY;

				// 获取后一段segment
				auto secondCurrPt = currPt;
				secondCurrPt.pos.lon += currX;
				secondCurrPt.pos.lat += currY;
				auto tmpNextPt = nextPt;
				tmpNextPt.pos.lon += currX;
				tmpNextPt.pos.lat += currY;

				MapPoint3D64 intersectPt = {};
				double distance = firstCurrPt.pos.distance(secondCurrPt.pos);
				if (floatEqualWithEpsilon(distance, 0, 30)) {
					newLoc.vertexes.push_back(secondCurrPt);
				}
				else if (PolylineIntersector::intersect(prevPt, firstCurrPt, secondCurrPt, tmpNextPt, tolerance, intersectPt, false) == 1) {
					currPt = intersectPt;
					newLoc.vertexes.push_back(currPt);
				}
				else {
					newLoc.vertexes.push_back(firstCurrPt);
					newLoc.vertexes.push_back(secondCurrPt);
				}
			}
			if (idx == endIdx - 1) {
				nextPt.pos.lon += currX;
				nextPt.pos.lat += currY;
				newLoc.vertexes.push_back(nextPt);
			}
		}
		return newLoc;
	}

	template <typename T>
	const MapPoint3D64& unwrapMapPoint3D64(const T& t) { return t; }
	template <>
	const MapPoint3D64& unwrapMapPoint3D64(const MapPoint3D64Ref& t) { return t.get(); }

	template<typename TPoint>
	DividedRoadNearestResult findDividedRoadInsertPosition(const MapPoint3D64& point, const std::vector<TPoint>& linestring, bool forwardSearch, std::size_t lastSegmentIndex)
	{
		using Searcher = NearestPointOnLineSegmentsSearcher;

		Searcher::Point searcherPoint = Searcher::Point_make(point);
		Searcher::SearchResult nearestResult = Searcher::SearchResult::invalidObj();

		// [indexFrom, indexTo)，同时确保indexTo落在Linestring索引范围内使得(v[i], v[i+1])有效。
		int indexFrom = forwardSearch ? int(lastSegmentIndex) : int(linestring.size() - 1) - int(lastSegmentIndex);
		int indexTo = forwardSearch ? int(linestring.size() - 1) : 0;
		int step = forwardSearch ? 1 : -1;
		for (int i = indexFrom; i < indexTo; i += step)
		{
			const MapPoint3D64& thisVertex = unwrapMapPoint3D64<TPoint>(linestring[i]);
			const MapPoint3D64& nextVertex = unwrapMapPoint3D64<TPoint>(linestring[i + step]);
			Searcher::Segment seg = Searcher::Segment_make(thisVertex, nextVertex);
			if (thisVertex == nextVertex)
			{
				double sqDist = Searcher::PointSqDist(searcherPoint, Searcher::Point_make(thisVertex));
				if (sqDist < nearestResult.sqDist)
				{
					nearestResult.point = seg[0];
					nearestResult.pointPosition = Searcher::NearestPointPosition::SEGMENT_MIDDLE;
					nearestResult.sqDist = sqDist;
					nearestResult.nearestSegmentIndex = i;

					lastSegmentIndex = std::size_t(i);
				}
				else if (sqDist > nearestResult.sqDist)
					break;
			}
			else
			{
				auto result = Searcher::findNearestPointToSegment(seg, searcherPoint);
				if (result.sqDist < nearestResult.sqDist)
				{
					nearestResult = result;
					nearestResult.nearestSegmentIndex = i;

					lastSegmentIndex = std::size_t(i);
				}
				else if (result.sqDist > nearestResult.sqDist)
					break;
			}
		}

		switch (nearestResult.pointPosition)
		{
		case Searcher::NearestPointPosition::SEGMENT_START:
			return { Searcher::Point_toMapPoint(nearestResult.point), std::size_t(nearestResult.nearestSegmentIndex), lastSegmentIndex };
		case Searcher::NearestPointPosition::SEGMENT_MIDDLE:
		case Searcher::NearestPointPosition::SEGMENT_END:
		default: // 必然能够找到最邻近点以及插入的点的位置
			return { Searcher::Point_toMapPoint(nearestResult.point), std::size_t(nearestResult.nearestSegmentIndex + 1), lastSegmentIndex };
		}
	}
	template DividedRoadNearestResult findDividedRoadInsertPosition<MapPoint3D64Ref>(const MapPoint3D64& point, const std::vector<MapPoint3D64Ref>& linestring, bool forwardSearch, std::size_t lastSegmentIndex);
	template DividedRoadNearestResult findDividedRoadInsertPosition<MapPoint3D64>(const MapPoint3D64& point, const std::vector<MapPoint3D64>& linestring, bool forwardSearch, std::size_t lastSegmentIndex);


	bool checkLink1CanTravelToLink2(std::pair<DbLink*, bool>& link1, std::pair<DbLink*, bool>& link2)
	{
		static const bool travelLookupTable[][4] =
		{
			{false, true, false, false},
			{true, false, false, false},
			{false, false, false, true},
			{false, false, true, false}
		};

		int lineDirCode = (int(link1.second) << 1) | (int(!link2.second));

		int d1 = 1 - (link1.first->direct - 2), d2 = 1 - (link2.first->direct - 2);
		int trafficCode = d1 << 1 | d2;

		return travelLookupTable[lineDirCode][trafficCode];
	}
}
