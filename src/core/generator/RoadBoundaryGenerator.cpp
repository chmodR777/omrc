#include "stdafx.h"
#include "GroupGenerator.h"
#include "RoadBoundaryGenerator.h"
#include "algorithm/grap_point_algorithm.h"
#include "algorithm/polyline_intersector.h"
namespace OMDB
{
    void RoadBoundaryGenerator::generate(DbMesh* const pMesh)
    {
		m_mesh = pMesh;
        std::vector<DbRecord*>& links = pMesh->query(RecordType::DB_HAD_LINK);
        for (auto hl : links) {
            DbLink* pLink = (DbLink*)hl;
			if (!generateHdData(pLink))
				continue;

			// 顺方向
			generate(pLink, 2);

			// 逆方向
			generate(pLink, 3);
        }
    }

	void RoadBoundaryGenerator::generate(DbLink* const pLink, int direct)
	{
		// 车信 pLink->relLaneInfos;
		auto directLanePas = getLanePas(pLink, pLink->lanePas, direct);
		if (directLanePas.empty()) {
			return;
		}

		LineString3d originLine = pLink->geometry;
		coordinatesTransform.convert(originLine.vertexes.data(), originLine.vertexes.size());

		auto lineLength = bg::length(LINESTRING_T(originLine.vertexes));
		auto boundaryPoints = buildBoundaryPoints(originLine.vertexes);

		auto generateDirect2Boundary = [&](int side) {
			double lastEndOffset{ DBL_MAX };
			DbRoadBoundNode* lastEndNode = nullptr;
			for (int idx = 0; idx < directLanePas.size(); idx++) {
				auto currLanePa = directLanePas[idx];
				auto currRelLinkPair = getRelLinkPair(pLink, currLanePa);
				if (87955278724609537 == currRelLinkPair.first)
					printf("");

				int64 rdBoundSeed = (currRelLinkPair.first << 1) | (side - 1);
				int64 rdNodeSeed = (rdBoundSeed << 1) | (side - 1);

				auto startOffset = currRelLinkPair.second.startOffset;
				auto endOffset = currRelLinkPair.second.endOffset;
				auto subLine = getBoundaryByOffset(originLine.vertexes, boundaryPoints, startOffset * lineLength, endOffset * lineLength);
				coordinatesTransform.invert(subLine.vertexes.data(), subLine.vertexes.size());

				auto laneNum = currLanePa->relLinkLanes.size();
				auto halfLaneOffset = 3500.0 * laneNum / 2;

				// boundary
				DbRoadBoundLink* pBoundary = (DbRoadBoundLink*)m_mesh->alloc(RecordType::DB_HAD_ROAD_BOUNDARY_LINK);
				pBoundary->uuid = currRelLinkPair.first + (rdBoundSeed++);
				pBoundary->boundaryType = 0; // TODO
				if (side == 1) { // right
					pBoundary->geometry = adjustBoundaryOffset(subLine, halfLaneOffset);
				} else if (side == 2) { // left
					pBoundary->geometry = adjustBoundaryOffset(subLine, -halfLaneOffset);
				}

				// startNode
				DbRoadBoundNode* pStartNode = nullptr;
				if (lastEndNode != nullptr && std::abs(startOffset - lastEndOffset) < 0.01) {
					pStartNode = lastEndNode;
				} else {
					pStartNode = (DbRoadBoundNode*)m_mesh->alloc(RecordType::DB_HAD_ROAD_BOUNDARY_NODE);
					pStartNode->uuid = currRelLinkPair.first + (rdNodeSeed++);
					pStartNode->geometry = pBoundary->geometry.vertexes.front();
					m_mesh->insert(pStartNode->uuid, pStartNode);
				}

				// endNode
				DbRoadBoundNode* pEndNode = (DbRoadBoundNode*)m_mesh->alloc(RecordType::DB_HAD_ROAD_BOUNDARY_NODE);
				pEndNode->uuid = currRelLinkPair.first + (rdNodeSeed++);
				pEndNode->geometry = pBoundary->geometry.vertexes.back();
				m_mesh->insert(pEndNode->uuid, pEndNode);

				// nodeId
				pBoundary->starRoadBoundNodeId = pStartNode->uuid;
				pBoundary->endRoadBoundNodeId = pEndNode->uuid;
				m_mesh->insert(pBoundary->uuid, pBoundary);

				// relLg
				DbRoadBoundLink::DbLgRoadBoundREL relLg;
				relLg.side = side;
				relLg.direction = direct;
				pBoundary->relLgs.emplace(currRelLinkPair.first, relLg);
				currLanePa->roadBoundaries.push_back(pBoundary);

				// backup last node
				lastEndOffset = endOffset;
				lastEndNode = pEndNode;
			}
		};

		auto generateDirect3Boundary = [&](int side) {
			double lastEndOffset{ DBL_MAX };
			DbRoadBoundNode* lastEndNode = nullptr;
			for (int idx = 0; idx < directLanePas.size(); idx++) {
				auto currLanePa = directLanePas[idx];
				auto currRelLinkPair = getRelLinkPair(pLink, currLanePa);

				int64 rdBoundSeed = (currRelLinkPair.first << 1) | (side - 1);
				int64 rdNodeSeed = (rdBoundSeed << 1) | (side - 1);

				auto startOffset = currRelLinkPair.second.startOffset;
				auto endOffset = currRelLinkPair.second.endOffset;
				auto subLine = getBoundaryByOffset(originLine.vertexes, boundaryPoints, startOffset * lineLength, endOffset * lineLength);
				coordinatesTransform.invert(subLine.vertexes.data(), subLine.vertexes.size());

				auto laneNum = currLanePa->relLinkLanes.size();
				auto halfLaneOffset = 3500.0 * laneNum / 2;

				// boundary
				DbRoadBoundLink* pBoundary = (DbRoadBoundLink*)m_mesh->alloc(RecordType::DB_HAD_ROAD_BOUNDARY_LINK);
				pBoundary->uuid = currRelLinkPair.first + (rdBoundSeed++);
				pBoundary->boundaryType = 0; // TODO
				if (side == 1) { // right
					pBoundary->geometry = adjustBoundaryOffset(subLine, -halfLaneOffset);
				} else if (side == 2) { // left
					pBoundary->geometry = adjustBoundaryOffset(subLine, halfLaneOffset);
				}

				// startNode
				DbRoadBoundNode* pStartNode = (DbRoadBoundNode*)m_mesh->alloc(RecordType::DB_HAD_ROAD_BOUNDARY_NODE);
				pStartNode->uuid = currRelLinkPair.first + (rdNodeSeed++);
				pStartNode->geometry = pBoundary->geometry.vertexes.back();
				m_mesh->insert(pStartNode->uuid, pStartNode);


				// endNode
				DbRoadBoundNode* pEndNode = nullptr;
				if (lastEndNode != nullptr && std::abs(startOffset - lastEndOffset) < 0.01) {
					pEndNode = lastEndNode;
				} else {
					pEndNode = (DbRoadBoundNode*)m_mesh->alloc(RecordType::DB_HAD_ROAD_BOUNDARY_NODE);
					pEndNode->uuid = currRelLinkPair.first + (rdNodeSeed++);
					pEndNode->geometry = pBoundary->geometry.vertexes.front();
					m_mesh->insert(pEndNode->uuid, pEndNode);
				}

				// nodeId
				pBoundary->starRoadBoundNodeId = pStartNode->uuid;
				pBoundary->endRoadBoundNodeId = pEndNode->uuid;
				m_mesh->insert(pBoundary->uuid, pBoundary);

				// relLg
				DbRoadBoundLink::DbLgRoadBoundREL relLg;
				relLg.side = side;
				relLg.direction = direct;
				pBoundary->relLgs.emplace(currRelLinkPair.first, relLg);
				currLanePa->roadBoundaries.push_back(pBoundary);

				// backup last node
				lastEndOffset = endOffset;
				lastEndNode = pStartNode;
			}
		};

		if (direct == 2)
		{
			// left boundary
			generateDirect2Boundary(2);

			// right boundary
			generateDirect2Boundary(1);
		}
		else if (direct == 3)
		{
			// left boundary
			generateDirect3Boundary(2);

			// right boundary
			generateDirect3Boundary(1);
		}
	}

    void RoadBoundaryGenerator::generateRelation(DbMesh* const pMesh, std::vector<DbMesh*>* nearby)
    {
		auto isForkRoad = [](DbNode* pNode)->bool {
			int sum = 0;
			for (DbLink* pLink : pNode->links)
			{
				if (pLink->startNode != pLink->endNode) {
					sum += 1;
				}
				else {
					sum += 2;
				}
			}
			return sum > 2;
		};

		m_mesh = pMesh;
		m_nearbyMesh = nearby;
		std::vector<DbRecord*>& nodes = pMesh->query(RecordType::DB_HAD_NODE);
		for (auto nd : nodes)
		{
			DbNode* pNode = (DbNode*)nd;
			if (pNode->links.size() <= 1)
				continue;

			if (!isForkRoad(pNode))
			{
				generateStraightRoad(pNode);
			}
			else
			{
				generateForkRoadClipPoint(pNode);
			}
		}

		// choose proper clipPoint
		std::vector<DbRecord*>& links = pMesh->query(RecordType::DB_HAD_LINK);
		for (auto hl : links) {
			DbLink* pLink = (DbLink*)hl;
			for (auto& clipPoint : pLink->clipPoints) {
				// 裁切点由近到远排序
				DbNode* pNode = (DbNode*)m_mesh->query(clipPoint.first, RecordType::DB_HAD_NODE);
				std::sort(clipPoint.second.begin(), clipPoint.second.end(),
					[&](const auto& first, const auto& second)->bool {
						auto firstDis = pNode->geometry.pos.distanceSquare(first.position.pos);
						auto secondDis = pNode->geometry.pos.distanceSquare(second.position.pos);
						return firstDis < secondDis;
					});
				auto ip = std::unique(clipPoint.second.begin(), clipPoint.second.end(),
					[&](const auto& first, const auto& second)->bool {
						return mapPoint3D64_compare(first.position, second.position);
					});
				clipPoint.second.resize(std::distance(clipPoint.second.begin(), ip));
			}
		}

		for (auto hl : links) {
			DbLink* pLink = (DbLink*)hl;
			generateForkRoad(pLink);
		}
    }

	void RoadBoundaryGenerator::generateStraightRoad(DbNode* pNode)
	{
		UNREFERENCED_PARAMETER(pNode);
	}

	void RoadBoundaryGenerator::generateForkRoadClipPoint(DbNode* pNode)
	{
		auto checkLanePaValid = [](DbLink* pLink, std::vector<DbRdLinkLanePa*>& lanePas)->bool {
			double offsetSum = 0;
			for (auto lanePa : lanePas) {
				auto relLinkPair = getRelLinkPair(pLink, lanePa);
				auto startOffset = relLinkPair.second.startOffset;
				auto endOffset = relLinkPair.second.endOffset;
				offsetSum += (endOffset - startOffset);
			}
			return offsetSum > 0.9;
		};

		auto expandNodeFace = RING_2T(pNode->geometry, 10000);
		bg::correct(expandNodeFace);

		std::unordered_map<DbLink*, std::vector<linestring_2t>> clipFaces;
		auto getClipFaces = [&](DbLink* pLink)->std::vector<linestring_2t>& {
			auto iter = clipFaces.find(pLink);
			if (iter != clipFaces.end()) {
				return iter->second;
			}

			std::vector<DbRdLinkLanePa*> lanePas;
			if (pLink->direct != 1) {
				auto tmpLanePas = getLanePas(pLink, pLink->lanePas, pLink->direct);
				if (checkLanePaValid(pLink, tmpLanePas)) {
					lanePas = tmpLanePas;
				}
			}
			else {
				auto tmpLanePas = getLanePas(pLink, pLink->lanePas, 2);
				if (checkLanePaValid(pLink, tmpLanePas)) {
					lanePas = tmpLanePas;
				}
				else {
					tmpLanePas = getLanePas(pLink, pLink->lanePas, 3);
					if (checkLanePaValid(pLink, tmpLanePas)) {
						lanePas = tmpLanePas;
					}
				}
			}
			if (!lanePas.empty()) {
				Polygon3d polygon;
				LineString3d leftSide, rightSide;
				getLanePasBoundary(lanePas, leftSide, rightSide);
				makeLanePaPolygon(leftSide.vertexes, rightSide.vertexes, polygon);

				auto roadFaceRing = RING_2T(polygon.vertexes);
				bg::correct(roadFaceRing);

				std::vector<ring_2t> resultRings;
				bg::intersection(roadFaceRing, expandNodeFace, resultRings);
				if (!resultRings.empty()) {
					std::vector<linestring_2t> resultLines;
					for (auto& resultRing : resultRings)
						resultLines.push_back(LINESTRING_2T(resultRing));
					clipFaces.emplace(pLink, resultLines);
				}
			}
			return clipFaces[pLink];
		};

		for (int m = 0; m < pNode->links.size(); m++)
		{
			DbLink* pFirst = (DbLink*)pNode->links[m];
			auto firstClipFaces = getClipFaces(pFirst);
			if (firstClipFaces.empty())
				continue;

			for (int n = m + 1; n < pNode->links.size(); n++)
			{
				DbLink* pSecond = (DbLink*)pNode->links[n];
				auto secondClipFaces = getClipFaces(pSecond);
				if (secondClipFaces.empty())
					continue;

				generateForkRoadClipPoint(pNode, pFirst, firstClipFaces, pSecond, secondClipFaces);
			}
		}
	}

	void RoadBoundaryGenerator::generateForkRoadClipPoint(DbNode* pNode, 
		DbLink* firstLink, std::vector<linestring_2t>& firstClipFaces,
		DbLink* secondLink, std::vector<linestring_2t>& secondClipFaces)
	{
		auto updateForkRoadClipPoint = [&](DbLink* pLink, point_2t& pos) {
			size_t si, ei;
			MapPoint3D64 grappedPt;
			MapPoint3D64 tmpPos = MapPoint3D64_make(pos.get<0>(), pos.get<1>(), 0);
			if (GrapPointAlgorithm::grapOrMatchNearestPoint(tmpPos, pLink->geometry.vertexes, grappedPt, si, ei)) {
				DbLink::ClipPoint clipPoint{ grappedPt, si, ei };
				auto iter = pLink->clipPoints.find(pNode->uuid);
				if (iter == pLink->clipPoints.end()) {
					std::vector<DbLink::ClipPoint> vec;
					vec.push_back(clipPoint);
					pLink->clipPoints.emplace(pNode->uuid, vec);
				}
				else {
					iter->second.push_back(clipPoint);
				}
			}
		};

		for (auto& firstClipFace : firstClipFaces) {
			for (auto& secondClipFace : secondClipFaces) {
				std::vector<point_2t> intersectionPts;
				bg::intersection(firstClipFace, secondClipFace, intersectionPts);
				for (auto& intersectionPt : intersectionPts) {
					// firstLink
					updateForkRoadClipPoint(firstLink, intersectionPt);

					// secondLink
					updateForkRoadClipPoint(secondLink, intersectionPt);
				}
			}
		}
	}

	void RoadBoundaryGenerator::generateForkRoad(DbLink* pLink)
	{
		std::unordered_set<uint64_t> visited;
		auto generateForkRoadFromNode = [&](DbNode* pNode) {
			std::vector<DbLink*> doubleDirectLinks;
			std::vector<DbLink*> singleDirectLinks;
			for (auto pLink : pNode->links) {
				if (pLink->direct == 1) {
					doubleDirectLinks.push_back(pLink);
				}
				else {
					singleDirectLinks.push_back(pLink);
				}
			}

			for (auto pLink : doubleDirectLinks) {
				// generate in forward direction
				DSegmentId forwardSeg = DSegmentId_getDSegmentId(pLink->uuid);
				generateDirectForkRoad(pLink, forwardSeg, pNode, visited);

				// generate in backward direction
				DSegmentId backwardSeg = DSegmentId_getReversed(forwardSeg);
				generateDirectForkRoad(pLink, backwardSeg, pNode, visited);
			}

			for (auto pLink : singleDirectLinks) {
				auto dSegmentId = DSegmentId_getDSegmentId(pLink->uuid);
				generateDirectForkRoad(pLink, dSegmentId, pNode, visited);
			}
		};

		// startNode
		auto pStartNode = m_mesh->query(pLink->startNode, RecordType::DB_HAD_NODE);
		generateForkRoadFromNode((DbNode*) pStartNode);

		// endNode
		auto pEndNode = m_mesh->query(pLink->endNode, RecordType::DB_HAD_NODE);
		generateForkRoadFromNode((DbNode*)pEndNode);
	}

	void RoadBoundaryGenerator::generateDirectForkRoad(DbLink* pLink, DSegmentId dsegId, DbNode* pNode, std::unordered_set<uint64_t>& visited)
	{
		if (84207887605792865 == pNode->uuid)
			printf("");
		auto direct = getDSegmentDirect(pLink, dsegId);
		auto previousLinks = getDSegmentPreviousLink(pLink, dsegId);
		auto nextLinks = getDSegmentNextLink(pLink, dsegId);
		auto lanePas = getLanePas(pLink, pLink->lanePas, direct);
		
		std::vector<DbLink*> connectedLinks = pNode->getLinksExcept(pLink);
		for (auto connectedLink : connectedLinks) {
			DbLink* currentLink = nullptr; 
			int currentDirect = -1;
			std::vector<DbRdLinkLanePa*> currentLanePas;

			DbLink* nextLink = nullptr;
			int nextDirect = -1;
			std::vector<DbRdLinkLanePa*> nextLanePas;
			
			DSegmentId connectedDSegId{ INVALID_DSEGMENT_ID };
			if (std::find(previousLinks.begin(), previousLinks.end(), connectedLink) != previousLinks.end()) {
				connectedDSegId = getPreviousDSegId(pLink, dsegId, connectedLink);
				auto connectedDirect = getDSegmentDirect(connectedLink, connectedDSegId);

				currentLink = connectedLink;
				currentDirect = connectedDirect;
				currentLanePas = getLanePas(connectedLink, connectedLink->lanePas, connectedDirect);

				nextLink = pLink;
				nextDirect = direct;
				nextLanePas = lanePas;
			}
			else if (std::find(nextLinks.begin(), nextLinks.end(), connectedLink) != nextLinks.end()) {
				connectedDSegId = getNextDSegId(pLink, dsegId, connectedLink);
				auto connectedDirect = getDSegmentDirect(connectedLink, connectedDSegId);

				currentLink = pLink;
				currentDirect = direct;
				currentLanePas = lanePas;

				nextLink = connectedLink;
				nextDirect = connectedDirect;
				nextLanePas = getLanePas(connectedLink, connectedLink->lanePas, connectedDirect);
			}
			if (connectedDSegId == INVALID_DSEGMENT_ID)
				continue;

			generateTopoDirectForkRoad(pNode,
				currentLink, currentDirect, currentLanePas, 
				nextLink, nextDirect, nextLanePas, visited);
		}
	}

	void RoadBoundaryGenerator::generateTopoDirectForkRoad(DbNode* pNode,
		DbLink* pCurrent, int currentDirect, std::vector<DbRdLinkLanePa*>& currentLanePas, 
		DbLink* pNext, int nextDirect, std::vector<DbRdLinkLanePa*>& nextLanePas, std::unordered_set<uint64_t>& visited)
	{
		// 进来该函数的都是有向路,方向不对直接返回
		if (currentDirect == 1 || nextDirect == 1)
			return;

		// 没有车道组的返回
		if (currentLanePas.empty() || nextLanePas.empty())
			return;

		DbRdLinkLanePa* pCurrentLastGroup = nullptr;
		DbRdLinkLanePa* pNextFirstGroup = nullptr;
		if (currentDirect == 2 && nextDirect == 2) {
			pCurrentLastGroup = *currentLanePas.rbegin();
			pNextFirstGroup = nextLanePas[0];
		}
		else if (currentDirect == 2 && nextDirect == 3) {
			pCurrentLastGroup = *currentLanePas.rbegin();
			pNextFirstGroup = *nextLanePas.rbegin();
		}
		else if (currentDirect == 3 && nextDirect == 2) {
			pCurrentLastGroup = currentLanePas[0];
			pNextFirstGroup = nextLanePas[0];
		}
		else if (currentDirect == 3 && nextDirect == 3) {
			pCurrentLastGroup = currentLanePas[0];
			pNextFirstGroup = *nextLanePas.rbegin();
		}
		generateTopoDirectForkRoadGroup(pNode, 
			pCurrent, currentDirect, pCurrentLastGroup, 
			pNext, nextDirect, pNextFirstGroup, visited);
	}

	void RoadBoundaryGenerator::generateTopoDirectForkRoadGroup(DbNode* pNode,
		DbLink* pCurrent, int currentDirect, DbRdLinkLanePa* pCurrentLanePa,
		DbLink* pNext, int nextDirect, DbRdLinkLanePa* pNextLanePa,
		std::unordered_set<uint64_t>& visited)
	{
		UNREFERENCED_PARAMETER(visited);
		auto generateLanePa = [&](DbLink* pLink, DbRdLinkLanePa* pLanePa)->DbRdLinkLanePa* {
			auto id = pLanePa->uuid << 1;
			id += pLanePa->generateLanePaCnt.fetch_add(1, std::memory_order_relaxed);
			DbRdLinkLanePa* pNewLanePa = (DbRdLinkLanePa*)m_mesh->query(id, RecordType::DB_RD_LINK_LANEPA);
			if (pNewLanePa == nullptr)
			{
				pNewLanePa = (DbRdLinkLanePa*)m_mesh->alloc(RecordType::DB_RD_LINK_LANEPA);
				pNewLanePa->uuid = id;
				pNewLanePa->isGenerated = true;
				m_mesh->insert(pNewLanePa->uuid, pNewLanePa);
			}

			int startOffset = 0; 
			int endOffset = 0;
			auto relLinkPair = getRelLinkPair(pLink, pLanePa);
			if (pLanePa == pCurrentLanePa) {
				if (currentDirect == 2) {
					startOffset = relLinkPair.second.endOffset;
					endOffset = 1;
				}
				else if (currentDirect == 3) {
					startOffset = 0;
					endOffset = relLinkPair.second.startOffset;
				}
			}
			else if (pLanePa == pNextLanePa) {
				if (nextDirect == 2) {
					startOffset = 0;
					endOffset = relLinkPair.second.startOffset;
				}
				else if (nextDirect == 3) {
					startOffset = relLinkPair.second.endOffset;
					endOffset = 1;
				}
			}

			DbRdLinkLanePa::DbRelLink relLink;
			relLink.relLinkid = relLinkPair.second.relLinkid;
			relLink.startOffset = startOffset;
			relLink.endOffset = endOffset;
			relLink.directType = relLinkPair.second.directType;
			pNewLanePa->relLinks.emplace(id, relLink);
			return pNewLanePa;
		};

		auto generateRoadBoundary = [&](DbLink* pLink, int direct, 
			DbRdLinkLanePa* pLanePa, LineString3d& geometry, int64 startId, int64 endId, int side) {
			auto currRelLinkPair = getRelLinkPair(pLink, pLanePa);
			int64 rdBoundSeed = (currRelLinkPair.first << 1) | (side - 1);
			int64 rdNodeSeed = (rdBoundSeed << 1) | (side - 1);

			// boundary
			DbRoadBoundLink* pBoundary = (DbRoadBoundLink*)m_mesh->alloc(RecordType::DB_HAD_ROAD_BOUNDARY_LINK);
			pBoundary->uuid = currRelLinkPair.first + (rdBoundSeed++);
			pBoundary->boundaryType = 0; // TODO
			pBoundary->geometry = geometry;

			// nodeId
			pBoundary->starRoadBoundNodeId = startId;
			pBoundary->endRoadBoundNodeId = endId;
			m_mesh->insert(pBoundary->uuid, pBoundary);

			// relLg
			DbRoadBoundLink::DbLgRoadBoundREL relLg;
			relLg.side = side;
			relLg.direction = direct;
			pBoundary->relLgs.emplace(currRelLinkPair.first, relLg);
			pLanePa->roadBoundaries.push_back(pBoundary);
		};

		// 进来该函数的都是有向路,方向不对直接返回
		if (currentDirect == 1 || nextDirect == 1)
			return;

		// 道路边界数不对返回
		if (pCurrentLanePa->roadBoundaries.size() != 2 || pNextLanePa->roadBoundaries.size() != 2)
			return;

		// clip current lanePa and update node position
		if (pCurrent->clipPoints.find(pNode->uuid) != pCurrent->clipPoints.end()) {
			clipForkRoadGroup(pNode, pCurrent, pCurrentLanePa);
		}

		// clip next lanePa and update node position
		if (pNext->clipPoints.find(pNode->uuid) != pNext->clipPoints.end()) {
			clipForkRoadGroup(pNode, pNext, pNextLanePa);
		}

		// 当前拓扑关系
		auto currentDSegId = getDirectDSegment(pCurrent, currentDirect);
		//auto currentPreviousLinks = getDSegmentPreviousLink(pCurrent, currentDSegId);
		auto currentNextLinks = getDSegmentNextLink(pCurrent, currentDSegId);

		// 下一个拓扑关系
		auto nextDSegId = getDirectDSegment(pNext, nextDirect);
		auto nextPreviousLinks = getDSegmentPreviousLink(pNext, nextDSegId);
		//auto nextNextLinks = getDSegmentNextLink(pNext, nextDSegId);

		// 左边
		auto& currentLeftBoundary = pCurrentLanePa->roadBoundaries[0];
		auto& currentRightBoundary = pCurrentLanePa->roadBoundaries[1];
		LineString3d currentLeftSide, currentRightSide;
		getLanePaBoundary(pCurrentLanePa, currentLeftSide, currentRightSide);

		// 右边
		auto& nextLeftBoundary = pNextLanePa->roadBoundaries[0];
		auto& nextRightBoundary = pNextLanePa->roadBoundaries[1];
		LineString3d nextLeftSide, nextRightSide;
		getLanePaBoundary(pNextLanePa, nextLeftSide, nextRightSide);

		// 生成边界线
		LineString3d leftSide, rightSide;
		generateLanePaBoundary(currentLeftSide.vertexes, nextLeftSide.vertexes, leftSide.vertexes);
		generateLanePaBoundary(currentRightSide.vertexes, nextRightSide.vertexes, rightSide.vertexes);

		std::vector<int64> currentLeftNodes, currentRightNodes;
		getLanePaBoundaryNodes(pCurrentLanePa, currentLeftNodes, currentRightNodes);

		std::vector<int64> nextLeftNodes, nextRightNodes;
		getLanePaBoundaryNodes(pNextLanePa, nextLeftNodes, nextRightNodes);

		// 生成车道组并建立拓扑关系
		if (currentNextLinks.size() > 1) {
			DbRdLinkLanePa* pLanePa = generateLanePa(pNext, pNextLanePa);
			printf("");
		}
		else if (nextPreviousLinks.size() > 1) {
			DbRdLinkLanePa* pLanePa = generateLanePa(pCurrent, pCurrentLanePa);
			auto leftStartNode = currentLeftNodes.back();
			if (directionEqual(currentLeftBoundary, pCurrentLanePa, 3)) {
				std::reverse(leftSide.vertexes.begin(), leftSide.vertexes.end());
				leftStartNode = currentLeftNodes.front();
			}
			auto leftEndNode = nextLeftNodes.front();
			if (directionEqual(nextLeftBoundary, pNextLanePa, 3)) {
				leftEndNode = currentLeftNodes.back();
			}
			generateRoadBoundary(pCurrent, currentDirect, pLanePa, leftSide, leftStartNode, leftEndNode, 2);

			auto rightStartNode = currentRightNodes.back();
			if (directionEqual(currentRightBoundary, pCurrentLanePa, 3)) {
				std::reverse(rightSide.vertexes.begin(), rightSide.vertexes.end());
				rightStartNode = currentRightNodes.front();
			}
			auto rightEndNode = nextRightNodes.front();
			if (directionEqual(nextRightBoundary, pNextLanePa, 3)) {
				rightEndNode = nextRightNodes.back();
			}
			generateRoadBoundary(pCurrent, currentDirect, pLanePa, rightSide, rightStartNode, rightEndNode, 1);

			// 生成车道组
			GroupGenerator::generateGroup(m_mesh, pCurrent, pLanePa);
			printf("");
		}
		else {

		}

		printf("");
	}

	void RoadBoundaryGenerator::getLanePaBoundaryNodes(DbRdLinkLanePa* lanePa, std::vector<int64>& leftSide, std::vector<int64>& rightSide)
	{
		leftSide.clear(); rightSide.clear();
		leftSide.push_back(lanePa->roadBoundaries[0]->starRoadBoundNodeId);
		leftSide.push_back(lanePa->roadBoundaries[0]->endRoadBoundNodeId);
		if (directionEqual(lanePa->roadBoundaries[0], lanePa, 3))
		{
			std::reverse(leftSide.begin(), leftSide.end());
		}
		rightSide.push_back(lanePa->roadBoundaries[1]->starRoadBoundNodeId);
		rightSide.push_back(lanePa->roadBoundaries[1]->endRoadBoundNodeId);
		if (directionEqual(lanePa->roadBoundaries[1], lanePa, 3))
		{
			// 反向，进行点的反转
			std::reverse(rightSide.begin(), rightSide.end());
		}
	}

	void RoadBoundaryGenerator::clipForkRoadGroup(DbNode* pNode, DbLink* pLink, DbRdLinkLanePa* lanePa)
	{
		auto getVerticalSeg = [](point_2t& firstPt, point_2t& secondPt)->segment_2t {
			auto _p1 = P2_V2(firstPt);
			auto _p2 = P2_V2(secondPt);
			double _l1 = bg::distance(_p2, _p1);

			vector_2t _v1 = _p2;
			bg::subtract_point(_v1, _p1);
			bg::divide_value(_v1, _l1);

			vector_2t _v2(_v1.get<1>(), -_v1.get<0>());
			point_2t _p3 = secondPt;
			bg::multiply_value(_v2, 10000);
			bg::add_point(_p3, V2_P2(_v2));

			vector_2t _v3(-_v1.get<1>(), _v1.get<0>());
			point_2t _p4 = secondPt;
			bg::multiply_value(_v3, 10000);
			bg::add_point(_p4, V2_P2(_v3));

			segment_2t seg2(_p3, _p4);
			return seg2;
		};

		auto getClipPolygon = [](segment_2t& firstSeg, segment_2t& secondSeg)->Polygon3d {
			Polygon3d polygon;
			polygon.vertexes.push_back(MapPoint3D64_make(firstSeg.first.get<0>(), firstSeg.first.get<1>(), 0));
			polygon.vertexes.push_back(MapPoint3D64_make(firstSeg.second.get<0>(), firstSeg.second.get<1>(), 0));
			polygon.vertexes.push_back(MapPoint3D64_make(secondSeg.first.get<0>(), secondSeg.first.get<1>(), 0));
			polygon.vertexes.push_back(MapPoint3D64_make(secondSeg.second.get<0>(), secondSeg.second.get<1>(), 0));
			polygon.vertexes.push_back(polygon.vertexes.front());
			return polygon;
		};

		auto clipPointIter = pLink->clipPoints.find(pNode->uuid);
		auto& backClipPoint = clipPointIter->second.back();

		auto nodePt = POINT_2T(pNode->geometry);
		auto clipPt = POINT_2T(backClipPoint.position);
		auto distance = bg::distance(nodePt, clipPt);

		segment_2t clipNodeSeg{ clipPt, nodePt };
		clipNodeSeg = SEGMENT_2T_EX_FRONT(clipNodeSeg, 300);
		auto clipNodeVerSeg = getVerticalSeg(clipPt, clipNodeSeg.second);
		auto nodeClipVerSeg = getVerticalSeg(nodePt, clipPt);
		Polygon3d clipPolygon = getClipPolygon(nodeClipVerSeg, clipNodeVerSeg);

		static const double tolerance = 10;
		std::vector<std::vector<MapPoint3D64>> locations;
		for (auto pBoundary : lanePa->roadBoundaries) {
			auto outPts = pBoundary->geometry.vertexes;
			bool containsIntersectPoint = false;
			for (size_t i = 0; i < outPts.size() - 1; i++) {
				auto& startPt = outPts[i];
				auto& endPt = outPts[i + 1];
				std::vector<MapPoint3D64> intersectPoints;
				if (PolylineIntersector::intersect(startPt, endPt, clipPolygon.vertexes, tolerance, intersectPoints)) {
					auto pStart = POINT_T(startPt);
					auto pEnd = POINT_T(endPt);
					segment_t segment{ pStart, pEnd };
					vector_t pabVec = V3_N(S3_V3(segment));
					std::sort(intersectPoints.begin(), intersectPoints.end(), [&](MapPoint3D64& pa, MapPoint3D64& pb) {
						vector_t tmp0V = S3_V3(pStart, POINT_T(pa));
						auto v0Sum = bg::dot_product(pabVec, tmp0V);

						vector_t tmp1V = S3_V3(pStart, POINT_T(pb));
						auto v1Sum = bg::dot_product(pabVec, tmp1V);
						return v0Sum < v1Sum;
						});
					auto insertIter = mapPoint3D64_iterator(outPts, endPt);
					for (auto& intersectPoint : intersectPoints) {
						containsIntersectPoint = true;
						auto intersectPt = POINT_2T(intersectPoint);
						if (isEqualPoint2T(P3_P2(pStart), intersectPt) || isEqualPoint2T(P3_P2(pEnd), intersectPt)) {
							continue;
						}
						insertIter = outPts.insert(insertIter, intersectPoint);
						insertIter++;// 指向插入点下一个位置
					}
				}
			}
			if (!containsIntersectPoint) {
				return;
			}

			// clip
			for (auto iter = outPts.begin(); iter != outPts.end(); ) {
				bool pointInPolygon = false;
				if (GrapPointAlgorithm::isPointInPolygon(*iter, clipPolygon.vertexes)) {
					pointInPolygon = true;
				}
				size_t si, ei;
				MapPoint3D64 minGrappedPt = {};
				GrapPointAlgorithm::grapOrMatchNearestPoint(*iter, clipPolygon.vertexes, minGrappedPt, si, ei);
				if (minGrappedPt.pos.distance(iter->pos) < 50) {
					pointInPolygon = false;
				}

				if (pointInPolygon) {
					iter = outPts.erase(iter);
					continue;
				}
				iter++; 
			}

			// 边界在裁切面内
			if (outPts.size() <= 1) {
				lanePa->inClipRing = true;
				continue;
			}
			locations.push_back(outPts);
		}

		// 不在裁切面内时更新数据
		if (lanePa->roadBoundaries.size() == locations.size()) {
			for (int idx = 0; idx < lanePa->roadBoundaries.size(); idx++) {
				auto& pBoundary = lanePa->roadBoundaries[idx];
				pBoundary->geometry.vertexes = locations[idx];
				auto& outPts = pBoundary->geometry.vertexes;

				auto startNode = (DbRoadBoundNode*)m_mesh->query(pBoundary->starRoadBoundNodeId, RecordType::DB_HAD_ROAD_BOUNDARY_NODE);
				auto endNode = (DbRoadBoundNode*)m_mesh->query(pBoundary->endRoadBoundNodeId, RecordType::DB_HAD_ROAD_BOUNDARY_NODE);
				auto startNodeDistance = pNode->geometry.pos.distanceSquare(startNode->geometry.pos);
				auto endNodeDistance = pNode->geometry.pos.distanceSquare(endNode->geometry.pos);
				auto updateNode = (startNodeDistance < endNodeDistance) ? startNode : endNode;

				auto frontPtDistance = pNode->geometry.pos.distanceSquare(outPts.front().pos);
				auto backPtDistance = pNode->geometry.pos.distanceSquare(outPts.back().pos);
				auto& updatePt = (frontPtDistance < backPtDistance) ? outPts.front() : outPts.back();
				updateNode->geometry = updatePt;
			}
		}
		
	}

}
