#pragma once
#include "Generator.h"
#include <unordered_map>
#include <unordered_set>
#include <functional>
namespace OMDB
{
	class RoadBoundaryGenerator : public Generator
	{
	public:
		RoadBoundaryGenerator(GeneratorData& data) :Generator(data) {};

	protected:
		typedef void (RoadBoundaryGenerator::* ProcessDirectRoadGroupFn)(DbNode*, DbLink*, int, DbRdLinkLanePa*, DbLink*, int, DbRdLinkLanePa*);
	protected:
		virtual void generate(DbMesh* const pMesh) override;
		void generateDirectNot23Boundary(DbLink* const pLink);
		void generateDirect23Boundary(DbLink* const pLink, int direct);

		void saveLanePaInfo(DbRdLinkLanePa* const lanePa);

		void generateDirect2Boundary(DbLink* const pLink, int direct, std::vector<DbRdLinkLanePa*>& directLanePas,
			LineString3d& offsetLine, std::vector<boundaryPoint>& boundaryPoints, double boundaryLength, double boundaryOffset, int side);
		void generateDirect3Boundary(DbLink* const pLink, int direct, std::vector<DbRdLinkLanePa*>& directLanePas,
			LineString3d& offsetLine, std::vector<boundaryPoint>& boundaryPoints, double boundaryLength, double boundaryOffset, int side);

		virtual void generateRelation(DbMesh* const pMesh, std::vector<DbMesh*>* nearby) override;

		bool isForkRoad(DbNode* pNode);

		void generateForkRoadClipPoint(DbNode* pNode);
		void generateForkRoadClipPoint(DbNode* pNode, DbLink* pLink, point_2t& pos);
		std::vector<linestring_2t> getClipFaces(DbLink* pLink, ring_2t& nodeFace);
		void fixupForkRoadClipPoint(DbMesh* const pMesh);

		void generateDirectRoadFromNode(DbNode* pNode, std::unordered_set<std::string>& visited,
			ProcessDirectRoadGroupFn processDirectRoadGroup);

		void generateDirectRoad(DbLink* pLink, DSegmentId dsegId, DbNode* pNode, std::unordered_set<std::string>& visited,
			ProcessDirectRoadGroupFn processDirectRoadGroup);
		void generateTopoDirectRoad(DbNode* pNode,
			DbLink* pCurrent, int currentDirect, std::vector<DbRdLinkLanePa*>& currentLanePas,
			DbLink* pNext, int nextDirect, std::vector<DbRdLinkLanePa*>& nextLanePas, std::unordered_set<std::string>& visited,
			ProcessDirectRoadGroupFn processDirectRoadGroup);

		void generateDirectStraightRoadGroup(DbNode* pNode,
			DbLink* pCurrent, int currentDirect, DbRdLinkLanePa* pCurrentLanePa,
			DbLink* pNext, int nextDirect, DbRdLinkLanePa* pNextLanePa);

		void generateDirectForkRoadGroup(DbNode* pNode, 
			DbLink* pCurrent, int currentDirect, DbRdLinkLanePa* pCurrentLanePa,
			DbLink* pNext, int nextDirect, DbRdLinkLanePa* pNextLanePa);

		DbRdLinkLanePa* allocForkRoadLanePa(DbRdLinkLanePa* pLanePa);
		void generateForkRoadBoundary(DbLink* pLink, int direct,
			DbRdLinkLanePa* pLanePa, LineString3d& geometry, int64 startId, int64 endId, int side);
		void getLanePaBoundaryNodes(DbRdLinkLanePa* lanePa, std::vector<int64>& leftSide, std::vector<int64>& rightSide);
		int calcLanePaAngle(LineString3d& currentRightSide, LineString3d& nextRightSide);
		void clipForkRoadGroup(DbNode* pNode, DbLink* pLink, DbRdLinkLanePa* lanePa);

		template <typename Iterator>
		bool checkPathCanTravel(Iterator begin, Iterator end);

		template <typename Iterator>
		std::vector<std::vector<DbRdLinkLanePa*>> findPathRightSidePas(Iterator begin, Iterator end);

		void RoadBoundaryGenerator::findDividedRoadRightSidePas(std::array<std::vector<std::pair<DbLink*, bool>>, 2>& dividedRoad, 
			std::array<std::vector<std::vector<DbRdLinkLanePa*>>, 2>& outDividedRoadPas, int& commonBoundaryIndex);

		void generateDividedRoadCommonBoundary(std::size_t index, std::array<std::vector<std::pair<DbLink*, bool>>, 2>& dividedRoad);

	private:
		DbMesh* m_mesh = nullptr;
		std::vector<DbMesh*>* m_nearbyMesh = nullptr;
	};
}

