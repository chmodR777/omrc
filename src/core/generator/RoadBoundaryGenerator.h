#pragma once
#include "Generator.h"
#include <unordered_map>
#include <unordered_set>
namespace OMDB
{
	class RoadBoundaryGenerator : public Generator
	{
	public:
		RoadBoundaryGenerator(GeneratorData& data) :Generator(data) {};
	protected:
		virtual void generate(DbMesh* const pMesh) override;
		void generate(DbLink* const pLink, int direct);

		virtual void generateRelation(DbMesh* const pMesh, std::vector<DbMesh*>* nearby) override;

		void generateStraightRoad(DbNode* pNode);

		void generateForkRoadClipPoint(DbNode* pNode);
		void generateForkRoadClipPoint(DbNode* pNode, 
			DbLink* firstLink, std::vector<linestring_2t>& firstClipFaces,
			DbLink* secondLink, std::vector<linestring_2t>& secondClipFaces);
		void generateForkRoad(DbLink* pLink);

		void generateDirectForkRoad(DbLink* pLink, DSegmentId dsegId, DbNode* pNode, std::unordered_set<uint64_t>& visited);
		void generateTopoDirectForkRoad(DbNode* pNode,
			DbLink* pCurrent, int currentDirect, std::vector<DbRdLinkLanePa*>& currentLanePas,
			DbLink* pNext, int nextDirect, std::vector<DbRdLinkLanePa*>& nextLanePas, std::unordered_set<uint64_t>& visited);
		void generateTopoDirectForkRoadGroup(DbNode* pNode, 
			DbLink* pCurrent, int currentDirect, DbRdLinkLanePa* pCurrentLanePa,
			DbLink* pNext, int nextDirect, DbRdLinkLanePa* pNextLanePa, std::unordered_set<uint64_t>& visited);
		void getLanePaBoundaryNodes(DbRdLinkLanePa* lanePa, std::vector<int64>& leftSide, std::vector<int64>& rightSide);
		void clipForkRoadGroup(DbNode* pNode, DbLink* pLink, DbRdLinkLanePa* lanePa);

	private:
		DbMesh* m_mesh = nullptr;
		std::vector<DbMesh*>* m_nearbyMesh = nullptr;
	};
}

