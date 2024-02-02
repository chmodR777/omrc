#pragma once
#include "clipper.hpp"
#include "Processor.h"
namespace OMDB
{
	class LinkGroupAssociationProcessor : public Processor
	{
		using HadRelLaneGroupAssociation = HadLaneGroupAssociation::HadRelLaneGroupAssociation;
	protected:
		virtual void process(DbMesh* const pMesh, HadGrid* pGrid) override;
		virtual void processRelation(DbMesh* const pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby) override;

		void generateLaneGroupAssociations(HadGrid* const pGrid,const std::vector<HadGrid*>& nearby);
		HadLaneGroupAssociation* generateLaneGroupAssociation(HadGrid* const pGrid, HadFillArea* pFillArea, HadRelLaneGroupAssociation& lgAssociation);
		bool isXUTurnLA(HadRelLaneGroupAssociation* lgAssociation, HadRelLaneGroupAssociation* pLA);
		std::vector<HadRelLaneGroupAssociation*>::iterator findLgAssociationIter(
			HadRelLaneGroupAssociation* lgAssociation, HadLaneGroup* firstLaneGroup, HadLaneGroup* secondLaneGroup,
			HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, std::vector<HadRelLaneGroupAssociation*>& allLaneGroupAssociations,
			std::map<int64, std::vector<HadRelLaneGroupAssociation*>>& groupedLaneGroupAssociations);
		bool laGeometrysOverFillArea(HadFillArea* pFillArea, HadRelLaneGroupAssociation& lgAssociation);
		void constructFillAreaPath(HadFillArea* pFillArea, ClipperLib::Path& fillAreaPath, double& fillAreaHight);
		void constructLgAssociationPath(HadRelLaneGroupAssociation& lgAssociation, ClipperLib::Path& laPath, double& laHight);
		void constructLaneGroupAssociation(HadRelLaneGroupAssociation& lgAssociation,
			int64 groupId, int directType, HadLaneGroup* first, HadLaneGroup* second);
		void getNearbyFillAreas(BoundingBox2d& extent, const std::vector<HadGrid*>& grids, std::vector<HadFillArea*>& nearbyFillAreas);
		MapPoint3D64 getBoundaryVector(std::vector<MapPoint3D64>& vertexes);
		bool isNotSemiTransparentGroup(HadLaneGroup* pLaneGroup);

		double findNearbyLaneBoundary(HadLaneGroup* pLaneGroup, HadLaneGroup* pNearbyLaneGroup,
			HadLaneBoundary*& pLaneGroupLaneBoundary, HadLaneBoundary*& pNearbyLaneGroupLaneBoundary);
		void groupLaneGroupAssociations(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, 
			std::vector<HadRelLaneGroupAssociation*>& allLaneGroupAssociations,
			std::map<int64, std::vector<HadRelLaneGroupAssociation*>>& groupedLaneGroupAssociations);
		void appendGroupLaneGroupAssociation(HadRelLaneGroupAssociation* const pLA, std::vector<HadRelLaneGroupAssociation*>& allLaneGroupAssociations,
			std::map<int64, std::vector<HadRelLaneGroupAssociation*>>& groupedLaneGroupAssociations);
		void findNearbyLaneGroups(HadGrid* const pGrid, HadFillArea* const pFillArea, const std::vector<HadGrid*>& nearby, std::vector<HadLaneGroup*>& pNearbyLaneGroups);
		BoundingBox2d makeBoundingBox2d(HadFillArea* const pFillArea);

	private:
		int64 m_maxGroupId = -1;
	};
}

