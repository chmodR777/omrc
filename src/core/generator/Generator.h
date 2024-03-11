#pragma once
#include <algorithm>
#include "CompileSetting.h"
#include "../record/DbMesh.h"
#include "map_point3d64_converter.h"
#include "Segment.h"

// had_link_form����
#define LINK_IS_IN_TUNNEL  (31)        //���
#define LINK_IS_IN_ROUNDABOUT (33)     //����
#define LINK_IS_IN_UTURN (35)          //��ͷ

namespace OMDB
{
	struct GeneratorData
	{
		bool coordinatesTransformGenerated = false;
		MapPoint3D64Converter coordinatesTransform;

		bg::strategy::buffer::join_miter join_strategy;
		bg::strategy::buffer::end_flat end_strategy;
		bg::strategy::buffer::point_square circle_strategy;
		bg::strategy::buffer::side_straight side_strategy;
	};

	struct boundaryPoint
	{
		boundaryPoint(
			const MapPoint3D64& originPoint,
			double fractionDistance,
			int id)
			: _originPoint(originPoint),
			_fractionDistance(fractionDistance),
			_id(id){}

		int _id;
		MapPoint3D64 _originPoint;
		double _fractionDistance;
	};

	class Generator
	{
	public:
		virtual ~Generator() = default;
		Generator(GeneratorData& data) :generatorData(data) {};

		void Generate(DbMesh* const pMesh);

		// ֻ���޸ĵ�ǰ���������,�����޸�nearby������,����processʱ������ʧЧ
		void GenerateRelation(DbMesh* pMesh, std::vector<DbMesh*>* nearby);

		static void getDSegmentTopoLinks(DbLink* pLink, DSegmentId dSegmenId,
			std::vector<DbSkeleton*>& previousLinks, std::vector<DbSkeleton*>& nextLinks);
		static std::vector<DbSkeleton*> getDSegmentPreviousLink(DbLink* pLink, DSegmentId dSegmenId);
		static std::vector<DbSkeleton*> getDSegmentNextLink(DbLink* pLink, DSegmentId dSegmenId);
		static DSegmentId getPreviousDSegId(DbLink* thisLink, DSegmentId thisDSegid, DbLink* previousLink);
		static DSegmentId getNextDSegId(DbLink* thisLink, DSegmentId thisDSegid, DbLink* nextLink);
		static int64 getDSegmentStartNode(DbLink* pLink, DSegmentId dSegmenId);
		static int64 getDSegmentEndNode(DbLink* pLink, DSegmentId dSegmenId);
		static bool getDSegmentDir(DbLink* pLink, DSegmentId dSegmenId);

		static int getDSegmentDirect(DbLink* pLink, DSegmentId dSegmenId);
		static DSegmentId getDirectDSegment(DbLink* pLink, int direct);

		static float getLinkLength(DbLink* pLink);
		static std::wstring* getLinkName(const DbLink* pLink);
		static std::set<int> getLinkNameGroups(const DbLink* pLink);
		static std::set<std::wstring*> getLinkNames(const DbLink* pLink);
		static bool containsLinkName(const DbLink* pLink, std::wstring* name);
		static bool containsLinkNames(const DbLink* pLink, std::set<std::wstring*>& names);

	protected:
		virtual void generate(DbMesh* const pMesh) = 0;
		virtual void generateRelation(DbMesh* pMesh, std::vector<DbMesh*>* nearby) = 0;

		void generateCoordinatesTransform(DbMesh* const pMesh);

		bool getExpandPolyByOffset(const ring_2t& ring,const int64& offsetSize, ring_2t& resultRing);

		// ��ѯ�ܱ�����
		DbRecord* queryNearby(DbMesh* pMesh, std::vector<DbMesh*>* nearby, int64 id, RecordType objectType);
		DbRecord* queryOrCreate(DbMesh* pMesh, std::vector<DbMesh*>* nearby, int64 id, RecordType objectType);

		static bool generateHdData(const DbLink* pLink);

		/// �жϱ߽������Ƿ���side��ͬ
		static bool sideEqual(DbRoadBoundLink* const pBoundary, DbRdLinkLanePa* const pGroup, int side);

		/// �жϱ߽緽���Ƿ���direction��ͬ
		static bool directionEqual(DbLaneMarkLink* const pBoundary, DbRdLinkLanePa* const pGroup, int direction);

		/// �жϱ߽緽���Ƿ���direction��ͬ
		static bool directionEqual(DbRoadBoundLink* const pBoundary, DbRdLinkLanePa* const pGroup, int direction);

		static std::pair<int64, DbRdLinkLanePa::DbRelLink> getRelLinkPair(const DbLink* pLink, const DbRdLinkLanePa* lanePa);

		static std::vector<DbRdLinkLanePa*> getLanePas(const DbLink* pLink, const std::vector<DbRdLinkLanePa*>& lanePas, int direct);
		static void getLanePasBoundary(std::vector<DbRdLinkLanePa*>& lanePas, int direct, LineString3d& leftSide, LineString3d& rightSide);
		static void getLanePaBoundary(DbRdLinkLanePa* lanePa, LineString3d& leftSide, LineString3d& rightSide);
		static void makeLanePaPolygon(const std::vector<MapPoint3D64>& leftSide, const std::vector<MapPoint3D64>& rightSide, Polygon3d& polygon);
		static MapPoint3D64 getBoundaryVector(std::vector<MapPoint3D64>& vertexes);
		static double getLanePaWidth(const DbLink* pLink, const DbRdLinkLanePa* lanePa);
		static double getLanePaWidthByLaneNum(const DbLink* pLink, const DbRdLinkLanePa* lanePa);
		static double getLanePaWidthByKind(const DbLink* pLink, const DbRdLinkLanePa* lanePa);

		/// ����������·�߽��firstLine��secondLine֮�������
		static void generateLanePaBoundary(
			const double lanePaAngle,
			const std::vector<MapPoint3D64>& firstLine, 
			const std::vector<MapPoint3D64>& secondLine,
			std::vector<MapPoint3D64>& boundaryVertexes);

		/// ����������·�߽��startMapPoint��endMapPoint֮�������
		static void generateLanePaBoundary(
			double lanePaAngle,
			MapPoint3D64 startMapPoint, MapPoint3D64 endMapPoint, 
			MapPoint3D64 prevStartPoint, MapPoint3D64 nextEndPoint,
			std::vector<MapPoint3D64>& boundaryVertexes);

		static std::vector<boundaryPoint> buildBoundaryPoints(const std::vector<MapPoint3D64>& originPoints);
		static LineString3d getBoundaryByOffset(std::vector<MapPoint3D64>& originPoints,
			std::vector<boundaryPoint> boundaryPoints, double startOffset, double endOffset);
		static LineString3d adjustBoundaryOffset(LineString3d& location, int offset);
	protected:
		MapPoint3D64Converter coordinatesTransform;
		GeneratorData& generatorData;
	};

	/**
	 * @brief �ж�Link1�Ƿ����ͨ�е�link2��Link1 & Link2���й�����Node��������ͨ������Node���ӡ�link1��bool��ʾ����Node�Ƿ���link1���߷��� ĩ�ˣ�link2��bool��ʾ����Node�Ƿ���link2���߷��� �׶ˡ�
	 *        ע�⣬Link1&Link2Ϊ�����е�·����direct��Ϊ1��ֻ��Ϊ2����3��
	 * @param link1 
	 * @param link2 
	 * @return 
	 */
	bool checkLink1CanTravelToLink2(std::pair<DbLink*, bool>& link1, std::pair<DbLink*, bool>& link2);


	struct DividedRoadNearestResult
	{
		MapPoint3D64 nearestPoint;
		std::size_t insertPos;
		std::size_t lastSegmentIndex;
	};
	/**
	 * @brief �����е�·��Ѱ��һ����״������ָ�������ڽ��ĵ�
	 * @tparam TPoint MapPoint3D64����MapPoint3D64Reft����
	 * @param point 
	 * @param linestring 
	 * @return first -> ���ڽ�����linestring�ܹ������λ�ã�ʹ��linestring����������second -> linestring�ϵ����ڽ���
	 */
	template<typename TPoint>
	DividedRoadNearestResult findDividedRoadInsertPosition(const MapPoint3D64& point, const std::vector<TPoint>& linestring, bool forwardSearch = true, std::size_t lastSegmentIndex = 0);

}


