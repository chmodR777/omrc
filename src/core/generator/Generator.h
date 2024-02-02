#pragma once
#include <algorithm>
#include "CompileSetting.h"
#include "../record/DbMesh.h"
#include "map_point3d64_converter.h"
#include "tool_kit/boost_geometry_utils.h"
#include "Segment.h"

namespace OMDB
{
	#define INVALID_INDEX -1

	struct GeneratorData
	{
		bool coordinatesTransformGenerated = false;
		MapPoint3D64Converter coordinatesTransform;
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
		void GenerateRelation(DbMesh* pMesh, std::vector<DbMesh*>* nearby);

		static float getLinkLength(DbLink* pLink);
		static std::wstring* getLinkName(const DbLink* pLink);
		static std::set<std::wstring*> getLinkNames(const DbLink* pLink);
		static bool containsLinkName(const DbLink* pLink, std::wstring* name);

		static std::vector<DbSkeleton*> getDSegmentPreviousLink(DbLink* pLink, DSegmentId dSegmenId);
		static std::vector<DbSkeleton*> getDSegmentNextLink(DbLink* pLink, DSegmentId dSegmenId);
		static DSegmentId getPreviousDSegId(DbLink* thisLink, DSegmentId thisDSegid, DbLink* previousLink);
		static DSegmentId getNextDSegId(DbLink* thisLink, DSegmentId thisDSegid, DbLink* nextLink);
		static int64 getDSegmentStartNode(DbLink* pLink, DSegmentId dSegmenId);
		static int64 getDSegmentEndNode(DbLink* pLink, DSegmentId dSegmenId);
		static bool getDSegmentDir(DbLink* pLink, DSegmentId dSegmenId);

		static int getDSegmentDirect(DbLink* pLink, DSegmentId dSegmenId);
		static DSegmentId getDirectDSegment(DbLink* pLink, int direct);

		/// 判断边界左右是否与side相同
		static bool sideEqual(DbRoadBoundLink* const pBoundary, DbRdLinkLanePa* const pGroup, int side);

		/// 判断边界方向是否与direction相同
		static bool directionEqual(DbLaneMarkLink* const pBoundary, DbRdLinkLanePa* const pGroup, int direction);

		/// 判断边界方向是否与direction相同
		static bool directionEqual(DbRoadBoundLink* const pBoundary, DbRdLinkLanePa* const pGroup, int direction);

	protected:
		virtual void generate(DbMesh* const pMesh) = 0;
		virtual void generateRelation(DbMesh* pMesh, std::vector<DbMesh*>* nearby) = 0;

		static bool generateHdData(const DbLink* pLink);

		static std::pair<int64, DbRdLinkLanePa::DbRelLink> getRelLinkPair(const DbLink* pLink, const DbRdLinkLanePa* lanePa);

		static std::vector<DbRdLinkLanePa*> getLanePas(const DbLink* pLink, const std::vector<DbRdLinkLanePa*>& lanePas, int direct);
		static void getLanePasBoundary(std::vector<DbRdLinkLanePa*>& lanePas, LineString3d& leftSide, LineString3d& rightSide);
		static void getLanePaBoundary(DbRdLinkLanePa* lanePa, LineString3d& leftSide, LineString3d& rightSide);
		static void makeLanePaPolygon(const std::vector<MapPoint3D64>& leftSide, const std::vector<MapPoint3D64>& rightSide, Polygon3d& polygon);

		/// 生成两条道路边界从firstLine到secondLine之间的连线
		static void generateLanePaBoundary(
			const std::vector<MapPoint3D64>& firstLine, 
			const std::vector<MapPoint3D64>& secondLine,
			std::vector<MapPoint3D64>& boundaryVertexes);

		/// 生成两条道路边界从startMapPoint到endMapPoint之间的连线
		static void generateLanePaBoundary(
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
}

