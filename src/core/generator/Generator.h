#pragma once
#include <algorithm>
#include "CompileSetting.h"
#include "../record/DbMesh.h"
#include "map_point3d64_converter.h"
#include "Segment.h"

// had_link_form属性
#define LINK_IS_IN_TUNNEL  (31)        //隧道
#define LINK_IS_IN_ROUNDABOUT (33)     //环岛
#define LINK_IS_IN_UTURN (35)          //掉头

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

		// 只能修改当前网格的数据,不能修改nearby的数据,否则process时会数据失效
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

		// 查询周边网格
		DbRecord* queryNearby(DbMesh* pMesh, std::vector<DbMesh*>* nearby, int64 id, RecordType objectType);
		DbRecord* queryOrCreate(DbMesh* pMesh, std::vector<DbMesh*>* nearby, int64 id, RecordType objectType);

		static bool generateHdData(const DbLink* pLink);

		/// 判断边界左右是否与side相同
		static bool sideEqual(DbRoadBoundLink* const pBoundary, DbRdLinkLanePa* const pGroup, int side);

		/// 判断边界方向是否与direction相同
		static bool directionEqual(DbLaneMarkLink* const pBoundary, DbRdLinkLanePa* const pGroup, int direction);

		/// 判断边界方向是否与direction相同
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

		/// 生成两条道路边界从firstLine到secondLine之间的连线
		static void generateLanePaBoundary(
			const double lanePaAngle,
			const std::vector<MapPoint3D64>& firstLine, 
			const std::vector<MapPoint3D64>& secondLine,
			std::vector<MapPoint3D64>& boundaryVertexes);

		/// 生成两条道路边界从startMapPoint到endMapPoint之间的连线
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
	 * @brief 判断Link1是否可以通行到link2。Link1 & Link2具有公共的Node，几何上通过公共Node连接。link1的bool表示公共Node是否在link1画线方向 末端，link2的bool表示公共Node是否在link2画线方向 首端。
	 *        注意，Link1&Link2为上下行道路，其direct不为1，只能为2或者3。
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
	 * @brief 上下行道路中寻找一侧形状线上与指定点最邻近的点
	 * @tparam TPoint MapPoint3D64或者MapPoint3D64Reft类型
	 * @param point 
	 * @param linestring 
	 * @return first -> 最邻近点在linestring能够插入的位置，使得linestring保持连续，second -> linestring上的最邻近点
	 */
	template<typename TPoint>
	DividedRoadNearestResult findDividedRoadInsertPosition(const MapPoint3D64& point, const std::vector<TPoint>& linestring, bool forwardSearch = true, std::size_t lastSegmentIndex = 0);

}


