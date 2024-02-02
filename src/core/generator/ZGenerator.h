#pragma once

#include <unordered_set>
#include <functional>

#include "Generator.h"
#include "math3d/vector_math.h"
#include "core/sd_road/DirLink.h"
#include "core/sd_road/DirLinkPath.h"
#include "core/sd_road/DivicedRoadSearcher.h"
#include "core/sd_road/DivicedRoadSearcher.h"

namespace OMDB
{
	using MapPoint3D64Ref = std::reference_wrapper<MapPoint3D64>;

	//////////////////////////////////////////////////////////////////////////

	struct AdjustHeightInfo
	{
		sd::DirLink dlink;
		double lastHeight;
		double length;
		double priority;
	};

	class AdjustHeightQueue
	{
	public:
		forceinline size_t size() { return m_data.size(); }
		forceinline void clear() { m_data.clear(); }

		void push(const AdjustHeightInfo& data);

		void pop(AdjustHeightInfo* data);

	private:
		cqstd::vector<AdjustHeightInfo> m_data;
	};

	class DirLinkPathVertexRefCollector : public sd::DirLinkPath::VertexAccessor
	{
		std::vector<MapPoint3D64Ref> m_verticesRef;

		int32 m_maxHeight{ 0 };

	public:

		std::vector<MapPoint3D64Ref> verticesRef() { return m_verticesRef; }

		int32 maxHeight() { return m_maxHeight; }

		void access(MapPoint3D64& vertex, const sd::DirLinkPath::VertexAccessor::AccessInfo& accessInfo) override
		{
			UNREFERENCED_PARAMETER(accessInfo);

			m_verticesRef.push_back(vertex);
			m_maxHeight = max(m_maxHeight, vertex.z);
		}
	};

	class GradientVertexModifier : public sd::DirLinkPath::VertexAccessor
	{
		double m_heightFrom;
		double m_heightTo;
		double m_length;

	public:
		GradientVertexModifier(double heightFrom, double heightTo, double length);

		void access(MapPoint3D64& vertex, const sd::DirLinkPath::VertexAccessor::AccessInfo& accessInfo) override;
	};

	class LeastHeightVertexModifier : public sd::DirLinkPath::VertexAccessor
	{
		double m_leastHeight;

	public:
		LeastHeightVertexModifier(double heightFrom, double heightTo);

		void access(MapPoint3D64& vertex, const sd::DirLinkPath::VertexAccessor::AccessInfo& accessInfo) override;
	};

	/**
	 * @brief ���������޸���߳��ܵ������ĵ�·�����ĵ�·�ĸ̣߳�ʹ��·�ڹ���ƽ��
	*/
	class ConnectedRoadVertexModifier : public sd::DirLinkPath::VertexAccessor
	{
		MapPoint3D64& m_mainRoadPoint;
		double m_heightDiff;
		double m_roadLength;

		MapPoint3D64* m_theFirstTwoPoints[2];

		bool m_skipTheRestPoints{ true };

	public:
		ConnectedRoadVertexModifier(MapPoint3D64& mainRoadPoint, double roadLength);

		ConnectedRoadVertexModifier& operator=(ConnectedRoadVertexModifier&) = delete;

		void access(MapPoint3D64& vertex, const sd::DirLinkPath::VertexAccessor::AccessInfo& accessInfo) override;
	};

	class SingleDirRoad : public sd::DirLinkPath {
	public:
		enum class StopReason
		{
			NONE,
			MEET_Z_LEVEL_POINT,
			NO_OUTLINKS,
			MULTI_OUT_LINKS,
		};

		struct  Portion : public sd::DirLinkPath::Portion
		{
			StopReason stopReason{ StopReason::NONE };
			int zLevelPointIndex{ INVALID_INDEX };

			std::size_t countVertexNum();

			bool forward() const { return dirLink.forward(); }

			/**
			 * @brief ��ȡ��·Link���ط����������Ӧ�ĵ㡣����˵����·Ϊ����indexΪ0���򷵻�link���εĵ�һ���㣬����·Ϊ�����򷵻�link�������һ���㡣
			 * @param index 
			 * @return 
			*/
			MapPoint3D64& getVertexInDir(std::size_t index);
		};

		Portion& front() { return _portionAt(0); }

		Portion& back() { return _portionAt(static_cast<int>(m_portions.size() - 1)); }

		MapPoint3D64* frontPoint();

		MapPoint3D64* backPoint();

		double getStartHeight();

		double getEndHeight();

		double length();

		void accessVertices(VertexAccessor& vertexModifier) override;

		bool hasPassedZeroLevelPoint() { return m_hasPassedZeroLevelPoint; };

		void setHasPassedZeroLevelPoint(bool val) { m_hasPassedZeroLevelPoint = val; };

		void addPortion(Portion portion);

	private:
		static void _accessVerticesSameLinkPortion(Portion& portion, Portion& nextPortion, std::size_t& vertexIndex, double& accLength, const std::size_t totalVertexNum, VertexAccessor& vertexModifier);
		/**
		 * @brief ����Portion����ĺ�������
		 * @param accLength
		 * @param vertexIndex
		 * @param totalVertexNum
		 * @param portion
		 * @param vertexModifier
		*/
		static void _accessVerticesDifferentLinkPortion(SingleDirRoad::Portion& portion, std::size_t& vertexIndex, double& accLength, const std::size_t totalVertexNum, VertexAccessor& vertexModifier);

		static std::size_t _calculateSameLinkPortionVertexNum(Portion& portion, Portion& nextPortion);

	private:
		std::vector<Portion> m_portions;
		bool m_hasPassedZeroLevelPoint{ false };

		std::size_t _calculateTotoalVertexNum();

		Portion& _portionAt(int index) { return m_portions[index]; }
	};

	class ZGenerator : public Generator
	{
	public:
		ZGenerator(GeneratorData& data) :Generator(data) {};
	protected:
		virtual void generate(DbMesh* const pMesh) override;
		virtual void generateRelation(DbMesh* const pMesh, std::vector<DbMesh*>* nearby) override;

		bool checkAndAdjustZLevel(std::vector<std::pair<DbZLevel*, std::vector<DbLink*>>>& zLevelLinks);
		bool checkIntersectingInfo(std::pair<DbZLevel*, std::vector<DbLink*>>& zLevelLinkPair);
		void adjustPointHeight(DbLink* pLink, int idx, double targetZ, double slope);
		void adjustDividedRoadHeight(sd::DividedRoad& dividedRoad);
		void bfs(AdjustHeightQueue& adjustQueue, std::set<int64>& adjustedSegment);
		double adjustZ(std::vector<MapPoint3D64>& points, bool isSameDir, double targetHeight, double length);
		void adjustZ2(std::vector<MapPoint3D64>& points, bool isSameDir, double startHeight, double endHeight);

		void processHeightFromZLevelPoints(std::vector<std::pair<DbZLevel*, std::vector<DbLink*>>>& zLevelLinks);
		void processHeightByZLevelPoint(DbZLevel* pZLevel, DbZLevel::DbRelLink& relLink, DbLink*& pLink, std::unordered_set<int64>& processedRoadPortion);
		SingleDirRoad extendRoadFromZLevelPoint(sd::DirLink& beginLink, int beginLinkZLevelPointIndex);
		/**
		 * @brief �������·��������������·��ֱ����������Z Level���������·�����������·�ж�����ߵ�·��
		*/
		SingleDirRoad extendRoadFromDSegment(sd::DirLink& beginLink);

		void adjustSingleDirRoadHeight(SingleDirRoad& single_dir_road);

		double getZByPoint(DbZLevel::DbRelLink& relLink, DbLink* pLink);
		bool isPedestrian(DbLink* pLink) { return pLink->kind == 10; };

	protected:
		std::vector<SingleDirRoad> m_roadsNeedAdjust;
		std::vector<ConnectedRoadVertexModifier> m_roadsModifier;

	private:
		/**
		 * @brief ʹ��һϵ�е�Link�����Z Level�����ɸ̡߳�
		 *        ���ɵĸ߳�ֻ��д����Link����
		 *        ���÷���������ṩ��¡�Ķ���������Щlink����ʹ���¸߳�Ҳ����ʹ�á�
		 * @param zLevelLinks 
		*/
		void generateHeight(std::vector<std::pair<DbZLevel*, std::vector<DbLink*>>>& zLevelLinks,
			std::vector<sd::DividedRoad>& dividedRoads);
	};
}

