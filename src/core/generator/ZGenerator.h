#pragma once

#include <unordered_set>
#include <queue> 

#include "Generator.h"
#include "math3d/vector_math.h"

namespace OMDB
{
	//////////////////////////////////////////////////////////////////////////

	class DirLink
	{
		int64 m_linkuuid; // ����ǰ�棬�������ʱ��һʱ����Բ鿴��link��id
		bool m_forward; // ����(forward)->��Link���߷�����ͬ������(backward)->��Link���߷����෴�������򼴷���
		// ԭʼLink
		DbLink* m_link;
		LineString3d* m_linkGeometry; // ��DirLink�ڲ���Զʹ��m_linkGeometry������m_link->geometry

	public:
		/**
		 * @brief ����һ���յ�DirectLink���󣬲�Ҫ���øö�����κη���(����valid)������
		*/
		DirLink() : m_linkuuid(0), m_forward(true), m_link(nullptr), m_linkGeometry(nullptr) { }

		/**
		 * @brief ʹ��ʹ�û��߷�����DirectLink
		 * @param link Not NULL!!!
		 * @param forward
		*/
		DirLink(DbLink* link, bool forward) : m_linkuuid(link->uuid), m_forward(forward), m_link(link), m_linkGeometry(nullptr) { }

		DirLink(DbLink* link, bool forward, LineString3d& linkGeometry) : m_linkuuid(link->uuid), m_forward(forward), m_link(link), m_linkGeometry(&linkGeometry) { }

		/**
		 * @brief ����һ�������DirectLink����
		 * @return
		*/
		DirLink getReverseDirLink() { return DirLink{ m_link, !m_forward, *m_linkGeometry }; }
		DirLink getForwardDirLink() { return  m_forward ? *this : getReverseDirLink(); }
		DirLink getBackwardDirLink() { return  !m_forward ? *this : getReverseDirLink(); }

		/**
		 * @brief ��鵱ǰDirectLink�����Ƿ���Ч����Ч�Ķ���������ó��������κη�������
		 * @return
		*/
		bool valid() const { return m_link != nullptr; }

		bool forward() const { return m_forward; }

		/**
		 * @brief ��ǰ������other���е��޷���Link�����Ƿ�һ����
		 * @param other
		 * @return
		*/
		bool rawLinkEqual(const DirLink& other) const { return this->m_link == other.m_link; }

		////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/////////                                                                                    ///////////////
		/////////                           DbLink���Դ���                                            //////////////
		/////////                                                                                   ///////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////////////

		int direct() { return m_link->direct; };

		int kind() { return m_link->kind; }

		int64 uuid() const { return m_link->uuid; }

		int multi_digitized() { return m_link->multi_digitized; }

		std::vector<DbZLevel::DbRelLink>& zLevels() { return m_link->zLevels; }

		const std::vector<int> wayTypes() { return m_link->wayTypes; }
		bool containsWayType(int type) { return std::find(m_link->wayTypes.cbegin(), m_link->wayTypes.cend(), type) != m_link->wayTypes.cend(); }

		LineString3d& geometry() { return *m_linkGeometry; }
		void setGeoemtry(LineString3d* geom) { m_linkGeometry = geom; }

		std::wstring* getLinkName() { return Generator::getLinkName(m_link); }
		std::set<std::wstring*> getLinkNames() { return Generator::getLinkNames(m_link); }
		bool containsLinkName(std::wstring* name) { return Generator::containsLinkName(m_link, name); }
		bool containsLinkNames(std::set<std::wstring*>& names) { return Generator::containsLinkNames(m_link, names); }

		/**
		 * @brief ��Ҫ�޸ĸö�����κζ���!!!
		*/
		DbLink* getLinkDangerous() { return m_link; }

		////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/////////                                                                                    ///////////////
		/////////                           ������غ���                                              ///////////////
		/////////                                                                                   ///////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////////////

		/**
		 * @brief DirLink���εڶ����㵽��һ������������Եڶ�����Ϊԭ���ƽ��ֱ������ϵ�µ���+X��ʱ�뷽��ļнǡ��нǵ�λ���ȡ�
		 * @return ���ȣ���Χ[0, 2PI)
		*/
		double startHeadingAngle();
		/**
		 * @brief DirLink���ε����ڶ����㵽������һ������������Ե����ڶ�����Ϊԭ���ƽ��ֱ������ϵ�µ���+X��ʱ�뷽��ļнǡ��нǵ�λ���ȡ�
		 * @return ���ȣ���Χ[0, 2PI)
		*/
		double endHeadingAngle();


		/**
		 * @brief ��ȡDirectLink ����ĩ�˵�ĸ̡߳��̵߳�λ���ס�
		 * @return
		*/
		double getStartHeight() { return getStartPointRef().z; }

		double getEndHeight() { return getEndPointRef().z; }

		/**
		 * @brief ��ȡDirectLink��һ��������á�ע���ⲿ���ܻ��޸�����㡣
		*/
		MapPoint3D64& getStartPointRef() { return m_forward ? m_linkGeometry->vertexes.front() : m_linkGeometry->vertexes.back(); }

		/**
		 * @brief ��ȡDirectLink���һ��������á�ע���ⲿ���ܻ��޸�����㡣
		*/
		MapPoint3D64& getEndPointRef() { return m_forward ? m_linkGeometry->vertexes.back() : m_linkGeometry->vertexes.front(); }
		MapPoint3D64 getEndPoint() const { return m_forward ? m_linkGeometry->vertexes.back() : m_linkGeometry->vertexes.front(); }
		/**
		 * @brief ��ȡDireLink˳�����ָ���������ĵ����á�ע���ⲿ���ܻ��޸�����㡣
		 * @param index
		 * @return
		*/
		MapPoint3D64& getPointRef(int index) { return m_forward ? m_linkGeometry->vertexes[index] : m_linkGeometry->vertexes[int(m_linkGeometry->vertexes.size()) - 1 - index]; }

		MapPoint3D64& getPointRefNoDir(int index) { return m_linkGeometry->vertexes[index]; }

		/**
		 * @brief ����������Link�ϴ���㵽�յ�ĳ����Ͼ������ָ���ٷֱȵĵ�
		 * @param perc [0.0, 1.0]
		 * @return 
		*/
		MapPoint3D64 calPointAt(double perc) const;

		/**
		 * @brief ��ȡ��ǰlink���γ��ȡ�ע����calLength����length()��Ϊ��ȡ���ԣ�����Ҫ�����ܵļ��㡣
		 * @return
		*/
		// TODO: ����ֱ�ӷ��ػ���ĳ��ȶ������ٴμ��㡣
		double length() { return calLength(); }

		/**
		 * @brief ����Link���ȣ���λ�ס�
		 * @return
		*/
		double calLength() const;

		/**
		* @brief ��������(indexFrom & indexTo)���ۼƳ��ȣ���λ�ס�����indexFrom & indexTo��С��ϵ����ν��
		*/
		double calPortionLength(int indexFrom, int indexTo) const;

		/**
		 * @brief ����DirectLink����㵽ָ����ĳ��ȡ���λ�ס�
		 *        Note: �ü����뵱ǰDirectLink��������ء�
		*/
		double calStartToMidLength(int index) const { return m_forward ? calToMidForwardLength(index) : calToMidBackwardLength(index); }

		/**
		 * @brief ����DirectLink��ָ���㵽ĩ�˵�ĳ��ȡ���λ�ס�
		 *        Note: �ü����뵱ǰDirectLink��������ء�
		*/
		double calMidToEndLength(int index) const { return m_forward ? calToEndForwardLength(index) : calToEndBackwardLength(index); }

		/**
		 * @brief ����Link����㿪ʼ��ָ����(λ���м�ĵ�)���ȡ���λ�ס�
		 *        Note: �ü����޹ص�ǰDirectLink������
		*/
		double calToMidForwardLength(int index) const { return calPortionLength(0, index); }

		/**
		 * @brief ����Link��ĩ�˵���ָ����ĳ��ȡ���λ�ס�
		 *        Note: �ü����޹ص�ǰDirectLink������
		*/
		double calToMidBackwardLength(int index) const { return calPortionLength(int(m_linkGeometry->vertexes.size()) - 1, index); }

		/**
		 * @brief ����Link��ָ���㿪ʼ�����һ������ۼƳ��ȡ���λ�ס�
		 *        Note: �ü����޹ص�ǰDirectLink������
		*/
		double calToEndForwardLength(int index) const { return calPortionLength(index, int(m_linkGeometry->vertexes.size()) - 1); }

		/**
		 * @brief ����Link��ָ���㵽��ʼ����ۼƳ��ȡ���λ�ס�
		 *        Note: �ü����޹ص�ǰDirectLink������
		*/
		double calToEndBackwardLength(int index) const { return calPortionLength(index, 0); }

		/**
		 * @brief ��Link���δ�indexFrom�Ķ�����ʵ�indexTo�Ķ��㣨������ģ�ordered����
		 * @param indexFrom
		 * @param indexTo
		 * @param accessor
		*/
		class VertexAccessor
		{
		public:
			virtual ~VertexAccessor() = default;
			virtual void accessFirst(MapPoint3D64& point) { point; }
			virtual void accessMiddle(MapPoint3D64& point, MapPoint3D64& prevPoint, double distToPrevPoint) { point; prevPoint; distToPrevPoint; }
			virtual void accessLast(MapPoint3D64& point, MapPoint3D64& prevPoint, double distToPrevPoint) { point; prevPoint; distToPrevPoint; }
		};
		void accessVerticesOrdered(int indexFrom, int indexTo, VertexAccessor& accessor);
		void accessVerticesToEnd(int indexFrom, VertexAccessor& accessor)
		{
			if (m_forward)
				accessVerticesOrdered(indexFrom, int(m_linkGeometry->vertexes.size()) - 1, accessor);
			else
				accessVerticesOrdered(indexFrom, 0, accessor);
		}
		void accessVerticesToEnd(VertexAccessor& accessor)
		{
			if (m_forward)
				accessVerticesOrdered(0, int(m_linkGeometry->vertexes.size()) - 1, accessor);
			else
				accessVerticesOrdered(int(m_linkGeometry->vertexes.size()) - 1, 0, accessor);
		}
		void accessVerticesToMid(int targetIndex, VertexAccessor& accessor)
		{
			if (m_forward)
				accessVerticesOrdered(0, targetIndex, accessor);
			else
				accessVerticesOrdered(int(m_linkGeometry->vertexes.size()) - 1, targetIndex, accessor);
		}

		// accessor���Կ��Ʒ��ʵ���Ϊ
		enum class AccessorControl
		{
			NONE,
			STOP,
		};

		////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/////////                                                                                    ///////////////
		/////////                           ·��������غ���                                           ///////////////
		/////////                                                                                   ///////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////////////

		/**
		 * @brief ��ȡ�������Linkĩ�˽ڵ�ֱ����������Link���������������Link����
		 * @return
		*/
		template <bool withLinkGeometry=false>
		std::vector<DirLink> getEndDirectOutlinks();
		std::vector<int64> getEndDirectOutlinksUUID();

		/**
		 * @brief ��ȡ�������Linkĩ�˽ڵ�ֱ����������Link�� �� �������������Link����
		 * @return
		*/
		std::vector<DirLink> getEndDirectOutlinksExceptSelf();
		std::vector<int64> getEndDirectOutlinksExceptSelfUUID();
	};

	class DirLinkVertexReader : public DirLink::VertexAccessor
	{
		std::vector<MapPoint3D64> m_vertices;
	public:

		const std::vector<MapPoint3D64>& vertices() { return m_vertices; }

		void accessFirst(MapPoint3D64& point) override { m_vertices.clear();  m_vertices.push_back(point); }
		void accessMiddle(MapPoint3D64& point, MapPoint3D64& prevPoint, double distToPrevPoint)override { m_vertices.push_back(point); prevPoint; distToPrevPoint; }
		void accessLast(MapPoint3D64& point, MapPoint3D64& prevPoint, double distToPrevPoint) override { m_vertices.push_back(point); prevPoint; distToPrevPoint; }
	};


	// bfs�м�¼����link��Ϣ���̵�link���ȷ��ʴ���
	struct AdjustHeightInfo
	{
		DirLink dlink;
		double lastHeight;
		double length;
		double priority;
	};

	struct AdjustHeightInfoComp
	{
		bool operator()(const AdjustHeightInfo& l, const AdjustHeightInfo& r) const { return l.priority > r.priority; }
	};

	using AdjustHeightQueue = std::priority_queue < AdjustHeightInfo, std::vector<AdjustHeightInfo>, AdjustHeightInfoComp>;

	/**
	 * @brief ��������·����ÿһ���ֵĵ�·��״�ߵĶ���
	*/
	class PathVerticesAccessor
	{
	public:
		struct AccessInfo
		{
			// ���ʵĶ�����·�����ζ����е�������
			std::size_t index;
			// ·�������м��ζ�������
			std::size_t totalVertexNum;
			// ���ʵĶ��㵽·�����ĳ��ȣ���λ�ס�
			double accLength;
		};
		/**
		 * @brief �ú������ն��㲢�����޸ģ���ѡ��
		 * @param vertex ��������
		 * @param index
		 * @param totalVertexNum  ͨ��SingleDirRoad������ʶ������ֵΪ���е�·�����ж����е�������
		 * @param accLength ͨ��SingleDirRoad������ʶ������ֵΪ��·�ۼƳ��ȡ�
		*/
		virtual void access(MapPoint3D64& vertex, const AccessInfo& accessInfo) = 0;

		virtual ~PathVerticesAccessor() {};
	};

	/**
	 * @brief ������ʹ���path��link�ϵĶ��㲢�ռ���Ϣ���ݸ�PathVerticesAccessor
	*/
	class PathLinkVerticesAccessor : public DirLink::VertexAccessor
	{
		PathVerticesAccessor& m_vertexAccessor;
		// ·�������м��ζ�������
		std::size_t m_totalVertexNum;
		// path��һ���ֵ�ĩ�˵�
		MapPoint3D64* m_lastPortionLastPoint;
		// ���ʵĶ�����·�����ζ����е�������
		std::size_t m_index;
		// ���ʵĶ��㵽·�����ĳ��ȣ���λ�ס�
		double m_accLength;

	public:
		PathLinkVerticesAccessor(PathVerticesAccessor& vertexAccessor, std::size_t totalVertexNum);

		void accessFirst(MapPoint3D64& point);
		void accessMiddle(MapPoint3D64& point, MapPoint3D64& prevPoint, double distToPrevPoint);
		void accessLast(MapPoint3D64& point, MapPoint3D64& prevPoint, double distToPrevPoint);
	};

	/**
	 * @brief ��ʾ��һ����������(Portion)���������Ӷ��ɵ�һ��·����
	*/
	class DirLinkPath final
	{
	public:
		/**
		 * @brief һ��Portion�ĺ�����DirLink��Ҳ����Я���������ԡ�
		*/
		struct Portion final
		{
			DirLink dirLink;
			// ��Path��㵽���Portion��DirLink��ĩ�˵���ۼƳ��ȣ���λ�ס�
			double accLength;

			/**
			 * @brief ��ת��ǰportion
			*/
			void reverse(double pathLength);

			/**
			 * @brief ͳ�Ƹ�Portion�Ķ���������һ��Portion��ȫ����һ��DirLink��
			 * @return
			*/
			std::size_t countVertexNum() { return dirLink.geometry().vertexes.size(); }
		};

		/**
		 * @brief Iterator
		 * @return
		*/
		std::vector<Portion>::iterator begin() { return m_portions.begin(); }
		std::vector<Portion>::iterator end() { return m_portions.end(); }

		std::vector<Portion>& portions() { return m_portions; }

		/**
		 * @brief �жϵ�ǰ·���Ƿ���Ч����Ч�Ķ���������������������
		 * @return
		*/
		bool valid() { return !m_portions.empty(); }

		/**
		 * @brief ����·��׷��һ����
		 * @param portion
		*/
		void addPortion(Portion portion) { m_portions.push_back(portion); }
		/**
		 * @brief ������ȫ��dirLink���ɵ�Portion
		 * @param dirLink
		*/
		void addPortion(DirLink& dirLink);

		/**
		 * @brief ����·�������һ����Ȼ���Ƴ���Note������ǰȷ��portions�ǿա�
		*/
		DirLinkPath::Portion popBack();

		/**
		 * @brief ����ǰ����·����ת��
		*/
		void reverse();

		/**
		 * @brief �����׷��path�������߸���֤�ṩ��path�뵱ǰpath����һ�¡�
		 *        Note: �ú����Ƿ����������滻ԭ����Ҫ���
		 * @param path
		*/
		void appendPath(DirLinkPath& path);

		/**
		 * @brief ���������㵽�յ���ʸ�·�������ж���
		 * @param accessor
		*/
		void accessVertices(PathVerticesAccessor& accessor);

	protected:
		/**
		 * @brief ͳ������Portion��link�Ķ��������ܺ�
		 * @return
		*/
		std::size_t calTotalVertexNum();

	private:
		std::vector<Portion> m_portions;
	};

	class SingleDirRoad {
	public:
		enum class StopReason
		{
			NONE,
			MEET_Z_LEVEL_POINT,
			NO_OUTLINKS,
			MULTI_OUT_LINKS,
		};

		struct  Portion
		{
			DirLink dirLink;
			// ��Path��㵽���Portion��DirLink��ĩ�˵���ۼƳ��ȣ���λ�ס�
			double accLength;
			StopReason stopReason{ StopReason::NONE };
			int zLevelPointIndex{ INVALID_INDEX };
			// ���Portion DirLinkĩ��������������path�ϵ�DirLink���������ҵ���SingleDirRoad����path�̵߳��������߳���Ҫ������
			std::vector<SingleDirRoad> connectedRoads;

			std::size_t countVertexNum();

			/**
			 * @brief �����ǰ����Ϊֹͣԭ��ΪMEET_Z_LEVEL_POINT���÷�����ת��������Ϊ���Portion��
			*/
			Portion asBeginPortion();
		};

		struct PortionLessComp
		{
			bool operator()(const Portion& l, const Portion& r) const;
		};

		/**
		 * @brief �жϵ�ǰ·���Ƿ���Ч����Ч�Ķ���������������������
		 * @return
		*/
		bool valid() { return !m_portions.empty(); }

		Portion& front() { return _portionAt(0); }

		Portion& back() { return _portionAt(static_cast<int>(m_portions.size() - 1)); }

		std::vector<Portion>& portions() { return m_portions; }

		MapPoint3D64* frontPoint();

		MapPoint3D64* backPoint();

		double getStartHeight() { return frontPoint()->z; }

		double SingleDirRoad::getEndHeight() { return backPoint()->z; }

		double length() { return m_portions.empty() ? 0.0 : back().accLength; }

		void accessVertices(PathVerticesAccessor& vertexModifier);

		bool hasPassedZeroLevelPoint() { return m_hasPassedZeroLevelPoint; };

		void setHasPassedZeroLevelPoint(bool val) { m_hasPassedZeroLevelPoint = val; };

		void addPortion(Portion portion) { m_portions.push_back(portion); }

	private:
		std::vector<Portion> m_portions;
		bool m_hasPassedZeroLevelPoint{ false };

		std::size_t _calculateTotoalVertexNum();

		Portion& _portionAt(int index) { return m_portions[index]; }
	};

	using SingleDirRoadPortionSet = std::set<SingleDirRoad::Portion, SingleDirRoad::PortionLessComp>;

	class DirLinkPathVertexRefCollector : public PathVerticesAccessor
	{
		std::vector<MapPoint3D64Ref> m_verticesRef;

		int32 m_heightSum{ 0 };

	public:
		std::vector<MapPoint3D64Ref> verticesRef() { return m_verticesRef; }

		int32 heightSum() { return m_heightSum; }

		void access(MapPoint3D64& vertex, const PathVerticesAccessor::AccessInfo& accessInfo) override;
	};


	class GradientVertexModifier : public PathVerticesAccessor
	{
		double m_heightFrom;
		double m_heightTo;
		double m_length;

	public:
		GradientVertexModifier(double heightFrom, double heightTo, double length) :m_heightFrom(heightFrom), m_heightTo(heightTo), m_length(length) {}

		void access(MapPoint3D64& vertex, const PathVerticesAccessor::AccessInfo& accessInfo);
	};

	class LeastHeightVertexModifier : public PathVerticesAccessor
	{
		double m_leastHeight;

	public:
		LeastHeightVertexModifier(double heightFrom, double heightTo);


		void access(MapPoint3D64& vertex, const PathVerticesAccessor::AccessInfo& accessInfo);

	};

	/**
	 * @brief ���������޸���߳��ܵ������ĵ�·�����ĵ�·�ĸ̣߳�ʹ��·�ڹ���ƽ��
	*/
	class ConnectedRoadVertexModifier : public PathVerticesAccessor
	{
		MapPoint3D64& m_mainRoadPoint;
		double m_heightDiff;
		double m_roadLength;

		MapPoint3D64* m_theFirstTwoPoints[2];

		bool m_skipTheRestPoints{ true };

	public:
		ConnectedRoadVertexModifier(MapPoint3D64& mainRoadPoint, double roadLength) : m_mainRoadPoint(mainRoadPoint), m_roadLength(roadLength)
		{
		}

		ConnectedRoadVertexModifier& operator=(ConnectedRoadVertexModifier&) = delete;

		void access(MapPoint3D64& vertex, const PathVerticesAccessor::AccessInfo& accessInfo);
	};

	struct DividedRoad
	{
		// �����е�·������·
		DirLinkPath leftPath;
		// �����е�·���Ҳ��·
		DirLinkPath rightPath;
	};

	/**
		 * @brief ��������������
		*/
	class DividedRoadSearcher
	{
		// ��������mesh����
		DbMesh* m_mesh;
		// ��������mesh����
		std::vector<DbMesh*> m_nearbyMesh;
		// m_mesh + m_nearbyMesh
		std::vector<DbMesh*> m_totalMeshes;
		// ·�������ĵ�·��ͬ��·���ݵ�Junction Road���˴�Ϊ��Щ��·��uuid��
		std::set<int64> m_centerLinks;

		/**
		 * @brief ��ȡMesh����·���ڵ�·
		 * @param mesh 
		*/
		void updateCenterLinks(DbMesh *mesh);

		/**
		 * @brief ��Links����ƽֱ�̶�����: Խ����ƽֱ������ǰ�棬�Ա��������в��������ȴ������ƽֱ�ĵ�·
		 * @param dividedLinks
		 * @return
		*/
		static void sortLinksByStraightness(std::vector<DirLink>& dividedLinks);

		/**
		 * @brief ��outlinks����startHeadingAngle��С�������򣬰���nextLinkInCCWȡ��ǰlink��һ��link��
		 * @param outlinks
		 * @param curLink
		 * @param nextLinkInCCW
		 * @return
		*/
		static DirLink getNextDirLink(std::vector<DirLink>& outlinks, DirLink& curLink, bool nextLinkInCCW);

		/**
		 * @brief ����ץ·��ʹ�õ�Link��״�ߵķ��򣬵�λ���ȡ��÷���ͨ����ָ�������ڽ������ϵĲ��������㡣
		 * @param linestring
		 * @param point
		 * @return
		*/
		double findLineDirectionWithPoint(const LineString3d& linestring, const MapPoint3D64& point, MapPoint64& nearestPointOnLineString);

		/**
		 * @brief �ж�Link�Ƿ��������е�·
		 * @param link
		 * @return
		*/
		bool isDividedRoadZGen(DirLink& link, bool ignoreCneterLink = false);

		/**
		 * @brief ���������õ�������link
		*/
		struct NeighborLink
		{
			DbLink* link;
			double direction; // link����
			double directionDiff; // link�������������������ƽ�г̶ȡ�
			MapPoint64 nearestPointOnLink; // link������������ĵ�����ĵ�
		};

		/**
		 * @brief ��Link�ϵ�ָ������һ�����뷶Χ����������link�����������Ľ��������direction�����ƽ�г̶ȴӴ�С���򣬷��ؽ����Խǰ���linkԽƽ�С�
		 * @param link ��ǰ���·
		 * @param point ��ǰ���·
		 * @param maxFindDist ����������롣��λ��γ�ȡ�
		 * @param direction
		 * @return
		*/
		std::vector<NeighborLink> searchParallelNeighborLinks(DirLink& link, const MapPoint3D64& point, const double maxFindDist, const double direction);

		struct GrabRoadResult
		{
			DirLink link;
			bool isLeft;

			bool valid() const { return link.valid(); }
		};
		/**
		 * @brief ZGenerator��ʹ�õ�ץ·�߼���ץȡ������·����ͬ����һ������·��
		 * @param grabRoadIsLeft ץȡ���ĶԲ��·�Ƿ������ŵ�·���������·
		 * @param pointOnLink link�ϵĵ㣬�Ӹõ�����Χץ·����
		 * @param pointOnLinkDirection �ڸõ㳯��·�н�����ĽǶ�
		 * @return ���ûץ��������ЧDirLink
		*/
		GrabRoadResult grabAnotherRoadZGen(DirLink& link, const MapPoint3D64& pointOnLink, double pointOnLinkDirection);

		/**
		 * @brief ���ץ·��ѡȡ���Ž��
		 * @param link 
		 * @param pointOnLink 
		 * @param grabRoadIsLeft 
		 * @return 
		*/
		GrabRoadResult grabAnotherRoadZGenMultiTimes(DirLink& link);

		/**
		 * @brief ��DirLink˳������淽������������·
		 * @param beginLinkOnLeft �Ƿ��������е��������Ҳ��·
		 * @return
		*/
		DirLinkPath extendRoad(DirLink& beginLink, bool beginLinkOnLeft, std::set<int64>& visitedLinks);

		std::vector<DividedRoad> searchZGen(std::vector<DirLink>& dividedLinks);

		/**
		 * @brief ������ĩ�˶����Ż��������߲��ҽ���о��������һ��·һ�˳�����һ��ǳ�����������ú����������������
		 * @return ���Ƴ��������е�·��[left links, right links]
		*/
		static std::array<std::vector<DirLink>, 2> trimEndRoadsPerDividedRoad(DividedRoad& dividedRoad, bool trimBegin, bool trimEnd);

	public:
		DividedRoadSearcher(DbMesh* currentMesh, std::vector<DbMesh*> nearbyMesh);

		/**
		 * @brief ZGenerator�е������������߼�
		 * @return
		*/
		std::vector<DividedRoad> searchZGen();
	};


	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//              ZGenerator
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	struct ZLevelLinks
	{
		DbZLevel* zlevel;
		std::vector<DirLink> links;
	};


	class AdjustZVertexAccessor : public DirLink::VertexAccessor
	{
		// ���߳�
		double m_targetHeight;
		// ��Զ�������룬��
		double m_influenceLength;
		// ����access���ۼƷ��ʾ��룬��
		double m_accLength;
		// ��ʼ��̲߳�
		double m_firstDeltaHeight;

	public:
		AdjustZVertexAccessor(double targetHeight, double firstDeltaHeight, double influenceLength);

		void accessFirst(MapPoint3D64& point) override;

		void accessMiddle(MapPoint3D64& point, MapPoint3D64& prevPoint, double distToPrevPoint) override;

		void accessLast(MapPoint3D64& point, MapPoint3D64& prevPoint, double distToPrevPoint) override { accessMiddle(point, prevPoint, distToPrevPoint); }

		double restLength() const { return m_influenceLength - m_accLength; }
	};

	class AdjustZ2VertexAccessor : public DirLink::VertexAccessor
	{
		double m_accessTotalLength;
		double m_startHeight;
		double m_endHeight;
		double m_startDeltaHeight;
		double m_endDeltaHeight;
		double m_accLength;

	public:
		AdjustZ2VertexAccessor(DirLink& link, double startHeight, double endHeight);

		void accessFirst(MapPoint3D64& point) override;

		void accessMiddle(MapPoint3D64& point, MapPoint3D64& prevPoint, double distToPrevPoint) override;

		void accessLast(MapPoint3D64& point, MapPoint3D64& prevPoint, double distToPrevPoint) override;
	};


	class ZGenerator : public Generator
	{
	public:
		ZGenerator(GeneratorData& data) :Generator(data) {};
	protected:
		virtual void generate(DbMesh* const pMesh) override;
		virtual void generateRelation(DbMesh* const pMesh, std::vector<DbMesh*>* nearby) override;

	private:

		int m_currentMeshID;
		std::map<int64, std::unique_ptr<LineString3d>> m_linkGeometryOwner; // DbLink.uuid -> geometry

		LineString3d& getOrAddLinkGeomCopy(DbLink& link);
		LineString3d* getLinkGeomCopyOrNull(int64 linkId);
		std::vector<DirLink> getEndDirectOutlinksExceptSelf(DirLink& link);

		/**
		 * @brief ʹ��һϵ�е�Link�����Z Level�����ɸ̡߳�
		 *        ���ɵĸ߳�ֻ��д����Link����
		 *        ���÷���������ṩ��¡�Ķ���������Щlink����ʹ���¸߳�Ҳ����ʹ�á�
		 * @param zLevelLinks
		*/
		void generateHeight(std::vector<ZLevelLinks>& zLevelLinksArray, std::vector<DividedRoad>& dividedRoads);

		/**
		 * @brief ̧�߸߼�·��Viaduct n. �߼��š�
		*/
		void adjustViaductRoad(DirLink& link);
		void adjustViaductRoad(const std::vector<DirLink>& links);

		/**
		 * @brief ��Mesh�����ȡZLevel & Links
		 * @param pMesh
		 * @return
		*/
		std::vector<ZLevelLinks> readZLevelLinks(DbMesh* pMesh);

		bool checkAndAdjustZLevel(std::vector<ZLevelLinks>& zLevelLinksArray);
		bool checkIntersectingInfo(ZLevelLinks& zLevelLinkPair);
		void adjustPointHeight(DirLink& pLink, int idx, double targetZ, double slope);
		bool checkDividedRoadNotModified(DividedRoad& dividedRoad);
		void adjustDividedRoadHeight(DividedRoad& dividedRoad);
		void bfs(AdjustHeightQueue& adjustQueue, std::set<int64>& adjustedLinks);

		void processRoadBetweenZLevels(std::vector<ZLevelLinks>& zLevelLinks);
		void processHeightByZLevelPoint(DbZLevel::DbRelLink& relLink, DirLink& pLink, SingleDirRoadPortionSet& processedRoadPortion);
		SingleDirRoad extendRoadFromZLevelPoint(DirLink& beginLink, int beginLinkZLevelPointIndex);
		/**
		 * @brief �������·��������������·��ֱ����������Z Level���������·�����������·�ж�����ߵ�·��
		*/
		SingleDirRoad extendRoadFromDSegment(DirLink& beginLink);

		void adjustSingleDirRoadHeight(SingleDirRoad& single_dir_road);

		bool isPedestrian(DirLink& link) { return link.kind() == 10; };
	};

}

