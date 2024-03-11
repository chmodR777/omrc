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
		int64 m_linkuuid; // 放在前面，方便调试时第一时间可以查看到link的id
		bool m_forward; // 正向(forward)->与Link画线方向相同，反向(backward)->与Link画线方向相反。非正向即反向。
		// 原始Link
		DbLink* m_link;
		LineString3d* m_linkGeometry; // 在DirLink内部永远使用m_linkGeometry而不是m_link->geometry

	public:
		/**
		 * @brief 构造一个空的DirectLink对象，不要调用该对象的任何方法(除了valid)！！！
		*/
		DirLink() : m_linkuuid(0), m_forward(true), m_link(nullptr), m_linkGeometry(nullptr) { }

		/**
		 * @brief 使用使用画线方向构造DirectLink
		 * @param link Not NULL!!!
		 * @param forward
		*/
		DirLink(DbLink* link, bool forward) : m_linkuuid(link->uuid), m_forward(forward), m_link(link), m_linkGeometry(nullptr) { }

		DirLink(DbLink* link, bool forward, LineString3d& linkGeometry) : m_linkuuid(link->uuid), m_forward(forward), m_link(link), m_linkGeometry(&linkGeometry) { }

		/**
		 * @brief 创建一个反向的DirectLink对象。
		 * @return
		*/
		DirLink getReverseDirLink() { return DirLink{ m_link, !m_forward, *m_linkGeometry }; }
		DirLink getForwardDirLink() { return  m_forward ? *this : getReverseDirLink(); }
		DirLink getBackwardDirLink() { return  !m_forward ? *this : getReverseDirLink(); }

		/**
		 * @brief 检查当前DirectLink对象是否无效。无效的对象不允许调用除了其它任何方法！！
		 * @return
		*/
		bool valid() const { return m_link != nullptr; }

		bool forward() const { return m_forward; }

		/**
		 * @brief 当前对象与other持有的无方向Link对象是否一样。
		 * @param other
		 * @return
		*/
		bool rawLinkEqual(const DirLink& other) const { return this->m_link == other.m_link; }

		////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/////////                                                                                    ///////////////
		/////////                           DbLink属性代理                                            //////////////
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
		 * @brief 不要修改该对象的任何东西!!!
		*/
		DbLink* getLinkDangerous() { return m_link; }

		////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/////////                                                                                    ///////////////
		/////////                           几何相关函数                                              ///////////////
		/////////                                                                                   ///////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////////////

		/**
		 * @brief DirLink几何第二个点到第一个点的向量在以第二个点为原点的平面直角坐标系下的与+X逆时针方向的夹角。夹角单位弧度。
		 * @return 弧度，范围[0, 2PI)
		*/
		double startHeadingAngle();
		/**
		 * @brief DirLink几何倒数第二个点到倒数第一个点的向量在以倒数第二个点为原点的平面直角坐标系下的与+X逆时针方向的夹角。夹角单位弧度。
		 * @return 弧度，范围[0, 2PI)
		*/
		double endHeadingAngle();


		/**
		 * @brief 获取DirectLink 起点和末端点的高程。高程单位厘米。
		 * @return
		*/
		double getStartHeight() { return getStartPointRef().z; }

		double getEndHeight() { return getEndPointRef().z; }

		/**
		 * @brief 获取DirectLink第一个点的引用。注意外部可能会修改这个点。
		*/
		MapPoint3D64& getStartPointRef() { return m_forward ? m_linkGeometry->vertexes.front() : m_linkGeometry->vertexes.back(); }

		/**
		 * @brief 获取DirectLink最后一个点的引用。注意外部可能会修改这个点。
		*/
		MapPoint3D64& getEndPointRef() { return m_forward ? m_linkGeometry->vertexes.back() : m_linkGeometry->vertexes.front(); }
		MapPoint3D64 getEndPoint() const { return m_forward ? m_linkGeometry->vertexes.back() : m_linkGeometry->vertexes.front(); }
		/**
		 * @brief 获取DireLink顺方向的指定点索引的点引用。注意外部可能会修改这个点。
		 * @param index
		 * @return
		*/
		MapPoint3D64& getPointRef(int index) { return m_forward ? m_linkGeometry->vertexes[index] : m_linkGeometry->vertexes[int(m_linkGeometry->vertexes.size()) - 1 - index]; }

		MapPoint3D64& getPointRefNoDir(int index) { return m_linkGeometry->vertexes[index]; }

		/**
		 * @brief 计算在有向Link上从起点到终点的长度上距离起点指定百分比的点
		 * @param perc [0.0, 1.0]
		 * @return 
		*/
		MapPoint3D64 calPointAt(double perc) const;

		/**
		 * @brief 获取当前link几何长度。注意与calLength区别，length()视为获取属性，不需要耗性能的计算。
		 * @return
		*/
		// TODO: 后续直接返回缓存的长度而不是再次计算。
		double length() { return calLength(); }

		/**
		 * @brief 计算Link长度，单位米。
		 * @return
		*/
		double calLength() const;

		/**
		* @brief 计算两点(indexFrom & indexTo)的累计长度，单位米。其中indexFrom & indexTo大小关系无所谓。
		*/
		double calPortionLength(int indexFrom, int indexTo) const;

		/**
		 * @brief 计算DirectLink从起点到指定点的长度。单位米。
		 *        Note: 该计算与当前DirectLink对象方向相关。
		*/
		double calStartToMidLength(int index) const { return m_forward ? calToMidForwardLength(index) : calToMidBackwardLength(index); }

		/**
		 * @brief 计算DirectLink从指定点到末端点的长度。单位米。
		 *        Note: 该计算与当前DirectLink对象方向相关。
		*/
		double calMidToEndLength(int index) const { return m_forward ? calToEndForwardLength(index) : calToEndBackwardLength(index); }

		/**
		 * @brief 计算Link从起点开始到指定点(位于中间的点)长度。单位米。
		 *        Note: 该计算无关当前DirectLink对象方向。
		*/
		double calToMidForwardLength(int index) const { return calPortionLength(0, index); }

		/**
		 * @brief 计算Link从末端点起到指定点的长度。单位米。
		 *        Note: 该计算无关当前DirectLink对象方向。
		*/
		double calToMidBackwardLength(int index) const { return calPortionLength(int(m_linkGeometry->vertexes.size()) - 1, index); }

		/**
		 * @brief 计算Link从指定点开始到最后一个点的累计长度。单位米。
		 *        Note: 该计算无关当前DirectLink对象方向。
		*/
		double calToEndForwardLength(int index) const { return calPortionLength(index, int(m_linkGeometry->vertexes.size()) - 1); }

		/**
		 * @brief 计算Link从指定点到起始点的累计长度。单位米。
		 *        Note: 该计算无关当前DirectLink对象方向。
		*/
		double calToEndBackwardLength(int index) const { return calPortionLength(index, 0); }

		/**
		 * @brief 对Link几何从indexFrom的顶点访问到indexTo的顶点（依次序的，ordered）。
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

		// accessor可以控制访问的行为
		enum class AccessorControl
		{
			NONE,
			STOP,
		};

		////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/////////                                                                                    ///////////////
		/////////                           路网拓扑相关函数                                           ///////////////
		/////////                                                                                   ///////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////////////

		/**
		 * @brief 获取与该有向Link末端节点直接相连有向Link。包含自身反方向的Link对象。
		 * @return
		*/
		template <bool withLinkGeometry=false>
		std::vector<DirLink> getEndDirectOutlinks();
		std::vector<int64> getEndDirectOutlinksUUID();

		/**
		 * @brief 获取与该有向Link末端节点直接相连有向Link。 不 包含自身反方向的Link对象。
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


	// bfs中记录相邻link信息，短的link优先访问处理
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
	 * @brief 访问这条路径上每一部分的道路形状线的顶点
	*/
	class PathVerticesAccessor
	{
	public:
		struct AccessInfo
		{
			// 访问的顶点在路径几何顶点中的索引。
			std::size_t index;
			// 路径中所有几何顶点数量
			std::size_t totalVertexNum;
			// 访问的顶点到路径起点的长度，单位米。
			double accLength;
		};
		/**
		 * @brief 该函数接收顶点并对其修改（可选）
		 * @param vertex 顶点坐标
		 * @param index
		 * @param totalVertexNum  通过SingleDirRoad对象访问顶点则该值为所有道路中所有顶点中的数量。
		 * @param accLength 通过SingleDirRoad对象访问顶点则该值为道路累计长度。
		*/
		virtual void access(MapPoint3D64& vertex, const AccessInfo& accessInfo) = 0;

		virtual ~PathVerticesAccessor() {};
	};

	/**
	 * @brief 负责访问构成path的link上的顶点并收集信息传递给PathVerticesAccessor
	*/
	class PathLinkVerticesAccessor : public DirLink::VertexAccessor
	{
		PathVerticesAccessor& m_vertexAccessor;
		// 路径中所有几何顶点数量
		std::size_t m_totalVertexNum;
		// path上一部分的末端点
		MapPoint3D64* m_lastPortionLastPoint;
		// 访问的顶点在路径几何顶点中的索引。
		std::size_t m_index;
		// 访问的顶点到路径起点的长度，单位米。
		double m_accLength;

	public:
		PathLinkVerticesAccessor(PathVerticesAccessor& vertexAccessor, std::size_t totalVertexNum);

		void accessFirst(MapPoint3D64& point);
		void accessMiddle(MapPoint3D64& point, MapPoint3D64& prevPoint, double distToPrevPoint);
		void accessLast(MapPoint3D64& point, MapPoint3D64& prevPoint, double distToPrevPoint);
	};

	/**
	 * @brief 表示由一个或多个部分(Portion)依次序连接而成的一条路径。
	*/
	class DirLinkPath final
	{
	public:
		/**
		 * @brief 一个Portion的核心是DirLink，也可以携带其它属性。
		*/
		struct Portion final
		{
			DirLink dirLink;
			// 从Path起点到达该Portion的DirLink的末端点的累计长度，单位米。
			double accLength;

			/**
			 * @brief 反转当前portion
			*/
			void reverse(double pathLength);

			/**
			 * @brief 统计该Portion的顶点数量。一个Portion完全包含一个DirLink。
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
		 * @brief 判断当前路径是否有效。有效的对象才允许访问其它方法。
		 * @return
		*/
		bool valid() { return !m_portions.empty(); }

		/**
		 * @brief 给该路径追加一部分
		 * @param portion
		*/
		void addPortion(Portion portion) { m_portions.push_back(portion); }
		/**
		 * @brief 新增完全由dirLink构成的Portion
		 * @param dirLink
		*/
		void addPortion(DirLink& dirLink);

		/**
		 * @brief 返回路径的最后一部分然后移除。Note：调用前确保portions非空。
		*/
		DirLinkPath::Portion popBack();

		/**
		 * @brief 将当前有向路径反转。
		*/
		void reverse();

		/**
		 * @brief 在最后追加path。调用者负责保证提供的path与当前path方向一致。
		 *        Note: 该函数是否满足里氏替换原则需要检查
		 * @param path
		*/
		void appendPath(DirLinkPath& path);

		/**
		 * @brief 依次序从起点到终点访问该路径的所有顶点
		 * @param accessor
		*/
		void accessVertices(PathVerticesAccessor& accessor);

	protected:
		/**
		 * @brief 统计所有Portion的link的顶点数量总和
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
			// 从Path起点到达该Portion的DirLink的末端点的累计长度，单位米。
			double accLength;
			StopReason stopReason{ StopReason::NONE };
			int zLevelPointIndex{ INVALID_INDEX };
			// 与该Portion DirLink末端相连的其它非path上的DirLink延伸搜索找到的SingleDirRoad，在path高程调整完后其高程需要调整。
			std::vector<SingleDirRoad> connectedRoads;

			std::size_t countVertexNum();

			/**
			 * @brief 如果当前部分为停止原因为MEET_Z_LEVEL_POINT，该方法将转换返回作为起点Portion。
			*/
			Portion asBeginPortion();
		};

		struct PortionLessComp
		{
			bool operator()(const Portion& l, const Portion& r) const;
		};

		/**
		 * @brief 判断当前路径是否有效。有效的对象才允许访问其它方法。
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
	 * @brief 该类用于修改与高程受到调整的道路相连的道路的高程，使得路口过度平滑
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
		// 上下行道路的左侧道路
		DirLinkPath leftPath;
		// 上下行道路的右侧道路
		DirLinkPath rightPath;
	};

	/**
		 * @brief 负责上下线搜索
		*/
	class DividedRoadSearcher
	{
		// 中心网格mesh数据
		DbMesh* m_mesh;
		// 相邻网格mesh数据
		std::vector<DbMesh*> m_nearbyMesh;
		// m_mesh + m_nearbyMesh
		std::vector<DbMesh*> m_totalMeshes;
		// 路口内中心道路，同算路数据的Junction Road。此处为该些道路的uuid。
		std::set<int64> m_centerLinks;

		/**
		 * @brief 读取Mesh加入路口内道路
		 * @param mesh 
		*/
		void updateCenterLinks(DbMesh *mesh);

		/**
		 * @brief 将Links按照平直程度排序: 越长又平直的排在前面，以便在上下行查找中优先处理更加平直的道路
		 * @param dividedLinks
		 * @return
		*/
		static void sortLinksByStraightness(std::vector<DirLink>& dividedLinks);

		/**
		 * @brief 将outlinks按照startHeadingAngle从小到大排序，按照nextLinkInCCW取当前link下一个link。
		 * @param outlinks
		 * @param curLink
		 * @param nextLinkInCCW
		 * @return
		*/
		static DirLink getNextDirLink(std::vector<DirLink>& outlinks, DirLink& curLink, bool nextLinkInCCW);

		/**
		 * @brief 计算抓路中使用的Link形状线的方向，单位弧度。该方向通过与指定点最邻近的线上的部分来计算。
		 * @param linestring
		 * @param point
		 * @return
		*/
		double findLineDirectionWithPoint(const LineString3d& linestring, const MapPoint3D64& point, MapPoint64& nearestPointOnLineString);

		/**
		 * @brief 判断Link是否是上下行道路
		 * @param link
		 * @return
		*/
		bool isDividedRoadZGen(DirLink& link, bool ignoreCneterLink = false);

		/**
		 * @brief 近邻搜索得到的其它link
		*/
		struct NeighborLink
		{
			DbLink* link;
			double direction; // link方向
			double directionDiff; // link与搜索输入的期望方向平行程度。
			MapPoint64 nearestPointOnLink; // link上与搜索输入的点最近的点
		};

		/**
		 * @brief 从Link上的指定点向一定距离范围内搜索其它link，并且搜索的结果按照与direction方向的平行程度从大到小排序，返回结果中越前面的link越平行。
		 * @param link 当前侧道路
		 * @param point 当前侧道路
		 * @param maxFindDist 最大搜索距离。单位经纬度。
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
		 * @brief ZGenerator中使用的抓路逻辑。抓取与有向路段相同的另一个有向路段
		 * @param grabRoadIsLeft 抓取到的对侧道路是否是沿着道路方向的左侧道路
		 * @param pointOnLink link上的点，从该点向周围抓路搜索
		 * @param pointOnLinkDirection 在该点朝道路行进方向的角度
		 * @return 如果没抓到返回无效DirLink
		*/
		GrabRoadResult grabAnotherRoadZGen(DirLink& link, const MapPoint3D64& pointOnLink, double pointOnLinkDirection);

		/**
		 * @brief 多次抓路，选取最优结果
		 * @param link 
		 * @param pointOnLink 
		 * @param grabRoadIsLeft 
		 * @return 
		*/
		GrabRoadResult grabAnotherRoadZGenMultiTimes(DirLink& link);

		/**
		 * @brief 沿DirLink顺方向和逆方向延伸搜索道路
		 * @param beginLinkOnLeft 是否是上下行的左侧或者右侧道路
		 * @return
		*/
		DirLinkPath extendRoad(DirLink& beginLink, bool beginLinkOnLeft, std::set<int64>& visitedLinks);

		std::vector<DividedRoad> searchZGen(std::vector<DirLink>& dividedLinks);

		/**
		 * @brief 上下线末端对齐优化。上下线查找结果中经常会出现一侧路一端超出另一侧非常长的情况，该函数处理这种情况。
		 * @return 被移除的上下行道路。[left links, right links]
		*/
		static std::array<std::vector<DirLink>, 2> trimEndRoadsPerDividedRoad(DividedRoad& dividedRoad, bool trimBegin, bool trimEnd);

	public:
		DividedRoadSearcher(DbMesh* currentMesh, std::vector<DbMesh*> nearbyMesh);

		/**
		 * @brief ZGenerator中的上下行搜索逻辑
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
		// 起点高程
		double m_targetHeight;
		// 最远调整距离，米
		double m_influenceLength;
		// 所有access中累计访问距离，米
		double m_accLength;
		// 起始点高程差
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
		 * @brief 使用一系列的Link对象和Z Level点生成高程。
		 *        生成的高程只管写出到Link对象，
		 *        调用方负责可能提供克隆的对象所以这些link对象即使更新高程也不会使用。
		 * @param zLevelLinks
		*/
		void generateHeight(std::vector<ZLevelLinks>& zLevelLinksArray, std::vector<DividedRoad>& dividedRoads);

		/**
		 * @brief 抬高高架路。Viaduct n. 高架桥。
		*/
		void adjustViaductRoad(DirLink& link);
		void adjustViaductRoad(const std::vector<DirLink>& links);

		/**
		 * @brief 从Mesh对象读取ZLevel & Links
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
		 * @brief 朝有向道路延伸搜索其它道路，直到碰到存在Z Level点的其它道路或者有有向道路有多个出边道路。
		*/
		SingleDirRoad extendRoadFromDSegment(DirLink& beginLink);

		void adjustSingleDirRoadHeight(SingleDirRoad& single_dir_road);

		bool isPedestrian(DirLink& link) { return link.kind() == 10; };
	};

}

