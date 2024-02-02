#pragma once
#include <string>
#include <vector>
#include <map>
#include "../Geometry.h"
namespace OMDB
{
    enum class RecordType : uint8
    {
        UNKONWN = 0,
		DB_HAD_PA_VALUE,
        DB_HAD_LINK,
        DB_HAD_LINK_PA,
        DB_HAD_NODE,
		DB_HAD_LINK_FIXED_SPEEDLIMIT,

		DB_HAD_ROAD_BOUNDARY_LINK,
        DB_HAD_ROAD_BOUNDARY_NODE,
        DB_HAD_ROAD_BOUNDARY_PA,

        DB_ROAD_NAME,
        DB_HAD_LINK_NAME,
        DB_HAD_LINK_LANEPA,
		DB_HAD_LG_LINK,
        DB_HAD_LG_ASSOCIATION,
        DB_HAD_LANE_LINK,
        DB_HAD_LANE_NODE,
        DB_HAD_LANE_MARK_LINK,
        DB_HAD_LANE_MARK_NODE,
        DB_HAD_LANE_MARK_PA,
		DB_HAD_LANE_FIXED_SPEEDLIMIT,
        DB_HAD_OBJECT_CROSS_WALK,
        DB_HAD_OBJECT_STOPLOCATION,
        DB_HAD_OBJECT_FILL_AREA,
        DB_HAD_OBJECT_TEXT,
        DB_HAD_OBJECT_ARROW,
        DB_HAD_OBJECT_WALL,
        DB_HAD_OBJECT_TRAFFIC_SIGN,
        DB_HAD_OBJECT_BARRIER,
		DB_HAD_OBJECT_LG_REL,
		DB_HAD_OBJECT_LANE_LINK_REL,
		DB_HAD_OBJECT_OBJECT_REL,
		DB_HAD_TOLLGATE,				//收费站
		DB_HAD_TOLL_LANE,				//收费站内车道收费信息
		DB_HAD_INTERSECTION,
		DB_HAD_OBJECT_TRAFFIC_LIGHTS,
		DB_HAD_OBJECT_POLE,
		DB_HAD_LANE_LINK_TURNWAITING,

		DB_HAD_ZLEVEL,
		DB_HAD_LANEINFO,
		DB_RD_LINK_LANEPA,
		DB_RD_LANE_LINK_CLM,
		DB_RD_LANE_TOPO_DETAIL,


    };

	class DbMesh;
    class DbRecord
	{
	public:
		int64 uuid{0};
		RecordType recordType{ RecordType ::UNKONWN};
		DbMesh* owner = nullptr;
	};

	class DbPAField
	{
	public:
		int featureType{ 0 };
		double startOffset{ 0.0 };
		double endOffset{ 0.0 };
		MultiPoint3d geometry;
	};

	class DbSkeleton : public DbRecord
	{
	public:
		std::vector<DbSkeleton*>    previous;
		std::vector<DbSkeleton*>    next;
	};

	class DbLgLink;
	class DbLinkDataLevel;
	class DbLinkSeparation;
	class DbLinkMedian;
	class DbLinkOverheadObstruction;
	class DbLinkTollArea;
	
	class DbNode;
	class DbZLevel;
	class DbLinkName;
	class DbLaneInfo;
	class DbRdLinkLanePa;
	class DbLink : public DbSkeleton
	{
	public:
		struct DbRelLaneInfo
		{
			int seqNum{0};
			int direct{0};
		};
		struct ClipPoint
		{
			MapPoint3D64 position;
			size_t startIndex;
			size_t endIndex;
		};

		int64            startNode{0};
		int64           endNode{0};
		int                     kind{0};
		LineString3d			geometry;
		int multi_digitized{0};
		int direct{0};
		int separation_left{0};
		int separation_right{0};
		int median_left{0};
		int median_right{0};
		int overhead_obstruction{0};
		std::vector<int> wayTypes;

		std::vector<DbLinkDataLevel> dataLevels;
		std::vector<DbLinkSeparation> separations;
		std::vector<DbLinkMedian> medians;
		std::vector<DbLinkOverheadObstruction> overheadObstructions;
		std::vector<DbLinkTollArea> tollAreas;

		// TODO 以下需要写出
		std::map<int64, DbRelLaneInfo> relLaneInfos;

		// 以下字段不需要输出
		std::vector<DbZLevel*> zLevels;
		std::vector<DbRdLinkLanePa*> lanePas;
		std::vector<DbLgLink*> groups;
		std::vector<DbLinkName*> linkNames;
		DbNode* startNodePtr{ nullptr };
		DbNode* endNodePtr{ nullptr };

		// 路面裁切点(首尾点相连时存DSegmentId?)
		std::map<int64, std::vector<ClipPoint>> clipPoints;

		// 是否跨网格
		bool crossGrid{ false };
		float length{0};
	};

	// 有向Link,该表为虚表,不需要写出数据
	// 注意,往该对象添加数据不会改变原始数据
	class DbDirectLink : public DbLink
	{
	public:
		// 单向
		int direct{ 0 };
		// 单向前置节点
		std::vector<DbSkeleton*>    previous;
		// 单向后置节点
		std::vector<DbSkeleton*>    next;
		// 单向车道组
		std::vector<DbRdLinkLanePa*> lanePas;

	public:
		static DbDirectLink* getDirectLink(DbLink* pLink, int direct);
	};

	class DbZLevel : public DbRecord
	{
	public:
		struct DbRelLink
		{
			int64					  relLinkid{ 0 };
			int						  shpSeqNum{ 0 };
			int						  startEnd{ 0 };
			int						  zLevel{ 0 };
			MapPoint3D64				  geometry;
		};
		std::vector<DbRelLink> relLinks;

	public:
		forceinline DbRelLink* relLink(int64 relLinkid) {
			return &(*std::find_if(relLinks.begin(), relLinks.end(), 
				[&](DbRelLink& l) {return l.relLinkid == relLinkid; }));
		}
	};

	class DbLaneInfo : public DbRecord
	{
	public:
		std::wstring inLaneInfo{L""};
		std::wstring extendLaneInfo{L""};
		std::wstring busLaneInfo{L""};
		int reachDir{0};
	};

	// DB_HAD_LG_LINK,DB_HAD_LINK_LANEPA,DB_RD_LINK_LANEPA三个表结构相同
	class DbRelLinkCommon : public DbSkeleton
	{
	public:
		struct DbRelLink
		{
			int64					  relLinkid{0};
			double                  startOffset{0.0};
			double                  endOffset{0.0};
			int						  type{0};
			int						  directType{0};
		};
		std::map<int64, DbRelLink> relLinks;
		bool generateHdData = false; // 是否生成的组
	};

	class DbLgLink : public DbRelLinkCommon{};

	class DbHadLinkLanePa : public DbRelLinkCommon{};

	// TODO 该表也需要写出
	// DbRd开头的为CLM表
	class DbRdLaneLinkCLM : public DbRecord
	{
	public:
		struct DbRdLaneLinkAccessCLM
		{
			int64					  characteristic{ 0 };
			std::wstring				  validPeriod{ L"" };
		};
		struct DbRdLaneLinkConditionCLM
		{
			int						  type{ 0 };
			int						  direct{ 0 };
			std::wstring				  validPeriod{ L"" };
		};
		struct DbRdLaneLinkSpeedLimitCLM
		{
			int64					  maxSpeed{ 0 };
			int						  minSpeed{ 0 };
		};

		int laneType{0};
		int tranType{0};
		std::wstring arrowDir{ L"" };

		// access
		std::vector<DbRdLaneLinkAccessCLM> accesses;

		// condition
		std::vector<DbRdLaneLinkConditionCLM> conditions;

		// speedlimit
		std::vector<DbRdLaneLinkSpeedLimitCLM> speedLimits;
	};

	class DbLaneMarkLink;
	class DbRoadBoundLink;
	class DbRdLinkLanePa : public DbRelLinkCommon {
	public:
		struct DbRelRdLinkLane
		{
			int64					  relLinkLaneId{ 0 };
			int						  seqNum{ 0 };
		};

		std::vector<DbRelRdLinkLane> relLinkLanes;

		// 以下字段不需要输出
		std::vector<DbRoadBoundLink*>	roadBoundaries;  //按在车道组内从左到右的顺序排序
		std::vector<DbLaneMarkLink*>	    laneBoundaries;  //按在车道组内从左到右的顺序排序

		// 是否跨网格
		bool crossGrid{ false };
		int64 inIntersection{ 0 };
		bool inClipRing{ false };
		bool isGenerated{ false };
		std::atomic_int generateLanePaCnt{ 0 };
	};

	// TODO 该表也需要写出
	class DbRdLaneTopoDetail : public DbRecord
	{
	public:
		struct DbRdLaneTopoCond
		{
			int						  vehicleType{ 0 };
			std::wstring				  validPeriod{ L"" };
		};
		struct DbRdLaneTopoVia
		{
			int64					  relLinkLaneId{ 0 };
			int						  seqNum{ 0 };
		};

		int64 inLaneLinkPid{0};
		int64 outLaneLinkPid{0};
		int throughTurn{0};
		int laneChange{0};

		// condition
		std::vector<DbRdLaneTopoCond> conditions;

		// 经过车道
		std::vector<DbRdLaneTopoVia> viaLinkLanes;
	};

	class DbLgAssociation : public DbRecord
	{
	public:
		struct DbRelLgAssociation
		{
			int                directType{0};
			int64            firstLgLinkId{0};
			int64            secondLgLinkId{0};
		};
		// 一个车道组可能与周边两个车道组形成两个LA关系,此时group_id可能相同
		std::vector<DbRelLgAssociation> relLgAssociations;
	};

	class DbNode : public DbRecord
	{
	public:
		int64             intersectionId{0};
		MapPoint3D64					geometry;
		std::vector<int32>             meshIds;
		int wayType{0};

		// 该字段不需要输出
		std::vector<DbLink*> links;
	public:
		forceinline std::vector<DbLink*> getLinksExcept(DbLink* const pLink) {
			std::vector<DbLink*> tmpLinks;
			for (auto link : links)
				if (pLink != link)
					tmpLinks.push_back(link);
			return tmpLinks;
		}
	};

	class DbPAValue : public DbRecord
	{
	public:
		int seqNum{-1};
		int attributeType{0};
		int attributeValue{0};
	};

	class DbLinkPA : public DbRecord, public DbPAField
	{
	public:
		int64 relLinkId{0};
		std::vector<DbPAValue*> paValues;
	};

	class DbLinkDataLevel : public DbRecord, public DbPAField
	{
	public:
		int dataLevel{ 0 };
	};

	class DbLinkSeparation : public DbRecord, public DbPAField
	{
	public:
		int side{ 0 };
		int separation{ 0 };
	};

	class DbLinkMedian : public DbRecord, public DbPAField
	{
	public:
		int side{ 0 };
		int median{ 0 };
	};

	class DbLinkOverheadObstruction : public DbRecord, public DbPAField
	{
	};

	class DbLinkTollArea : public DbRecord, public DbPAField
	{
	};

	class DbRoadName;
	class DbLinkName : public DbRecord
	{
	public:
		struct DbLinkNamePa : public DbPAField
		{
			int nameGroup{ 0 };
			int seqNum{ 0 };
			int nameClass{ 0 };
			int nameType{ 0 };

			// 该字段不需要输出
			std::vector<DbRoadName*> roadNames;
		};
		std::vector<DbLinkNamePa> linkNamePas;
	};

	class DbRoadName : public DbRecord
	{
	public:
		struct DbRoadNameGroup
		{
			std::wstring langCode{ L"" };
			std::wstring name{ L"" };
			std::wstring type{ L"" };
			std::wstring base{ L"" };
			std::wstring prefix{ L"" };
			std::wstring suffix{ L"" };
			int codeType{ 0 };
		};

		std::vector<DbRoadNameGroup> nameGroups;
	};

	class DbRoadBoundNode : public DbRecord
	{
	public:
		MapPoint3D64 geometry;
	};

	class DbRoadBoundPA : public DbRecord, public DbPAField
	{
    public:
		int64 relRoadBoundLinkId{0};
		std::vector<DbPAValue*> paValues;
	};

    class DbRoadBoundLink : public DbSkeleton
    {
    public:
		//HAD_LG_RDBOUND_REL
		struct DbLgRoadBoundREL
		{
			int side{0};
			int direction{0};
		};
		int64	starRoadBoundNodeId{0};
		int64	endRoadBoundNodeId{0};
		// -1标识表里没查到边界类型
		int boundaryType{-1};
		LineString3d geometry;

		std::map<int64, DbLgRoadBoundREL> relLgs;
    };

	class DbLaneLink : public DbSkeleton
	{
	public:
		uint32 laneType{0};
		int isIntersection{0};
		int	arrowDir{0};
		int width{0};	//车道平均宽度（单位：厘米）

		// HAD_LANE_LINK_LG
		int64	relLgId{0};
		uint32	laneNum{0};
		int64	startLaneNodeId{0};
		int64	endLaneNodeId{0};
		uint32 seqNumber{0};
		int laneDir{0};

		// HAD_LANE_LINK_CONDITION
		int conditionType{0};

        LineString3d geometry;
	};

	class DbLaneNode : public DbRecord
	{
	public:
		MapPoint3D64 geometry;
	};

	enum class DbLaneBoundaryPAType
	{
		BOUNDARY_TYPE = 7,
		TRAVERSAL = 8,
		MARK_COUNT = 10,
		MARK_SEQ_NUM = 11,
		MARK_TYPE = 12,
		MARK_COLOR = 13,
		//MARK_WIDTH = 14,
		//MARK_MATERIAL = 15,
		LATERAL_OFFSET = 16,
	};

	class DbLaneMarkLink : public DbSkeleton
	{
	public:

		struct DbLaneMarkLinkMarking
		{
			int markSeqNum{0};
			int markType{ 0 };
			int markColor{ 0 };
			int markWidth{ 0 };
			int markMaterial{ 0 };
			int lateralOffset{ 0 };
		};
		// HAD_LG_MARK_REL
		struct DbLgMarkRel
		{
			int lgMarkSeqNum{ 0 };
			int lgMarkDirect{ 0 };
		};

		// HAD_LANE_MARK_REL
		struct DbLaneMarkRel
		{
			int64 relLaneLinkId{ 0 };
			int side{ 0 };
			int direct{ 0 };
		};

		int64	startLaneMarkNodeId{ 0 };
		int64	endLaneMarkNodeId{ 0 };
		int	boundaryType{ 0 };
		int markingCount{ 0 };
		LineString3d geometry;
		// HAD_LANE_MARK_TRAVERSAL
		int traversal{ 0 };

		// 一个车道边界可能有多个marking
		std::vector<DbLaneMarkLinkMarking> markings;

		// 一个车道边界可能由多个车道组共用
		std::map<int64, DbLgMarkRel> relLgs;

		//// HAD_LANE_MARK_REL
		/////@warning 1条车道边界线应对应1-2条车道中心线，DbLaneMarkLink::relLaneLinkId中缺失了部分。小心使用！
		//int64 relLaneLinkId{ 0 };
		//int side{ 0 };
		//int direct{ 0 };

		std::vector<DbLaneMarkRel> laneMarkRels;
	};

	class DbLaneMarkNode : public DbRecord
	{
	public:
		MapPoint3D64 geometry;
	};

	class DbLaneMarkPA : public DbRecord, public DbPAField
	{
	public:
		int64 relLaneMarkLinkId{ 0 };
		std::vector<DbPAValue*> paValues;
	};

    class DbLaneLinkTurnwaiting : public DbRecord, public DbPAField
    {
    };

// 	class DbObject : public DbRecord
// 	{
// 	public:
// 		int64 groupId;
// 		int64 relLgLinkId;
// 		int64 relLaneLinkId;
// 		int64 relObjectId;
// 	};

	enum class DbObjectType
	{
		CURB = 1,
		BARRIER = 2,
		TUNNEL = 4,
		TRAFFIC_SIGN = 5,
		MESSAGE_SIGN = 6,
		DELINEATOR = 7,
		WALL = 13,
		ARROW = 14,
		TEXT = 15,
		SYMBOL = 16,
		WARNING_AREA = 17,
		FILL_AREA = 18,
		TRAFFIC_LIGHTS = 19,
		POLE = 22,
		WALL_P = 23,
		OH_STRUCT = 24,
		STOPLOCATION = 25,
		CROSS_WALK = 26,
		BUS_STOP = 27,
		SPEED_BUMP = 28,
		CROSS_BIKE = 29,
		PIER = 30,
		SIDE_WALK = 31,
		NOPARKING = 32
	};

	class DbBarrier : public DbRecord
	{
	public:
		int barrierType{ 0 };
		MultiLineString3d geometry;
	};

	class DbWall : public DbRecord
	{
	public:
		int wallType{ 0 };
		MultiLineString3d geometry;
	};

	class DbFillArea : public DbRecord
	{
	public:
		Polygon3d geometry;
	};

	class DbCrossWalk : public DbRecord
	{
	public:
		int color{ 0 };

		Polygon3d geometry;
	};

	class DbTrafficSign : public DbRecord
	{
	public:
		int trafSignShape{ 0 };
		int signType{ 0 };
		int color{ 0 };
		double heading{ 0 };
		Polygon3d geometry;
	};

	class DbArrow : public DbRecord
	{
	public:
		MapPoint3D64 center;
		int length{ 0 };
		int width{ 0 };
		int color{ 0 };
		int arrowClass{ 0 };
		Polygon3d geometry;
	};
	class DbStopLocation : public DbRecord
	{
	public:
		int width{ 0 };
		int color{ 0 };
		int locationType{ 0 };
		LineString3d geometry;
	};
	class DbLgRel : public DbRecord
	{
	public:
		struct DbRelLg
		{
			int64 relLgId{ 0 };
			int objectType{ 0 };
		};
		std::map<int64, DbRelLg> relLgs;
	};
	class DbLaneLinkRel : public DbRecord
	{
	public:
		struct DbRelLaneLInk
		{
			int64 relLaneLinkId{ 0 };
			int objectType{ 0 };
		};
		std::map<int64, DbRelLaneLInk> relLaneLinkIds;
	};

	class DbText : public DbRecord
	{
	public:
		MapPoint3D64 center;
		int length{ 0 };
		int width{ 0 };
		int color{ 0 };
		Polygon3d geometry;
		//std::string	textContent;
		std::wstring 	textContent{L""};
	};

	/** 限速值的来源 */
	enum class DbSpeedLimitSource
	{
		DbSpeedLimitSource_none = 0,
		DbSpeedLimitSource_real = 1,        ///< 真实限速牌
		DbSpeedLimitSource_theoretical = 2  ///< 理论限速值
	};

	/** 最高限速等级*/
	enum class MaxSpeedLimitClass
	{
		MaxSpeedLimitClass_none = 0,    // 无记录
		MaxSpeedLimitClass_first = 1,   // > 120
		MaxSpeedLimitClass_second = 2,  // (100,120]
		MaxSpeedLimitClass_third = 3,   // (90,100]
		MaxSpeedLimitClass_fourth = 4,  // (70,90]
		MaxSpeedLimitClass_fifth = 5,	// (50,70]
		MaxSpeedLimitClass_sixth = 6,	// (30,50]
		MaxSpeedLimitClass_seventh = 7,	// (10,30]
		MaxSpeedLimitClass_eighth = 8,	// (0,10]
	};

	class DbFixedSpeedLimit : public DbRecord
	{
	public:
		int maxSpeedLimit{ 0 };                        // 速度上限，单位：km/h
		DbSpeedLimitSource maxSpeedLimitSource{ DbSpeedLimitSource::DbSpeedLimitSource_none };   // 速度上线的来源
		int minSpeedLimit{ 0 };                        // 速度下限，单位：km/h
		DbSpeedLimitSource minSpeedLimitSource{ DbSpeedLimitSource::DbSpeedLimitSource_none };   // 速度下限的来源
	};

	class DbLinkSpeedLimit : public DbFixedSpeedLimit, public DbPAField
	{
	public:
		int64 relLinkId{ 0 };
		int direction{ 0 };                            // 2 顺向， 3 逆向
		bool isLaneDependent{ 0 };                     // 是否车道依赖,0否，1是
		MaxSpeedLimitClass maxSpeedLimitClass{ MaxSpeedLimitClass::MaxSpeedLimitClass_none };    // 最高限速等级
	};

	class DbIntersection : public DbRecord
	{
	public:
		struct OutLink : public DbPAField
		{
			int64 linkId{ 0 };
			int direction{ 0 };    //2 顺方向, 3 逆方向
			int isCenterLink{ 0 }; //0 否, 1 是
		};

		struct OutlinePA 
		{
			int64 linkId{ 0 };
			double offset{ 0.0 };
			MapPoint3D64 position;
		};
	public:
		int intersectionType{ 0 };       // 0 简单路口，1 复合路口HAD_INTERSECTION,V1.6中是单独的表
		std::vector<int64> refLaneGroups;// 1:N 关联的车道组 HAD_INTERSECTION_LG
		std::vector<int64> refLanePAs;// 1:N HAD_INTERSECIOTN_LANEPA
		std::vector<int32> refMeshs;// 路口覆盖的所有tile
		std::vector<int64> refNodes;// 1:N,路口组成NODE(HAD_NODE), HAD_INTERSECTION_NODE
		std::vector<int64> refPoints;// 1:N, 路口边界点PA,关联HAD_LINK_PA, HAD_INTERSECTION_POINT V1.5

		std::vector<OutLink> outLinks;// 1:N, 路口组成LINK或LINKPA, HAD_INTERSECTION_LINK
		std::vector<OutlinePA> outLines;// 1:N, 路口边界点PA,关联HAD_LINK_PA, HAD_INTERSECTION_POINT V1.6
	};

	/** 收费站区域内所有道路的收费方式 */
	class DbTollLane : public DbRecord
	{
	public:
		int64	laneLinkPid{ 0 };		///< 车道中心线ID
		int32	seqNum{ 0 };			///< 车道中心线序号
		int32	cardType{ 0 };			///< 领卡类型（实际使用3bit位）。类型参见《OMDB交换规格书》HAD_TOLL_LANE表。
		int32	payMethod{ 0 };			///< 收费方式（实际使用9bit位）。类型参见《OMDB交换规格书》HAD_TOLL_LANE表。
	};

	/** 收费站 */
	class DbTollGate : public DbRecord
	{
	public:
		int64 paPid{ 0 };			///< HAD_LINK_PA外键，即：DbLinkPA.uuid。
		int32 tollType{ 0 };			///< 收费类型。类型参见《OMDB交换规格书》HAD_TOLLGATE表。
		std::wstring tollName{ L"" };       ///< 收费站名称
		int64 linkPid{ 0 };
		double offset{ 0.0 };
		MapPoint3D64 geometry;
		std::vector<DbTollLane*> tollLanes;
	};

	/** 收费站区域内所有通道(中间结构,不需要序列化) */
	class DbTollGatePassage : public DbRecord
	{
	public:
		std::vector<DbTollLane*> tollLanes;
	};

	/** 交通信号灯 */
	class DbTrafficLights : public DbRecord
	{
	public:
		int32 type{0};				//	1 路口交通流控制信号灯;2 匝道控制信号灯;3 收费站信号灯;4 车道状态信号灯;5 计时器;6 行人灯;7 非机动车灯
		int32 orientation;			//	1 垂直; 2 水平
		int32 rowNumber{0};			//	层数,灯箱数	
		int32 columnNumber{ 0 };	//	灯箱的灯泡数
		double heading{ 0.0 };		//	朝向
		Polygon3d geometry;			//	几何
	};

	/** 杆状物 */
	class DbPole: public DbRecord
	{
	public:
		int32 type{ 0 };			//	1 其他;2 树干
		int diameterTop{ 0 };		//	顶面直径 mm
		int	diameterBottom{ 0 };	//	底面直径	mm
		LineString3d geometry;			//	几何
	};



}

