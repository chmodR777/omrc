#pragma once
#include <vector>
#include <string>
#include <map>
#include "../Geometry.h"
#include "math3d/vector2.h"
#include "math3d/vector3.h"
namespace OMDB
{
	enum class ElementType
	{
		UNKONWN,
		HAD_GRID,
		HAD_LINK,
		HAD_NODE,
		HAD_ROAD_BOUNDARY,
		HAD_ROAD_BOUNDARY_NODE,
		HAD_LANE,
		HAD_LANE_NODE,
        HAD_LANE_BOUNDARY,
        HAD_LANE_BOUNDARY_NODE,
		HAD_LANE_GROUP,
		HAD_LG_ASSOCIATIOIN,
		HAD_PART_ATTRIBUTE,
        HAD_OBJECT_CROSS_WALK,
        HAD_OBJECT_STOPLOCATION,
        HAD_OBJECT_TRAFFIC_LIGHTS,
        HAD_OBJECT_FILL_AREA,
        HAD_OBJECT_SYMBOL,
        HAD_OBJECT_TEXT,
        HAD_OBJECT_ARROW,
        HAD_OBJECT_WALL, 
        HAD_OBJECT_TRAFFIC_SIGN,
        HAD_OBJECT_BARRIER,
		HAD_INTERSECTION,
		HAD_TOLLGATE,
		HAD_OBJECT_POLE,
		HAD_LANE_TURNWAITING,
		HAD_OBJECT_SPEED_BUMP
	
	};
	enum class BoundaryType : int
	{
		NOT_APPLY = 0,
		NONE = 1,
		MARKING = 2,
		CURB = 3,
		GURADRAIL = 4,
		WALL = 5,
		EDGE = 6,
		GORE = 7,
		BARRIER = 8,
		PA_REFERENCE = -99
	};

	enum class LinkSeparationType : int
	{
		NO = 0,				//无
		SOLID = 1,			//坚固护栏
		NON_SOLID = 2,		//非坚固护栏
		PA_REFERENCE = -99	//PA参考
	};

    enum class Color : int
    {
		UNKONWN = 0,
		WHITE = 1,
		YELLOW = 2,
		OTHER = 9,
    };

	/** 限速值的来源 */
	enum class FixedSpeedLimitSource
	{
		FixedSpeedLimitSource_none = 0,
		FixedSpeedLimitSource_real = 1,        ///< 真实限速牌
		FixedSpeedLimitSource_theoretical = 2  ///< 理论限速值
	};

	struct FixedSpeedLimit
	{
		int maxSpeedLimit;                        // 速度上限，单位：km/h
		FixedSpeedLimitSource maxSpeedLimitSource;   // 速度上线的来源
		int minSpeedLimit;                        // 速度下限，单位：km/h
		FixedSpeedLimitSource minSpeedLimitSource;   // 速度下限的来源
	};


	typedef long long int64;
	class HadGrid;
	class HadElement
	{
	public:
		int64				originId;
		ElementType          objectType;
		HadGrid* owner = nullptr;
	};

	class HadLaneGroup;
	class HadLane;
    class HadObject : public HadElement
    {
    public:
		// 一个object可能属于多个车道组
		std::vector<HadLaneGroup*> laneGroups;
		std::vector<HadLane*> refLanes;
    };

    class HadPartAttribute : public HadElement
    {
    public:
        double start;
        double end;
        int name;
        int value;
		int seqNum{-1};
		MultiPoint3d points;
		// 限速
		FixedSpeedLimit speedLimit;
    };

	class HadSkeleton : public HadElement
	{
	public:
		std::vector<HadSkeleton*>    previous;
		std::vector<HadSkeleton*>    next;
		std::vector<HadPartAttribute*>   attributes;
		LineString3d             location;
	};

	// link
	class HadLink;
    class HadNode : public HadElement
    {
    public:
        MapPoint3D64		position;
		int64         intersectionId;
		int                 wayType;
		std::vector<int32>	meshIds;
		std::vector<HadLink*> links;
    };

	class HadLaneGroup;
    class HadLink : public HadSkeleton
    {
    public:

		struct HadLinkDataLevel
		{
			int64 linkId{ 0 };
			int dataLevel{ 0 };
			int featureType{ 0 };
			double startOffset{ 0.0 };
			double endOffset{ 0.0 };
			MultiPoint3d geometry;
		};

        HadNode* startNode;
        HadNode* endNode;
		
		int multi_digitized;
		int direct;
		int separation_left;
		int separation_right;
		int median_left;
		int median_right;
		int overhead_obstruction;
		std::vector<int> wayTypes;

		// 点间距信息
		std::vector<double> distances;

		// 点方向信息，current -> next
		std::vector<Vector2> directions;

		std::vector<HadLaneGroup*> groups;

		// 限速
		FixedSpeedLimit speedLimit;

		// 是否跨网格
		bool crossGrid { false };

		//data level
		std::vector<HadLinkDataLevel> dataLevels;
    };

	class HadRoadBoundaryNode : public HadElement
	{
	public:
		MapPoint3D64 position;
	};
    class HadRoadBoundary : public HadSkeleton
    {
	public:
		//HAD_LG_RDBOUND_REL
		struct HADLgRoadBoundREL
		{
			// 在LINK的左右侧，1 - 右侧, 2 - 左侧
			int side;

			// 是否与LINK同方向,2 - 同向, 3 - 反向
			int direction;
		};

		BoundaryType boundaryType;
		HadRoadBoundaryNode* startNode;
		HadRoadBoundaryNode* endNode;

		// 是否悬空尾结点
		bool danglingEndNode{ true };

		// 一个道路边界可能由两个临接的车道组共用
		std::map<int64, HadLaneGroup*> linkGroups;
		std::map<int64, HADLgRoadBoundREL> relLgs;

		// 保存道路边界横跨的护栏/强对象
		std::vector<HadObject*> barrierObjects;
    };

	class HadLaneGroup;
	class HadLaneTurnwaiting;
    class HadLaneBoundaryNode : public HadElement
    {
	public:
        MapPoint3D64 position;
    };

	/// @brief 车道边界线
	/// @node 
	///		从原始数据HAD_LANE_LINK_LG表中可知：车道中心线通行方向=车道中心线画线方向=车道组通行方向。
	///		从HAD_LG_MARK_REL表可知，车道组通行方向并不一定等于车道边界线画线方向。
	///		因为涉及startNode、endNode及可能的一些拓扑和方向相关的属性，目前没有在本结构体中捋顺车道边界线点序。
	/// 也就是说，理论上车道边界线的点序有可能和车道线画线方向相反。
	class HadLaneBoundary : public HadSkeleton
	{
    public:
		struct HadLaneBoundaryMarking
		{
			int markSeqNum;
			int markType;
			int markColor;
			int markWidth;
			int markMaterial;
			int lateralOffset;
		};
		// 对应DB_LG_MARK_REL
		struct HadLgMarkRel
		{
			int lgMarkSeqNum;
			int lgMarkDirect;
		};

		std::vector<HadLaneBoundaryMarking> markings;
		std::map<int64, short> seqNumberOnLGs;
		short seqNumberOnLG;	//位于所在车道面上的序号（从左到右排序，最左为1）
		// 由于车道边界可能由两个临接的车道组共用,因此该字段在各车道组都不一样,改为存在relLgs字段
		// short lgMarkDirect;	//画线方向与所在车道组通行方向的关系：2同向；3反向。
        BoundaryType boundaryType;
		HadLaneBoundaryNode* startNode;
		HadLaneBoundaryNode* endNode;

		// 一个车道边界可能由两个临接的车道组共用,另外跨网格时存在车道组为空的情况
		std::map<int64, HadLaneGroup*> linkGroups;
		std::map<int64, HadLgMarkRel> relLgs;

		// 是否为过滤的Overlapping边界
		bool filterOverlappingBoundary{ false };

		//待转线PA
		std::vector<HadLaneTurnwaiting*> turnwaitingPAs;


	public:
		forceinline short getSeqNumberOnLG(int64 lgId) const {
			return seqNumberOnLGs.count(lgId) ? seqNumberOnLGs.find(lgId)->second : seqNumberOnLG;
		}
	};


	enum class HadLaneBoundaryPAType
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


	class HadLaneNode : public HadElement
	{
	public:
        HadLaneNode* startNode;
        HadLaneNode* endNode;
        MapPoint3D64 position;
	};

	class HadText;
	
	class HadLane : public HadSkeleton
	{
	public:
		enum class LaneType : uint32
		{
			LaneType_regular                 = 1 << 0,    // 常规车道
			LaneType_auxiliary               = 1 << 1,    // 复合车道
			LaneType_accelerate              = 1 << 2,    // 加速车道
			LaneType_decelerate              = 1 << 3,    // 减速车道
			LaneType_hov                     = 1 << 4,    // 满载车道
			LaneType_slow                    = 1 << 6,    // 慢车道
			LaneType_shoulder                = 1 << 8,    // 路肩车道
			LaneType_regulated_access        = 1 << 10,   // 管制车道
			LaneType_drivable_parking        = 1 << 13,   // 可行驶停车道
			LaneType_drivable_shoulder       = 1 << 14,   // 可行驶路肩车道
			LaneType_emergency_parking_strip = 1 << 17,   // 紧急停车带
			LaneType_bus                     = 1 << 18,   // 公交车道
			LaneType_bicycle                 = 1 << 19,   // 自行车道
			LaneType_turn                    = 1 << 20,   // 转向车道
			LaneType_tidal                   = 1 << 21,   // 潮汐车道
			LaneType_hedge                   = 1 << 23,   // 避险车道
			LaneType_variable_driving        = 1 << 24,   // 可变车道
			LaneType_parking                 = 1 << 25,   // 停车车道
			LaneType_other                   = 1 << 26,   // 其它
			LaneType_turn_left               = 1 << 27,   // 借道左转
		};
	public:
		uint32 laneType;
		int isIntersection;
		std::string	arrowDir;
		int width;	//车道平均宽度（单位：厘米）

		// HAD_LANE_LINK_LG
		HadLaneGroup* linkGroup = nullptr;
		int laneNumber;
		HadLaneNode* startNode;
		HadLaneNode* endNode;
		int seqNumber;
		int laneDirection;

		// HAD_LANE_LINK_CONDITION
		int conditionType;

		HadLaneBoundary* leftBoundary;
		HadLaneBoundary* rightBoundary;

		std::vector<HadObject*> objects;
		HadLaneTurnwaiting* laneTurnwaiting{nullptr};

		// 限速
		FixedSpeedLimit speedLimit;
		std::vector<HadText*> speedLimitTexts;
		std::vector<HadText*> busAndHovTexts;

	public:
		forceinline bool isDesignedLaneType(LaneType type) { return (laneType & (uint32)type); };
	};


	class HadLaneTurnwaiting:public HadElement
	{
	public:
		double startOffset;
		double endOffset;
		
		MultiPoint3d positions;

	};


	class HadBarrier;
	class HadLaneGroupAssociation;
	// 车道组通行方向与LINK划线方向关系
	// 一个车道组可能属于多条LINK,不同的LINK方向可能不同,即车道组的方向需取LINK对应段的方向
	class HadLaneGroup : public HadSkeleton
	{
	public:
		struct HadRelLink
		{
			HadLink* link;
			double							start;
			double							end;
			int							    direct;
		};

		int width = 0;	//车道组面的平均宽度（单位：厘米）

		std::map<HadLink*, HadRelLink> relLinks;

		std::vector<HadLane*>			lanes;				//按在车道组内从左到右的顺序排序
        std::vector<HadLaneBoundary*>	laneBoundaries;		//按在车道组内从左到右的顺序排序
		std::vector<HadObject*>			objects;
		std::vector<HadRoadBoundary*>	roadBoundaries;
		std::vector<HadLaneGroupAssociation*> associations;

		// AABB,包围盒，用于空间范围的搜索，使用道路边界计算包围盒
		BoundingBox3d extent;

		int maxSpeedLimit;                        // 速度上限，单位：km/h
		int minSpeedLimit;                        // 速度下限，单位：km/h

		// 是否跨网格
		bool crossGrid { false };
		int64 inIntersection { 0 };
		bool isGenerated{ false };
	};


	
	class HadRoadsideObject : public HadObject
	{

	};

	class HadOverheadObject : public HadObject
	{
    public:
        Polygon3d polygon;
	};

	class HadRoadSurfaceObject : public HadObject
	{
    public:
        Polygon3d polygon;
	};

	class HadBarrier : public HadRoadsideObject
	{
	public:
		enum class BarrierType
		{
			BarrierType_Barrier,			//护栏
			BarrierType_JerseyBarrier,		//新泽西护栏
			BarrierType_Guardrail,			//安全护栏
			BarrierType_Fence,				//围栏
			BarrierType_OtherBarrier		//其他护栏
		};
	public:
		MultiLineString3d location;
		BarrierType barrierType;
	};

	class HadFillArea;
	class HadLaneGroupAssociation : public HadElement
	{
	public:
		struct HadRelLaneGroupAssociation
		{
			int64 groupId = 0;
			int directType;
			HadLaneGroup* first;
			HadLaneGroup* second;

			HadLaneBoundary* firstLaneBoundary;
			HadLaneBoundary* secondLaneBoundary;

			// 是否已应用构建导流带
			bool applyDiversion{ true };

			// 当前LA关系压盖的导流区
			std::vector<HadFillArea*> overedFillAreas;

			// 两条车道边界的平均距离
			double avgDistance{ 0 };
		};
		// 一个车道组可能与周边两个车道组形成两个LA关系,此时group_id可能相同
		std::vector<HadRelLaneGroupAssociation> relLaneGroupAssociations;
	};

	class HadFillArea : public HadRoadsideObject
	{
	public:
		Polygon3d polygon;

		// 是否三角岛
		bool isGore{ false };

		// 是否已应用构建导流带
		bool applyDiversion{ false };

		// AABB,包围盒，用于空间范围的搜索，使用导流带边界计算
		BoundingBox2d extent;

		// 当前导流区被融合的导流区
		HadFillArea* mergedFillArea = nullptr;

		// 压盖当前导流区的LA关系
		std::vector<HadLaneGroupAssociation::HadRelLaneGroupAssociation*> overedLgAssociations;
	};

	class HadOnStruct : public HadOverheadObject
	{
	public:
		Polygon3d polygon;
	};

	class HadCrossWalk : public HadObject
	{
	public:
		Color color;
		Polygon3d polygon;
	};

	class HadTrafficSign : public HadObject
	{
	public:
		enum class ShapeType
		{
			IRREGULAR,
			RECTANGLE,
			TRIANGLE,
			CIRCLE,
			RHOMBUS,
			INVERTED_TRIANGLE,
			SQUARE,
			OCTAGON
		};

		//此枚举值的数值与规格书保持一致。
		enum class SignType
		{
			OTHERS						= 1,
			MAX_SPEED_LIMIT_VALUE		= 2,
			MIN_SPEED_LIMIT_VALUE		= 3,
			END_SPEED_LIMIT_VALUE		= 4,
			ADVISORY_SPEED_LIMIT_VALUE	= 5,
		};

		SignType signType;
		ShapeType shapeType;
		Color color;
		//std::string content;
		int32 speed;
		Polygon3d polygon;
		MapPoint3D64 centerPt;
		MapPoint3D64 grapPt;

	};

	class HadStopLocation : public HadObject
	{
	public:
		unsigned short width;
		unsigned short color;
		unsigned short locationType;
		LineString3d location;
	};
    class HadMessageSign : public HadOverheadObject
    {
    public:
        enum class SignType
        {

        };

        double heading;
		SignType signType;
    };

	
	class HadWall : public HadObject
	{
	public:
		enum class WallType
		{
			TUNNEL,
			OTHER
		};
	public:
		MultiLineString3d location;
		WallType wallType;
	};

    class HadSymbol : public HadRoadSurfaceObject
    {
    public:
        MapPoint3D64 postion;
        double length;
        double width;
        Color color;
    };
	class HadArrow :public HadSymbol
	{
	public:

		enum class ArrowType
		{
			UNKNOWN = 0,
			STRAIGHT = 1,
			RIGHT = 2,
			STRAIGHT_OR_RIGHT = 3,
			LEFT = 4,
			STRAIGHT_OR_LEFT = 5,
			LEFT_OR_RIGHT = 6,
			LEFT_OR_STRAIGHT_OR_RIGHT = 7,
			U_TURN = 8,
			STRAIGHT_OR_U_TURN = 9,
			U_TURN_OR_RIGHT = 10,
			U_TURN_OR_STRAIGHT_OR_RIGHT = 11,
			U_TURN_OR_LEFT = 12,
			U_TURN_OR_LEFT_OR_STRAIGHT = 13,
			U_TURN_OR_LEFT_OR_RIGHT = 14,
			LEFT_MIX = 16,
			RIGHT_MIX = 32,
			N_TURN = 64,
			N_TURN_OR_STRAIGHT = 65,
			LEFT_FRONT = 128,
			RIGHT_FRONT = 258,
		};

		ArrowType arrowClass;
		int direction;
	};
	class HadText : public HadSymbol
	{
    public:

		enum class SpeedLimitTextType
		{
			UNKNOWN = 0,
			minSpeed = 1,
			maxSpeed = 2,
			singleSpeed = 3,
			totalSpeed = 4,
			rangeSymbol = 5,
		};

		//std::string content;
		std::wstring content;

		//impl
		HadText* speedLimitRangeText;
		HadText* nextText;
		HadText* previousText;
		SpeedLimitTextType speedLimitType;
		bool isTimeText{false};

	};
	class HadWarningArea : public HadRoadSurfaceObject
	{
	public:
		enum class MaterialType
		{

		};

	public:
		MaterialType material;
		Color color;
	};

	class HadIntersection : public HadElement
	{
	public:
		struct OutLink
		{
			int64 linkId;
			int linkType; //1 link, 2 link_pa
			double sOffset;
			double eOffset;
			MultiPoint3d geometry;
			int direction;    //2 顺方向, 3 逆方向
			int isCenterLink; //0 否, 1 是
		};

		struct OutlinePA
		{
			int64 linkId;
			double offset;
			MapPoint3D64 position;
		};
	public:
		int intersectionType;       // 0 简单路口，1 复合路口
		std::vector<int64> refLanePAs;
		//std::vector<int32> refMeshs;//路口覆盖的所有tile
		//std::vector<int64> refNodes;
		//std::vector<int64> refPoints;
		//std::vector<OutLink> outLinks;
		//std::vector<OutlinePA> outLines;

		std::vector<HadGrid*> refMeshs;//路口覆盖的所有tile
		std::vector<HadLaneGroup*> refLaneGroups;
		std::vector<HadLink*> refOutLinks;
		std::vector<HadPartAttribute*> refLinkPAs;
		std::vector<HadNode*> refNodes;
		std::vector<HadPartAttribute*> refPointPAs;

		// 处理直行路口标识,只处理一次
		bool doStraightIntersection{ true };
		bool isUTurn{ false };
		bool isRoundabout{ false };
	};

	/** 收费站 */
	class HadTollGate : public HadElement
	{
	public:
		//车道收费类型。对应源数据中领卡类型与收费方式两个字段。
		enum class LanePayType
		{
			eUnkonwn	= 0x00,		///< 值为0，表示未应用。
			eETC		= 0x01,		///< 1bit:ETC
			eManual		= 0x02,		///< 2bit:人工
			eSelf		= 0x04		///< 3bit:自助
		};

		/** 收费站区域内所有道路的收费方式 */
		struct TollLane
		{
			int64	laneLinkPid;		///< 车道中心线ID
			int32	seqNum;				///< 车道中心线序号
			//int32	cardType;			///< 领卡类型（实际使用3bit位）。类型参见《OMDB交换规格书》HAD_TOLL_LANE表.
			LanePayType	payType;
			MapPoint3D64	tollPoint; ///< 编译完收费站的几何位置
		};
		using PtrTollLane = std::shared_ptr<TollLane>;

	public:
		int32 tollType;						///< 收费类型。类型参见《OMDB交换规格书》HAD_TOLLGATE表。
		std::wstring tollName{ L"" };    	///< 收费站中文名称
		MapPoint3D64 position;				///< 收费站位置（几何位置位于link上）。
		std::vector<PtrTollLane> tollLanes;	///< 所在车道线信息。
	};

	class HadTrafficLights : public HadObject
	{
	public:
		uint8 type{ 0 };				//	1 路口交通流控制信号灯;2 匝道控制信号灯;3 收费站信号灯;4 车道状态信号灯;5 计时器;6 行人灯;7 非机动车灯
		uint8 orientation;				//	1 垂直; 2 水平
		uint8 rowNumber{ 0 };			//	层数,灯箱数	
		uint8 columnNumber{ 0 };		//	灯箱的灯泡数
		double heading{ 0.0 };			//	朝向 正北为0
		Polygon3d location;				//	几何
	};

	class HadPole : public HadObject
	{
	public:
		uint8 type{ 0 };				//	1 其他;2 树干
		LineString3d location;				//	几何
	};

	class HadSpeedBump : public HadObject
	{
	public:
		double heading{ 0.0 };
		Polygon3d polygon;
	};

}