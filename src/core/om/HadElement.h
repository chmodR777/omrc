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
		NO = 0,				//��
		SOLID = 1,			//��̻���
		NON_SOLID = 2,		//�Ǽ�̻���
		PA_REFERENCE = -99	//PA�ο�
	};

    enum class Color : int
    {
		UNKONWN = 0,
		WHITE = 1,
		YELLOW = 2,
		OTHER = 9,
    };

	/** ����ֵ����Դ */
	enum class FixedSpeedLimitSource
	{
		FixedSpeedLimitSource_none = 0,
		FixedSpeedLimitSource_real = 1,        ///< ��ʵ������
		FixedSpeedLimitSource_theoretical = 2  ///< ��������ֵ
	};

	struct FixedSpeedLimit
	{
		int maxSpeedLimit;                        // �ٶ����ޣ���λ��km/h
		FixedSpeedLimitSource maxSpeedLimitSource;   // �ٶ����ߵ���Դ
		int minSpeedLimit;                        // �ٶ����ޣ���λ��km/h
		FixedSpeedLimitSource minSpeedLimitSource;   // �ٶ����޵���Դ
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
		// һ��object�������ڶ��������
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
		// ����
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

		// ������Ϣ
		std::vector<double> distances;

		// �㷽����Ϣ��current -> next
		std::vector<Vector2> directions;

		std::vector<HadLaneGroup*> groups;

		// ����
		FixedSpeedLimit speedLimit;

		// �Ƿ������
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
			// ��LINK�����Ҳ࣬1 - �Ҳ�, 2 - ���
			int side;

			// �Ƿ���LINKͬ����,2 - ͬ��, 3 - ����
			int direction;
		};

		BoundaryType boundaryType;
		HadRoadBoundaryNode* startNode;
		HadRoadBoundaryNode* endNode;

		// �Ƿ�����β���
		bool danglingEndNode{ true };

		// һ����·�߽�����������ٽӵĳ����鹲��
		std::map<int64, HadLaneGroup*> linkGroups;
		std::map<int64, HADLgRoadBoundREL> relLgs;

		// �����·�߽���Ļ���/ǿ����
		std::vector<HadObject*> barrierObjects;
    };

	class HadLaneGroup;
	class HadLaneTurnwaiting;
    class HadLaneBoundaryNode : public HadElement
    {
	public:
        MapPoint3D64 position;
    };

	/// @brief �����߽���
	/// @node 
	///		��ԭʼ����HAD_LANE_LINK_LG���п�֪������������ͨ�з���=���������߻��߷���=������ͨ�з���
	///		��HAD_LG_MARK_REL���֪��������ͨ�з��򲢲�һ�����ڳ����߽��߻��߷���
	///		��Ϊ�漰startNode��endNode�����ܵ�һЩ���˺ͷ�����ص����ԣ�Ŀǰû���ڱ��ṹ������˳�����߽��ߵ���
	/// Ҳ����˵�������ϳ����߽��ߵĵ����п��ܺͳ����߻��߷����෴��
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
		// ��ӦDB_LG_MARK_REL
		struct HadLgMarkRel
		{
			int lgMarkSeqNum;
			int lgMarkDirect;
		};

		std::vector<HadLaneBoundaryMarking> markings;
		std::map<int64, short> seqNumberOnLGs;
		short seqNumberOnLG;	//λ�����ڳ������ϵ���ţ���������������Ϊ1��
		// ���ڳ����߽�����������ٽӵĳ����鹲��,��˸��ֶ��ڸ������鶼��һ��,��Ϊ����relLgs�ֶ�
		// short lgMarkDirect;	//���߷��������ڳ�����ͨ�з���Ĺ�ϵ��2ͬ��3����
        BoundaryType boundaryType;
		HadLaneBoundaryNode* startNode;
		HadLaneBoundaryNode* endNode;

		// һ�������߽�����������ٽӵĳ����鹲��,���������ʱ���ڳ�����Ϊ�յ����
		std::map<int64, HadLaneGroup*> linkGroups;
		std::map<int64, HadLgMarkRel> relLgs;

		// �Ƿ�Ϊ���˵�Overlapping�߽�
		bool filterOverlappingBoundary{ false };

		//��ת��PA
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
			LaneType_regular                 = 1 << 0,    // ���泵��
			LaneType_auxiliary               = 1 << 1,    // ���ϳ���
			LaneType_accelerate              = 1 << 2,    // ���ٳ���
			LaneType_decelerate              = 1 << 3,    // ���ٳ���
			LaneType_hov                     = 1 << 4,    // ���س���
			LaneType_slow                    = 1 << 6,    // ������
			LaneType_shoulder                = 1 << 8,    // ·�糵��
			LaneType_regulated_access        = 1 << 10,   // ���Ƴ���
			LaneType_drivable_parking        = 1 << 13,   // ����ʻͣ����
			LaneType_drivable_shoulder       = 1 << 14,   // ����ʻ·�糵��
			LaneType_emergency_parking_strip = 1 << 17,   // ����ͣ����
			LaneType_bus                     = 1 << 18,   // ��������
			LaneType_bicycle                 = 1 << 19,   // ���г���
			LaneType_turn                    = 1 << 20,   // ת�򳵵�
			LaneType_tidal                   = 1 << 21,   // ��ϫ����
			LaneType_hedge                   = 1 << 23,   // ���ճ���
			LaneType_variable_driving        = 1 << 24,   // �ɱ䳵��
			LaneType_parking                 = 1 << 25,   // ͣ������
			LaneType_other                   = 1 << 26,   // ����
			LaneType_turn_left               = 1 << 27,   // �����ת
		};
	public:
		uint32 laneType;
		int isIntersection;
		std::string	arrowDir;
		int width;	//����ƽ����ȣ���λ�����ף�

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

		// ����
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
	// ������ͨ�з�����LINK���߷����ϵ
	// һ��������������ڶ���LINK,��ͬ��LINK������ܲ�ͬ,��������ķ�����ȡLINK��Ӧ�εķ���
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

		int width = 0;	//���������ƽ����ȣ���λ�����ף�

		std::map<HadLink*, HadRelLink> relLinks;

		std::vector<HadLane*>			lanes;				//���ڳ������ڴ����ҵ�˳������
        std::vector<HadLaneBoundary*>	laneBoundaries;		//���ڳ������ڴ����ҵ�˳������
		std::vector<HadObject*>			objects;
		std::vector<HadRoadBoundary*>	roadBoundaries;
		std::vector<HadLaneGroupAssociation*> associations;

		// AABB,��Χ�У����ڿռ䷶Χ��������ʹ�õ�·�߽�����Χ��
		BoundingBox3d extent;

		int maxSpeedLimit;                        // �ٶ����ޣ���λ��km/h
		int minSpeedLimit;                        // �ٶ����ޣ���λ��km/h

		// �Ƿ������
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
			BarrierType_Barrier,			//����
			BarrierType_JerseyBarrier,		//����������
			BarrierType_Guardrail,			//��ȫ����
			BarrierType_Fence,				//Χ��
			BarrierType_OtherBarrier		//��������
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

			// �Ƿ���Ӧ�ù���������
			bool applyDiversion{ true };

			// ��ǰLA��ϵѹ�ǵĵ�����
			std::vector<HadFillArea*> overedFillAreas;

			// ���������߽��ƽ������
			double avgDistance{ 0 };
		};
		// һ��������������ܱ������������γ�����LA��ϵ,��ʱgroup_id������ͬ
		std::vector<HadRelLaneGroupAssociation> relLaneGroupAssociations;
	};

	class HadFillArea : public HadRoadsideObject
	{
	public:
		Polygon3d polygon;

		// �Ƿ����ǵ�
		bool isGore{ false };

		// �Ƿ���Ӧ�ù���������
		bool applyDiversion{ false };

		// AABB,��Χ�У����ڿռ䷶Χ��������ʹ�õ������߽����
		BoundingBox2d extent;

		// ��ǰ���������ںϵĵ�����
		HadFillArea* mergedFillArea = nullptr;

		// ѹ�ǵ�ǰ��������LA��ϵ
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

		//��ö��ֵ����ֵ�����鱣��һ�¡�
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
			int direction;    //2 ˳����, 3 �淽��
			int isCenterLink; //0 ��, 1 ��
		};

		struct OutlinePA
		{
			int64 linkId;
			double offset;
			MapPoint3D64 position;
		};
	public:
		int intersectionType;       // 0 ��·�ڣ�1 ����·��
		std::vector<int64> refLanePAs;
		//std::vector<int32> refMeshs;//·�ڸ��ǵ�����tile
		//std::vector<int64> refNodes;
		//std::vector<int64> refPoints;
		//std::vector<OutLink> outLinks;
		//std::vector<OutlinePA> outLines;

		std::vector<HadGrid*> refMeshs;//·�ڸ��ǵ�����tile
		std::vector<HadLaneGroup*> refLaneGroups;
		std::vector<HadLink*> refOutLinks;
		std::vector<HadPartAttribute*> refLinkPAs;
		std::vector<HadNode*> refNodes;
		std::vector<HadPartAttribute*> refPointPAs;

		// ����ֱ��·�ڱ�ʶ,ֻ����һ��
		bool doStraightIntersection{ true };
		bool isUTurn{ false };
		bool isRoundabout{ false };
	};

	/** �շ�վ */
	class HadTollGate : public HadElement
	{
	public:
		//�����շ����͡���ӦԴ�������쿨�������շѷ�ʽ�����ֶΡ�
		enum class LanePayType
		{
			eUnkonwn	= 0x00,		///< ֵΪ0����ʾδӦ�á�
			eETC		= 0x01,		///< 1bit:ETC
			eManual		= 0x02,		///< 2bit:�˹�
			eSelf		= 0x04		///< 3bit:����
		};

		/** �շ�վ���������е�·���շѷ�ʽ */
		struct TollLane
		{
			int64	laneLinkPid;		///< ����������ID
			int32	seqNum;				///< �������������
			//int32	cardType;			///< �쿨���ͣ�ʵ��ʹ��3bitλ�������Ͳμ���OMDB��������顷HAD_TOLL_LANE��.
			LanePayType	payType;
			MapPoint3D64	tollPoint; ///< �������շ�վ�ļ���λ��
		};
		using PtrTollLane = std::shared_ptr<TollLane>;

	public:
		int32 tollType;						///< �շ����͡����Ͳμ���OMDB��������顷HAD_TOLLGATE��
		std::wstring tollName{ L"" };    	///< �շ�վ��������
		MapPoint3D64 position;				///< �շ�վλ�ã�����λ��λ��link�ϣ���
		std::vector<PtrTollLane> tollLanes;	///< ���ڳ�������Ϣ��
	};

	class HadTrafficLights : public HadObject
	{
	public:
		uint8 type{ 0 };				//	1 ·�ڽ�ͨ�������źŵ�;2 �ѵ������źŵ�;3 �շ�վ�źŵ�;4 ����״̬�źŵ�;5 ��ʱ��;6 ���˵�;7 �ǻ�������
		uint8 orientation;				//	1 ��ֱ; 2 ˮƽ
		uint8 rowNumber{ 0 };			//	����,������	
		uint8 columnNumber{ 0 };		//	����ĵ�����
		double heading{ 0.0 };			//	���� ����Ϊ0
		Polygon3d location;				//	����
	};

	class HadPole : public HadObject
	{
	public:
		uint8 type{ 0 };				//	1 ����;2 ����
		LineString3d location;				//	����
	};

	class HadSpeedBump : public HadObject
	{
	public:
		double heading{ 0.0 };
		Polygon3d polygon;
	};

}