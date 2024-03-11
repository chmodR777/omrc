#pragma once

#include "cqstl/vector.h"
#include "geometry/map_point3d64.h"
#include "math3d/dvector2.h"
#include "math3d/vector2.h"
#include "math3d/vector3.h"
#include "core/Geometry.h"

#include <array>

namespace hadc
{
	// �ж�����������Ƿ������������Ӧ�ñ��ж�Ϊһ����
	forceinline bool GeometryUtil_tooCloseBetweenPoints(const MapPoint64& l, const MapPoint64& r)
	{
		return nc_abs(l.lon - r.lon) < 10 && nc_abs(l.lat - r.lat) < 10;
	}

	// �ҵ�һϵ�е� points �о��� p ����ĵ������
	int GeometryUtil_findNearestPoint(const Vector3* points, int n, Vector2 p);

	// �������ߺ�polyline�Ľ���;���
	bool GeometryUtil_calRayCutWithLine(const MapPoint3D64* points, size_t ptNum, const MapPoint3D64& p, const DVector2& dir, MapPoint3D64& rstPt, double& rstDis);

	// ����ĳ������ָ�������Polyline�ľ����Ƿ�С��ĳ��ֵ
	bool GeometryUtil_testIsPointWithinDistanceFromPolyline(const MapPoint3D64& p, const DVector2& dir,
		const MapPoint3D64* points, size_t ptNum, double testDistanceInMeter, MapPoint3D64& rstPt, double& rstDis);

	// ������Polyline�ľ��룬����ֵ��λ��cm
	int64 GeometryUtil_calcDistanceBetweenPointAndPolyline(MapPoint64 pos, const MapPoint3D64* pts, size_t ptCount);
	/**
	* @brief ͨ���������߶ȵ���ֵ
	* @param[in] lon, lat
	*		�߶ȶ�Ӧ�����꣬��λ����
	* @return
	*		���صĸ߶ȵ���ֵ����λ��cm
	* @details
	*		�߶ȵ���ֵ�ľ���ļ��㷽���ǣ�
	*		���Ƚ����꣨��Ϊ��GCJ02���꣩ת����WGS84���꣬Ȼ��������������ƫ��ֵ��
	*		Ȼ���ƫ��ֵ����һ��ϵ�������ս������Ϊ�߶���Ҫ���ϵ�ƫ��ֵ��
	*/
	int32 GeometryUtil_calcHeightAdjustmentByPosition(double lon, double lat);
	// Polyline
	struct Polyline
	{
		const MapPoint3D64* pts;
		size_t ptCount;
	};

	// polylineץȡ����
	struct PolylineGrabParams
	{
		PolylineGrabParams() { reset(); }

		void reset();

		MapPoint64 position;
		/**
		* ץ·��������Ϊ0����ʱ��Ϊ������λ���ȡ�
		* ȱʡΪ JUST_ANY_ANGLE����ʾ��������ץ·�������ʱ�Ƕ�ƫ�ȡ0��
		*/
		uint16 queryAngle;
		/**
		* ����ƫ����ϵ��
		*	deviationFactor: ����Ĵ��ϵ���� angleDeviationFactor: �ǶȵĴ��ϵ��
		*	score = deviationFactor * GrabResult.deviation + angleDeviationFactor * GrabResult.angleDeviation * 100.
		*	����ԽСԽ�á�ץ����·������ score ��С��������ȱʡֵ�� deviationFactor = 1, angleDeviationFactor = 0��
		*/
		int32 deviationFactor;
		/**
		* �Ƕ�ƫ����ϵ�� @sa deviationFactor
		*/
		int32 angleDeviationFactor;
	};

	// Polyline ץȡ���
	struct PolylineGrabResult
	{
		MapPoint3D64 intersection;     ///< Polyline�Ͼ���ץȡ�㣨pos������ĵ�
		uint16 intersectionDir;        ///< Polyline�� intersection ��λ�ô��ķ���������Ϊ0����ʱ��Ϊ������λ���ȡ�
		uint16 interPointId;           ///< intersection ��֮ǰ���һ����״����±�
		uint32 deviation;              ///< ��ץ·�㵽��Polyline�ľ��룬��λ cm
		int32 angleDeviation;          ///< ��ץ·����queryAngle���� intersectionDir �ĽǶȲ��λ���ȣ���Χ��[-180, 180]
									   ///< ���ץ·����queryAngle���� JUST_ANY_ANGLE���� angleDeviation Ϊ 0��

		uint32 disForward;             ///< �� intersection ���ظ�Polyline�ߵ�ĩ�˵ľ��룬��λ cm
		uint32 disBackward;            ///< �Ӹ�HadPolyline��ʼ����·�ߵ� intersection ��ľ��룬��λ cm
		uint32 score;                  ///< ץ·����ķ�ֵ���ο� PolylineGrabParams.deviationFactor �� PolylineGrabParams.angleDeviationFactor
		Sideness sideness;             ///< ץ·����HadPolyline����໹���Ҳ࣬���ץ·��������Polyline�ϣ���ȡֵΪ Sideness_unknown��
	};

	void GeometryUtil_grabPolyline(const PolylineGrabParams& params, const MapPoint3D64* pts, size_t ptCount, PolylineGrabResult& resultOut);
	void GeometryUtil_grabPolyline(const PolylineGrabParams& params, const Polyline& polyline, PolylineGrabResult& resultOut);
	void GeometryUitl_grabMultiPolylines(const PolylineGrabParams& params, const Polyline* polylines, size_t lineCount, cqstd::vector<PolylineGrabResult>& resultsOut);
}


namespace OMDB
{

	/**
	* �����������������߶�����ָ�������������߶Σ�
	* ����ø��߶�����ָ�����������ĵ㡣
	* Note: ���ڶ�άƽ���ϼ������ڽ��㡣
	*/
	class NearestPointOnLineSegmentsSearcher
	{
	public:
		using Point = std::array<double, 3>;
		using Segment = std::array<Point, 2>;

		static Point Point_make(const MapPoint3D64& pt)
		{
			return Point{ double(pt.pos.lon), double(pt.pos.lat), double(pt.z) };
		}

		static Point Point_make(const MapPoint64& pt)
		{
			return Point{ double(pt.lon), double(pt.lat), 0.0 };
		}

		static Segment Segment_make(const MapPoint3D64& start, const MapPoint3D64& end)
		{
			return Segment{ Point_make(start), Point_make(end) };
		}

		static Segment Segment_make(const MapPoint64& start, const MapPoint64& end)
		{
			return Segment{ Point_make(start), Point_make(end) };
		}

		static MapPoint3D64 Point_toMapPoint(const Point& pt)
		{
			return MapPoint3D64{ MapPoint64{ int64(pt[0]), int64(pt[1]) }, int32(pt[2]) };
		}

		static double PointSqDist(const Point& pt1, const Point& pt2)
		{
			return std::pow(pt1[0] - pt2[0], 2.0) + std::pow(pt1[1] - pt2[1], 2.0);
		}

		// �洢�߶������ڽ������߶��ϵ�λ��
		enum class NearestPointPosition
		{
			SEGMENT_START, // ���߶����
			SEGMENT_MIDDLE, // ���߶��м䲿�֣������յ㣩
			SEGMENT_END // ���߶��յ�
		};

		struct SearchResult
		{
			Point point;
			NearestPointPosition pointPosition;
			double sqDist;
			int nearestSegmentIndex;

			static SearchResult invalidObj()
			{
				return SearchResult{ Point{0, 0},  NearestPointPosition::SEGMENT_START, DBL_MAX, -1 };
			}

			bool valid()
			{
				return sqDist != DBL_MAX;
			}
		};

	private:

		std::vector<Segment> m_segments;


	public:
		static SearchResult findNearestPointToSegment(const Segment& segment, const Point& point);

		NearestPointOnLineSegmentsSearcher(std::vector<Segment> segments) : m_segments(std::move(segments)) {}

		const std::vector<Segment>& segments() const { return m_segments; }

		SearchResult searchNearest(const Point& sourcePoint) const;
	};

	NearestPointOnLineSegmentsSearcher NearestPointOnLineSegmentsSearcher_make(const LineString3d& linestring);
	NearestPointOnLineSegmentsSearcher NearestPointOnLineSegmentsSearcher_make(const std::vector<MapPoint3D64>& linestringVertices);

	NearestPointOnLineSegmentsSearcher NearestPointOnLineSegmentsSearcher_make(const std::vector<Vector3>& linestring);

}
