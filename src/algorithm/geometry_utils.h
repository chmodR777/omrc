#pragma once

#include "cqstl/vector.h"
#include "geometry/map_point3d64.h"
#include "math3d/dvector2.h"
#include "math3d/vector2.h"
#include "math3d/vector3.h"

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

