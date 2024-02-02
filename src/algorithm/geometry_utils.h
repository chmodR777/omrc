#pragma once

#include "cqstl/vector.h"
#include "geometry/map_point3d64.h"
#include "math3d/dvector2.h"
#include "math3d/vector2.h"
#include "math3d/vector3.h"

namespace hadc
{
	// 判断两个坐标点是否过近，以至于应该被判断为一个点
	forceinline bool GeometryUtil_tooCloseBetweenPoints(const MapPoint64& l, const MapPoint64& r)
	{
		return nc_abs(l.lon - r.lon) < 10 && nc_abs(l.lat - r.lat) < 10;
	}

	// 找到一系列点 points 中距离 p 最近的点的索引
	int GeometryUtil_findNearestPoint(const Vector3* points, int n, Vector2 p);

	// 计算射线和polyline的交点和距离
	bool GeometryUtil_calRayCutWithLine(const MapPoint3D64* points, size_t ptNum, const MapPoint3D64& p, const DVector2& dir, MapPoint3D64& rstPt, double& rstDis);

	// 计算某个点在指定方向和Polyline的距离是否小于某个值
	bool GeometryUtil_testIsPointWithinDistanceFromPolyline(const MapPoint3D64& p, const DVector2& dir,
		const MapPoint3D64* points, size_t ptNum, double testDistanceInMeter, MapPoint3D64& rstPt, double& rstDis);

	// 计算点和Polyline的距离，返回值单位：cm
	int64 GeometryUtil_calcDistanceBetweenPointAndPolyline(MapPoint64 pos, const MapPoint3D64* pts, size_t ptCount);
	/**
	* @brief 通过坐标计算高度调整值
	* @param[in] lon, lat
	*		高度对应的坐标，单位：度
	* @return
	*		返回的高度调整值，单位：cm
	* @details
	*		高度调整值的具体的计算方法是：
	*		首先将坐标（认为是GCJ02坐标）转换成WGS84坐标，然后计算两个坐标的偏差值，
	*		然后该偏差值乘以一个系数，最终结果是作为高度需要加上的偏差值。
	*/
	int32 GeometryUtil_calcHeightAdjustmentByPosition(double lon, double lat);
	// Polyline
	struct Polyline
	{
		const MapPoint3D64* pts;
		size_t ptCount;
	};

	// polyline抓取参数
	struct PolylineGrabParams
	{
		PolylineGrabParams() { reset(); }

		void reset();

		MapPoint64 position;
		/**
		* 抓路方向，正东为0，逆时针为正，单位：度。
		* 缺省为 JUST_ANY_ANGLE，表示不带方向抓路，即打分时角度偏差都取0。
		*/
		uint16 queryAngle;
		/**
		* 距离偏差打分系数
		*	deviationFactor: 距离的打分系数， angleDeviationFactor: 角度的打分系数
		*	score = deviationFactor * GrabResult.deviation + angleDeviationFactor * GrabResult.angleDeviation * 100.
		*	分数越小越好。抓到的路将按照 score 从小到大排序。缺省值是 deviationFactor = 1, angleDeviationFactor = 0。
		*/
		int32 deviationFactor;
		/**
		* 角度偏差打分系数 @sa deviationFactor
		*/
		int32 angleDeviationFactor;
	};

	// Polyline 抓取结果
	struct PolylineGrabResult
	{
		MapPoint3D64 intersection;     ///< Polyline上距离抓取点（pos）最近的点
		uint16 intersectionDir;        ///< Polyline在 intersection 点位置处的方向，以正东为0，逆时针为正，单位：度。
		uint16 interPointId;           ///< intersection 点之前最后一个形状点的下标
		uint32 deviation;              ///< 从抓路点到该Polyline的距离，单位 cm
		int32 angleDeviation;          ///< 从抓路方向（queryAngle）到 intersectionDir 的角度差，单位：度，范围：[-180, 180]
									   ///< 如果抓路方向（queryAngle）是 JUST_ANY_ANGLE，则 angleDeviation 为 0。

		uint32 disForward;             ///< 从 intersection 点沿该Polyline走到末端的距离，单位 cm
		uint32 disBackward;            ///< 从该HadPolyline的始端沿路走到 intersection 点的距离，单位 cm
		uint32 score;                  ///< 抓路结果的分值，参考 PolylineGrabParams.deviationFactor 和 PolylineGrabParams.angleDeviationFactor
		Sideness sideness;             ///< 抓路点在HadPolyline的左侧还是右侧，如果抓路点正好在Polyline上，则取值为 Sideness_unknown。
	};

	void GeometryUtil_grabPolyline(const PolylineGrabParams& params, const MapPoint3D64* pts, size_t ptCount, PolylineGrabResult& resultOut);
	void GeometryUtil_grabPolyline(const PolylineGrabParams& params, const Polyline& polyline, PolylineGrabResult& resultOut);
	void GeometryUitl_grabMultiPolylines(const PolylineGrabParams& params, const Polyline* polylines, size_t lineCount, cqstd::vector<PolylineGrabResult>& resultsOut);
}

