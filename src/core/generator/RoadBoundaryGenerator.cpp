#include "stdafx.h"
#include <deque>
#include "GroupGenerator.h"
#include "RoadBoundaryGenerator.h"
#include "algorithm/grap_point_algorithm.h"
#include "algorithm/polyline_intersector.h"
#include "algorithm/geometry_utils.h"
#include "math3d/dvector3.h"
#include "math3d/vector3.h"
#include "math3d/plane.h"
#include "cq_types_basic.h"
#include "math3d/coordinate_converter.h"
namespace OMDB
{

	/// 两条线需要四个贝塞尔曲线的控制点，每条线两个点，每个点的表示方式为距离折线第一个点的距离。
	struct CurvePointsPos
	{
		std::pair<float, float> trimLengthA;// first <= second
		std::pair<float, float> trimLengthB;
	};

	forceinline Vector2 _Vector2_fromVector3(Vector3 p) { return vec2(p.x, p.y); }
	forceinline Vector2 _Vector2_fromVector3(DVector3 p) { return vec2((float)p.x, (float)p.y); }
	bool vector3Near2D(Vector3 l, Vector3 r) { return floatEqual(l.x, r.x) && floatEqual(l.y, r.y); }


	struct PolylineIntersection
	{
		Vector3 pos;

		// when dir == true:
		//        | A
		//        |
		// <------------- B
		//        |
		//        |
		//        v
		bool dir;
		float distA;// lineA：交点距离起点的距离
		float distB;// lineB：交点距离起点的距离
	};

	int PolylineCalculator_calcIntersections(const std::vector<Vector3>& polylineA, const std::vector<Vector3>& polylineB, PolylineIntersection* intersections, int intersectionCount, bool ignoreHeightdif)
	{
		auto util_segmentIntersection = [](Vector3 A, Vector3 B, Vector3 C, Vector3 D, Vector3* projectAB, Vector3* projectCD) -> bool
			{
				Plane ab, cd;
				Vector3 vAB = B - A;
				Vector3 vCD = D - C;
				ab.setWithNormalPoint(vec3(vAB.y, -vAB.x, 0), A);
				cd.setWithNormalPoint(vec3(vCD.y, -vCD.x, 0), C);

				return ab.intersectSegment(C, D, projectCD) && cd.intersectSegment(A, B, projectAB);
			};


		int count = 0;
		Vector3 intersecA, intersecB, dirA, dirB;
		float lengthA = 0, lengthB = 0;
		for (size_t i = 1; i < polylineA.size(); i++)
		{
			dirA = polylineA[i] - polylineA[i - 1];
			lengthB = 0;
			for (size_t j = 1; j < polylineB.size(); j++)
			{
				dirB = polylineB[j] - polylineB[j - 1];
				if (util_segmentIntersection(polylineA[i], polylineA[i - 1],
					polylineB[j], polylineB[j - 1],
					&intersecA, &intersecB))
				{
					//避免匝道错误
					if (!ignoreHeightdif && fabs(intersecA.z - intersecB.z) > 3.0f)
					{
						continue;
					}
					float crossProduct = Vector2_cross(_Vector2_fromVector3(dirA), _Vector2_fromVector3(dirB));
					if (crossProduct != 0)
					{
						intersections[count].pos = (intersecA + intersecB) * 0.5;
						intersections[count].dir = crossProduct < 0;
						intersections[count].distA = lengthA + (polylineA[i - 1] - intersecA).length();
						intersections[count].distB = lengthB + (polylineB[j - 1] - intersecB).length();
						count++;
					}
					if (count == intersectionCount)
						return count;
				}

				lengthB += dirB.length();
			}

			lengthA += dirA.length();
		}
		return count;
	};

	float PolylineCalculator_calcLength(const std::vector<Vector3>& polyline)
	{
		float sum = 0;
		for (size_t i = 1; i < polyline.size(); i++)
			sum += (polyline[i] - polyline[i - 1]).length();
		return sum;
	};

	float PolylineCalculator_cutAtLength(const std::vector<Vector3>& polyline, float cutLength, Vector3* cutPointOut, size_t* nextIndexOut)
	{
		if (cutLength < 0)
			return false;

		float accumLength = 0;

		for (size_t i = 1; i < polyline.size(); i++)
		{
			Vector3 segment = polyline[i] - polyline[i - 1];
			if (accumLength + segment.length() > cutLength)
			{
				if (cutPointOut)
					*cutPointOut = polyline[i - 1] + normalize(segment) * (cutLength - accumLength);
				if (nextIndexOut)
					*nextIndexOut = i;

				//fix the floating point precision error
				if (cutPointOut)
				{
					if (vector3Near2D(*cutPointOut, polyline[i]))
						*cutPointOut = polyline[i];
					else if (vector3Near2D(*cutPointOut, polyline[i - 1]))
						*cutPointOut = polyline[i - 1];
				}

				return true;
			}
			accumLength += segment.length();
		}


		if (nc_abs(cutLength - accumLength) <= CQ_EPSILON)
		{
			if (cutPointOut)
				*cutPointOut = polyline.back();
			if (nextIndexOut)
				*nextIndexOut = polyline.size() - 1u;
			return true;
		}

		return false;
	};

	// 在线上与指定点最近的点获取裁剪信息
	void PolylineCalculator_cutAtNearestPoint(const std::vector<Vector3>& polyline, const Vector3& point, Vector3* cutPointOut, size_t* nextIndexOut)
	{
		NearestPointOnLineSegmentsSearcher searcher = NearestPointOnLineSegmentsSearcher_make(polyline);
		NearestPointOnLineSegmentsSearcher::SearchResult result = searcher.searchNearest({ point.x, point.y, point.z });
		using Position = NearestPointOnLineSegmentsSearcher::NearestPointPosition;

		if (cutPointOut)
			*cutPointOut = vec3(result.point[0], result.point[1], result.point[2]);
		if (nextIndexOut)
			*nextIndexOut = result.pointPosition == Position::SEGMENT_START ? result.nearestSegmentIndex : result.nearestSegmentIndex + 1;
	};


	float PolylineCalculator_calcDistanceOfParallelPart(const std::vector<Vector3>& lineA, const std::vector<Vector3>& lineB)
	{
		constexpr float MAX_PARALLEL_ANGLE = 5;
		float MAX_PARALLEL_COS_VALUE = cos(MAX_PARALLEL_ANGLE * MATH_DEGREE2RADIAN);

		float dist = FLT_MAX;
		PointF _;
		for (size_t i = 1; i < lineA.size(); i++)
		{
			for (size_t j = 1; j < lineB.size(); j++)
			{
				Vector2 dirA = normalize(_Vector2_fromVector3(lineA[i] - lineA[i - 1]));
				Vector2 dirB = normalize(_Vector2_fromVector3(lineB[j] - lineB[j - 1]));
				if (dot(dirA, dirB) > MAX_PARALLEL_COS_VALUE)
				{
					dist = nc_min(dist, Math_segmentPointDistanceSquaredF(PointF_make(lineA[i].x, lineA[i].y),
						PointF_make(lineA[i - 1].x, lineA[i - 1].y), PointF_make(lineB[j].x, lineB[j].y), &_, NULL));
					dist = nc_min(dist, Math_segmentPointDistanceSquaredF(PointF_make(lineA[i].x, lineA[i].y),
						PointF_make(lineA[i - 1].x, lineA[i - 1].y), PointF_make(lineB[j - 1].x, lineB[j - 1].y), &_, NULL));
					dist = nc_min(dist, Math_segmentPointDistanceSquaredF(PointF_make(lineB[j].x, lineB[j].y),
						PointF_make(lineB[j - 1].x, lineB[j - 1].y), PointF_make(lineA[i].x, lineA[i].y), &_, NULL));
					dist = nc_min(dist, Math_segmentPointDistanceSquaredF(PointF_make(lineB[j].x, lineB[j].y),
						PointF_make(lineB[j - 1].x, lineB[j - 1].y), PointF_make(lineA[i - 1].x, lineA[i - 1].y), &_, NULL));
				}
			}
		}

		return sqrtf(dist);
	};

	// 原有3DR生成两道路边界连接处bezier曲线控制点
	void CurveGenerator_generateCurvePoints(const std::vector<Vector3>& lineA, float lineAExtend, const std::vector<Vector3>& lineB, float lineBExtend,
		CurvePointsPos* curvePointsPosOut, bool bCW, bool bIgnoreHeightdif)
	{
		constexpr float CURVE_BACK_DISTENCE = 3.f;
		constexpr int POLYGON_CURVE_POINT_COUNT = 5;
		constexpr int POLYGON_INTERSECTION_MAX_LENGTH_METERS = 100;

		std::vector<Vector3> m_bufferA = lineA;
		std::vector<Vector3> m_bufferB = lineB;

		Vector3 startPosA = m_bufferA.front();
		Vector3 startPosB = m_bufferB.front();

		auto _extendLineFromStart = [](std::vector<Vector3>& points, float length)
			{
				Vector3 dir = normalize(points[0] - points[1]);
				points[0] = points[0] + dir * length;
			};

		_extendLineFromStart(m_bufferA, lineAExtend);
		_extendLineFromStart(m_bufferB, lineBExtend);

		PolylineIntersection intersections[1];
		Vector3 dirA = m_bufferA[1] - m_bufferA[0];
		Vector3 dirB = m_bufferB[1] - m_bufferB[0];

		int relCount = PolylineCalculator_calcIntersections(m_bufferA, m_bufferB, intersections, 1, bIgnoreHeightdif);

		m_bufferA[0] = startPosA;
		m_bufferB[0] = startPosB;
		float lengthA = PolylineCalculator_calcLength(m_bufferA);
		float lengthB = PolylineCalculator_calcLength(m_bufferB);
		if (relCount == 1 && (bCW || !intersections[0].dir)
			&& intersections[0].distA - lineAExtend < POLYGON_INTERSECTION_MAX_LENGTH_METERS
			&& intersections[0].distB - lineBExtend < POLYGON_INTERSECTION_MAX_LENGTH_METERS)
		{
			curvePointsPosOut->trimLengthA.first = intersections[0].distA - lineAExtend;
			curvePointsPosOut->trimLengthA.second = nc_clamp(intersections[0].distA - lineAExtend + CURVE_BACK_DISTENCE, 0.f, lengthA);
			curvePointsPosOut->trimLengthB.first = intersections[0].distB - lineBExtend;
			curvePointsPosOut->trimLengthB.second = nc_clamp(intersections[0].distB - lineBExtend + CURVE_BACK_DISTENCE, 0.f, lengthB);
		}
		else
		{
			curvePointsPosOut->trimLengthA.second = CURVE_BACK_DISTENCE * 2;
			curvePointsPosOut->trimLengthA.first = CURVE_BACK_DISTENCE;
			curvePointsPosOut->trimLengthB.second = CURVE_BACK_DISTENCE * 2;
			curvePointsPosOut->trimLengthB.first = CURVE_BACK_DISTENCE;

			// 同向
			if (dot(dirA, dirB) > 0)
			{
				//20221009主路与辅路或非机动车道交汇的大路口
				Vector3 dirA1 = dirA.normalizedCopy();
				Vector3 dirB1 = dirB.normalizedCopy();
				float dotAB = dot(dirA1, dirB1);
				float dist = PolylineCalculator_calcDistanceOfParallelPart(lineA, lineB);
				if (dotAB > 0.965f && dist < 5.0f)
				{
					curvePointsPosOut->trimLengthA.second = CURVE_BACK_DISTENCE;
					curvePointsPosOut->trimLengthA.first = CURVE_BACK_DISTENCE;
					curvePointsPosOut->trimLengthB.second = CURVE_BACK_DISTENCE;
					curvePointsPosOut->trimLengthB.first = CURVE_BACK_DISTENCE;

					//
					Vector3 dirAB = m_bufferB[0] - m_bufferA[0];
					float dotAAB = dot(dirA, dirAB);
					float len = fabs(dot(dirAB, dirA1));
					if (dotAAB > 0)
					{
						curvePointsPosOut->trimLengthA.second += len;
						curvePointsPosOut->trimLengthA.first += len;
					}
					else
					{
						curvePointsPosOut->trimLengthB.second += len;
						curvePointsPosOut->trimLengthB.first += len;
					}

					curvePointsPosOut->trimLengthA.second += 1.2f;
					curvePointsPosOut->trimLengthB.second += 1.2f;
				}

			}
			else // 对向
			{
				if (Vector2_cross(_Vector2_fromVector3(dirA), _Vector2_fromVector3(startPosB - startPosA)) > 0)
				{
					curvePointsPosOut->trimLengthA.second = CURVE_BACK_DISTENCE * 4;
					curvePointsPosOut->trimLengthA.first = CURVE_BACK_DISTENCE * 2;
					curvePointsPosOut->trimLengthB.first = -CURVE_BACK_DISTENCE;
					curvePointsPosOut->trimLengthB.second = 0;
				}
				else
				{
					curvePointsPosOut->trimLengthA.second = 0;
					curvePointsPosOut->trimLengthA.first = -CURVE_BACK_DISTENCE;

					curvePointsPosOut->trimLengthB.first = CURVE_BACK_DISTENCE * 2;
					curvePointsPosOut->trimLengthB.second = CURVE_BACK_DISTENCE * 4;
				}
			}
		}
	}

	const std::vector<Vector3> CurveGenerator_generateCurve(const std::vector<Vector3>& lineA,
		const std::vector<Vector3>& lineB, const CurvePointsPos& curvePointsPos)
	{
		constexpr int POLYGON_CURVE_POINT_COUNT = 7;

		auto _extendStartPos = [](const std::vector<Vector3>& points, float length) -> Vector3
			{
				Vector3 dir = normalize(points[0] - points[1]);
				return points[0] + dir * length;
			};

		Vector3 p0, p1, p2, p3;

		// calc point
		if (curvePointsPos.trimLengthA.second < 0 || curvePointsPos.trimLengthB.second < 0)
		{
			CQ_LOG_ERROR("curvePointsPos second trim length should >= 0, otherwise the Bezier curve will be very exaggerated");
			return std::vector<Vector3>{};
		}

		if (!PolylineCalculator_cutAtLength(lineA, curvePointsPos.trimLengthA.second, &p0, NULL))
			p0 = lineA.back();

		if (curvePointsPos.trimLengthA.first < 0)
		{
			p1 = _extendStartPos(lineA, -curvePointsPos.trimLengthA.first);
			p1.z = cq_max(lineA.front().z, 0.0f);
		}
		else if (!PolylineCalculator_cutAtLength(lineA, curvePointsPos.trimLengthA.first, &p1, NULL))
			p1 = lineA.back();

		if (curvePointsPos.trimLengthB.first < 0)
		{
			p2 = _extendStartPos(lineB, -curvePointsPos.trimLengthB.first);
			p2.z = cq_max(lineB.front().z, 0.0f);
		}
		else if (!PolylineCalculator_cutAtLength(lineB, curvePointsPos.trimLengthB.first, &p2, NULL))
			p2 = lineB.back();

		CQ_ASSERT(curvePointsPos.trimLengthB.second >= 0);
		if (!PolylineCalculator_cutAtLength(lineB, curvePointsPos.trimLengthB.second, &p3, NULL))
			p3 = lineB.back();

		std::vector<Vector3> curve(POLYGON_CURVE_POINT_COUNT);
		for (int i = 0; i < POLYGON_CURVE_POINT_COUNT; i++)
		{
			float t = (float)i / (POLYGON_CURVE_POINT_COUNT - 1);
			curve[i] = Math_sampleCubicBezier3D(p0, p1, p2, p3, t);
		}

		return curve;
	}

	/**
     * @brief 将polyline从起点开始裁剪掉dist长度
     */
	bool PolylineCalculator_trimStart(std::vector<Vector3>& polyline, float dist)
	{
		Vector3 cutPoint;
		size_t cutNextIndex;
		if (!PolylineCalculator_cutAtLength(polyline, dist, &cutPoint, &cutNextIndex))
			return false;

		polyline[cutNextIndex - 1] = cutPoint;
		if (vector3Near2D(cutPoint, polyline[cutNextIndex]))
			cutNextIndex++;
		if (cutNextIndex >= polyline.size())
			return false;
		polyline.erase(polyline.begin(), polyline.begin() + cutNextIndex - 1);
		return true;
	}

	/**
	 * @brief 修改lineA & lineB几何使其连接处平滑过渡
	 * @param lineA 同lineB一样，连接路口，剩余部分远离路口
	 * @param lineB 
	 * @param linkNodePosition 车道组所对应的两个link的几何。
	*/
	void generateStraightRoadCurvePoints(std::vector<MapPoint3D64>& lineA, std::vector<MapPoint3D64>& lineB, const MapPoint3D64& linkNodePosition)
	{
		const MapPoint3D64 lineAOriginBack = lineA.back();
		const MapPoint3D64 lineBOriginBack = lineB.back();

		double centerLon{ 0.0 }, centerLat{ 0.0 };
		{
			for (const MapPoint3D64& pt : lineA)
			{
				centerLon += pt.pos.lon;
				centerLat += pt.pos.lat;
			}
			for (const MapPoint3D64& pt : lineB)
			{
				centerLon += pt.pos.lon;
				centerLat += pt.pos.lat;
			}
			centerLon /= (lineA.size() + lineB.size());
			centerLat /= (lineA.size() + lineB.size());
		}


		MapPoint64 center{ int64(centerLon), int64(centerLat) };
		double x2yRatio = coordinate_converter::calcLonLatScale(center.toNdsPoint());
		constexpr double MAPPOINT3D64LonLatToMeter = 0.00111;

		auto toLocal = [&center, &x2yRatio, &MAPPOINT3D64LonLatToMeter](const MapPoint3D64& pt)  -> Vector3
			{
				Vector3 newPoint;
				newPoint.x = (pt.pos.lon - center.lon) * x2yRatio;
				newPoint.y = (pt.pos.lat - center.lat);
				newPoint.z = pt.z * 0.01;

				// 单位转为米
				newPoint.x *= MAPPOINT3D64LonLatToMeter;
				newPoint.y *= MAPPOINT3D64LonLatToMeter;

				return newPoint;
			};

		auto toOrigin = [&center, &x2yRatio, &MAPPOINT3D64LonLatToMeter](const Vector3& pt) -> MapPoint3D64
			{
				double lon = pt.x / MAPPOINT3D64LonLatToMeter;
				double lat = pt.y / MAPPOINT3D64LonLatToMeter;
				double z = pt.z / 0.01;
				lon /= x2yRatio;

				return  MapPoint3D64_make(int64(lon + center.lon), int64(lat + center.lat), int32(z));
			};

		std::vector<Vector3> localLineA;
		for (const MapPoint3D64& pt : lineA)
			localLineA.push_back(toLocal(pt));

		std::vector<Vector3> localLineB;
		for (const MapPoint3D64& pt : lineB)
			localLineB.push_back(toLocal(pt));

		CurvePointsPos trimLengths;
		constexpr float EXTEND_FOR_NORMAL_ROAD{ 5.0f };
		float lineAExtend{ EXTEND_FOR_NORMAL_ROAD };
		float lineBExtend{ EXTEND_FOR_NORMAL_ROAD };

		bool bCW = false;
		{
			Vector3 dirA = normalize(localLineA[1] - localLineA[0]);
			Vector3 dirB = normalize(localLineB[1] - localLineB[0]);
			float dotAB = dot(dirA, dirB);
			if (fabs(dotAB) < 0.5f)// dirA & dirB夹角大于60度
			{
				Vector3 crossV = cross(normalize(dirA), normalize(dirB));
				bCW = dot(vec3(0.0f, 0.0f, 1.0f), crossV) > 0 ? false : true;
				lineAExtend = EXTEND_FOR_NORMAL_ROAD * 6.0f;
				lineBExtend = EXTEND_FOR_NORMAL_ROAD * 6.0f;
			}
		}
		
		CurveGenerator_generateCurvePoints(localLineA, lineAExtend, localLineB, lineBExtend, &trimLengths, bCW, true);

		float lineALength = PolylineCalculator_calcLength(localLineA);
		float lineBLength = PolylineCalculator_calcLength(localLineB);
		trimLengths.trimLengthA.first = min(lineALength, trimLengths.trimLengthA.first);
		trimLengths.trimLengthA.second = min(lineALength, trimLengths.trimLengthA.second);
		trimLengths.trimLengthB.first = min(lineBLength, trimLengths.trimLengthB.first);
		trimLengths.trimLengthB.second = min(lineBLength, trimLengths.trimLengthB.second);


	// 生成曲线，并在恰当地方分割成两半分别赋值给lineA & lineB

		std::vector<Vector3> curve = CurveGenerator_generateCurve(localLineA, localLineB, trimLengths);
		if (curve.empty()) return;

		PolylineCalculator_trimStart(localLineA, trimLengths.trimLengthA.second);
		PolylineCalculator_trimStart(localLineB, trimLengths.trimLengthB.second);

		float curveLength = PolylineCalculator_calcLength(curve);
		Vector3 curveMidPoint;
		size_t curveMidPointNextIndex = -1;
		// 使用Link连接处Node位置裁切使得车道组边界划分正确
		PolylineCalculator_cutAtNearestPoint(curve, toLocal(linkNodePosition), &curveMidPoint, &curveMidPointNextIndex);
		// 将curve的前一半添加到localLineA，后一半添加到localLineB
		// 由于都需要将curve加到两线的前部分，因此将线翻转追加，然后再翻转
		std::reverse(localLineA.begin(), localLineA.end());
		for (std::size_t i = 1; i < curveMidPointNextIndex; ++i)
			localLineA.push_back(curve[i]);
		localLineA.push_back(curveMidPoint);
		std::reverse(localLineA.begin(), localLineA.end());

		std::reverse(localLineB.begin(), localLineB.end());
		for (int i = curve.size() - 1; i >= int(curveMidPointNextIndex); --i)
			localLineB.push_back(curve[i]);
		localLineB.push_back(curveMidPoint);
		std::reverse(localLineB.begin(), localLineB.end());

		// localLine所使用的局部坐标系转回MapPoint3D64
		lineA.clear();
		for (const auto& pt : localLineA)
		{
			MapPoint3D64 newPoint = toOrigin(pt);
			if (lineA.empty() || newPoint != lineA.back())
				lineA.push_back(newPoint);
		}
		lineB.clear();
		for (const auto& pt : localLineB)
		{
			MapPoint3D64 newPoint = toOrigin(pt);
			if (lineB.empty() || newPoint != lineB.back())
				lineB.push_back(toOrigin(pt));
		}

		// 上述一系列处理加上局部坐标和MapPoint3D64的转换有可能导致最后一个点坐标出现些许偏差，此处恢复，使得与上一车道线坐标完全重合
		lineA.back() = lineAOriginBack;
		lineB.back() = lineBOriginBack;
	}

    void RoadBoundaryGenerator::generate(DbMesh* const pMesh)
    {
		m_mesh = pMesh;
        std::vector<DbRecord*>& links = pMesh->query(RecordType::DB_HAD_LINK);
        for (auto hl : links) {
            DbLink* pLink = (DbLink*)hl;
			if (!generateHdData(pLink))
				continue;

			if (pLink->direct == 2 || pLink->direct == 3) {
				generateDirect23Boundary(pLink, pLink->direct);
			}
			else {
				generateDirectNot23Boundary(pLink);
			}
        }

		// 车道组索引
		std::vector<DbRecord*>& lanePas = m_mesh->query(RecordType::DB_RD_LINK_LANEPA);
		for (auto lp : lanePas) {
			DbRdLinkLanePa* lanePa = (DbRdLinkLanePa*)lp;
			if (lanePa->roadBoundaries.size() == 2)
				saveLanePaInfo(lanePa);
		}
    }

	void RoadBoundaryGenerator::generateDirectNot23Boundary(DbLink* const pLink)
	{
		// 顺方向
		auto rightLanePas = getLanePas(pLink, pLink->lanePas, 2);
		if (!rightLanePas.empty()) {
			double boundaryOffset = DBL_MAX;
			for (int idx = 0; idx < rightLanePas.size(); idx++) {
				auto lanePa = rightLanePas[idx];
				auto halfBoundaryOffset = getLanePaWidth(pLink, lanePa) / 2;
				boundaryOffset = min(boundaryOffset, halfBoundaryOffset);
			}

			// left boundary
			LineString3d originLine = pLink->geometry;
			coordinatesTransform.convert(originLine.vertexes.data(), originLine.vertexes.size());
			auto boundaryPoints = buildBoundaryPoints(originLine.vertexes);
			auto boundaryLength = bg::length(LINESTRING_T(originLine.vertexes));
			generateDirect2Boundary(pLink, 2, rightLanePas, originLine, boundaryPoints, boundaryLength, 0, 2);

			// right boundary
			auto rightOffsetLine = adjustBoundaryOffset(pLink->geometry, boundaryOffset);
			coordinatesTransform.convert(rightOffsetLine.vertexes.data(), rightOffsetLine.vertexes.size());
			auto rightBoundaryPoints = buildBoundaryPoints(rightOffsetLine.vertexes);
			auto rightBoundaryLength = bg::length(LINESTRING_T(rightOffsetLine.vertexes));
			generateDirect2Boundary(pLink, 2, rightLanePas, rightOffsetLine, rightBoundaryPoints, rightBoundaryLength, boundaryOffset, 1);
		}

		// 逆方向
		auto leftLanePas = getLanePas(pLink, pLink->lanePas, 3);
		if (!leftLanePas.empty()) {
			double boundaryOffset = DBL_MAX;
			for (int idx = 0; idx < leftLanePas.size(); idx++) {
				auto lanePa = leftLanePas[idx];
				auto halfBoundaryOffset = getLanePaWidth(pLink, lanePa) / 2;
				boundaryOffset = min(boundaryOffset, halfBoundaryOffset);
			}

			// left boundary
			auto leftOffsetLine = adjustBoundaryOffset(pLink->geometry, boundaryOffset);
			coordinatesTransform.convert(leftOffsetLine.vertexes.data(), leftOffsetLine.vertexes.size());
			auto leftBoundaryPoints = buildBoundaryPoints(leftOffsetLine.vertexes);
			auto leftBoundaryLength = bg::length(LINESTRING_T(leftOffsetLine.vertexes));
			for (auto leftLanePa : leftLanePas) {
				auto leftRelLinkPair = getRelLinkPair(pLink, leftLanePa);

				DbRdLinkLanePa* rightLanePa = nullptr;
				for (auto tmpLanePa : rightLanePas) {
					auto tmpRelLinkPair = getRelLinkPair(pLink, tmpLanePa);
					if (std::abs(leftRelLinkPair.second.startOffset - tmpRelLinkPair.second.startOffset) < 0.01 
						&& std::abs(leftRelLinkPair.second.endOffset - tmpRelLinkPair.second.endOffset) < 0.01) {
						rightLanePa = tmpLanePa;
						break;
					}
				}

				// relLg
				if (rightLanePa != nullptr) {
					DbRoadBoundLink::DbLgRoadBoundREL relLg;
					relLg.side = 2;
					relLg.direction = 3;
					DbRoadBoundLink* pBoundary = rightLanePa->roadBoundaries[0];
					pBoundary->relLgs.emplace(leftRelLinkPair.first, relLg);
					leftLanePa->roadBoundaries.push_back(pBoundary);
				} 
				else {
					std::vector<DbRdLinkLanePa*> tmpLeftLanePas{ leftLanePa };
					generateDirect3Boundary(pLink, 3, tmpLeftLanePas, leftOffsetLine, leftBoundaryPoints, leftBoundaryLength, boundaryOffset, 2);
				}
			}

			// right boundary
			auto rightOffsetLine = adjustBoundaryOffset(pLink->geometry, -boundaryOffset);
			coordinatesTransform.convert(rightOffsetLine.vertexes.data(), rightOffsetLine.vertexes.size());
			auto rightBoundaryPoints = buildBoundaryPoints(rightOffsetLine.vertexes);
			auto rightBoundaryLength = bg::length(LINESTRING_T(rightOffsetLine.vertexes));
			generateDirect3Boundary(pLink, 3, leftLanePas, rightOffsetLine, rightBoundaryPoints, rightBoundaryLength, -boundaryOffset, 1);
		}
	}

	void RoadBoundaryGenerator::generateDirect23Boundary(DbLink* const pLink, int direct)
	{
		// 车信 pLink->relLaneInfos;
		auto directLanePas = getLanePas(pLink, pLink->lanePas, direct);
		if (directLanePas.empty()) {
			return;
		}

		double boundaryOffset = DBL_MAX;
		for (int idx = 0; idx < directLanePas.size(); idx++) {
			auto lanePa = directLanePas[idx];
			auto halfBoundaryOffset = getLanePaWidth(pLink, lanePa) / 2;
			boundaryOffset = min(boundaryOffset, halfBoundaryOffset);
		}

		if (direct == 2)
		{
			// left boundary
			auto leftOffsetLine = adjustBoundaryOffset(pLink->geometry, -boundaryOffset);
			coordinatesTransform.convert(leftOffsetLine.vertexes.data(), leftOffsetLine.vertexes.size());
			auto leftBoundaryPoints = buildBoundaryPoints(leftOffsetLine.vertexes);
			auto leftBoundaryLength = bg::length(LINESTRING_T(leftOffsetLine.vertexes));
			generateDirect2Boundary(pLink, direct, directLanePas, leftOffsetLine, leftBoundaryPoints, leftBoundaryLength, -boundaryOffset, 2);

			// right boundary
			auto rightOffsetLine = adjustBoundaryOffset(pLink->geometry, boundaryOffset);
			coordinatesTransform.convert(rightOffsetLine.vertexes.data(), rightOffsetLine.vertexes.size());
			auto rightBoundaryPoints = buildBoundaryPoints(rightOffsetLine.vertexes);
			auto rightBoundaryLength = bg::length(LINESTRING_T(rightOffsetLine.vertexes));
			generateDirect2Boundary(pLink, direct, directLanePas, rightOffsetLine, rightBoundaryPoints, rightBoundaryLength, boundaryOffset, 1);
		}
		else if (direct == 3)
		{
			// left boundary
			auto leftOffsetLine = adjustBoundaryOffset(pLink->geometry, boundaryOffset);
			coordinatesTransform.convert(leftOffsetLine.vertexes.data(), leftOffsetLine.vertexes.size());
			auto leftBoundaryPoints = buildBoundaryPoints(leftOffsetLine.vertexes);
			auto leftBoundaryLength = bg::length(LINESTRING_T(leftOffsetLine.vertexes));
			generateDirect3Boundary(pLink, direct, directLanePas, leftOffsetLine, leftBoundaryPoints, leftBoundaryLength, boundaryOffset, 2);

			// right boundary
			auto rightOffsetLine = adjustBoundaryOffset(pLink->geometry, -boundaryOffset);
			coordinatesTransform.convert(rightOffsetLine.vertexes.data(), rightOffsetLine.vertexes.size());
			auto rightBoundaryPoints = buildBoundaryPoints(rightOffsetLine.vertexes);
			auto rightBoundaryLength = bg::length(LINESTRING_T(rightOffsetLine.vertexes));
			generateDirect3Boundary(pLink, direct, directLanePas, rightOffsetLine, rightBoundaryPoints, rightBoundaryLength, -boundaryOffset, 1);
		}
	}

	void RoadBoundaryGenerator::saveLanePaInfo(DbRdLinkLanePa* const lanePa)
	{
		LineString3d leftSide, rightSide;
		getLanePaBoundary(lanePa, leftSide, rightSide);
		if (!leftSide.vertexes.empty() && !rightSide.vertexes.empty()) {
			lanePaInfo info;
			info._lanePa = lanePa;

			auto& tmpLeftLine = leftSide.vertexes;
			//coordinatesTransform.convert(tmpLeftLine.data(), tmpLeftLine.size());
			auto& tmpRightLine = rightSide.vertexes;
			//coordinatesTransform.convert(tmpRightLine.data(), tmpRightLine.size());

			lanePa->updateBoundingBox();
			std::vector<MapPoint3D64> bboxVertexes;
			bboxVertexes.push_back(lanePa->extent.min);
			bboxVertexes.push_back(lanePa->extent.max);
			info._lanePaBox2T = BOX_2T(bboxVertexes);

			info._leftLine = LINESTRING_T(tmpLeftLine);
			info._rightLine = LINESTRING_T(tmpRightLine);
			m_mesh->insertLanePaInfo(info);
		}
	}

	void RoadBoundaryGenerator::generateDirect2Boundary(DbLink* const pLink, int direct, std::vector<DbRdLinkLanePa*>& directLanePas,
		LineString3d& offsetLine, std::vector<boundaryPoint>& boundaryPoints, double boundaryLength, double boundaryOffset, int side)
	{
		double lastEndOffset{ DBL_MAX };
		DbRoadBoundNode* lastEndNode = nullptr;
		for (int idx = 0; idx < directLanePas.size(); idx++) {
			auto currLanePa = directLanePas[idx];
			auto currRelLinkPair = getRelLinkPair(pLink, currLanePa);

			int64 rdBoundSeed = (currRelLinkPair.first << 1) | (side - 1);
			int64 rdNodeSeed = (rdBoundSeed << 1) | (side - 1);

			auto startOffset = currRelLinkPair.second.startOffset;
			auto endOffset = currRelLinkPair.second.endOffset;
			auto subLine = getBoundaryByOffset(offsetLine.vertexes, boundaryPoints, startOffset * boundaryLength, endOffset * boundaryLength);
			coordinatesTransform.invert(subLine.vertexes.data(), subLine.vertexes.size());

			// boundary
			DbRoadBoundLink* pBoundary = (DbRoadBoundLink*)m_mesh->alloc(RecordType::DB_HAD_ROAD_BOUNDARY_LINK);
			pBoundary->uuid = currRelLinkPair.first + (rdBoundSeed++);
			pBoundary->boundaryType = 0; // TODO
			pBoundary->geometry = subLine;
			pBoundary->geometryOffset = boundaryOffset;

			// startNode
			DbRoadBoundNode* pStartNode = nullptr;
			if (lastEndNode != nullptr && std::abs(startOffset - lastEndOffset) < 0.01) {
				pStartNode = lastEndNode;
			}
			else {
				pStartNode = (DbRoadBoundNode*)m_mesh->alloc(RecordType::DB_HAD_ROAD_BOUNDARY_NODE);
				pStartNode->uuid = currRelLinkPair.first + (rdNodeSeed++);
				pStartNode->geometry = pBoundary->geometry.vertexes.front();
				m_mesh->insert(pStartNode->uuid, pStartNode);
			}

			// endNode
			DbRoadBoundNode* pEndNode = (DbRoadBoundNode*)m_mesh->alloc(RecordType::DB_HAD_ROAD_BOUNDARY_NODE);
			pEndNode->uuid = currRelLinkPair.first + (rdNodeSeed++);
			pEndNode->geometry = pBoundary->geometry.vertexes.back();
			m_mesh->insert(pEndNode->uuid, pEndNode);

			// nodeId
			pBoundary->starRoadBoundNodeId = pStartNode->uuid;
			pBoundary->endRoadBoundNodeId = pEndNode->uuid;
			m_mesh->insert(pBoundary->uuid, pBoundary);

			// relLg
			DbRoadBoundLink::DbLgRoadBoundREL relLg;
			relLg.side = side;
			relLg.direction = direct;
			pBoundary->relLgs.emplace(currRelLinkPair.first, relLg);
			currLanePa->roadBoundaries.push_back(pBoundary);

			// backup last node
			lastEndOffset = endOffset;
			lastEndNode = pEndNode;
		}
	}

	void RoadBoundaryGenerator::generateDirect3Boundary(DbLink* const pLink, int direct, std::vector<DbRdLinkLanePa*>& directLanePas,
		LineString3d& offsetLine, std::vector<boundaryPoint>& boundaryPoints, double boundaryLength, double boundaryOffset, int side)
	{
		double lastEndOffset{ DBL_MAX };
		DbRoadBoundNode* lastEndNode = nullptr;
		for (int idx = 0; idx < directLanePas.size(); idx++) {
			auto currLanePa = directLanePas[idx];
			auto currRelLinkPair = getRelLinkPair(pLink, currLanePa);

			int64 rdBoundSeed = (currRelLinkPair.first << 1) | (side - 1);
			int64 rdNodeSeed = (rdBoundSeed << 1) | (side - 1);

			auto startOffset = currRelLinkPair.second.startOffset;
			auto endOffset = currRelLinkPair.second.endOffset;
			auto subLine = getBoundaryByOffset(offsetLine.vertexes, boundaryPoints, startOffset * boundaryLength, endOffset * boundaryLength);
			coordinatesTransform.invert(subLine.vertexes.data(), subLine.vertexes.size());

			// boundary
			DbRoadBoundLink* pBoundary = (DbRoadBoundLink*)m_mesh->alloc(RecordType::DB_HAD_ROAD_BOUNDARY_LINK);
			pBoundary->uuid = currRelLinkPair.first + (rdBoundSeed++);
			pBoundary->boundaryType = 0; // TODO
			pBoundary->geometry = subLine;
			pBoundary->geometryOffset = boundaryOffset;

			// startNode
			DbRoadBoundNode* pStartNode = (DbRoadBoundNode*)m_mesh->alloc(RecordType::DB_HAD_ROAD_BOUNDARY_NODE);
			pStartNode->uuid = currRelLinkPair.first + (rdNodeSeed++);
			pStartNode->geometry = pBoundary->geometry.vertexes.back();
			m_mesh->insert(pStartNode->uuid, pStartNode);


			// endNode
			DbRoadBoundNode* pEndNode = nullptr;
			if (lastEndNode != nullptr && std::abs(startOffset - lastEndOffset) < 0.01) {
				pEndNode = lastEndNode;
			}
			else {
				pEndNode = (DbRoadBoundNode*)m_mesh->alloc(RecordType::DB_HAD_ROAD_BOUNDARY_NODE);
				pEndNode->uuid = currRelLinkPair.first + (rdNodeSeed++);
				pEndNode->geometry = pBoundary->geometry.vertexes.front();
				m_mesh->insert(pEndNode->uuid, pEndNode);
			}

			// nodeId
			pBoundary->starRoadBoundNodeId = pStartNode->uuid;
			pBoundary->endRoadBoundNodeId = pEndNode->uuid;
			m_mesh->insert(pBoundary->uuid, pBoundary);

			// relLg
			DbRoadBoundLink::DbLgRoadBoundREL relLg;
			relLg.side = side;
			relLg.direction = direct;
			pBoundary->relLgs.emplace(currRelLinkPair.first, relLg);
			currLanePa->roadBoundaries.push_back(pBoundary);

			// backup last node
			lastEndOffset = endOffset;
			lastEndNode = pStartNode;
		}
	}

    void RoadBoundaryGenerator::generateRelation(DbMesh* const pMesh, std::vector<DbMesh*>* nearby)
    {
		m_mesh = pMesh;
		m_nearbyMesh = nearby;
		std::vector<DbRecord*>& nodes = pMesh->query(RecordType::DB_HAD_NODE);
		std::unordered_set<std::string> visitedNodes;

		// generateDirectStraightRoadGroup
		for (auto nd : nodes)
		{
			DbNode* pNode = (DbNode*)nd;
			if (!isForkRoad(pNode)) {
				generateDirectRoadFromNode(pNode, visitedNodes, &RoadBoundaryGenerator::generateDirectStraightRoadGroup);
			}
		}

		// 生成上下行道路车道组公共边界
		for (std::size_t i = 0; i < pMesh->getDividedRoads().size(); ++i)
		{
			auto& dividedRoad = pMesh->getDividedRoads()[i];
			generateDividedRoadCommonBoundary(i, dividedRoad);
		}	
		pMesh->getDividedRoads().clear(); // 后续Generator应该不会再用到上下行道路，此处将其清理。

		for (auto nd : nodes)
		{
			DbNode* pNode = (DbNode*)nd;
			if (pNode->links.size() <= 1)
				continue;
			if (isForkRoad(pNode))
			{
				generateForkRoadClipPoint(pNode);
			}
		}

		// choose proper clipPoint
		fixupForkRoadClipPoint(pMesh);

		// generateDirectForkRoadGroup
		for (auto nd : nodes)
		{
			DbNode* pNode = (DbNode*)nd;
			if (isForkRoad(pNode)) {
				generateDirectRoadFromNode(pNode, visitedNodes, &RoadBoundaryGenerator::generateDirectForkRoadGroup);
			}
		}
    }

	bool RoadBoundaryGenerator::isForkRoad(DbNode* pNode)
	{
		int sum = 0;
		for (DbLink* pLink : pNode->links)
		{
			// 没有车道组暂时不算在路口里
			if (pLink->lanePas.empty())
				continue;
			if (pLink->startNode != pLink->endNode) {
				sum += 1;
			}
			else {
				sum += 2;
			}
		}
		return sum > 2;
	}

	void RoadBoundaryGenerator::generateForkRoadClipPoint(DbNode* pNode)
	{
		auto expandNodeFace = RING_2T(pNode->geometry, 10000);
		bg::correct(expandNodeFace);

		for (int m = 0; m < pNode->links.size(); m++)
		{
			DbLink* pFirst = (DbLink*)pNode->links[m];
			auto firstClipFaces = getClipFaces(pFirst, expandNodeFace);
			if (firstClipFaces.empty())
				continue;

			for (int n = m + 1; n < pNode->links.size(); n++)
			{
				DbLink* pSecond = (DbLink*)pNode->links[n];
				auto secondClipFaces = getClipFaces(pSecond, expandNodeFace);
				if (secondClipFaces.empty())
					continue;

				for (auto& firstClipFace : firstClipFaces) {
					for (auto& secondClipFace : secondClipFaces) {
						std::vector<point_2t> intersectionPts;
						bg::intersection(firstClipFace, secondClipFace, intersectionPts);
						for (auto& intersectionPt : intersectionPts) {
							// firstLink
							generateForkRoadClipPoint(pNode, pFirst, intersectionPt);

							// secondLink
							generateForkRoadClipPoint(pNode, pSecond, intersectionPt);
						}
					}
				}
			}
		}
	}

	void RoadBoundaryGenerator::generateForkRoadClipPoint(DbNode* pNode, DbLink* pLink, point_2t& pos)
	{
		size_t si, ei;
		MapPoint3D64 grappedPt;
		MapPoint3D64 tmpPos = MapPoint3D64_make(pos.get<0>(), pos.get<1>(), 0);
		if (GrapPointAlgorithm::grapOrMatchNearestPoint(tmpPos, pLink->geometry.vertexes, grappedPt, si, ei)) {
			DbLink::ClipPoint clipPoint{ grappedPt, si, ei };
			auto iter = pLink->clipPoints.find(pNode->uuid);
			if (iter == pLink->clipPoints.end()) {
				std::vector<DbLink::ClipPoint> vec;
				vec.push_back(clipPoint);
				pLink->clipPoints.emplace(pNode->uuid, vec);
			}
			else {
				iter->second.push_back(clipPoint);
			}
		}
	}

	std::vector<linestring_2t> RoadBoundaryGenerator::getClipFaces(DbLink* pLink, ring_2t& nodeFace)
	{
		// 20169149 link 84208805243722729占比只有80%
		auto checkLanePaValid = [](DbLink* pLink, std::vector<DbRdLinkLanePa*>& lanePas)->bool {
			double offsetSum = 0;
			for (auto lanePa : lanePas) {
				auto relLinkPair = getRelLinkPair(pLink, lanePa);
				auto startOffset = relLinkPair.second.startOffset;
				auto endOffset = relLinkPair.second.endOffset;
				offsetSum += (endOffset - startOffset);
			}
			return offsetSum > 0.8;
		};

		LineString3d leftSide, rightSide;
		if (pLink->direct != 1) {
			auto tmpLanePas = getLanePas(pLink, pLink->lanePas, pLink->direct);
			if (checkLanePaValid(pLink, tmpLanePas)) {
				auto& lanePas = tmpLanePas;
				getLanePasBoundary(lanePas, pLink->direct, leftSide, rightSide);
			}
		}
		else {
			auto tmpRightLanePas = getLanePas(pLink, pLink->lanePas, 2);
			if (checkLanePaValid(pLink, tmpRightLanePas)) {
				LineString3d tmpLeftSide;
				auto& lanePas = tmpRightLanePas;
				getLanePasBoundary(lanePas, 2, tmpLeftSide, rightSide);
			}
			auto tmpLeftLanePas = getLanePas(pLink, pLink->lanePas, 3);
			if (checkLanePaValid(pLink, tmpLeftLanePas)) {
				LineString3d tmpLeftSide;
				auto& lanePas = tmpLeftLanePas;
				getLanePasBoundary(lanePas, 3, tmpLeftSide, leftSide);
				std::reverse(leftSide.vertexes.begin(), leftSide.vertexes.end());
			}
		}
		if (!leftSide.vertexes.empty() && !rightSide.vertexes.empty()) {
			Polygon3d polygon;
			makeLanePaPolygon(leftSide.vertexes, rightSide.vertexes, polygon);

			ring_2t tmpExpandRing;
			auto roadFaceRing = RING_2T(polygon.vertexes);
			bg::correct(roadFaceRing);
			if (getExpandPolyByOffset(roadFaceRing, 1000, tmpExpandRing)) {
				roadFaceRing = tmpExpandRing;
			}

			std::vector<ring_2t> resultRings;
			bg::intersection(roadFaceRing, nodeFace, resultRings);
			if (!resultRings.empty()) {
				std::vector<linestring_2t> resultLines;
				for (auto& resultRing : resultRings)
					resultLines.push_back(LINESTRING_2T(resultRing));
				return resultLines;
			}
		}
		return std::vector<linestring_2t>();
	}

	void RoadBoundaryGenerator::fixupForkRoadClipPoint(DbMesh* const pMesh)
	{
		auto sortClipPoint = [](DbNode* pNode, std::pair<const int64, std::vector<DbLink::ClipPoint>>& clipPoint) {
			// 裁切点由近到远排序
			std::sort(clipPoint.second.begin(), clipPoint.second.end(),
				[&](const auto& first, const auto& second)->bool {
					auto firstDis = pNode->geometry.pos.distanceSquare(first.position.pos);
					auto secondDis = pNode->geometry.pos.distanceSquare(second.position.pos);
					return firstDis < secondDis;
				});
			auto ip = std::unique(clipPoint.second.begin(), clipPoint.second.end(),
				[&](const auto& first, const auto& second)->bool {
					return mapPoint3D64_compare(first.position, second.position);
				});
			clipPoint.second.resize(std::distance(clipPoint.second.begin(), ip));
		};

		auto dropClipPoint = [&](DbNode* pNode, std::pair<const int64, std::vector<DbLink::ClipPoint>>& clipPoint) {
			auto nodePt = POINT_2T(pNode->geometry);
			while (!clipPoint.second.empty()) {
				auto& backClipPoint = clipPoint.second.back();
				auto distance = bg::distance(nodePt, POINT_2T(backClipPoint.position));
				if (distance > 8000) {
					clipPoint.second.pop_back();
				}
				else {
					break;
				}
			}
		};

		std::unordered_map<DbLink*, std::vector<linestring_2t>> clipFaces;
		std::vector<DbRecord*>& links = pMesh->query(RecordType::DB_HAD_LINK);
		for (auto hl : links) {
			DbLink* pLink = (DbLink*)hl;
			for (auto& clipPoint : pLink->clipPoints) {
				DbNode* pNode = (DbNode*)m_mesh->query(clipPoint.first, RecordType::DB_HAD_NODE);
				if (84207887605792865 == pNode->uuid)
					printf("");

				// 裁切点由近到远排序
				sortClipPoint(pNode, clipPoint);

				// 丢弃太长的裁切点
				dropClipPoint(pNode, clipPoint);

				double distance = 0;
				if (!clipPoint.second.empty()) {
					auto& backClipPoint = clipPoint.second.back();
					auto clipPt = POINT_2T(backClipPoint.position);
					distance = bg::distance(POINT_2T(pNode->geometry), clipPt);
				}
				if (distance < 2000) { // 太短的使用默认值
					auto expandNodeFace = RING_2T(pNode->geometry, 2000);
					bg::correct(expandNodeFace);

					std::vector<point_2t> intersectionPts;
					auto& linkInfo = pMesh->queryLinkInfo(pLink->uuid);
					bg::intersection(expandNodeFace, linkInfo._linkPoints2T, intersectionPts);
					for (auto& intersectionPt : intersectionPts) {
						generateForkRoadClipPoint(pNode, pLink, intersectionPt);
					}
					if (!intersectionPts.empty()) {
						sortClipPoint(pNode, clipPoint);
					}
				}
			}
		}
	}

	void RoadBoundaryGenerator::generateDirectRoadFromNode(DbNode* pNode, std::unordered_set<std::string>& visited,
		ProcessDirectRoadGroupFn processDirectRoadGroup)
	{
		std::vector<DbLink*> doubleDirectLinks;
		std::vector<DbLink*> singleDirectLinks;
		for (auto pLink : pNode->links) {
			if (pLink->direct == 1) {
				doubleDirectLinks.push_back(pLink);
			}
			else {
				singleDirectLinks.push_back(pLink);
			}
		}

		for (auto pLink : doubleDirectLinks) {
			// generate in forward direction
			DSegmentId forwardSeg = DSegmentId_getDSegmentId(pLink->uuid);
			generateDirectRoad(pLink, forwardSeg, pNode, visited, processDirectRoadGroup);

			// generate in backward direction
			DSegmentId backwardSeg = DSegmentId_getReversed(forwardSeg);
			generateDirectRoad(pLink, backwardSeg, pNode, visited, processDirectRoadGroup);
		}

		for (auto pLink : singleDirectLinks) {
			auto dSegmentId = DSegmentId_getDSegmentId(pLink->uuid);
			generateDirectRoad(pLink, dSegmentId, pNode, visited, processDirectRoadGroup);
		}
	}

	void RoadBoundaryGenerator::generateDirectRoad(DbLink* pLink, DSegmentId dsegId, DbNode* pNode, std::unordered_set<std::string>& visited,
		ProcessDirectRoadGroupFn processDirectRoadGroup)
	{
		if (84207887605792865 == pNode->uuid)
			printf("");
		auto direct = getDSegmentDirect(pLink, dsegId);
		auto lanePas = getLanePas(pLink, pLink->lanePas, direct);

		std::vector<DbSkeleton*> previousLinks, nextLinks;
		getDSegmentTopoLinks(pLink, dsegId, previousLinks, nextLinks);
		
		std::vector<DbLink*> connectedLinks = pNode->getLinksExcept(pLink);
		for (auto connectedLink : connectedLinks) {
			DbLink* currentLink = nullptr; 
			int currentDirect = -1;
			std::vector<DbRdLinkLanePa*> currentLanePas;

			DbLink* nextLink = nullptr;
			int nextDirect = -1;
			std::vector<DbRdLinkLanePa*> nextLanePas;
			
			DSegmentId connectedDSegId{ INVALID_DSEGMENT_ID };
			if (std::find(previousLinks.begin(), previousLinks.end(), connectedLink) != previousLinks.end()) {
				connectedDSegId = getPreviousDSegId(pLink, dsegId, connectedLink);
				auto connectedDirect = getDSegmentDirect(connectedLink, connectedDSegId);

				currentLink = connectedLink;
				currentDirect = connectedDirect;
				currentLanePas = getLanePas(connectedLink, connectedLink->lanePas, connectedDirect);

				nextLink = pLink;
				nextDirect = direct;
				nextLanePas = lanePas;
			}
			else if (std::find(nextLinks.begin(), nextLinks.end(), connectedLink) != nextLinks.end()) {
				connectedDSegId = getNextDSegId(pLink, dsegId, connectedLink);
				auto connectedDirect = getDSegmentDirect(connectedLink, connectedDSegId);

				currentLink = pLink;
				currentDirect = direct;
				currentLanePas = lanePas;

				nextLink = connectedLink;
				nextDirect = connectedDirect;
				nextLanePas = getLanePas(connectedLink, connectedLink->lanePas, connectedDirect);
			}
			if (connectedDSegId == INVALID_DSEGMENT_ID)
				continue;

			generateTopoDirectRoad(pNode,
				currentLink, currentDirect, currentLanePas, 
				nextLink, nextDirect, nextLanePas, visited, processDirectRoadGroup);
		}
	}

	void RoadBoundaryGenerator::generateTopoDirectRoad(DbNode* pNode,
		DbLink* pCurrent, int currentDirect, std::vector<DbRdLinkLanePa*>& currentLanePas, 
		DbLink* pNext, int nextDirect, std::vector<DbRdLinkLanePa*>& nextLanePas, std::unordered_set<std::string>& visited,
		ProcessDirectRoadGroupFn processDirectRoadGroup)
	{
		if (84208791712897823 == pCurrent->uuid && 84206794410498994 == pNext->uuid)
			printf("");
		// 进来该函数的都是有向路,方向不对直接返回
		if (currentDirect == 1 || nextDirect == 1)
			return;

		// 没有车道组的返回
		if (currentLanePas.empty() || nextLanePas.empty())
			return;

		DbRdLinkLanePa* pCurrentLastGroup = nullptr;
		DbRdLinkLanePa* pNextFirstGroup = nullptr;
		if (currentDirect == 2 && nextDirect == 2) {
			pCurrentLastGroup = *currentLanePas.rbegin();
			pNextFirstGroup = nextLanePas[0];
		}
		else if (currentDirect == 2 && nextDirect == 3) {
			pCurrentLastGroup = *currentLanePas.rbegin();
			pNextFirstGroup = *nextLanePas.rbegin();
		}
		else if (currentDirect == 3 && nextDirect == 2) {
			pCurrentLastGroup = currentLanePas[0];
			pNextFirstGroup = nextLanePas[0];
		}
		else if (currentDirect == 3 && nextDirect == 3) {
			pCurrentLastGroup = currentLanePas[0];
			pNextFirstGroup = *nextLanePas.rbegin();
		}

		if (pCurrentLastGroup != nullptr && pNextFirstGroup != nullptr) {
			auto getTopoIndex = [](int64 currentLgId, int64 nextLgId) {
				return std::to_string(currentLgId) + "_" + std::to_string(nextLgId);};
			auto topoIndex = getTopoIndex(pCurrentLastGroup->uuid, pNextFirstGroup->uuid);
			if (visited.count(topoIndex) == 0) {
				((*this).*processDirectRoadGroup)(pNode,
					pCurrent, currentDirect, pCurrentLastGroup,
					pNext, nextDirect, pNextFirstGroup);
				visited.insert(topoIndex);
			}
		}
	}

	void RoadBoundaryGenerator::generateDirectStraightRoadGroup(DbNode* pNode, 
		DbLink* pCurrent, int currentDirect, DbRdLinkLanePa* pCurrentLanePa, 
		DbLink* pNext, int nextDirect, DbRdLinkLanePa* pNextLanePa)
	{
		// 进来该函数的都是有向路,方向不对直接返回
		if (currentDirect == 1 || nextDirect == 1)
			return;

		// 道路边界数不对返回
		if (pCurrentLanePa->roadBoundaries.size() != 2 || pNextLanePa->roadBoundaries.size() != 2)
			return;

		if (pCurrentLanePa->straightRoadGeometryTopoHandled)
			return;
		else
		{
			pCurrentLanePa->straightRoadGeometryTopoHandled = true;
			pNextLanePa->straightRoadGeometryTopoHandled = true;
		}

		// 从起点或者终点截取line顶点的一部分，使得该部分顶点的长度至少大于dist，若line的长度小于dist，返回完整的line
		auto Subline = [](const LineString3d& line, double minDist, bool fromBegin) -> LineString3d
			{
				LineString3d subline;
				double distAcc{ 0.0 };

				const std::size_t iInit = fromBegin ? 0 : line.vertexes.size() - 1;
				const std::size_t iStep = fromBegin ? 1 : -1;

				for (std::size_t i = iInit; distAcc < minDist && (i + iStep) < line.vertexes.size(); i += iStep)
				{
					MapPoint3D64 point = line.vertexes[i];
					MapPoint3D64 nextPoint = line.vertexes[i + iStep];
					distAcc += MapPoint64::geodistance(point.pos, nextPoint.pos);

					if (i == iInit)
						subline.vertexes.push_back(point);
					subline.vertexes.push_back(nextPoint);
				}

				return subline;
			};

		bool curLinkBeginConnecteToNode = pCurrent->startNode == pNode->uuid;
		bool nextLinkBeginConnectToNode = pNext->startNode == pNode->uuid;

		// 调整左侧连接处顶点坐标
		auto adjustConnectdBound = [&Subline, &pCurrent, &pNext, &pNode, curLinkBeginConnecteToNode, nextLinkBeginConnectToNode]
		(DbRoadBoundLink* curBound, DbRoadBoundLink* nextBound, double offsetSign)
			{
				UNREFERENCED_PARAMETER(offsetSign);

				constexpr int BIT_CUR{ 1 };
				constexpr int BIT_NEXT{ 0 };

				// connected表示link起点连接到node，disconnected不连接
				constexpr int CONNECTED{ 1 };
				constexpr int DISCONNECTED{ 0 };

				constexpr int CUR_CONNECTED = CONNECTED << BIT_CUR;
				constexpr int CUR_DISCONNECTED = DISCONNECTED << BIT_CUR;
				constexpr int NEXT_CONNECTED = CONNECTED << BIT_NEXT;
				constexpr int NEXT_DISCONNECTED = DISCONNECTED << BIT_NEXT;

				const int actionCode = (static_cast<int>(curLinkBeginConnecteToNode) << BIT_CUR) | (static_cast<int>(nextLinkBeginConnectToNode) << BIT_NEXT);
				switch (actionCode)
				{
				case CUR_CONNECTED | NEXT_CONNECTED:
					curBound->starRoadBoundNodeId = nextBound->starRoadBoundNodeId;
					break;
				case CUR_CONNECTED | NEXT_DISCONNECTED:
					curBound->starRoadBoundNodeId = nextBound->endRoadBoundNodeId;
					break;
				case CUR_DISCONNECTED | NEXT_CONNECTED:
					curBound->endRoadBoundNodeId = nextBound->starRoadBoundNodeId;
					break;
				case CUR_DISCONNECTED | NEXT_DISCONNECTED:
					curBound->endRoadBoundNodeId = nextBound->endRoadBoundNodeId;
					break;
				default:
					break;
					// impossible case
				}

				if (curBound->geometryOffset != 0.0 && nextBound->geometryOffset != 0.0)
				{
					LineString3d fullLine;
					std::size_t fullLineNodeVertexIndex = -1;
					{
						const double sublineMinDist{ 30.0 };
						LineString3d curSubline = Subline(pCurrent->geometry, sublineMinDist, curLinkBeginConnecteToNode);
						LineString3d nextSubline = Subline(pNext->geometry, sublineMinDist, nextLinkBeginConnectToNode);

						// TODO: 规定几何检查规则
						if (curSubline.vertexes.empty() || nextSubline.vertexes.empty())
							return; // 不应该出现该种情况

						fullLineNodeVertexIndex = curSubline.vertexes.size() - 1;
						fullLine = std::move(curSubline);
						std::reverse(fullLine.vertexes.begin(), fullLine.vertexes.end());
						fullLine.vertexes.insert(fullLine.vertexes.end(), nextSubline.vertexes.begin() + 1, nextSubline.vertexes.end());
					}

					double avgOffset = offsetSign * (std::abs(curBound->geometryOffset) + std::abs(nextBound->geometryOffset)) * 0.5;
					LineString3d bufferLine = adjustBoundaryOffset(fullLine, avgOffset);
					assert(bufferLine.vertexes.size() == fullLine.vertexes.size());
					MapPoint3D64 bufferLineNodeVertex = bufferLine.vertexes[fullLineNodeVertexIndex];

					switch (actionCode)
					{
					case CUR_CONNECTED | NEXT_CONNECTED:
						curBound->geometry.vertexes.front() = bufferLineNodeVertex;
						nextBound->geometry.vertexes.front() = bufferLineNodeVertex;
						break;
					case CUR_CONNECTED | NEXT_DISCONNECTED:
						curBound->geometry.vertexes.front() = bufferLineNodeVertex;
						nextBound->geometry.vertexes.back() = bufferLineNodeVertex;
						break;
					case CUR_DISCONNECTED | NEXT_CONNECTED:
						curBound->geometry.vertexes.back() = bufferLineNodeVertex;
						nextBound->geometry.vertexes.front() = bufferLineNodeVertex;
						break;
					case CUR_DISCONNECTED | NEXT_DISCONNECTED:
						curBound->geometry.vertexes.back() = bufferLineNodeVertex;
						nextBound->geometry.vertexes.back() = bufferLineNodeVertex;
						break;
					default:
						break;
						// impossible case
					}
				}
				else if (curBound->geometryOffset != 0.0 || nextBound->geometryOffset != 0.0)
				{
					switch (actionCode)
					{
					case CUR_CONNECTED | NEXT_CONNECTED:
						generateStraightRoadCurvePoints(curBound->geometry.vertexes, nextBound->geometry.vertexes, pNode->geometry);
						break;
					case CUR_CONNECTED | NEXT_DISCONNECTED:
						std::reverse(nextBound->geometry.vertexes.begin(), nextBound->geometry.vertexes.end());
						generateStraightRoadCurvePoints(curBound->geometry.vertexes, nextBound->geometry.vertexes, pNode->geometry);
						std::reverse(nextBound->geometry.vertexes.begin(), nextBound->geometry.vertexes.end());

						break;
					case CUR_DISCONNECTED | NEXT_CONNECTED:
						std::reverse(curBound->geometry.vertexes.begin(), curBound->geometry.vertexes.end());
						generateStraightRoadCurvePoints(curBound->geometry.vertexes, nextBound->geometry.vertexes, pNode->geometry);
						std::reverse(curBound->geometry.vertexes.begin(), curBound->geometry.vertexes.end());

						break;
					case CUR_DISCONNECTED | NEXT_DISCONNECTED:
						std::reverse(curBound->geometry.vertexes.begin(), curBound->geometry.vertexes.end());
						std::reverse(nextBound->geometry.vertexes.begin(), nextBound->geometry.vertexes.end());
						generateStraightRoadCurvePoints(curBound->geometry.vertexes, nextBound->geometry.vertexes, pNode->geometry);
						std::reverse(curBound->geometry.vertexes.begin(), curBound->geometry.vertexes.end());
						std::reverse(nextBound->geometry.vertexes.begin(), nextBound->geometry.vertexes.end());

						break;
					default:
						break;
						// impossible case
					}
				}
				else
				{
					// 无需处理，车道组边界与Link边界相同，故而能确保curBound->geometry与nextBound->geometry连续。
				}

			};

		// 调整左侧
		adjustConnectdBound(pCurrentLanePa->roadBoundaries[0], pNextLanePa->roadBoundaries[0], -1.0);
		// 调整右侧
		adjustConnectdBound(pCurrentLanePa->roadBoundaries[1], pNextLanePa->roadBoundaries[1], 1.0);
	}

	void RoadBoundaryGenerator::generateDirectForkRoadGroup(DbNode* pNode,
		DbLink* pCurrent, int currentDirect, DbRdLinkLanePa* pCurrentLanePa,
		DbLink* pNext, int nextDirect, DbRdLinkLanePa* pNextLanePa)
	{
		// 进来该函数的都是有向路,方向不对直接返回
		if (currentDirect == 1 || nextDirect == 1)
			return;

		// 道路边界数不对返回
		if (pCurrentLanePa->roadBoundaries.size() != 2 || pNextLanePa->roadBoundaries.size() != 2)
			return;

		auto generateLanePa = [&](DbLink* pLink, DbRdLinkLanePa* pLanePa)->DbRdLinkLanePa* {
			DbRdLinkLanePa* pNewLanePa = allocForkRoadLanePa(pLanePa);
			auto iter = pLanePa->generatedLanePas.find(pNode->uuid);
			if (iter == pLanePa->generatedLanePas.end()) {
				std::vector<DbRdLinkLanePa*> vec;
				vec.push_back(pNewLanePa);
				pLanePa->generatedLanePas.emplace(pNode->uuid, vec);
			}
			else {
				iter->second.push_back(pNewLanePa);
			}

			int startOffset = 0; 
			int endOffset = 0;
			auto relLinkPair = getRelLinkPair(pLink, pLanePa);
			if (pLanePa == pCurrentLanePa) {
				if (currentDirect == 2) {
					startOffset = relLinkPair.second.endOffset;
					endOffset = 1;
				}
				else if (currentDirect == 3) {
					startOffset = 0;
					endOffset = relLinkPair.second.startOffset;
				}
			}
			else if (pLanePa == pNextLanePa) {
				if (nextDirect == 2) {
					startOffset = 0;
					endOffset = relLinkPair.second.startOffset;
				}
				else if (nextDirect == 3) {
					startOffset = relLinkPair.second.endOffset;
					endOffset = 1;
				}
			}

			DbRdLinkLanePa::DbRelLink relLink;
			relLink.relLinkid = relLinkPair.second.relLinkid;
			relLink.startOffset = startOffset;
			relLink.endOffset = endOffset;
			relLink.directType = relLinkPair.second.directType;
			pNewLanePa->relLinks.emplace(pNewLanePa->uuid, relLink);
			// 暂时只在新生成的车道组保存拓扑关系,后面会执行车道组选择
			pNewLanePa->previous.push_back(pCurrentLanePa);
			pNewLanePa->next.push_back(pNextLanePa);
			return pNewLanePa;
		};

		// clip current lanePa and update node position
		if (pCurrent->clipPoints.find(pNode->uuid) != pCurrent->clipPoints.end()) {
			clipForkRoadGroup(pNode, pCurrent, pCurrentLanePa);
		}

		// clip next lanePa and update node position
		if (pNext->clipPoints.find(pNode->uuid) != pNext->clipPoints.end()) {
			clipForkRoadGroup(pNode, pNext, pNextLanePa);
		}

		// 当前拓扑关系
		auto currentDSegId = getDirectDSegment(pCurrent, currentDirect);
		std::vector<DbSkeleton*> currentPreviousLinks, currentNextLinks;
		getDSegmentTopoLinks(pCurrent, currentDSegId, currentPreviousLinks, currentNextLinks);

		// 下一个拓扑关系
		auto nextDSegId = getDirectDSegment(pNext, nextDirect);
		std::vector<DbSkeleton*> nextPreviousLinks, nextNextLinks;
		getDSegmentTopoLinks(pNext, nextDSegId, nextPreviousLinks, nextNextLinks);

		// 左边
		auto& currentLeftBoundary = pCurrentLanePa->roadBoundaries[0];
		auto& currentRightBoundary = pCurrentLanePa->roadBoundaries[1];
		LineString3d currentLeftSide, currentRightSide;
		getLanePaBoundary(pCurrentLanePa, currentLeftSide, currentRightSide);

		// 右边
		auto& nextLeftBoundary = pNextLanePa->roadBoundaries[0];
		auto& nextRightBoundary = pNextLanePa->roadBoundaries[1];
		LineString3d nextLeftSide, nextRightSide;
		getLanePaBoundary(pNextLanePa, nextLeftSide, nextRightSide);

		// 计算通过当前车道组角度
		auto lanePaAngle = calcLanePaAngle(currentRightSide, nextRightSide);

		// 生成边界线
		LineString3d leftSide, rightSide;
		generateLanePaBoundary(lanePaAngle, currentLeftSide.vertexes, nextLeftSide.vertexes, leftSide.vertexes);
		generateLanePaBoundary(lanePaAngle, currentRightSide.vertexes, nextRightSide.vertexes, rightSide.vertexes);

		std::vector<int64> currentLeftNodes, currentRightNodes;
		getLanePaBoundaryNodes(pCurrentLanePa, currentLeftNodes, currentRightNodes);

		std::vector<int64> nextLeftNodes, nextRightNodes;
		getLanePaBoundaryNodes(pNextLanePa, nextLeftNodes, nextRightNodes);

		// 保存生成的车道组在current车道组尾部
		auto storeToCurrentLanePa = [&](int angle)->DbRdLinkLanePa* {
			DbRdLinkLanePa* pLanePa = nullptr;
			if (pCurrent->owner == m_mesh) {
				pLanePa = generateLanePa(pCurrent, pCurrentLanePa);
				auto leftStartNode = currentLeftNodes.back();
				auto leftEndNode = nextLeftNodes.front();
				if (directionEqual(currentLeftBoundary, pCurrentLanePa, 3)) {
					std::reverse(leftSide.vertexes.begin(), leftSide.vertexes.end());
					std::swap(leftStartNode, leftEndNode);
				}
				generateForkRoadBoundary(pCurrent, currentDirect, pLanePa, leftSide, leftStartNode, leftEndNode, 2);

				auto rightStartNode = currentRightNodes.back();
				auto rightEndNode = nextRightNodes.front();
				if (directionEqual(currentRightBoundary, pCurrentLanePa, 3)) {
					std::reverse(rightSide.vertexes.begin(), rightSide.vertexes.end());
					std::swap(rightStartNode, rightEndNode);
				}
				generateForkRoadBoundary(pCurrent, currentDirect, pLanePa, rightSide, rightStartNode, rightEndNode, 1);

				// 生成车道组
				pLanePa->lanePaAngle = angle;
				GroupGenerator::generateGroup(m_mesh, pCurrent, pLanePa);
			}
			return pLanePa;
		};

		// 保存生成的车道组在next车道组首部
		auto storeToNextLanePa = [&](int angle)->DbRdLinkLanePa* {
			DbRdLinkLanePa* pLanePa = nullptr;
			if (pNext->owner == m_mesh) {
				pLanePa = generateLanePa(pNext, pNextLanePa);
				auto leftStartNode = currentLeftNodes.back();
				auto leftEndNode = nextLeftNodes.front();
				if (directionEqual(nextLeftBoundary, pNextLanePa, 3)) {
					std::reverse(leftSide.vertexes.begin(), leftSide.vertexes.end());
					std::swap(leftStartNode, leftEndNode);
				}
				generateForkRoadBoundary(pNext, nextDirect, pLanePa, leftSide, leftStartNode, leftEndNode, 2);

				auto rightStartNode = currentRightNodes.back();
				auto rightEndNode = nextRightNodes.front();
				if (directionEqual(nextRightBoundary, pNextLanePa, 3)) {
					std::reverse(rightSide.vertexes.begin(), rightSide.vertexes.end());
					std::swap(rightStartNode, rightEndNode);
				}
				generateForkRoadBoundary(pNext, nextDirect, pLanePa, rightSide, rightStartNode, rightEndNode, 1);

				// 生成车道组
				pLanePa->lanePaAngle = angle;
				GroupGenerator::generateGroup(m_mesh, pNext, pLanePa);
			}
			return pLanePa;
		};

		// 生成车道组并保存拓扑关系
		DbRdLinkLanePa* pLanePa = nullptr;
		if (nextPreviousLinks.size() == 1) {
			pLanePa = storeToNextLanePa(lanePaAngle);
		}
		else if (currentNextLinks.size() == 1 || std::abs(lanePaAngle) < 30) {
			pLanePa = storeToCurrentLanePa(lanePaAngle);
		}
		else {
			pLanePa = storeToNextLanePa(lanePaAngle);
		}

		// TODO,此处是否要保存?
		if (pLanePa != nullptr) {
			saveLanePaInfo(pLanePa);
		}
	}

	DbRdLinkLanePa* RoadBoundaryGenerator::allocForkRoadLanePa(DbRdLinkLanePa* pLanePa)
	{
		auto id = pLanePa->uuid << 1;
		auto index = pLanePa->getGeneratedLanePaCnt() / 2;
		if (index && pLanePa->getGeneratedLanePaCnt() == (index << 1)) {
			// TODO 此处可能会溢出,后续可能需要加全局ID列表排重
			pLanePa->generatedLanePaIdx = 0;
			id = pLanePa->uuid << (index << 1);
		}
		id += pLanePa->generatedLanePaIdx++;
		DbRdLinkLanePa* pNewLanePa = (DbRdLinkLanePa*)m_mesh->query(id, RecordType::DB_RD_LINK_LANEPA);
		if (pNewLanePa == nullptr)
		{
			pNewLanePa = (DbRdLinkLanePa*)m_mesh->alloc(RecordType::DB_RD_LINK_LANEPA);
			pNewLanePa->uuid = id;
			pNewLanePa->isGenerated = true;
			m_mesh->insert(pNewLanePa->uuid, pNewLanePa);
		}
		return pNewLanePa;
	}

	void RoadBoundaryGenerator::generateForkRoadBoundary(DbLink* pLink, int direct, 
		DbRdLinkLanePa* pLanePa, LineString3d& geometry, int64 startId, int64 endId, int side)
	{
		auto currRelLinkPair = getRelLinkPair(pLink, pLanePa);
		int64 rdBoundSeed = (currRelLinkPair.first << 1) | (side - 1);
		int64 rdNodeSeed = (rdBoundSeed << 1) | (side - 1);

		// boundary
		int64 id = currRelLinkPair.first + (rdBoundSeed++);
		DbRoadBoundLink* pBoundary = (DbRoadBoundLink*)m_mesh->query(id, RecordType::DB_HAD_ROAD_BOUNDARY_LINK);
		if (pBoundary == nullptr)
		{
			pBoundary = (DbRoadBoundLink*)m_mesh->alloc(RecordType::DB_HAD_ROAD_BOUNDARY_LINK);
			pBoundary->uuid = id;
			m_mesh->insert(pBoundary->uuid, pBoundary);

			pBoundary->boundaryType = 0; // TODO
			pBoundary->geometry = geometry;

			// nodeId
			pBoundary->starRoadBoundNodeId = startId;
			pBoundary->endRoadBoundNodeId = endId;

			// relLg
			DbRoadBoundLink::DbLgRoadBoundREL relLg;
			relLg.side = side;
			relLg.direction = direct;
			pBoundary->relLgs.emplace(currRelLinkPair.first, relLg);
			pLanePa->roadBoundaries.push_back(pBoundary);
		}
	}

	void RoadBoundaryGenerator::getLanePaBoundaryNodes(DbRdLinkLanePa* lanePa, std::vector<int64>& leftSide, std::vector<int64>& rightSide)
	{
		leftSide.clear(); rightSide.clear();
		leftSide.push_back(lanePa->roadBoundaries[0]->starRoadBoundNodeId);
		leftSide.push_back(lanePa->roadBoundaries[0]->endRoadBoundNodeId);
		if (directionEqual(lanePa->roadBoundaries[0], lanePa, 3))
		{
			std::reverse(leftSide.begin(), leftSide.end());
		}
		rightSide.push_back(lanePa->roadBoundaries[1]->starRoadBoundNodeId);
		rightSide.push_back(lanePa->roadBoundaries[1]->endRoadBoundNodeId);
		if (directionEqual(lanePa->roadBoundaries[1], lanePa, 3))
		{
			// 反向，进行点的反转
			std::reverse(rightSide.begin(), rightSide.end());
		}
	}

	int RoadBoundaryGenerator::calcLanePaAngle(LineString3d& currentRightSide, LineString3d& nextRightSide)
	{
		const double SLICE_VERTEXES_DISTANCE = 3000;
		auto getEndVertexes = [&](const std::vector<MapPoint3D64>& points)->std::vector<MapPoint3D64> {
			std::vector<MapPoint3D64> vertexes;
			double distance = 0;
			for (int i = (int)points.size() - 1; i >= 0; i--) {
				auto& point = points[i];
				if (i == (int)points.size() - 1) {
					vertexes.push_back(point);
					continue;
				}
				auto& frontPt = vertexes.front();
				vertexes.insert(vertexes.begin(), point);
				distance += frontPt.pos.distance(point.pos);
				if (distance >= SLICE_VERTEXES_DISTANCE)
					break;
			}
			return vertexes;
		};

		auto getStartVertexes = [&](const std::vector<MapPoint3D64>& points)->std::vector<MapPoint3D64> {
			std::vector<MapPoint3D64> vertexes;
			double distance = 0;
			for (size_t i = 0; i < points.size(); i++) {
				auto& point = points[i];
				if (i == 0) {
					vertexes.push_back(point);
					continue;
				}
				auto& backPt = vertexes.back();
				vertexes.push_back(point);
				distance += backPt.pos.distance(point.pos);
				if (distance >= SLICE_VERTEXES_DISTANCE)
					break;
			}
			return vertexes;
		};

		auto currentRightVertexes = getEndVertexes(currentRightSide.vertexes);
		auto nextRightVertexes = getStartVertexes(nextRightSide.vertexes);

		auto currentRightVector = getBoundaryVector(currentRightVertexes);
		auto nextRightVector = getBoundaryVector(nextRightVertexes);
		int angle = minimalDegree(POINT_2T(currentRightVector), POINT_2T(nextRightVector));

		coordinatesTransform.convert(nextRightVertexes.data(), nextRightVertexes.size());
		coordinatesTransform.convert(currentRightVertexes.data(), currentRightVertexes.size());
		if (GrapPointAlgorithm::isOnLeft(nextRightVertexes.back(), currentRightVertexes.data(), currentRightVertexes.size())) {
			angle = -angle;
		}
		return angle;
	}

	void RoadBoundaryGenerator::clipForkRoadGroup(DbNode* pNode, DbLink* pLink, DbRdLinkLanePa* lanePa)
	{
		auto getVerticalSeg = [](point_2t& firstPt, point_2t& secondPt)->segment_2t {
			auto _p1 = P2_V2(firstPt);
			auto _p2 = P2_V2(secondPt);
			double _l1 = bg::distance(_p2, _p1);

			vector_2t _v1 = _p2;
			bg::subtract_point(_v1, _p1);
			bg::divide_value(_v1, _l1);

			vector_2t _v2(_v1.get<1>(), -_v1.get<0>());
			point_2t _p3 = secondPt;
			bg::multiply_value(_v2, 10000);
			bg::add_point(_p3, V2_P2(_v2));

			vector_2t _v3(-_v1.get<1>(), _v1.get<0>());
			point_2t _p4 = secondPt;
			bg::multiply_value(_v3, 10000);
			bg::add_point(_p4, V2_P2(_v3));

			segment_2t seg2(_p3, _p4);
			return seg2;
		};

		auto getClipPolygon = [](segment_2t& firstSeg, segment_2t& secondSeg)->Polygon3d {
			Polygon3d polygon;
			polygon.vertexes.push_back(MapPoint3D64_make(firstSeg.first.get<0>(), firstSeg.first.get<1>(), 0));
			polygon.vertexes.push_back(MapPoint3D64_make(firstSeg.second.get<0>(), firstSeg.second.get<1>(), 0));
			polygon.vertexes.push_back(MapPoint3D64_make(secondSeg.first.get<0>(), secondSeg.first.get<1>(), 0));
			polygon.vertexes.push_back(MapPoint3D64_make(secondSeg.second.get<0>(), secondSeg.second.get<1>(), 0));
			polygon.vertexes.push_back(polygon.vertexes.front());
			return polygon;
		};

		auto clipPointIter = pLink->clipPoints.find(pNode->uuid);
		auto& backClipPoint = clipPointIter->second.back();

		auto nodePt = POINT_2T(pNode->geometry);
		auto clipPt = POINT_2T(backClipPoint.position);
		auto distance = bg::distance(nodePt, clipPt);

		segment_2t clipNodeSeg{ clipPt, nodePt };
		clipNodeSeg = SEGMENT_2T_EX_FRONT(clipNodeSeg, 300);
		auto clipNodeVerSeg = getVerticalSeg(clipPt, clipNodeSeg.second);
		auto nodeClipVerSeg = getVerticalSeg(nodePt, clipPt);
		Polygon3d clipPolygon = getClipPolygon(nodeClipVerSeg, clipNodeVerSeg);

		static const double tolerance = 10;
		for (auto pBoundary : lanePa->roadBoundaries) {
			auto outPts = pBoundary->geometry.vertexes;
			bool containsIntersectPoint = false;
			for (size_t i = 0; i < outPts.size() - 1; i++) {
				auto& startPt = outPts[i];
				auto& endPt = outPts[i + 1];
				std::vector<MapPoint3D64> intersectPoints;
				if (PolylineIntersector::intersect(startPt, endPt, clipPolygon.vertexes, tolerance, intersectPoints)) {
					auto pStart = POINT_T(startPt);
					auto pEnd = POINT_T(endPt);
					segment_t segment{ pStart, pEnd };
					vector_t pabVec = V3_N(S3_V3(segment));
					std::sort(intersectPoints.begin(), intersectPoints.end(), [&](MapPoint3D64& pa, MapPoint3D64& pb) {
						vector_t tmp0V = S3_V3(pStart, POINT_T(pa));
						auto v0Sum = bg::dot_product(pabVec, tmp0V);

						vector_t tmp1V = S3_V3(pStart, POINT_T(pb));
						auto v1Sum = bg::dot_product(pabVec, tmp1V);
						return v0Sum < v1Sum;
						});
					auto insertIter = mapPoint3D64_iterator(outPts, endPt);
					for (auto& intersectPoint : intersectPoints) {
						containsIntersectPoint = true;
						auto intersectPt = POINT_2T(intersectPoint);
						if (isEqualPoint2T(P3_P2(pStart), intersectPt) || isEqualPoint2T(P3_P2(pEnd), intersectPt)) {
							continue;
						}
						insertIter = outPts.insert(insertIter, intersectPoint);
						insertIter++;// 指向插入点下一个位置
					}
				}
			}
			if (!containsIntersectPoint) {
				continue;
			}

			// clip
			for (auto iter = outPts.begin(); iter != outPts.end(); ) {
				bool pointInPolygon = false;
				if (GrapPointAlgorithm::isPointInPolygon(*iter, clipPolygon.vertexes)) {
					pointInPolygon = true;
				}
				size_t si, ei;
				MapPoint3D64 minGrappedPt = {};
				GrapPointAlgorithm::grapOrMatchNearestPoint(*iter, clipPolygon.vertexes, minGrappedPt, si, ei);
				if (minGrappedPt.pos.distance(iter->pos) < 50) {
					pointInPolygon = false;
				}

				if (pointInPolygon) {
					iter = outPts.erase(iter);
					continue;
				}
				iter++; 
			}

			// 边界在裁切面内
			if (outPts.size() <= 1) {
				lanePa->inClipRing = true;
				continue;
			}
			
			// 不在裁切面内时更新数据
			pBoundary->geometry.vertexes = outPts;
			auto& geometryVertexes = pBoundary->geometry.vertexes;

			auto startNode = (DbRoadBoundNode*)m_mesh->query(pBoundary->starRoadBoundNodeId, RecordType::DB_HAD_ROAD_BOUNDARY_NODE);
			if (startNode == nullptr) {
				startNode = (DbRoadBoundNode*)queryNearby(m_mesh, m_nearbyMesh, pBoundary->starRoadBoundNodeId, RecordType::DB_HAD_ROAD_BOUNDARY_NODE);
			}
			auto endNode = (DbRoadBoundNode*)m_mesh->query(pBoundary->endRoadBoundNodeId, RecordType::DB_HAD_ROAD_BOUNDARY_NODE);
			if (endNode == nullptr) {
				endNode = (DbRoadBoundNode*)queryNearby(m_mesh, m_nearbyMesh, pBoundary->endRoadBoundNodeId, RecordType::DB_HAD_ROAD_BOUNDARY_NODE);
			}
			auto startNodeDistance = pNode->geometry.pos.distanceSquare(startNode->geometry.pos);
			auto endNodeDistance = pNode->geometry.pos.distanceSquare(endNode->geometry.pos);
			auto updateNode = (startNodeDistance < endNodeDistance) ? startNode : endNode;

			auto frontPtDistance = pNode->geometry.pos.distanceSquare(geometryVertexes.front().pos);
			auto backPtDistance = pNode->geometry.pos.distanceSquare(geometryVertexes.back().pos);
			auto& updatePt = (frontPtDistance < backPtDistance) ? geometryVertexes.front() : geometryVertexes.back();
			updateNode->geometry = updatePt;
		}
		
	}

	/**
	 * @brief 上下行道路的一侧path能够保证相邻的link的几何首尾连接，但搜索时没有保证一定能通行过去，按理说能够通行。如果出现上下行道路的一侧不能够通行则不处理，后续应当调查该种情况
	 * @return true -> 能够通行，false -> 不能够通行
	 */
	template <typename Iterator>
	bool RoadBoundaryGenerator::checkPathCanTravel(Iterator begin, Iterator end)
	{
		using PathRevIterator = std::vector < std::pair<DbLink*, bool>>::reverse_iterator;

		if (begin == end)
			return false;
		if (begin + 1 == end)
		{
			//
			std::pair<DbLink*, bool> link = *begin;
			if (std::is_same<Iterator, PathRevIterator>())
				link.second = !link.second;

			switch (link.first->direct)
			{
			case 2: return link.second;
			case 3: return !link.second;
			default: return false;
			}
		}

		for (auto it = begin; it + 1 != end; ++it)
		{
			std::pair<DbLink*, bool> thisLink = *it;
			std::pair<DbLink*, bool> nextLink = *(it + 1);
			if (std::is_same<Iterator, PathRevIterator>())
			{
				thisLink.second = !thisLink.second;
				nextLink.second = !nextLink.second;
			}

			if (!checkLink1CanTravelToLink2(thisLink, nextLink))
				return false;
		}
		return true;
	}

	/**
	 * @brief 从可连续通行的path读取Link右侧的车道组，若Link为单向通行，则使用该车道组。
	 */
	template <typename Iterator>
	std::vector<std::vector<DbRdLinkLanePa*>> RoadBoundaryGenerator::findPathRightSidePas(Iterator begin, Iterator end)
	{
		using PathRevIterator = std::vector < std::pair<DbLink*, bool>>::reverse_iterator;

		auto findRightSidePas = [this](std::pair<DbLink*, bool>& link) -> std::vector<DbRdLinkLanePa*>
			{
				switch (link.first->direct)
				{
				case 1:
					return link.second ? this->getLanePas(link.first, link.first->lanePas, 2) : this->getLanePas(link.first, link.first->lanePas, 3);
				case 2:
				case 3:
				default:
					return this->getLanePas(link.first, link.first->lanePas, link.first->direct);
				}
			};

		std::vector<std::vector<DbRdLinkLanePa*>> result;
		for (auto it = begin; it != end; ++it)
		{
			std::pair<DbLink*, bool> link = *it;
			if (std::is_same<Iterator, PathRevIterator>())
				link.second = !link.second;

			std::vector<DbRdLinkLanePa*> pas = findRightSidePas(link);
			result.push_back(pas);
		}

		return result;
	}

	void RoadBoundaryGenerator::findDividedRoadRightSidePas(std::array<std::vector<std::pair<DbLink*, bool>>, 2>& dividedRoad, 
		std::array<std::vector<std::vector<DbRdLinkLanePa*>>, 2>& outDividedRoadPas, int& outCommonBoundaryIndex)
	{
		using PathIterator = std::vector < std::pair<DbLink*, bool>>::iterator;
		using PathRevIterator = std::vector < std::pair<DbLink*, bool>>::reverse_iterator;

		auto& leftPath = dividedRoad[0];
		auto& rightPath = dividedRoad[1];

		// 左侧path正向通行，右侧道路逆向通行
		bool leftForwardRightBackward = checkPathCanTravel(leftPath.begin(), leftPath.end())
			&& checkPathCanTravel(rightPath.rbegin(), rightPath.rend());
		bool leftBackwardRightForward = checkPathCanTravel(leftPath.rbegin(), leftPath.rend())
			&& checkPathCanTravel(rightPath.begin(), rightPath.end());

		// Note: outDividedRoadPas左右侧pa按照divided road中的link排序

		if (leftForwardRightBackward && leftBackwardRightForward)
		{
			; // 不应该出现，不然上下行查找可能有误。
		}
		else if (leftForwardRightBackward)
		{
			outDividedRoadPas[0] = findPathRightSidePas(leftPath.begin(), leftPath.end());
			outDividedRoadPas[1] = findPathRightSidePas(rightPath.rbegin(), rightPath.rend());
			std::reverse(outDividedRoadPas[1].begin(), outDividedRoadPas[1].end());

			outCommonBoundaryIndex = 1;
		}
		else if (leftBackwardRightForward)
		{
			outDividedRoadPas[0] = findPathRightSidePas(leftPath.rbegin(), leftPath.rend());
			std::reverse(outDividedRoadPas[0].begin(), outDividedRoadPas[0].end());
			outDividedRoadPas[1] = findPathRightSidePas(rightPath.begin(), rightPath.end());

			outCommonBoundaryIndex = 0;
		}
		else
		{
			; // 不应该出现，不然上下行查找可能有误。
		}
	}

	void RoadBoundaryGenerator::generateDividedRoadCommonBoundary(std::size_t index, std::array<std::vector<std::pair<DbLink*, bool>>, 2>& dividedRoad)
	{
		UNREFERENCED_PARAMETER(index);
		UNREFERENCED_PARAMETER(dividedRoad);

		std::array<std::vector<std::vector<DbRdLinkLanePa*>>, 2> dividedRoadPa;
		int commonBoundaryIndex{ -1 };
		findDividedRoadRightSidePas(dividedRoad, dividedRoadPa, commonBoundaryIndex);

		if (commonBoundaryIndex == -1)
			return; // 不应该出现，否则上下行查找结果有误

		// 上下行道路顺方向的边界偏移量
		std::array<std::vector<double>, 2> dividedRoadOffset;
		auto genPathPaOffsetOrdered = [commonBoundaryIndex](std::vector<std::pair<DbLink*, bool>>& path, std::vector<std::vector<DbRdLinkLanePa*>>& pathPa)
			{
				std::vector<double> offsets;
				auto appendPaOffsets = [&offsets, commonBoundaryIndex](const DbRdLinkLanePa* pa)
					{
						offsets.push_back(pa->roadBoundaries[commonBoundaryIndex]->geometryOffset);
					};
				for (std::size_t i = 0; i < path.size(); ++i)
				{
					std::pair<DbLink*, bool>& pathPortion = path[i];
					std::vector<DbRdLinkLanePa*>& linkPas = pathPa[i];
					if (pathPortion.second)
						std::for_each(linkPas.begin(), linkPas.end(), appendPaOffsets);
					else
						std::for_each(linkPas.rbegin(), linkPas.rend(), appendPaOffsets);
				}

				return offsets;
			};
		dividedRoadOffset[0] = genPathPaOffsetOrdered(dividedRoad[0], dividedRoadPa[0]);
		dividedRoadOffset[1] = genPathPaOffsetOrdered(dividedRoad[1], dividedRoadPa[1]);

		struct PathPa
		{
			DbLink* link;
			bool linkForward;
			DbRdLinkLanePa* linkPa;
			std::size_t indexFrom; // pa在线上的索引范围：[indexFrom, indexTo]
			std::size_t indexTo;
		};

		std::vector<std::pair<DbLink*, bool>>& leftPath = dividedRoad[0], & rightPath = dividedRoad[1];
		std::vector<std::vector<DbRdLinkLanePa*>>& leftLinksPa = dividedRoadPa[0], & rightLinksPa = dividedRoadPa[1];
		std::vector<MapPoint3D64> leftPoints, rightPoints;
		std::vector<PathPa> leftPathPas, rightPathPas;
		{
			auto calLinestringLength = [](const std::vector<MapPoint3D64>& linestring)
				{
					double len = 0.0;
					for (std::size_t i = 0; i + 1 < linestring.size(); ++i)
					{
						auto& point = linestring[i];
						auto& nextPoint = linestring[i + 1];
						len += double(point.pos.distance(nextPoint.pos));
					}
					return len;
				};
			// 在linestring与起点为指定距离的位置上插入点，计算该点的坐标以及在该在哪个位置上插入该点。
			// pointLenFromStart: 待插入的点到linestring起点的距离。outInsertPoint：linestring在指定距离位置的点，outInsertPosition：在该位置插入点能保持线连续。
			// 返回值：true -> 应当插入，false -> 不应当插入
			auto findInsertPositionAt = [](const std::vector<MapPoint3D64>& linestring, double pointLenFromStart,
				MapPoint3D64& outInsertPoint, std::size_t& outInsertPosition, int64 tolerance) -> bool
				{
					// Note：需要确保计算link长度的流程和下面一样才能确保避免浮点数运算带来的微小误差
					double len = 0.0;
					for (std::size_t i = 0; i + 1 < linestring.size(); ++i)
					{
						auto& point = linestring[i];
						auto& nextPoint = linestring[i + 1];
						double segmentLen = double(point.pos.distance(nextPoint.pos));

						if ((len + segmentLen) >= pointLenFromStart)
						{
							double restPerc = (pointLenFromStart - len) / segmentLen;
							restPerc = nc_clamp(restPerc, 0.0, 1.0);

							outInsertPoint = mapPoint3D64_lerp(point, nextPoint, restPerc);
							if (mapPoint3D64_equalEpsilon2D(point, outInsertPoint, tolerance))
							{
								outInsertPosition = i;
								return false;
							}
							if (mapPoint3D64_equalEpsilon2D(nextPoint, outInsertPoint, tolerance))
							{
								outInsertPosition = i + 1;
								return false;
							}
							outInsertPosition = i + 1;
							return true;
						}
						len += segmentLen;
					}

					// 不应到达此处，否则程序错误，也可能是浮点数计算误差
					// 使用最后一个点新插入position和坐标
					outInsertPoint = linestring.back();
					outInsertPosition = linestring.size() - 1;
					return false;
				};
			// 在线(linestring)上寻找与指定点(point)距离最近的点，返回该点的位置以及在线上的何位置插入该点使得线保持连续
			auto readPathPoints = [&calLinestringLength, &findInsertPositionAt](std::vector<std::pair<DbLink*, bool>>& path, std::vector<std::vector<DbRdLinkLanePa*>> linksPa, std::vector<MapPoint3D64>& outPoints, std::vector<PathPa>& outPas)
				{
					// 顺 画线方向顺序插入pa点和记录PaOnPath
					auto insertPas = [&calLinestringLength, &findInsertPositionAt](DbLink* link, std::vector<DbRdLinkLanePa*> pas, std::vector<MapPoint3D64>& outPoints, std::vector<PathPa>& outPas)
						{
							outPoints = link->geometry.vertexes;
							double len = calLinestringLength(outPoints);
							for (DbRdLinkLanePa* pa : pas)
							{
								MapPoint3D64 outInsertPoint;
								std::size_t outInsertPosition;

								PathPa paOnPath;
								paOnPath.link = link;
								paOnPath.linkPa = pa;

								auto& paRelLink = pa->relLinks.begin()->second;// 一个PA关联的Link只有一个
								static const int64 NEAR_POINT_TOLERANCE{ 10 }; // 10单位，约为1cm
								// pa起点
								double paBeginPos = paRelLink.startOffset * len;
								if (findInsertPositionAt(outPoints, paBeginPos, outInsertPoint, outInsertPosition, NEAR_POINT_TOLERANCE))
								{
									outPoints.insert(outPoints.begin() + outInsertPosition, outInsertPoint);
								}
								paOnPath.indexFrom = outInsertPosition;
								// pa终点
								double paEndPos = paRelLink.endOffset * len;
								if (findInsertPositionAt(outPoints, paEndPos, outInsertPoint, outInsertPosition, NEAR_POINT_TOLERANCE))
								{
									outPoints.insert(outPoints.begin() + outInsertPosition, outInsertPoint);
								}
								paOnPath.indexTo = outInsertPosition;

								outPas.push_back(paOnPath);
							}
						};

					for (std::size_t i = 0; i < path.size(); ++i)
					{
						DbLink* link = path[i].first;
						bool linkForward = path[i].second;
						std::vector<DbRdLinkLanePa*> linkPas = linksPa[i];

						std::vector<MapPoint3D64> linkOutPoints;
						std::vector<PathPa> linkOutPas;
						insertPas(link, linkPas, linkOutPoints, linkOutPas);
						if (!linkForward)
						{
							for (auto& pa : linkOutPas)
							{
								pa.indexFrom = (linkOutPoints.size() - 1) - pa.indexFrom;
								pa.indexTo = (linkOutPoints.size() - 1) - pa.indexTo;
								std::swap(pa.indexFrom, pa.indexTo);
							}
							std::reverse(linkOutPoints.begin(), linkOutPoints.end());
							std::reverse(linkOutPas.begin(), linkOutPas.end());
						}
						for (auto& pa : linkOutPas)
						{
							pa.linkForward = linkForward;
						}
						if (i == 0)
						{
							outPoints = linkOutPoints;
							outPas = linkOutPas;
						}
						else
						{
							// 去除link连接处公共点。此处要求数据两相邻link连接处点必须相同
							for (auto& pa : linkOutPas)
							{
								pa.indexFrom += (outPoints.size() - 1);
								pa.indexTo += (outPoints.size() - 1);
							}
							outPoints.insert(outPoints.end(), linkOutPoints.begin() + 1, linkOutPoints.end());
							outPas.insert(outPas.end(), linkOutPas.begin(), linkOutPas.end());
						}
					}
				};

			readPathPoints(leftPath, leftLinksPa, leftPoints, leftPathPas);
			readPathPoints(rightPath, rightLinksPa, rightPoints, rightPathPas);

			// 左右侧形状线向对向插入最邻近点并更新pa的索引范围
			auto insertNearestPointOneSideToAnother = [](const std::vector<MapPoint3D64>& oneSideVertices, std::vector<MapPoint3D64>& anotherSideVerticesCopy)
				{
					std::size_t lastSegmentIndex = 0;
					anotherSideVerticesCopy.reserve(anotherSideVerticesCopy.size() + oneSideVertices.size());
					for (auto& vertex : oneSideVertices)
					{
						auto insertPosition = findDividedRoadInsertPosition(vertex, anotherSideVerticesCopy, true, lastSegmentIndex);
						lastSegmentIndex = insertPosition.lastSegmentIndex;
						anotherSideVerticesCopy.insert(anotherSideVerticesCopy.begin() + insertPosition.insertPos, insertPosition.nearestPoint);
					}
				};
			std::vector<MapPoint3D64> leftPointsCopy = leftPoints, rightPointsCopy = rightPoints;
			insertNearestPointOneSideToAnother(leftPoints, rightPointsCopy);
			insertNearestPointOneSideToAnother(rightPoints, leftPointsCopy);

			// 更新pa范围
			auto updatePaIndexRange = [](const std::vector<MapPoint3D64>& originPoints, const std::vector<MapPoint3D64>& newPoints, std::vector<PathPa>& pas)
				{
					auto findNextPointIndex = [&newPoints](const MapPoint3D64& targetPoint, std::size_t& inoutLastIndex, std::size_t& outTargetPointIndex)
						{
							for (std::size_t i = inoutLastIndex; i < newPoints.size(); ++i)
							{
								if (newPoints[i] == targetPoint)
								{
									inoutLastIndex = i;
									outTargetPointIndex = i;
									return;
								}
							}
						};

					std::size_t lastIndex = 0;
					for (auto& pa : pas)
					{
						const MapPoint3D64& fromPoint = originPoints[pa.indexFrom];
						const MapPoint3D64& toPoint = originPoints[pa.indexTo];
						findNextPointIndex(fromPoint, lastIndex, pa.indexFrom);
						findNextPointIndex(toPoint, lastIndex, pa.indexTo);
					}
				};

			updatePaIndexRange(leftPoints, leftPointsCopy, leftPathPas);
			updatePaIndexRange(rightPoints, rightPointsCopy, rightPathPas);

			leftPoints = std::move(leftPointsCopy);
			rightPoints = std::move(rightPointsCopy);
		}

		// 返回的宽度总是为正数
		auto findLinkLanePaWidth = [commonBoundaryIndex](const std::vector<PathPa>& pas, int pointIndex) -> double
			{
				for (auto& pa : pas)
				{
					if (pointIndex >= pa.indexFrom)
						return std::abs(pa.linkPa->roadBoundaries[commonBoundaryIndex]->geometryOffset);
				}
				// 不应该出现，否则给一个不合理的值使其效果特别的差，让其醒目些能方便看出来

				// 可能到达此处的情况：
				// 1. 上下行道路搜索结果中出现Link的的PA为空
				// 2. 上下行道路搜索结果中出现Link的的PA不为空，但是不完整，缺少其中一个范围。

				// Note: 注释此报错不影响后续流程进行，没有pa的link或却缺少pa的link不会编译路面。
				// throw std::runtime_error("error"); // 临时调试使用


				return 0.0;
			};

		std::vector<MapPoint3D64> leftRoadRightSidePoints, rightRoadLeftSidePoints, middlePoints;
		std::vector<bool> middlePointsShouldMrege;
		// assert( leftPoints.size() == rightPoints.size() )
		for (std::size_t sidePointIndex = 0; sidePointIndex < leftPoints.size(); ++sidePointIndex)
		{
			const MapPoint3D64& leftPoint = leftPoints[sidePointIndex];
			const MapPoint3D64& rightPoint = rightPoints[sidePointIndex];
			double leftOffset = findLinkLanePaWidth(leftPathPas, sidePointIndex);
			double rightOffset = findLinkLanePaWidth(rightPathPas, sidePointIndex);

			bool shouldMerge = false;
			{
				static const float MERGE_EXTRA_DISTANCE = 36.f;

				double lrDistance = MapPoint64::geodistance(leftPoint.pos, rightPoint.pos);
				double mergeLen = (leftOffset + rightOffset) * 0.001 + MERGE_EXTRA_DISTANCE;

				shouldMerge = lrDistance < mergeLen;

				// TODO: 添加首尾禁止合并距离逻辑
			}
			middlePointsShouldMrege.push_back(shouldMerge);


			MapPoint3D64 leftPointRight, rightPointLeft;
			{
				// distance需要为正数才能保证leftPointOut为左侧的点
				auto calcSidePoint = [](const std::vector<MapPoint3D64>& line, std::size_t index, double distance, MapPoint3D64* leftPointOut, MapPoint3D64* rightPointOut) -> bool
					{
						std::size_t linePointNum = line.size();
						if (index >= linePointNum || linePointNum < 2)
							return false;

						DVector3 v, p;

						if (index == 0 || linePointNum == 2)
							v = line[1].toDVector3() - line[0].toDVector3();
						else if (index == linePointNum - 1)
							v = line[linePointNum - 1].toDVector3() - line[linePointNum - 2].toDVector3();
						else
							v = line[index + 1].toDVector3() - line[index - 1].toDVector3();

						if (v.length() < 10.0) // 10个单位，约1cm
							return false;

						p = line[index].toDVector3();
						v.normalize();
						DVector3 v0 = dvec3(-v.y, v.x, 0);
						DVector3 v1 = dvec3(v.y, -v.x, 0);
						if (leftPointOut)
							*leftPointOut = MapPoint3D64_make(p + (v0 * distance));
						if (rightPointOut)
							*rightPointOut = MapPoint3D64_make(p + (v1 * distance));
						return true;
					};

				if (!calcSidePoint(leftPoints, sidePointIndex, leftOffset, nullptr, &leftPointRight))
					leftPointRight.invalidate();
				if (!calcSidePoint(rightPoints, sidePointIndex, rightOffset, &rightPointLeft, nullptr))
					rightPointLeft.invalidate();
			}
			leftRoadRightSidePoints.push_back(leftPointRight);
			rightRoadLeftSidePoints.push_back(rightPointLeft);
			middlePoints.push_back((leftPoint + rightPoint) * 0.5);
		}
		{
			// 上述处理后，leftRoadRightSidePoints和rightRoadLeftSidePoints可能存在一些无效的点
			auto fillInvalidSidePoints = [](std::vector<MapPoint3D64>& sidePoints, const std::vector<MapPoint3D64>& fallbackPoints) 
				{
					// 首部出现的无效点设置为后续第一个有效的点
					{
						MapPoint3D64* firstValidPoint{ nullptr };
						std::vector<MapPoint3D64Ref> frontInvalidPoints;
						for (std::size_t i = 0; i < sidePoints.size(); ++i)
						{
							if (!sidePoints[i].valid())
								frontInvalidPoints.emplace_back(sidePoints[i]);
							else
							{
								firstValidPoint = &sidePoints[i];
								break;
							}
						}
						if (firstValidPoint)
						{
							for (auto& ptRef : frontInvalidPoints)
							{
								ptRef.get() = *firstValidPoint;
							}
						}
						else
						{
							// 到达此处说明全部都是无效点，不大可能出现这种情况。
							// 姑且先赋值为fallbackPoints
							sidePoints = fallbackPoints;
							return;
						}
					}
					// 尾部出现的无效点设置为前方第一个有效点
					{
						MapPoint3D64* firstValidPoint{ nullptr };
						std::vector<MapPoint3D64Ref> backInvalidPoints;
						for (std::size_t i = sidePoints.size() - 1; i < sidePoints.size(); --i)
						{
							if (!sidePoints[i].valid())
								backInvalidPoints.emplace_back(sidePoints[i]);
							else
							{
								firstValidPoint = &sidePoints[i];
								break;
							}
						}
						if (firstValidPoint)
						{
							for (auto& ptRef : backInvalidPoints)
							{
								ptRef.get() = *firstValidPoint;
							}
						}
						else
						{
							// 到达此处说明全部都是无效点，不大可能出现这种情况。
							// 姑且先赋值为fallbackPoints
							sidePoints = fallbackPoints;
							return;
						}
					}
					// 剩余部分的invalidPoint
					std::vector<std::pair<std::deque<MapPoint3D64Ref>, std::array<MapPoint3D64, 2>>> invalidPointRanges;
					{
						std::deque<MapPoint3D64Ref> invalidPoints;
						MapPoint3D64* frontValidPoint{ nullptr }, * backValidPoint{ nullptr };
						for (MapPoint3D64& pt : sidePoints)
						{
							if (pt.valid())
							{
								if (invalidPoints.empty())
									frontValidPoint = &pt;
								else
								{
									backValidPoint = &pt;

									auto invalidPointRange = std::make_pair(invalidPoints, std::array<MapPoint3D64, 2>{*frontValidPoint, * backValidPoint});
									invalidPointRanges.push_back(invalidPointRange);

									frontValidPoint = backValidPoint = nullptr;
									invalidPoints.clear();
								}
							}
							else
							{
								invalidPoints.emplace_back(pt);
							}
						}
					}
					for (auto& invalidPointRange : invalidPointRanges)
					{
						std::deque<MapPoint3D64Ref>& invalidPoints = invalidPointRange.first;
						std::array<MapPoint3D64, 2> frontBackValidPoints = invalidPointRange.second;

						while (!invalidPoints.empty())
						{
							invalidPoints.front().get() = frontBackValidPoints[0];
							invalidPoints.pop_front();
							if (!invalidPoints.empty())
							{
								invalidPoints.back().get() = frontBackValidPoints[1];
								invalidPoints.pop_back();
							}
						}
					}
				};

				fillInvalidSidePoints(leftRoadRightSidePoints, leftPoints);
				fillInvalidSidePoints(rightRoadLeftSidePoints, rightPoints);
		}

		// 计算middle points每个点的距离起点的长度
		std::vector<double> middlePointsLengthAcc;
		if (!middlePoints.empty())
		{
			middlePointsLengthAcc.push_back(0.0f);
			for (size_t i = 1; i < middlePoints.size(); ++i)
			{
				double length = MapPoint64::geodistance(middlePoints[i].pos, middlePoints[i - 1].pos);
				middlePointsLengthAcc.push_back(middlePointsLengthAcc.back() + length);
			}
		}
		auto calculateRangeLength = [&middlePointsLengthAcc](const std::array<std::size_t, 2>& range)
			{
				return middlePointsLengthAcc[range[1]] - middlePointsLengthAcc[range[0]];
			};

		std::vector<std::array<std::size_t, 2>> rejectMergedRanges; // 记录不合并部分点的索引范围: [i, j]
		for (std::size_t lower = 0; lower < middlePointsShouldMrege.size(); ++lower)
		{
			if (!middlePointsShouldMrege[lower])
			{
				std::size_t high = lower;
				while ((++high) < middlePointsShouldMrege.size() && !middlePointsShouldMrege[high])
					; // pass

				rejectMergedRanges.push_back(std::array<std::size_t, 2>{lower, high - 1});

				lower = high - 1;
			}
		}

		// 移除过短的不合并部分，注意首尾不合并部分无论多长多短均不移除
		if (!rejectMergedRanges.empty())
		{
			static const float MIN_REJECT_MERGE_LENGTH = 700.0f;

			// 只有一个的特殊处理
			if (rejectMergedRanges.size() == 1)
			{
				// 该不合并部分起点为道路起点或终点为道路终点，不能移除
				if (rejectMergedRanges.front()[0] == 0 || (rejectMergedRanges.front()[1] + 1) == middlePoints.size())
				{
					// nothing to do
				}
				else if (calculateRangeLength(rejectMergedRanges.front()) < MIN_REJECT_MERGE_LENGTH)
				{
					rejectMergedRanges.clear();
				}
			}
			else
			{
				std::vector<std::array<std::size_t, 2>> newRejectMergedRanges; // 记录不合并部分点的索引范围: [i, j]

				// 查首
				if (rejectMergedRanges.front()[0] == 0 || calculateRangeLength(rejectMergedRanges.front()) > MIN_REJECT_MERGE_LENGTH)
				{
					newRejectMergedRanges.push_back(rejectMergedRanges.front());
				}
				// 查中
				for (std::size_t i = 1; i + 1 < rejectMergedRanges.size(); ++i)
				{
					if (calculateRangeLength(rejectMergedRanges[i]) > MIN_REJECT_MERGE_LENGTH)
					{
						newRejectMergedRanges.push_back(rejectMergedRanges[i]);
					}
				}
				// 查尾
				if (rejectMergedRanges.back()[1] + 1 == middlePoints.size() || calculateRangeLength(rejectMergedRanges.back()) > MIN_REJECT_MERGE_LENGTH)
				{
					newRejectMergedRanges.push_back(rejectMergedRanges.back());
				}

				rejectMergedRanges = std::move(newRejectMergedRanges);
			}
		}

		// 移除不合并部分之间过短的合并部分，长度使用相邻不合并部分的首尾之间点的距离来判断
		{
			static const float MIN_MERGE_LENGTH = 200.0f; // 最短的合并区域长度

			for (bool hasProcessed = true; hasProcessed;)
			{
				hasProcessed = false;

				std::vector<std::array<std::size_t, 2>> newRejectMergedRanges; // 记录不合并部分点的索引范围: [i, j]

				for (std::size_t i = 0; i + 1 < rejectMergedRanges.size(); ++i)
				{
					std::array<std::size_t, 2>& range = rejectMergedRanges[i];
					std::array<std::size_t, 2>& nextRange = rejectMergedRanges[i + 1];

					float mergeRangeLength = calculateRangeLength({ range[1], nextRange[0] });
					if (mergeRangeLength < MIN_MERGE_LENGTH && mergeRangeLength > 0.0f)
					{
						newRejectMergedRanges.insert(newRejectMergedRanges.begin(), rejectMergedRanges.begin(), rejectMergedRanges.begin() + i);
						newRejectMergedRanges.push_back({ range[0], nextRange[1] });
						newRejectMergedRanges.insert(newRejectMergedRanges.end(), rejectMergedRanges.begin() + i + 2, rejectMergedRanges.end());

						rejectMergedRanges = newRejectMergedRanges;
						hasProcessed = true;
						break;
					}
				}
			}
		}

		std::vector<MapPoint3D64> newLeftRoadRightSidePoints, newRightRoadLeftSidePoints;
		newLeftRoadRightSidePoints.reserve(leftRoadRightSidePoints.size());
		newRightRoadLeftSidePoints.reserve(rightRoadLeftSidePoints.size());
		//
		if (rejectMergedRanges.empty())
		{
			for (const MapPoint3D64& middlePoint : middlePoints)
			{
				newLeftRoadRightSidePoints.push_back(middlePoint);
				newRightRoadLeftSidePoints.push_back(middlePoint);
			}
		}
		else
		{
			std::size_t rejectMregeRangeIndex = 0;
			for (std::size_t i = 0; i < middlePoints.size(); ++i)
			{
				if (rejectMregeRangeIndex < rejectMergedRanges.size() && i == rejectMergedRanges[rejectMregeRangeIndex][0])
				{
					std::size_t lower = rejectMergedRanges[rejectMregeRangeIndex][0];
					const std::size_t high = rejectMergedRanges[rejectMregeRangeIndex][1];
					for (; lower <= high; ++lower)
					{
						const MapPoint3D64& rSidePoint = leftRoadRightSidePoints[lower];
						const MapPoint3D64& lSidePoint = rightRoadLeftSidePoints[lower];
						newLeftRoadRightSidePoints.push_back(rSidePoint);
						newRightRoadLeftSidePoints.push_back(lSidePoint);
					}
					i = high;
					rejectMregeRangeIndex += 1;
				}
				else
				{
					const MapPoint3D64& middlePoint = middlePoints[i];
					newLeftRoadRightSidePoints.push_back(middlePoint);
					newRightRoadLeftSidePoints.push_back(middlePoint);
				}
			}
		}

		// assert( newLeftRoadRightSidePoints.size() == leftPoints.size() )
		// assert( newRightRoadLeftSidePoints.size() == rightPoints.size() )
		// newLeftRoadRightSidePoints & newRightRoadLeftSidePoints重新赋值给 LinkLanePa的公共侧的道路边界
		auto recoverPaBoundary = [commonBoundaryIndex](std::vector<PathPa>& pathPas, const std::vector<MapPoint3D64>& sidePoints, int currentMeshId) 
			{
				for (PathPa& pa : pathPas)
				{
					if (pa.link->owner->getId() != currentMeshId)
						continue;

					auto& linestring = pa.linkPa->roadBoundaries[commonBoundaryIndex]->geometry.vertexes;
					linestring.clear();
					linestring.reserve(pa.indexTo - pa.indexFrom);
					{
						int indexFrom = pa.linkForward ? int(pa.indexFrom) : int(pa.indexTo);
						int	indexTo = pa.linkForward ? int(pa.indexTo) + 1 : int(pa.indexFrom) - 1;
						int step = pa.linkForward ? 1 : -1;
						for (int i = indexFrom; i != indexTo ; i += step)
						{
							linestring.push_back(sidePoints[i]);
						}
					}
				}
			};

		recoverPaBoundary(leftPathPas, newLeftRoadRightSidePoints, m_mesh->getId());
		recoverPaBoundary(rightPathPas, newRightRoadLeftSidePoints, m_mesh->getId());
	}

}
