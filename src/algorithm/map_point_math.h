#pragma once

#include "geometry/map_point3d64.h"

#include <cmath>
#include <array>

namespace OMDB
{
namespace map_pt_math
{

	/**
	 * @brief ������ָ������תָ���Ƕȡ�ע�⣬��ƽ�漸�δ��������Ǿ�γ�ȡ�
	 * @param pt 
	 * @param origin 
	 * @param angle ���ȡ���ֵΪ��ʱ����ת����ֵΪ˳ʱ����ת��
	 * @return 
	*/
	static MapPoint64 rotatePoint(MapPoint64 pt, const MapPoint64& origin, const double angle)
	{
		pt.lon -= origin.lon;
		pt.lat -= origin.lat;

		double cosAngle = std::cos(angle);
		double sinAngle = std::sin(angle);

		double newLon = pt.lon * cosAngle - pt.lat * sinAngle;
		double newLat = pt.lon * sinAngle + pt.lat * cosAngle;

		return MapPoint64{ int64(newLon) + origin.lon, int64(newLat) + origin.lat };
	}

	/**
	 * @param angle �Ƕȡ�
	*/
	static MapPoint64 rotatePointDegAngle(MapPoint64 pt, const MapPoint64& origin, const double angle)
	{
		constexpr double degree2Radian = MATH_PI_D / 180.0;
		return rotatePoint(pt, origin, angle * degree2Radian);
	}

	/**
	 * @brief ������originΪԭ���ƽ��ֱ������ϵ��point��+x��ʱ�뷽��ļнǣ���Χ[0, 2PI]����λ���ȡ�
	 * @param origin 
	 * @param point 
	 * @return 
	*/
	static double twoPointAngle(const MapPoint64& origin, MapPoint64 point)
	{
		point.lon -= origin.lon;
		point.lat -= origin.lat;

		double angle = std::atan2(double(point.lat), double(point.lon));
		return angle < 0.0 ? std::abs(angle + 2.0 * MATH_PI_D) : angle;
	}

	/**
	 * @brief ���߶���ͷ��Է���������ָ������
	 * @param segment start&end
	 * @param shrinkDist
	 * @return
	*/
	static std::array<MapPoint64, 2> shrinkSegment(const std::array<MapPoint64, 2>& segment, double shrinkDist)
	{
		double dx = static_cast<double>(segment[1].lon - segment[0].lon);
		double dy = static_cast<double>(segment[1].lat - segment[0].lat);
		double length = std::sqrt(dx * dx + dy * dy);
		// unit vector from segment start to segment end
		std::array<double, 2> unitVector{ dx / length, dy / length };
		std::array<double, 2> unitVectorInv{ -dx / length, -dy / length };

		return {
			MapPoint64{segment[0].lon + int64(unitVector[0] * shrinkDist), segment[0].lat + int64(unitVector[1] * shrinkDist)},
			MapPoint64{segment[1].lon + int64(unitVectorInv[0] * shrinkDist), segment[1].lat + int64(unitVectorInv[1] * shrinkDist)},
		};
	}

	static std::array<MapPoint3D64, 2> shrinkSegment(const std::array<MapPoint3D64, 2>& segment, double shrinkDist)
	{
		std::array<MapPoint64, 2> newSegment = shrinkSegment({ segment[0].pos, segment[1].pos }, shrinkDist);
		return {
			MapPoint3D64{newSegment[0], segment[0].z},
			MapPoint3D64{newSegment[1], segment[1].z},
		};
	}



}
}

