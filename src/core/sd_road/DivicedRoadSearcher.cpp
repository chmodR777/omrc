#include "stdafx.h"
#include "core/framework/RTreeSeacher.h"
#include "core/generator/Generator.h"
#include "algorithm/map_point_math.h"
#include "DivicedRoadSearcher.h"

#include <set>
#include <cmath>

namespace OMDB
{
namespace sd
{

	/**
	 * @brief �жϴӽ�1ת���2�Ƿ���Ҫ��ת������ת����1 & ��2 ��Ϊ���ȡ�
	 * @return true -> ��ת, false -> ��ת
	*/
	bool angleTurnLeft(double angle1, double angle2)
	{
		std::array<double, 2> target{ std::cos(angle2), std::sin(angle2) };
		std::array<double, 2> heading{ std::cos(angle1), std::sin(angle1) };

		double cross = target[0] * heading[1] - target[1] * heading[0];

		return cross < 0.0;
	}

	/**
	 * @brief �ж������Ƕ��Ƿ�ͬ�򡣽�1 & ��2 ��Ϊ���ȡ�
	 * @param angle1 radians
	 * @param angle2 radians
	 * @return 
	*/
	bool angleIsTheSameDirection(double angle1, double angle2)
	{
		double x1 = std::cos(angle1), y1 = std::sin(angle1);
		double x2 = std::cos(angle2), y2 = std::sin(angle2);

		double dot = x1 * x2 + y1 * y2;
		double  angle = std::acos(dot);
		return angle < (90.0 * MATH_DEGREE2RADIAN_D);
	}

	/**
	 * @brief ��outlinks����startHeadingAngle��С�������򣬰���nextLinkInCCWȡ��ǰlink��һ��link��
	 * @param outlinks 
	 * @param curLink 
	 * @param nextLinkInCCW 
	 * @return 
	*/
	static DirLink getNextDirLink(std::vector<DirLink>& outlinks, DirLink& curLink, bool nextLinkInCCW)
	{
		// outlinks����startHeadingAngle��С��������
		std::sort(outlinks.begin(), outlinks.end(), [](DirLink& l, DirLink& r) {return l.startHeadingAngle() < r.startHeadingAngle(); });
		// �ҵ�curLink����
		auto curLinkIt = std::find_if(outlinks.begin(), outlinks.end(), [&curLink](DirLink& dirLink) {return dirLink.link()->uuid == curLink.link()->uuid; });
		if (curLinkIt == outlinks.end())
			return DirLink{};
		else
		{
			std::size_t curLinkIndex = curLinkIt - outlinks.begin();
			if (nextLinkInCCW)
				return outlinks[(curLinkIndex + 1) % outlinks.size()];
			else
				return outlinks[(curLinkIndex - 1) % outlinks.size()];
		}
	}

	/**
	 * @brief ����ץ·��ʹ�õ�Link��״�ߵķ��򣬵�λ���ȡ��÷���ͨ����ָ�������ڽ������ϵĲ��������㡣
	 * @param linestring
	 * @param point
	 * @return
	*/
	double findLineDirectionWithPoint(const LineString3d& linestring, const MapPoint3D64& point, MapPoint64& nearestPointOnLineString)
	{
		using Point = std::array<double, 2>; // ��ͬ��MapPoint64��ֻ��ʹ��double�洢ԭ����int64
		using Segment = std::array<Point, 2>;

		auto MapPoint64ToPoint = [](const MapPoint64& point) { return Point{ static_cast<double>(point.lon), static_cast<double>(point.lat) }; };

		auto MapPoint64OnLine = [&linestring](std::size_t index) {return linestring.vertexes[index].pos; };

		auto findNearestPointOnSegment = [](const Segment& segment, const Point& point, Point& nearestPoint, int& nearestPointType, double &sqDist)
			{
				// point a is segment[0], point b is segment[1], point c is point
				double diffBAX = segment[1][0] - segment[0][0];
				double diffBAY = segment[1][1] - segment[0][1];
				double diffCAX = point[0] - segment[0][0];
				double diffCAY = point[1] - segment[0][1];
				double r = (diffBAX * diffCAX + diffBAY * diffCAY) / (diffBAX * diffBAX + diffBAY * diffBAY);

				// nearestPointType: 0 -> �߶�����ָ��������ĵ�Ϊ�߶����
				//                   1 -> �߶�����ָ��������ĵ�Ϊ�߶��յ�
				//                   3 -> �߶�����ָ��������ĵ�Ϊ�߶��м��

				if (r <= 0.0)
				{
					nearestPointType = 0;
					nearestPoint = segment[0];
				}
				else if (r >= 1.0)
				{
					nearestPointType = 1;
					nearestPoint = segment[1];
				}
				else
				{
					nearestPointType = 2;
					nearestPoint = Point{ segment[0][0] + r * diffBAX,
										  segment[0][1] + r * diffBAY };
				}

				sqDist = std::pow(nearestPoint[0] - point[0], 2.0) + std::pow(nearestPoint[1] - point[1], 2.0);
			};


		int nearestPointType;
		double sqDistToNearestPoint = DBL_MAX;
		std::size_t nearestSegmentIndex; // ������߶ε��յ���linestring���������
		for (std::size_t i = 1; i < linestring.vertexes.size(); ++i)
		{
			Segment segment{ MapPoint64ToPoint(MapPoint64OnLine(i - 1)), MapPoint64ToPoint(MapPoint64OnLine(i)) };
			int nearestPointTypeOnSegment;
			Point nearestPoint;
			double sqDist;
			findNearestPointOnSegment(segment, MapPoint64ToPoint(point.pos), nearestPoint, nearestPointTypeOnSegment, sqDist);
			if (sqDist < sqDistToNearestPoint)
			{
				nearestPointType = nearestPointTypeOnSegment;
				sqDistToNearestPoint = sqDist;
				nearestSegmentIndex = i;
				nearestPointOnLineString.lon = nearestPoint[0];
				nearestPointOnLineString.lat = nearestPoint[1];
			}
		}

		if (sqDistToNearestPoint == DBL_MAX)
		{
			return DBL_MAX;
		}
		else
		{
			switch (nearestPointType)
			{
			case 0:
				// ���ڽ�����Ϊ�߶���㣬ʹ�øõ�ǰ���߶η����֮ƽ��
				// ע���ٽ������������ǰ�߶�
				if (nearestSegmentIndex > 1)
				{
					double prevAngle = map_pt_math::twoPointAngle(MapPoint64OnLine(nearestSegmentIndex - 2), MapPoint64OnLine(nearestSegmentIndex - 1));
					double curAngle = map_pt_math::twoPointAngle(MapPoint64OnLine(nearestSegmentIndex - 1), MapPoint64OnLine(nearestSegmentIndex));
					return (prevAngle + curAngle) * 0.5;
				}
				else
				{
					return map_pt_math::twoPointAngle(MapPoint64OnLine(1), MapPoint64OnLine(0));
				}
				break;
			case 1:
				// ���ڽ�����Ϊ�߶��յ㣬ʹ�øõ�ǰ���߶η����֮ƽ��
				// ע���ٽ�����������ں��߶�
				if (nearestSegmentIndex < linestring.vertexes.size() - 1)
				{
					double prevAngle = map_pt_math::twoPointAngle(MapPoint64OnLine(nearestSegmentIndex - 1), MapPoint64OnLine(nearestSegmentIndex));
					double curAngle = map_pt_math::twoPointAngle(MapPoint64OnLine(nearestSegmentIndex), MapPoint64OnLine(nearestSegmentIndex + 1));
					return (prevAngle + curAngle) * 0.5;
				}
				else
				{
					return map_pt_math::twoPointAngle(MapPoint64OnLine(nearestSegmentIndex - 1), MapPoint64OnLine(nearestSegmentIndex));
				}
				break;
			case 3:
			default:
				// ���ڽ������߶��м䣬ʹ�ø��߶μ��㷽���
				return map_pt_math::twoPointAngle(MapPoint64OnLine(nearestSegmentIndex - 1), MapPoint64OnLine(nearestSegmentIndex));
			}
		}
	}

	DivicedRoadSearcher::DivicedRoadSearcher(DbMesh* mesh) : m_mesh(mesh)
	{
	}

	/**
	 * @brief ��Link�ϵ�ָ������һ�����뷶Χ����������link�����������Ľ��������direction�����ƽ�г̶ȴӴ�С���򣬷��ؽ����Խǰ���linkԽƽ�С�
	 * @param link ��ǰ���·
	 * @param point ��ǰ���·
	 * @param maxFindDist ����������롣��λ��γ�ȡ�
	 * @param direction ���򣬻��ȡ�
	 * @return
	*/
	std::vector<DivicedRoadSearcher::NeighborLink> DivicedRoadSearcher::searchParallelNeighborLinks(DbLink& link, const MapPoint3D64& point, const double maxFindDist, const double direction)
	{
		// �ж������н�(��λradians, ��Χ[0, 2PI) )֮��ƽ�г̶ȣ��������н�֮��ļнǴ�С���������޷���ֵԽС���������н�Խ�ӽ���
		auto angleParallelDiff = [](double angle1, double angle2) -> double
			{
				double maxAngle = max(angle1, angle2);
				double minAngle = min(angle1, angle2);

				double diff = maxAngle - minAngle;
				// diff�ֱ��0�ȡ�180�ȡ�360�ȵĲ������ֵ����Сֵ
				return min(min(diff, std::abs(diff - MATH_PI_D)), std::abs(2.0 * MATH_PI_D - diff));
			};

		// ��pointΪԲ�ģ�maxFindDistΪ�뾶��Բ�ڵ���������Link������
		std::vector<DbLink*> neighborLinks = RTreeSeacher::seachNearby2d(m_mesh, &link, point, maxFindDist);
		// ����������������յ�·�������Ƴ̶ȴӴ�С����
		std::vector<NeighborLink> neighborLinksParallel; // ����pair.secondΪ�ǶȲ���
		neighborLinksParallel.reserve(neighborLinks.size());
		for (DbLink* neighborLink : neighborLinks)
		{
			MapPoint64 nearestPoint;
			double lineDirection = findLineDirectionWithPoint(neighborLink->geometry, point, nearestPoint);
			double diff = angleParallelDiff(lineDirection, direction);

			NeighborLink neighborLinkObj;
			neighborLinkObj.link = neighborLink;
			neighborLinkObj.direction = lineDirection;
			neighborLinkObj.directionDiff = diff;
			neighborLinkObj.nearestPointOnLink = nearestPoint;

			neighborLinksParallel.push_back(neighborLinkObj);
		}
		// ����
		std::sort(neighborLinksParallel.begin(), neighborLinksParallel.end(), [](auto& l, auto& r) {return l.directionDiff < r.directionDiff; });

		return neighborLinksParallel;
	}

	bool DivicedRoadSearcher::isDividedRoadZGen(DbLink& link)
	{
		// TODO: �ж�Link�Ƿ������·�����·����Ϊ�������е�·
		// TODO: �ж�Link�Ƿ���·����·��

		return link.multi_digitized == 1;
	}


	DirLink DivicedRoadSearcher::grabAnotherRoadZGen(DirLink& link, bool& grabRoadIsLeft)
	{
		double startHeadingAngle = link.startHeadingAngle();
		double startHeadingAngleNegativeDir = startHeadingAngle + MATH_PI_D;
		MapPoint3D64 pointOnLink = link.getStartPointRef();
		std::set<int64> linkOutlinksUUID;
		{
			std::vector<int64> forwardOutlinks = link.getEndDirectOutlinksExceptSelfUUID();
			std::vector<int64> backwardOutlinks = link.getReverseDirLink().getEndDirectOutlinksExceptSelfUUID();
			linkOutlinksUUID.insert(forwardOutlinks.begin(), forwardOutlinks.end());
			linkOutlinksUUID.insert(backwardOutlinks.begin(), backwardOutlinks.end());
		}

		std::wstring* linkName = Generator::getLinkName(link.link());

		auto intersectsWithOtherLink = [](DbMesh* pGrid, const MapPoint64& segStart, const MapPoint64& segEnd) 
			{

				segment_2t bgSegment{ POINT_2T(segStart), POINT_2T(segEnd) };

				// segment����Լ10cm������
				std::array<MapPoint64, 2> segment = map_pt_math::shrinkSegment({ segStart, segEnd }, 100);


				segment_2t bgSegmentNew{ POINT_2T(segment[0]), POINT_2T(segment[1])};

				// 
				return RTreeSeacher::intersectsWithLinks2d(pGrid, segment[0], segment[1]);
			};


		auto isLeftRightParalel = [](const LineString3d &line1, const LineString3d &line2)
			{
				linestring_2t bgLine1 = LINESTRING_2T(line1.vertexes);
				linestring_2t bgLine2 = LINESTRING_2T(line2.vertexes);

				// ȡline2�����յ㵽Line1�����ڽ��㣬������������ڽ��㶼��ͬ����Ϊ�������߲�������ƽ��
				segment_2t seg1;
				bg::closest_points(bgLine2.front(), bgLine1, seg1);

				segment_2t seg2;
				bg::closest_points(bgLine2.back(), bgLine1, seg2);

				bool isLeftRightParalel = seg1.second != seg2.second;

				return isLeftRightParalel;
			};

		// Note: MapPoint64���԰뾶Ϊ6378137�������ϵľ���Ϊ0.0011131949079327358�ף�ԼΪ0.111cm
		// �ʶ��˴�30000��λ��Զ����Լ33.3��
		int maxFindDist = 30000;
		std::vector<NeighborLink> paralelLinks = searchParallelNeighborLinks(*link.link(), pointOnLink, maxFindDist, startHeadingAngle);
		// ����ƽ�еĵ�·��ʼ�ң����п����������е���һ��
		for (auto& parallelLink : paralelLinks)
		{
			if (Generator::containsLinkName(parallelLink.link, linkName) // ������ͬ
				&& linkOutlinksUUID.count(parallelLink.link->uuid) == 0 // ���뵱ǰ��·������������·
				&& !intersectsWithOtherLink(m_mesh, pointOnLink.pos, parallelLink.nearestPointOnLink) // ���ܺ��������·
				&& isLeftRightParalel(link.link()->geometry, parallelLink.link->geometry) // ȷ��������·������ƽ�У������� ||��������ǰ��ƽ�У������� --��
				)
			{
				// �ӵ�ǰ��·ָ��Բ��·���ڽ��������ת���·�����Ƿ�����
				double angleToOpposite = map_pt_math::twoPointAngle(pointOnLink.pos, parallelLink.nearestPointOnLink);
				bool linkIsLeft = angleTurnLeft(angleToOpposite, startHeadingAngleNegativeDir);
				grabRoadIsLeft = !linkIsLeft;

				// �жϵ�ǰlink���ҵ���link�Ƿ���Ȼ�����ɹ����DirLink
				if (angleIsTheSameDirection(startHeadingAngleNegativeDir, parallelLink.direction))
					return DirLink{ parallelLink.link, true };
				else
					return DirLink{ parallelLink.link, false };
			}
		}

		return DirLink{};
	}

	DirLinkPath DivicedRoadSearcher::extendRoad(DirLink& beginLink, bool beginLinkOnLeft, std::set<int64>& visitedLinks)
	{
		std::wstring* beginLinkName = Generator::getLinkName(beginLink.link());

		auto extendInOneDirection = [&](DirLink curLink, bool nextRoadInCCW) -> DirLinkPath
			{
				DirLinkPath path;

				while (true)
				{
					int highwayOutlinkNum;
					std::vector<DirLink> outlinks = curLink.getEndDirectOutlinks();
					if (outlinks.size() != 2) // ����������������·����ֹͣ����
						break;

					DirLink nextLink = getNextDirLink(outlinks, curLink, nextRoadInCCW);

					if (!nextLink.valid()
						|| !isDividedRoadZGen(*nextLink.link()))
						break;
					// ��ǰ��·����һ��·��Ϊ�����У��������߲�Ϊ������
					{
						int dividedRoadNum = 0;
						for (DirLink& outlink : outlinks)
							dividedRoadNum += static_cast<int>(isDividedRoadZGen(*outlink.link()));
						if (dividedRoadNum != 2)
							break;
					}

					// ��ǰ��·����һ��·������ͬ
					if (!Generator::containsLinkName(nextLink.link(), beginLinkName))
						break;

					// ��һ��·�Ѿ��������������е�·
					if (visitedLinks.count(nextLink.link()->uuid) > 0)
						break;

					visitedLinks.insert(nextLink.link()->uuid);
					curLink = nextLink;

					path.addPortion(nextLink);
				}

				return path;
			};

		// ��beginLink����������
		DirLinkPath pathBackward = extendInOneDirection(beginLink.getReverseDirLink(), !beginLinkOnLeft);

		pathBackward.reverse(); // ��ʱpathBackward�����Ѿ�����backward�ˣ�����forward
		pathBackward.addPortion(beginLink);

		DirLinkPath pathForward = extendInOneDirection(beginLink, beginLinkOnLeft);

		pathBackward.appendPath(pathForward);

		return pathBackward;
	}

	std::vector<DividedRoad> DivicedRoadSearcher::searchZGen()
	{
		std::set<int64> visitedLinks;

		std::vector<DividedRoad> dividedRoads;

		// TODO: ���ǿ�����
		for (auto& hl : m_mesh->query(RecordType::DB_HAD_LINK)) {
			DbLink* link = (DbLink*)hl;

			if (!isDividedRoadZGen(*link)
				|| visitedLinks.count(link->uuid) > 0)
				continue;

			// ץȡ�Բ��·
			DirLink oneSideLink{ link, true };
			bool anotherSideLinkOnLeft;
			DirLink anotherSideLink = grabAnotherRoadZGen(oneSideLink, anotherSideLinkOnLeft);

			if (!anotherSideLink.valid()
				|| visitedLinks.count(anotherSideLink.link()->uuid) > 0)
				continue;

			visitedLinks.insert(oneSideLink.link()->uuid);
			visitedLinks.insert(anotherSideLink.link()->uuid);

			// TODO: �õ���ߺ��ұߵ�link����Ϊ����linkͬ������һ����������֮�֡�
			DirLink leftSideLink = anotherSideLink;
			DirLink rightSideLink = oneSideLink;
			if (!anotherSideLinkOnLeft)
				std::swap(leftSideLink, rightSideLink);

			DirLinkPath leftPath = extendRoad(leftSideLink, true, visitedLinks);
			DirLinkPath rightPath = extendRoad(rightSideLink, false, visitedLinks);
			dividedRoads.push_back(DividedRoad{ leftPath, rightPath });
		}

		return dividedRoads;
	}

}
}