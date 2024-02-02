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
	 * @brief 判断从角1转向角2是否需要左转或者右转。角1 & 角2 均为弧度。
	 * @return true -> 左转, false -> 右转
	*/
	bool angleTurnLeft(double angle1, double angle2)
	{
		std::array<double, 2> target{ std::cos(angle2), std::sin(angle2) };
		std::array<double, 2> heading{ std::cos(angle1), std::sin(angle1) };

		double cross = target[0] * heading[1] - target[1] * heading[0];

		return cross < 0.0;
	}

	/**
	 * @brief 判断两个角度是否同向。角1 & 角2 均为弧度。
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
	 * @brief 将outlinks按照startHeadingAngle从小到大排序，按照nextLinkInCCW取当前link下一个link。
	 * @param outlinks 
	 * @param curLink 
	 * @param nextLinkInCCW 
	 * @return 
	*/
	static DirLink getNextDirLink(std::vector<DirLink>& outlinks, DirLink& curLink, bool nextLinkInCCW)
	{
		// outlinks按照startHeadingAngle从小到大排序
		std::sort(outlinks.begin(), outlinks.end(), [](DirLink& l, DirLink& r) {return l.startHeadingAngle() < r.startHeadingAngle(); });
		// 找到curLink索引
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
	 * @brief 计算抓路中使用的Link形状线的方向，单位弧度。该方向通过与指定点最邻近的线上的部分来计算。
	 * @param linestring
	 * @param point
	 * @return
	*/
	double findLineDirectionWithPoint(const LineString3d& linestring, const MapPoint3D64& point, MapPoint64& nearestPointOnLineString)
	{
		using Point = std::array<double, 2>; // 等同于MapPoint64，只是使用double存储原来的int64
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

				// nearestPointType: 0 -> 线段上与指定点最近的点为线段起点
				//                   1 -> 线段上与指定点最近的点为线段终点
				//                   3 -> 线段上与指定点最近的点为线段中间点

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
		std::size_t nearestSegmentIndex; // 最近的线段的终点在linestring顶点的索引
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
				// 最邻近点在为线段起点，使用该点前后线段方向角之平均
				// 注意临界情况，不存在前线段
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
				// 最邻近点在为线段终点，使用该点前后线段方向角之平均
				// 注意临界情况，不存在后线段
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
				// 最邻近点在线段中间，使用该线段计算方向角
				return map_pt_math::twoPointAngle(MapPoint64OnLine(nearestSegmentIndex - 1), MapPoint64OnLine(nearestSegmentIndex));
			}
		}
	}

	DivicedRoadSearcher::DivicedRoadSearcher(DbMesh* mesh) : m_mesh(mesh)
	{
	}

	/**
	 * @brief 从Link上的指定点向一定距离范围内搜索其它link，并且搜索的结果按照与direction方向的平行程度从大到小排序，返回结果中越前面的link越平行。
	 * @param link 当前侧道路
	 * @param point 当前侧道路
	 * @param maxFindDist 最大搜索距离。单位经纬度。
	 * @param direction 方向，弧度。
	 * @return
	*/
	std::vector<DivicedRoadSearcher::NeighborLink> DivicedRoadSearcher::searchParallelNeighborLinks(DbLink& link, const MapPoint3D64& point, const double maxFindDist, const double direction)
	{
		// 判断两个夹角(单位radians, 范围[0, 2PI) )之间平行程度，返回两夹角之间的夹角大小，正数，无方向，值越小表明两个夹角越接近。
		auto angleParallelDiff = [](double angle1, double angle2) -> double
			{
				double maxAngle = max(angle1, angle2);
				double minAngle = min(angle1, angle2);

				double diff = maxAngle - minAngle;
				// diff分别和0度、180度、360度的差异绝对值的最小值
				return min(min(diff, std::abs(diff - MATH_PI_D)), std::abs(2.0 * MATH_PI_D - diff));
			};

		// 以point为圆心，maxFindDist为半径的圆内的其它所有Link的数组
		std::vector<DbLink*> neighborLinks = RTreeSeacher::seachNearby2d(m_mesh, &link, point, maxFindDist);
		// 将上述搜索结果按照道路方向相似程度从大到小排序
		std::vector<NeighborLink> neighborLinksParallel; // 其中pair.second为角度差异
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
		// 排序
		std::sort(neighborLinksParallel.begin(), neighborLinksParallel.end(), [](auto& l, auto& r) {return l.directionDiff < r.directionDiff; });

		return neighborLinksParallel;
	}

	bool DivicedRoadSearcher::isDividedRoadZGen(DbLink& link)
	{
		// TODO: 判断Link是否是隧道路，隧道路不认为是上下行道路
		// TODO: 判断Link是否是路口内路段

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

				// segment缩短约10cm，足矣
				std::array<MapPoint64, 2> segment = map_pt_math::shrinkSegment({ segStart, segEnd }, 100);


				segment_2t bgSegmentNew{ POINT_2T(segment[0]), POINT_2T(segment[1])};

				// 
				return RTreeSeacher::intersectsWithLinks2d(pGrid, segment[0], segment[1]);
			};


		auto isLeftRightParalel = [](const LineString3d &line1, const LineString3d &line2)
			{
				linestring_2t bgLine1 = LINESTRING_2T(line1.vertexes);
				linestring_2t bgLine2 = LINESTRING_2T(line2.vertexes);

				// 取line2的起终点到Line1的最邻近点，如果这两个最邻近点都相同则认为这两条线不是左右平行
				segment_2t seg1;
				bg::closest_points(bgLine2.front(), bgLine1, seg1);

				segment_2t seg2;
				bg::closest_points(bgLine2.back(), bgLine1, seg2);

				bool isLeftRightParalel = seg1.second != seg2.second;

				return isLeftRightParalel;
			};

		// Note: MapPoint64在以半径为6378137的球体上的精度为0.0011131949079327358米，约为0.111cm
		// 故而此处30000单位最远可找约33.3米
		int maxFindDist = 30000;
		std::vector<NeighborLink> paralelLinks = searchParallelNeighborLinks(*link.link(), pointOnLink, maxFindDist, startHeadingAngle);
		// 从最平行的道路开始找，最有可能是上下行的另一对
		for (auto& parallelLink : paralelLinks)
		{
			if (Generator::containsLinkName(parallelLink.link, linkName) // 名称相同
				&& linkOutlinksUUID.count(parallelLink.link->uuid) == 0 // 非与当前道路相连的其它道路
				&& !intersectsWithOtherLink(m_mesh, pointOnLink.pos, parallelLink.nearestPointOnLink) // 不能横跨其它道路
				&& isLeftRightParalel(link.link()->geometry, parallelLink.link->geometry) // 确保两条道路是左右平行（类似于 ||）而不是前后平行（类似于 --）
				)
			{
				// 从当前道路指向对侧道路最邻近点的向量转向道路方向是否往左
				double angleToOpposite = map_pt_math::twoPointAngle(pointOnLink.pos, parallelLink.nearestPointOnLink);
				bool linkIsLeft = angleTurnLeft(angleToOpposite, startHeadingAngleNegativeDir);
				grabRoadIsLeft = !linkIsLeft;

				// 判断当前link与找到的link是否共向，然后生成共向的DirLink
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
					if (outlinks.size() != 2) // 不能碰到第三条道路否则停止搜索
						break;

					DirLink nextLink = getNextDirLink(outlinks, curLink, nextRoadInCCW);

					if (!nextLink.valid()
						|| !isDividedRoadZGen(*nextLink.link()))
						break;
					// 当前道路和下一道路都为上下行，其它出边不为上下行
					{
						int dividedRoadNum = 0;
						for (DirLink& outlink : outlinks)
							dividedRoadNum += static_cast<int>(isDividedRoadZGen(*outlink.link()));
						if (dividedRoadNum != 2)
							break;
					}

					// 当前道路和下一道路名称相同
					if (!Generator::containsLinkName(nextLink.link(), beginLinkName))
						break;

					// 下一道路已经纳入其它上下行道路
					if (visitedLinks.count(nextLink.link()->uuid) > 0)
						break;

					visitedLinks.insert(nextLink.link()->uuid);
					curLink = nextLink;

					path.addPortion(nextLink);
				}

				return path;
			};

		// 往beginLink反方向搜索
		DirLinkPath pathBackward = extendInOneDirection(beginLink.getReverseDirLink(), !beginLinkOnLeft);

		pathBackward.reverse(); // 此时pathBackward方向已经不是backward了，而是forward
		pathBackward.addPortion(beginLink);

		DirLinkPath pathForward = extendInOneDirection(beginLink, beginLinkOnLeft);

		pathBackward.appendPath(pathForward);

		return pathBackward;
	}

	std::vector<DividedRoad> DivicedRoadSearcher::searchZGen()
	{
		std::set<int64> visitedLinks;

		std::vector<DividedRoad> dividedRoads;

		// TODO: 考虑跨网格
		for (auto& hl : m_mesh->query(RecordType::DB_HAD_LINK)) {
			DbLink* link = (DbLink*)hl;

			if (!isDividedRoadZGen(*link)
				|| visitedLinks.count(link->uuid) > 0)
				continue;

			// 抓取对侧道路
			DirLink oneSideLink{ link, true };
			bool anotherSideLinkOnLeft;
			DirLink anotherSideLink = grabAnotherRoadZGen(oneSideLink, anotherSideLinkOnLeft);

			if (!anotherSideLink.valid()
				|| visitedLinks.count(anotherSideLink.link()->uuid) > 0)
				continue;

			visitedLinks.insert(oneSideLink.link()->uuid);
			visitedLinks.insert(anotherSideLink.link()->uuid);

			// TODO: 得到左边和右边的link。因为两个link同向，所以一定会有左右之分。
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