#include "stdafx.h"
#include <algorithm>
#include "GreenbeltUrbanCompiler.h"
#include "../framework/SpatialSeacher.h"
#include "clipper.hpp"
#include "math3d/vector_math.h"
#include "algorithm/grap_point_algorithm.h"
namespace OMDB
{

	// 构造函数
	GreenbeltUrbanCompiler::GreenbeltUrbanCompiler(CompilerData& data)
	:Compiler(data),
	m_1stLaneGroups(),
	m_2ndLaneGroups(),
#ifdef __DEBUG_GREENBELTURBAN__
	m_AllLaneGroups(),
	m_debugofs()
#else
	m_AllLaneGroups()
#endif
	{

	};

	// 编译开始
	void GreenbeltUrbanCompiler::compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile)
	{

#ifdef __DEBUG_GREENBELTURBAN__
		m_debugofs = std::ofstream(std::to_string(pGrid->getId())+".txt", std::ios_base::out | std::ios_base::trunc);
		//printf("---------------compile::start()  pGrid[m_meshId=%d]----------------------", pGrid->getId());
#endif

		std::vector<HadGrid*> nearbyGrids = { pGrid };
		for_each(nearby.begin(), nearby.end(), [&](HadGrid* g)->void {nearbyGrids.push_back(g); });

		std::vector<HadLaneGroup*> laneGroups;
		getGreenbeltLaneGroups(pGrid, laneGroups);		

		m_AllLaneGroups.clear();
		for (auto obj : laneGroups)
		{
			m_1stLaneGroups.clear();
			m_2ndLaneGroups.clear();
			HadLaneGroup* pGroup = (HadLaneGroup*)obj;

			if(std::find(m_AllLaneGroups.begin(), m_AllLaneGroups.end(), pGroup) != m_AllLaneGroups.end())
			{
				continue;
			}

#ifdef __DEBUG_GREENBELTURBAN__
			printf("=====compile::start() laneGroups(laneGroups.roadBoundaries[0].originId[%lld])=====\n",
				obj->roadBoundaries.size() ? (*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId : 0);
			PrintLaneGroup(pGroup, "inLaneGroup start:"); //debug
#endif
		
			std::vector<HadLaneGroup*> pNearbyLinkGroups;
			if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (225256394329629957))
//			if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227955360334811393))
			{
				expandConnectionLaneGroups(pGroup, nearbyGrids, pNearbyLinkGroups);  // 先做当前pGroup的连接。提前做出一条边。抓取的时候也容易排除抓取到同一条边的情况
				if (pNearbyLinkGroups.empty())
				{
					continue;
				}
				m_1stLaneGroups = pNearbyLinkGroups;

				HadLaneGroup* otherLaneGroup = nullptr;
				pNearbyLinkGroups.clear();
				bool bRet = grabOtherLaneGroup(pGrid, pGroup, otherLaneGroup);
				if (!bRet || !otherLaneGroup)
				{					
					continue;
				}

#ifdef __DEBUG_GREENBELTURBAN__
				PrintLaneGroup(pGroup, "inLaneGroup OK:"); //debug
				PrintLaneGroup(otherLaneGroup, "grabLaneGroup OK:"); //debug
#endif

				expandConnectionLaneGroups(otherLaneGroup, nearbyGrids, pNearbyLinkGroups);  // 通过抓取到的LaneGroup，做出另一条隔离带
				if (pNearbyLinkGroups.empty())
				{					
					continue;
				}
				m_2ndLaneGroups = pNearbyLinkGroups;

				alignLaneGroupsbylength(m_1stLaneGroups, m_2ndLaneGroups);
								
				std::vector<MapPoint3D64> Points1st;
				getPointsByLaneGroups(m_1stLaneGroups,Points1st);

				std::vector <MapPoint3D64> Points2nd;
				getPointsByLaneGroups(m_2ndLaneGroups,Points2nd);

				if (alignPoints(Points1st, Points2nd)==false)
				{
					continue;
				}

				RdsGreenbelt* pGreenbelt = (RdsGreenbelt*)createObject(pTile, EntityType::RDS_GREENBELT); // debug 复用高速道路
			//	RdsGreenbeltUrban* pGreenbelt = (RdsGreenbeltUrban*)createObject(pTile, EntityType::RDS_GREENBELT_URBAN);
				OMDB::LineString3d location = { Points1st };
				RDS::LineString3d line;
				convert(location, line);
				pGreenbelt->contour.lines.push_back(line);

				OMDB::LineString3d nearbyLocation = { Points2nd };
				RDS::LineString3d nearbyLine;
				convert(nearbyLocation, nearbyLine);
				pGreenbelt->contour.lines.push_back(nearbyLine);

				m_AllLaneGroups.insert(m_AllLaneGroups.end(), m_1stLaneGroups.begin(), m_1stLaneGroups.end());
				m_AllLaneGroups.insert(m_AllLaneGroups.end(), m_2ndLaneGroups.begin(), m_2ndLaneGroups.end());

#ifdef __DEBUG_GREENBELTURBAN__
				PrintPoints(Points1st, "Points1st"); //debug
				PrintPoints(Points2nd, "Points2nd"); //debug
				PrintLaneGroup(pGroup, "inLaneGroup end:"); //debug
#endif

			}			

		}

#ifdef __DEBUG_GREENBELTURBAN__	    
		m_debugofs.close();
#endif

	}

	//  得到能制作普通隔离带的道路的LaneGroups
	bool GreenbeltUrbanCompiler::getGreenbeltLaneGroups(HadGrid* const pGrid, std::vector<HadLaneGroup*>& laneGroups)
	{		
		for (auto obj : pGrid->query(ElementType::HAD_LANE_GROUP))
		{
			HadLaneGroup* pGroup = (HadLaneGroup*)obj;

			//PrintLaneGroup(pGroup); //debug

			//得到普通路绿化带的lanegroup
			if(ableGreenbeltLaneGroup(pGroup)) // 
			{
				laneGroups.push_back(pGroup);

#ifdef __DEBUG_GREENBELTURBAN__	    
				bool isRet1 = isProDataLevel(pGroup);
				bool isRet2 = isMultiDigitized(pGroup);
				std::string str = "isProDataLevel[" + std::to_string(isRet1) + "],isMultiDigitized[" + std::to_string(isRet2) + "]," + "inIntersection[" + std::to_string(pGroup->inIntersection) + "]";
				PrintLaneGroup(pGroup,"able greenbelt:");
#endif
			}
			else
			{

#ifdef __DEBUG_GREENBELTURBAN__	    
				bool isRet1 = isProDataLevel(pGroup);
				bool isRet2 = isMultiDigitized(pGroup);				
				std::string str = "isProDataLevel[" + std::to_string(isRet1) + "],isMultiDigitized[" + std::to_string(isRet2) + "]," + "inIntersection[" + std::to_string(pGroup->inIntersection) + "]";
				PrintLaneGroup(pGroup, "not greenbelt:"+ str);
#endif

			}
		}

		return true;
	}

	// 抓得对向道路的LaneGroup
	bool GreenbeltUrbanCompiler::grabOtherLaneGroup(HadGrid* const pGrid,HadLaneGroup* inLaneGroup, HadLaneGroup*& outNearLaneGroup)
	{

		std::vector<HadLaneGroup*> NearbyLaneGroups = SpatialSeacher::seachNearby2d(pGrid, inLaneGroup, 1e4);

		getNearbyLaneGropuByAngle(inLaneGroup, NearbyLaneGroups); // 按照一定角度取得周边的lanegroups

		getNearbyLaneGropuByDistance(inLaneGroup, NearbyLaneGroups, outNearLaneGroup);

		// 不同的LaneGrop存在相同的roadboundary

		if (outNearLaneGroup && (inLaneGroup->originId != outNearLaneGroup->originId))
		{
			if (inLaneGroup->roadBoundaries.size() && outNearLaneGroup->roadBoundaries.size())
			{
				if (inLaneGroup->roadBoundaries[0]->originId == outNearLaneGroup->roadBoundaries[0]->originId)
				{
					outNearLaneGroup = nullptr;
					return false;
				}
			}
			else if(inLaneGroup->laneBoundaries.size() && outNearLaneGroup->laneBoundaries.size())
			{
				if (inLaneGroup->laneBoundaries[0]->originId == outNearLaneGroup->laneBoundaries[0]->originId)
				{
					outNearLaneGroup = nullptr;
					return false;
				}
			}
		}
				
		return true;
	}

	// 得到周边一定角度范围内的LaneGroups。（删除inoutNearbyLaneGroups中，不满足角度要求的）
	bool GreenbeltUrbanCompiler::getNearbyLaneGropuByAngle(HadLaneGroup* inLaneGroup, std::vector<HadLaneGroup*>& inoutNearbyLaneGroups)
	{
		std::vector<HadLaneGroup*>  inlaneGropus = inoutNearbyLaneGroups;
		std::vector<HadLaneGroup*>  outlaneGropus;
		
		auto getLinkGroupAngle = [&](HadLaneGroup* inLaneGroup, HadLaneGroup* currentLaneGroup)
		{
			LineString3d inLine;
			if (inLaneGroup->roadBoundaries.size())
			{
				inLine = inLaneGroup->roadBoundaries[0]->location;
			}
			else if (inLaneGroup->laneBoundaries.size())
			{
				inLine = inLaneGroup->laneBoundaries[0]->location;
			}
			else
			{
				assert(0);
			}

			LineString3d currentLine;
			if (currentLaneGroup->roadBoundaries.size())
			{
				currentLine = currentLaneGroup->roadBoundaries[0]->location;
			}
			else if (currentLaneGroup->laneBoundaries.size())
			{
				currentLine = currentLaneGroup->laneBoundaries[0]->location;
			}
			else
			{
				assert(0);
			}
			int64  a = inLine.vertexes.front().pos.lon;
			point_2t startPtDir = { inLine.vertexes.front().pos.lon - inLine.vertexes.back().pos.lon , inLine.vertexes.front().pos.lat - inLine.vertexes.back().pos.lat };
			point_2t endPtDir = { currentLine.vertexes.front().pos.lon - currentLine.vertexes.back().pos.lon , currentLine.vertexes.front().pos.lat - currentLine.vertexes.back().pos.lat };
			uint16 uiAngle = minimalDegree(startPtDir, endPtDir);
			return uiAngle;
		};

		// 当前取到的currentLaneGroup同inLaneGroup在一条路上，也不需要。
		auto isInConnectionLaneGroups = [&,this](HadLaneGroup* inLaneGroup)
		{
			for (auto obj : this->m_1stLaneGroups)
			{
				if (obj->originId == inLaneGroup->originId)
				{
					return true;
				}
			}

			return false;
		};

		for(auto laneGroup : inlaneGropus)
		{
			if (isInConnectionLaneGroups(laneGroup) == false && ableGreenbeltLaneGroup(laneGroup)) // 过滤掉无效的lanegroup
			{
				if (getLinkGroupAngle(inLaneGroup, laneGroup) < 30) // 小于30度考虑抓取
				{				
					outlaneGropus.push_back(laneGroup);
				}				
			}			
		}
		inoutNearbyLaneGroups = outlaneGropus;
		return true;
	}
	
	// 得到距离（投影）最短的outNearLaneGroup
	bool GreenbeltUrbanCompiler::getNearbyLaneGropuByDistance(HadLaneGroup* inLaneGroup, std::vector<HadLaneGroup*>& inNearbyLaneGroups, HadLaneGroup*& outNearLaneGroup)
	{	
		//struct
		//{
		//	HadLaneGroup* laneGroup;
		//	double distance;
		//}laneGroupDistance;

		//std::map<HadLaneGroup*, double> laneGroupDistance;
		
		std::vector<HadLaneGroup*>  inlaneGropus = inNearbyLaneGroups;

		auto getLinkGroupDistance = [&](HadLaneGroup* inLaneGroup, HadLaneGroup* currentLaneGroup)
		{
			LineString3d inLine;
			if (inLaneGroup->roadBoundaries.size())
			{
				inLine = inLaneGroup->roadBoundaries[0]->location;
			}
			else if (inLaneGroup->laneBoundaries.size())
			{
				inLine = inLaneGroup->laneBoundaries[0]->location;
			}
			else
			{
				assert(0);
			}

			point_2t a= { inLine.vertexes.front().pos.lon ,inLine.vertexes.front().pos.lat }; // debug
			point_2t b = { inLine.vertexes.back().pos.lon ,inLine.vertexes.back().pos.lat };  // debug 
			point_2t sourcePoint = { (inLine.vertexes.front().pos.lon + inLine.vertexes.back().pos.lon)/2 ,(inLine.vertexes.front().pos.lat + inLine.vertexes.back().pos.lat)/2 }; 

			LineString3d currentLine;
			if (currentLaneGroup->roadBoundaries.size())
			{
				currentLine = currentLaneGroup->roadBoundaries[0]->location;
			}
			else if (currentLaneGroup->laneBoundaries.size())
			{
				currentLine = currentLaneGroup->laneBoundaries[0]->location;
			}
			else
			{
				assert(0);
			}
						
			point_2t startPt = { currentLine.vertexes.front().pos.lon,currentLine.vertexes.front().pos.lat };
			point_2t endPt =   { currentLine.vertexes.back().pos.lon, currentLine.vertexes.back().pos.lat };
			segment_2t segment = { startPt,endPt };

			std::vector<point_2t> tmpSegVector;
			tmpSegVector.push_back(startPt);
			tmpSegVector.push_back(endPt);
			std::vector<point_2t>::iterator ita, itb;
			getClosestSeg(sourcePoint, tmpSegVector, ita, itb);
			a = *ita;
			b = *itb;
			double distance = bg::distance(sourcePoint, segment);  // 中心点到线段，取最短距离。
			//laneGroupDistance.insert(map_pair)
			return distance;
		};

		double distance = DBL_MAX;
		for (auto laneGroup : inlaneGropus)
		{
			double tempDistance = getLinkGroupDistance(inLaneGroup, laneGroup);
			if (tempDistance < distance)
			{
				distance = tempDistance;
				outNearLaneGroup = laneGroup;
			}
		}

		return true;
	}

	// 通过inLaneGroup得到previou，next的接续
	bool GreenbeltUrbanCompiler::expandConnectionLaneGroups(HadLaneGroup* inLaneGroup, const std::vector<HadGrid*>& nearby, std::vector<HadLaneGroup*>& outConnectionLaneGroups)
	{
		enum Direction
		{
			previous = 0,
			next
		};
		
		auto getNextLaneGroup = [&](HadLaneGroup* inLaneGroup,Direction dir)
		{
			HadLaneGroup* LaneGroup = nullptr;
			std::map<int64, HadLaneGroup*> laneGroups;
			if (dir== previous)
			{
				if (inLaneGroup->roadBoundaries.size())
				{
					if (inLaneGroup->roadBoundaries[0]->previous.size())
					{
						HadRoadBoundary* predRoadBoundary = static_cast<HadRoadBoundary*>(inLaneGroup->roadBoundaries[0]->previous[0]);
						laneGroups = predRoadBoundary->linkGroups;
					}
				}
				else
				{
					if (inLaneGroup->laneBoundaries[0]->previous.size())
					{
						HadLaneBoundary* predLaneBoundary = static_cast<HadLaneBoundary*>(inLaneGroup->laneBoundaries[0]->previous[0]);
						laneGroups = predLaneBoundary->linkGroups;
					}
				}
			}
			else
			{
				if (inLaneGroup->roadBoundaries.size())
				{
					if (inLaneGroup->roadBoundaries[0]->next.size())
					{
						HadRoadBoundary* predRoadBoundary = static_cast<HadRoadBoundary*>(inLaneGroup->roadBoundaries[0]->next[0]);
						laneGroups = predRoadBoundary->linkGroups;
					}
				}
				else
				{
					if (inLaneGroup->laneBoundaries[0]->next.size())
					{
						HadLaneBoundary* predLaneBoundary = static_cast<HadLaneBoundary*>(inLaneGroup->laneBoundaries[0]->next[0]);
						laneGroups = predLaneBoundary->linkGroups;
					}
				}
			}

			if (laneGroups.size() == 1)
			{
				auto it = laneGroups.begin();
				LaneGroup = it->second;
			}
			else
			{
				// 两个laneGroup共用一个roadboundary
			}

			return LaneGroup;

		};

		auto availableLaneGroup = [&](HadLaneGroup* inLaneGroup)
		{
			if (inLaneGroup->inIntersection != 0)
			{
				return false;
			}
			return true;
		};

		std::vector<HadLaneGroup*> preLaneGroups;
		std::vector<HadLaneGroup*> nextLaneGroups;
		HadLaneGroup* currentLaneGroup = inLaneGroup;

		for(;;)
		{
			currentLaneGroup = getNextLaneGroup(currentLaneGroup, previous);
			if (currentLaneGroup && (availableLaneGroup(currentLaneGroup)==true))
			{
				preLaneGroups.push_back(currentLaneGroup);

#ifdef __DEBUG_GREENBELTURBAN__
				PrintLaneGroup(currentLaneGroup, "Prev->");
				printf("expandConnectionLaneGroups(previous ) currentLaneGroup(laneGroups.roadBoundaries[0].originId[%lld])\n",
					(*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)currentLaneGroup->roadBoundaries[0])))).originId);
#endif
			}
			else
			{
				break;
			}			
		}
		std::reverse(preLaneGroups.begin(), preLaneGroups.end());  // 从头开始
		preLaneGroups.push_back(inLaneGroup);
		outConnectionLaneGroups = preLaneGroups;

		currentLaneGroup = inLaneGroup;
		for (;;)
		{
			currentLaneGroup = getNextLaneGroup(currentLaneGroup, next);
			if (currentLaneGroup && (availableLaneGroup(currentLaneGroup) == true))
			{
				nextLaneGroups.push_back(currentLaneGroup);

#ifdef __DEBUG_GREENBELTURBAN__
				PrintLaneGroup(currentLaneGroup, "Next->"); //debug
				printf("expandConnectionLaneGroups( next ) currentLaneGroup(laneGroups.roadBoundaries[0].originId[%lld])\n",
					(*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)currentLaneGroup->roadBoundaries[0])))).originId);  //debug
#endif
			}
			else
			{
				break;
			}
		}
		outConnectionLaneGroups.insert(outConnectionLaneGroups.end(), nextLaneGroups.begin(), nextLaneGroups.end());

		
		size_t i = nearby.size();

		return true;
	}

	bool GreenbeltUrbanCompiler::ableGreenbeltLaneGroup(HadLaneGroup* inLaneGroup)
	{
		
		if (!inLaneGroup)
		{
			assert(0);
			return false;
		}
		// 普通路，上线分离，不是路口
		bool isRet1 = isProDataLevel(inLaneGroup);
		bool isRet2 = isMultiDigitized(inLaneGroup);
		if (isProDataLevel(inLaneGroup)==false
			&& isMultiDigitized(inLaneGroup)==false
			&& inLaneGroup->inIntersection == 0
			)  		
		{
			return true;
		}
		return false;
	}


	// 得到LaneGroups中对应的Points
	bool GreenbeltUrbanCompiler::getPointsByLaneGroups(std::vector<HadLaneGroup*> inLaneGroups, std::vector<MapPoint3D64>& Points)
	{
		std::vector<MapPoint3D64>  currentPoints;
		for (auto obj : inLaneGroups)
		{
			if (obj->roadBoundaries.size())
			{
				if (obj->roadBoundaries[0]->location.vertexes.size())
				{					
					currentPoints.insert(currentPoints.end(), obj->roadBoundaries[0]->location.vertexes.begin(), obj->roadBoundaries[0]->location.vertexes.end());
				}
				else
				{
					assert(0);
					return false;
				}
			}
			else
			{
				if (obj->laneBoundaries[0]->location.vertexes.size())
				{
					currentPoints.insert(currentPoints.end(), obj->laneBoundaries[0]->location.vertexes.begin(), obj->laneBoundaries[0]->location.vertexes.end());
				}
				else
				{
					assert(0);
					return false;
				}
			}
		}

		for (int i =0;i < currentPoints.size();i++)
		{
			if (currentPoints[i] != currentPoints[i+1] && i!= currentPoints.size()) // 有重复相邻点的时候，保留一个
			{
				Points.push_back(currentPoints[i]);
			}
		}
		return true;
	}

	bool GreenbeltUrbanCompiler::alignLaneGroupsbylength(std::vector<HadLaneGroup*>& in1stLaneGroups, std::vector<HadLaneGroup*>& in2ndLaneGroups)
	{
		struct LaneGroupPoints
		{
			HadLaneGroup* lanegroup;
			std::vector<MapPoint3D64> Points;
		};

		auto getLaneGroupPoints = [&](std::vector<HadLaneGroup*>& inLaneGroups, std::vector<LaneGroupPoints>& outLaneGroupsPoints)
		{
			for (auto it : inLaneGroups)
			{
				LaneGroupPoints st;
				st.lanegroup = it;
				std::vector<HadLaneGroup*> LaneGroups;
				LaneGroups.push_back(it);
				std::vector<MapPoint3D64> temPoints;
				getPointsByLaneGroups(LaneGroups, temPoints);
				st.Points = temPoints;
				outLaneGroupsPoints.push_back(st);
			}
		};

		std::vector<LaneGroupPoints> in1stLaneGroupsPoints;
		getLaneGroupPoints(in1stLaneGroups, in1stLaneGroupsPoints);

		std::vector<LaneGroupPoints> in2ndLaneGroupsPoints;
		getLaneGroupPoints(in2ndLaneGroups, in2ndLaneGroupsPoints);


		std::vector<MapPoint3D64> Points1st;
		getPointsByLaneGroups(in1stLaneGroups, Points1st);
		double leng1stLaneGropups = calcLength(Points1st);

		std::vector<MapPoint3D64> Points2nd;
		getPointsByLaneGroups(in2ndLaneGroups, Points2nd);
		double leng2ndLaneGropups = calcLength(Points2nd);

		std::vector<MapPoint3D64>::iterator ita, itb;		
		getClosestSeg(Points1st.front(), Points2nd, ita, itb);
		if(ita== Points2nd.begin())  // Points1st头长于Points2nd
		{
		//	point_2t  start = POINT_2T(Points1st.front());
		//	point_2t  end = POINT_2T(Points2nd.front());
		//	point_2t startPoints1st = POINT_2T(points1st.front());
		//	bg::length(start,end);
			float len = calcLength(Points1st.front(), *Points2nd.begin());
			if (len > 10000)
			{

			}

		}
		else if (itb == Points2nd.end() - 1)
		{
			float len = calcLength(Points1st.front(), *(Points2nd.end() - 1));
			if (len > 10000)
			{

			}

		}
			
		getClosestSeg(Points1st.back(), Points2nd, ita, itb);
		if (ita == Points2nd.begin())  // Points1st尾长于Points2nd
		{
			float len = calcLength(Points1st.back(), *(Points2nd.begin())); //10000(10m)
			if (len > 10000)
			{

			}
		}
		else  if (itb == Points2nd.end() - 1)
		{
			float len = calcLength(Points1st.back(), * (Points2nd.end() - 1));
			if (len > 10000)
			{

			}
		}
	









		auto delLaneGroupbylength = [&,this](std::vector<HadLaneGroup*>& inLaneGroups, double length)
		{
			std::vector<MapPoint3D64> Points;
			std::vector<HadLaneGroup*> LaneGroups;
			for (auto it : inLaneGroups)
			{
				LaneGroups.push_back(it);
				std::vector<MapPoint3D64> temPoints;
				getPointsByLaneGroups(LaneGroups, temPoints);
				Points.insert(Points.end(), temPoints.begin(), temPoints.end());
				double len = calcLength(Points);
				if (len> length)
				{
					inLaneGroups.clear();
					inLaneGroups = LaneGroups;				
				}
			}
			return true;
		};



		if (leng1stLaneGropups> leng2ndLaneGropups)
		{

			delLaneGroupbylength(in1stLaneGroups, leng2ndLaneGropups);
		}
		else
		{
			delLaneGroupbylength(in2ndLaneGroups, leng1stLaneGropups);
		}

		return true;
	}


	bool GreenbeltUrbanCompiler::alignPoints(std::vector <MapPoint3D64>& points1st, std::vector <MapPoint3D64>& points2nd)
	{
	//	point_2t startPoints1st = POINT_2T(points1st.front());
		point_t  startPoints1st = POINT_T(points1st.front());
	//	point_2t endPoints1st = POINT_2T(points1st.back());
		point_t endPoints1st = POINT_T(points1st.back());

	//	point_2t startPoints2nd = POINT_2T(points2nd.front());
		point_t startPoints2nd = POINT_T(points2nd.front());
	//	point_2t endPoints2nd = POINT_2T(points2nd.back());
		point_t endPoints2nd = POINT_T(points2nd.back());

		linestring_2t line2t1st = LINESTRING_2T(points1st);
		linestring_t  line1st = LINESTRING_T(points1st);
		linestring_2t line2t2nd = LINESTRING_2T(points2nd);
		linestring_t  line2nd = LINESTRING_T(points1st);


		std::vector<point_t>::iterator ita, itb;
		getClosestSeg(startPoints1st, line2nd, ita, itb);


		getClosestSeg(endPoints1st, line1st, ita, itb);


		getClosestSeg(startPoints2nd, line1st, ita, itb);

		getClosestSeg(endPoints2nd, line1st, ita, itb);

		//bool ret = bg::within(line1st, line2nd);

		bool ret = bg::intersects(line2t1st, line2t2nd);
		if (ret)
		{
			return false;
		}


		point_t grapedPt; // 判断那头长，那头短。能投到对面说明短。从短的做映射
		if(GRAP_POINT(startPoints1st, segment_t(line2nd.front(), line2nd.back()), grapedPt, 300))
		{
			getClosestSeg(startPoints1st, line2nd, ita, itb);
			if (GRAP_POINT(startPoints1st, segment_t(*ita, *itb), grapedPt, 300))
			{
				line2nd.erase(itb, line2nd.end());
				line2nd.push_back(grapedPt);
			}
			else
			{
				assert(0); // 不能投上
				return false;
			}
		}
		else if(GRAP_POINT(endPoints2nd, segment_t(line1st.front(), line1st.back()), grapedPt, 300)) //0.3m
		{
			getClosestSeg(endPoints2nd, line1st, ita, itb);
			if (GRAP_POINT(endPoints2nd, segment_t(*ita, *itb), grapedPt, 300))
			{
				line1st.insert(itb, grapedPt); //ita,itb之间插入。
				line1st.erase(line1st.begin(), itb); // 删除插入点之前的所有点
			}
			else
			{
				assert(0); // 不能投上
				return false;
			}

		}

		if (GRAP_POINT(endPoints1st, segment_t(line2nd.front(), line2nd.back()), grapedPt, 300))
		{
			getClosestSeg(endPoints1st, line2nd, ita, itb);
			if (GRAP_POINT(endPoints1st, segment_t(*ita, *itb), grapedPt, 300))
			{
				line2nd.insert(itb, grapedPt);
				line2nd.erase(line2nd.begin(), itb);
			}
			else
			{
				assert(0); // 不能投上
				return false;
			}

		}
		else if (GRAP_POINT(startPoints2nd, segment_t(line1st.front(), line1st.back()), grapedPt, 300))
		{
			getClosestSeg(endPoints1st, line1st, ita, itb);
			if (GRAP_POINT(endPoints1st, segment_t(*ita, *itb), grapedPt, 300))
			{
				line1st.erase(itb, line2nd.end());
				line1st.push_back(grapedPt);
			}
			else
			{
				assert(0); // 不能投上
				return false;
			}
		}

		return true;
	}



#ifdef __DEBUG_GREENBELTURBAN__

	void GreenbeltUrbanCompiler::PrintPoints(std::vector<MapPoint3D64> lanepoints, std::string mark)
	{
		
		std::string temp = mark+"(lon,lat)\n";

		for (auto ite : lanepoints)
		{
			temp += std::to_string(ite.pos.lon);
			temp += ",";
			temp += std::to_string(ite.pos.lat);
		//	temp += ",";
		//	temp += std::to_string(ite.z);
			temp += "\n";
		}
		m_debugofs << temp.c_str();
		
	}

	void GreenbeltUrbanCompiler::PrintLaneGroup(HadLaneGroup* inLaneGroup, std::string mark)
	{

		std::string temp = mark+"LaneGroup[" + std::to_string(inLaneGroup->originId) + "],";

		std::string strRoad;
		std::string strLane;
		if (inLaneGroup->roadBoundaries.size())
		{
			strRoad = "roadBoundaries[0]=(" + std::to_string(inLaneGroup->roadBoundaries[0]->originId) +"),";
		}
		else if(inLaneGroup->laneBoundaries.size())
		{
			strLane = "laneBoundaries[0]=(" + std::to_string(inLaneGroup->laneBoundaries[0]->originId) + "),";
		}

		temp += strRoad;
		temp += strLane;
		temp += "\n";

		m_debugofs << temp.c_str();
		
	}
#endif

}