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
	m_pGrid(nullptr),
	m_1stLaneGroups(),
	m_2ndLaneGroups(),
#ifdef __DEBUG_GREENBELTURBAN__
	m_VisitLaneGroups(),
	m_debugofs()
#else
	m_VisitLaneGroups()
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
		m_pGrid = pGrid;
		std::vector<HadGrid*> nearbyGrids = { pGrid };
		for_each(nearby.begin(), nearby.end(), [&](HadGrid* g)->void {nearbyGrids.push_back(g); });

		std::vector<HadLaneGroup*> laneGroups;
		getGreenbeltLaneGroups(pGrid, laneGroups);		

		m_VisitLaneGroups.clear();
		for (auto obj : laneGroups)
		{
			m_1stLaneGroups.clear();
			m_2ndLaneGroups.clear();
			HadLaneGroup* pGroup = (HadLaneGroup*)obj;

			if(std::find(m_VisitLaneGroups.begin(), m_VisitLaneGroups.end(), pGroup) != m_VisitLaneGroups.end())
			{
				continue;
			}

#ifdef __DEBUG_GREENBELTURBAN__
			//printf("=====compile::start() laneGroups(laneGroups.roadBoundaries[0].originId[%lld])=====\n",
				//obj->roadBoundaries.size() ? (*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId : 0);
			PrintLaneGroup(pGroup, "inLaneGroup start:"); //debug
#endif
		
			std::vector<HadLaneGroup*> p1stLaneGroups;
			std::vector<HadLaneGroup*> p2ndLaneGroups;
//			if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (225256394329629957))
//			if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227955360334811393))
//			if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (225238699592858881)) // debug freeze
//			if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227960477150810369)) //20169185	
//			if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227938694053957633)) //20169192
//			if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (225242841761658117))
//			if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227957495147467009))// 20169187
//			if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227936666317688833))// 20169187
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227957490126885121))// 20169187
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227940624788557825))// 20169187
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227953554456252673))// 20169187	
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227955252629278977))// 20169187
			if (
			    (*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (225621966049452293)// 20169187		
		//	|| (*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227942684338622465)// 20169187
		//	||(*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227955252629278977)
		//	|| (*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (225639467978270472)// 20169187
				)
			{
				expandConnectionLaneGroups(pGroup, nearbyGrids, p1stLaneGroups);  // 先做当前pGroup的连接。提前做出一条边。抓取的时候也容易排除抓取到同一条边的情况
				if (p1stLaneGroups.empty())
				{
					continue;
				}
				m_1stLaneGroups = p1stLaneGroups;

				HadLaneGroup* otherLaneGroup = nullptr;
				bool bRet = grabOtherLaneGroup(pGrid, pGroup, otherLaneGroup);
				if (!bRet || !otherLaneGroup)
				{					
					continue;
				}

#ifdef __DEBUG_GREENBELTURBAN__
				PrintLaneGroup(pGroup, "inLaneGroup OK:"); //debug
				PrintLaneGroup(otherLaneGroup, "grabLaneGroup OK:"); //debug
#endif

				expandConnectionLaneGroups(otherLaneGroup, nearbyGrids, p2ndLaneGroups);  // 通过抓取到的LaneGroup，做出另一条隔离带
				if (p2ndLaneGroups.empty())
				{					
					continue;
				}
				m_2ndLaneGroups = p2ndLaneGroups;

				alignLaneGroupsbylength(p1stLaneGroups, p2ndLaneGroups);

#ifdef __DEBUG_GREENBELTURBAN__
				PrintLaneGroup(m_1stLaneGroups, "Align 1st");
				PrintLaneGroup(m_2ndLaneGroups, "Align 2nd");
#endif
								
				std::vector<MapPoint3D64> Points1st;
				getPointsByLaneGroups(m_1stLaneGroups,Points1st);

				std::vector <MapPoint3D64> Points2nd;
				getPointsByLaneGroups(m_2ndLaneGroups,Points2nd);

				//if (alignPoints(Points1st, Points2nd)==false)
				{
					//continue;
				}


				linestring_2t line2t1st = LINESTRING_2T(Points1st);
			
				linestring_2t line2t2nd = LINESTRING_2T(Points2nd);
								
				if (bg::intersects(line2t1st, line2t2nd)) // 有相交不做隔离带
				{
#ifdef __DEBUG_GREENBELTURBAN__
					PrintPoints(Points1st, "Points1st intersects:"); //debug
					PrintPoints(Points2nd, "Points2nd intersects:"); //debug
#endif
					continue;
				}

			//	RdsGreenbelt* pGreenbelt = (RdsGreenbelt*)createObject(pTile, EntityType::RDS_GREENBELT); // debug 复用高速道路
				RdsGreenbeltUrban* pGreenbelt = (RdsGreenbeltUrban*)createObject(pTile, EntityType::RDS_GREENBELT_URBAN);
				OMDB::LineString3d location = { Points1st };
				RDS::LineString3d line;
				convert(location, line);
				pGreenbelt->contour.lines.push_back(line);

				OMDB::LineString3d nearbyLocation = { Points2nd };
				RDS::LineString3d nearbyLine;
				convert(nearbyLocation, nearbyLine);
				pGreenbelt->contour.lines.push_back(nearbyLine);

				m_VisitLaneGroups.insert(m_VisitLaneGroups.end(), m_1stLaneGroups.begin(), m_1stLaneGroups.end());
				m_VisitLaneGroups.insert(m_VisitLaneGroups.end(), m_2ndLaneGroups.begin(), m_2ndLaneGroups.end());

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
				PrintLaneGroup(pGroup,"able greenbelt:"+ str);
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

	//
	bool GreenbeltUrbanCompiler::getLaneGroupLocationLine(HadLaneGroup* inLaneGroup, LineString3d& line)
	{
		if (inLaneGroup->roadBoundaries.size())
		{
			line = inLaneGroup->roadBoundaries[0]->location;
		}
		else if (inLaneGroup->laneBoundaries.size())
		{
			line = inLaneGroup->laneBoundaries[0]->location;
		}
		else
		{
#ifdef __DEBUG_GREENBELTURBAN__
			printf("=====compile::getLaneGroupPoints() laneGroup(laneGroup.roadBoundaries[0].originId[%lld],\
					laneGroup.laneBoundaries[0].originId[% lld]) points null=====\n",
				inLaneGroup->roadBoundaries[0]->originId, \
				inLaneGroup->laneBoundaries[0]->originId);
			PrintLaneGroup(inLaneGroup, "LaneGroup points null!!!:"); //debug
#endif		
			return false;
		}
		return true;
	}



	// 得到周边一定角度范围内的LaneGroups。（删除inoutNearbyLaneGroups中，不满足角度要求的）
	bool GreenbeltUrbanCompiler::getNearbyLaneGropuByAngle(HadLaneGroup* inLaneGroup, std::vector<HadLaneGroup*>& inoutNearbyLaneGroups)
	{
		std::vector<HadLaneGroup*>  inlaneGropus = inoutNearbyLaneGroups;
		std::vector<HadLaneGroup*>  outlaneGropus;

		auto getLinkGroupAngle = [&,this](HadLaneGroup* inLaneGroup, HadLaneGroup* currentLaneGroup)
		{			
			LineString3d inLine;
			getLaneGroupLocationLine(inLaneGroup, inLine);

			LineString3d currentLine;
			getLaneGroupLocationLine(currentLaneGroup, currentLine);

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
			getLaneGroupLocationLine(inLaneGroup, inLine);

			point_2t a= { inLine.vertexes.front().pos.lon ,inLine.vertexes.front().pos.lat }; // debug
			point_2t b = { inLine.vertexes.back().pos.lon ,inLine.vertexes.back().pos.lat };  // debug 
			point_2t sourcePoint = { (inLine.vertexes.front().pos.lon + inLine.vertexes.back().pos.lon)/2 ,(inLine.vertexes.front().pos.lat + inLine.vertexes.back().pos.lat)/2 }; 


			LineString3d currentLine;			
			getLaneGroupLocationLine(currentLaneGroup, currentLine);
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

		std::vector<HadLaneGroup*> preLaneGroups;
		std::vector<HadLaneGroup*> nextLaneGroups;
		HadLaneGroup* currentLaneGroup = inLaneGroup;

		auto availableLaneGroup = [&,this](HadLaneGroup* inLaneGroup)
		{
			bool bRetPreFind = std::find(preLaneGroups.begin(), preLaneGroups.end(),inLaneGroup) != preLaneGroups.end(); // pre中是否存在
			bool bRetNextFind = std::find(nextLaneGroups.begin(), nextLaneGroups.end(), inLaneGroup) != nextLaneGroups.end(); // next中是否存在
			if ((inLaneGroup->inIntersection != 0)  // 路口不在接续
				|| bRetPreFind			// pre 已经存在，不再接续
				|| bRetNextFind			// next 已经存在，不再接续
				|| inLaneGroup->owner->getId() != m_pGrid->getId()  // 跨网格的处理
				)        
			{
				return false;
			}
			return true;
		};

		for(;;)
		{
			currentLaneGroup = getNextLaneGroup(currentLaneGroup, previous);			
			if (currentLaneGroup && (availableLaneGroup(currentLaneGroup)==true))
			{
				preLaneGroups.push_back(currentLaneGroup);
			//	bool ret = directionEqual(currentLaneGroup->roadBoundaries[0], currentLaneGroup, 3); // debug
			//	printf("===currentLaneGroup->roadBoundaries[0]->originId[%lld],currentLaneGroup->originId[%lld],ret[%d]======\n", currentLaneGroup->roadBoundaries[0]->originId,currentLaneGroup->originId,ret);
			}
			else
			{
				break;
			}			
		}
		std::reverse(preLaneGroups.begin(), preLaneGroups.end());  // 从头开始

#ifdef __DEBUG_GREENBELTURBAN__
		PrintLaneGroup(preLaneGroups, "Prev->");		
		PrintLaneGroup(inLaneGroup, "Current->");
#endif

		preLaneGroups.push_back(inLaneGroup);
		outConnectionLaneGroups = preLaneGroups;

		currentLaneGroup = inLaneGroup;
		for (;;)
		{
			currentLaneGroup = getNextLaneGroup(currentLaneGroup, next);
			if (currentLaneGroup && (availableLaneGroup(currentLaneGroup) == true))
			{
				nextLaneGroups.push_back(currentLaneGroup);
			}
			else
			{
				break;
			}
		}

#ifdef __DEBUG_GREENBELTURBAN__		
		PrintLaneGroup(nextLaneGroups, "Next->");
#endif

		outConnectionLaneGroups.insert(outConnectionLaneGroups.end(), nextLaneGroups.begin(), nextLaneGroups.end());
		
		size_t i = nearby.size();// for build
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
			LineString3d line;
			getLaneGroupLocationLine(obj, line);
			currentPoints.insert(currentPoints.end(), line.vertexes.begin(), line.vertexes.end());
		}

		Points.push_back(currentPoints.front()); //先放第一个
		for (int i = 1; i < currentPoints.size(); i++)
		{
			//	if ( i+1 < currentPoints.size() && currentPoints[i] != currentPoints[i+1]) // 有重复相邻点的时候，保留一个
			if (currentPoints[i - 1] != currentPoints[i])
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
		//double leng1stLaneGropups = calcLength(Points1st);

		std::vector<MapPoint3D64> Points2nd;
		getPointsByLaneGroups(in2ndLaneGroups, Points2nd);
		//double leng2ndLaneGropups = calcLength(Points2nd);

		
		auto delLaneGroupbylength = [&](MapPoint3D64 originPoint,std::vector<HadLaneGroup*>& inLaneGroups)
		{	
			float currntLen = DBL_MAX;
			float nextLen = 0;
			point_t  startPoint = POINT_T(originPoint);
			for (std::vector<HadLaneGroup*>::iterator ita = inLaneGroups.begin(); ita != inLaneGroups.end(); ita++)
			{
				std::vector<HadLaneGroup*> LaneGroups;
				LaneGroups.push_back(*ita);
				std::vector<MapPoint3D64> temPoints;
				getPointsByLaneGroups(LaneGroups, temPoints);// 找到最近点对应的lanegroup	

				for(int i =1; i< temPoints.size();i++)
				{
					point_t p1 = POINT_T(temPoints[i - 1]);
					point_t p2 = POINT_T(temPoints[i]);
					point_t grapedPt;
					if (GRAP_POINT(startPoint, segment_t(p1, p2), grapedPt, 300))  // 找到最近的一个lanegroup
					{
						if (ita != inLaneGroups.begin())						
						{
							inLaneGroups.erase(inLaneGroups.begin(), ita); // 最近之前的都删除掉
						}
						else
						{
							assert(0);
						}
						return true;
					}
				}				
			}
			return true;
		};

		auto isSameDirection = [&](std::vector<MapPoint3D64> Points1st, std::vector<MapPoint3D64> Points2nd)
		{
			float x = Points1st.front().pos.lat - Points1st.back().pos.lat;
			float y = Points1st.front().pos.lon - Points1st.back().pos.lon;

			float x1 = Points2nd.front().pos.lat - Points2nd.back().pos.lat;
			float y1 = Points2nd.front().pos.lon - Points2nd.back().pos.lon;

			float dot_product = x * x1 + y * y1; // 
			return dot_product > 0 ? true : false;  // 计算点积  正值方向相同，负值方向相反
		};

		bool bSameDirection = isSameDirection(Points1st, Points2nd);
		
		std::vector<MapPoint3D64>::iterator ita, itb;

		if (bSameDirection)   // 方向相同，1st头对应2nd头
		{
			// 1st头的处理	
			getClosestSeg(Points1st.front(), Points2nd, ita, itb);// 确定1st头和2nd边的长短关系。
			if (*ita == Points2nd.front())  // 1st头长于2nd的头。 需要删除1st的头，接近2nd的头
			{
				float len = calcLength(Points1st.front(), Points2nd.front());  // 计算1st头到2nd头的距离。
				if (len > 10000) // 10m范围内不做处理。
				{
					delLaneGroupbylength(Points2nd.front(), m_1stLaneGroups);  // 删除1st的头，同2nd的头对齐
				}
			}
			else //需要删除2nd的头 
			{
				getClosestSeg(Points2nd.front(), Points1st, ita, itb);// 确定2nd头和1st边的长短关系。
				if (*ita == Points1st.front())  // 2nd头长于1st的头。 需要删除2nd的头，接近1st的头
				{
					float len = calcLength(Points2nd.front(), Points1st.front());  // 计算2nd头到1st的头距离。
					if (len > 10000) // 10m范围内不做处理。
					{
						delLaneGroupbylength(Points1st.front(), m_2ndLaneGroups); // 删除2nd的头，同1st的头对齐。
					}
				}
				else
				{
					// 1st的头
					assert(0);
				}
			}

			// 1st尾的处理	
			getClosestSeg(Points1st.back(), Points2nd, ita, itb);// 确定1st尾和2nd边的长短关系。
			if (*itb == Points2nd.back())  // 1st尾巴长于2nd的尾。 需要删除1st的尾，接近2nd的尾
			{
				float len = calcLength(Points1st.back(), Points2nd.back());  // 计算1st尾到2nd的头距离。
				if (len > 10000) // 10m范围内不做处理。
				{
					std::reverse(m_1stLaneGroups.begin(), m_1stLaneGroups.end());  // 从尾部删除。
					delLaneGroupbylength(Points2nd.back(), m_1stLaneGroups);	   // 删除1st的尾，同2nd的尾对齐
					std::reverse(m_1stLaneGroups.begin(), m_1stLaneGroups.end());
				}
			}
			else // 2nd的尾长于1st的尾
			{
				getClosestSeg(Points2nd.back(), Points1st, ita, itb);// 确定2nd尾和1st边的长短关系。
				if (*itb == Points1st.back())  // 2nd尾长于1st的尾。 需要删除2nd尾，接近1st的尾
				{
					float len = calcLength(Points2nd.back(), Points1st.back());  // 计算2nd尾到1st的尾距离。
					if (len > 10000) // 10m范围内不做处理。
					{
						std::reverse(m_2ndLaneGroups.begin(), m_2ndLaneGroups.end());
						delLaneGroupbylength(Points1st.back(), m_2ndLaneGroups); // 删除2nd的尾巴，同1st的尾对齐。
						std::reverse(m_2ndLaneGroups.begin(), m_2ndLaneGroups.end());
					}
				}
				else
				{
					assert(0);
				}
			}
		}
		else  //方向相反，1st头对应2nd的尾巴
		{
			// 1st头的处理	
			getClosestSeg(Points1st.front(), Points2nd, ita, itb);// 确定1st头和2nd边的长短关系。
			if (*itb == Points2nd.back())  // 1st头长于2nd的尾。 需要删除1st的头，接近2nd的尾
			{
				float len = calcLength(Points1st.front(), Points2nd.back());  // 计算1st头到2nd尾的距离。
				if (len > 10000) // 10m范围内不做处理。
				{
					delLaneGroupbylength(Points2nd.back(), m_1stLaneGroups); // 删除1的头，同2nd的尾对齐
				}
			}
			else //需要删除2nd的尾巴
			{
				getClosestSeg(Points2nd.back(), Points1st, ita, itb);// 确定2nd尾和1st边的长短关系。
				if (*ita == Points1st.front())  // 2nd尾长于1st的头。 需要删除2nd尾，接近1st的头
				{
					float len = calcLength(Points2nd.back(), Points1st.front());  // 计算2nd尾到1st的头距离。
					if (len > 10000) // 10m范围内不做处理。
					{
						std::reverse(m_2ndLaneGroups.begin(), m_2ndLaneGroups.end());
						delLaneGroupbylength(Points1st.front(), m_2ndLaneGroups); // 删除2nd的尾巴,同1st头对齐。
						std::reverse(m_2ndLaneGroups.begin(), m_2ndLaneGroups.end());
					}
				}
				else
				{
					assert(0);
				}
			}

			// 1st尾的处理.
			getClosestSeg(Points1st.back(), Points2nd, ita, itb);// 确定1st尾和2nd边的长短关系。
			if (*ita == Points2nd.front())  // 1st尾长于2nd的头。 需要删除1st的尾，接近2nd的头
			{
				float len = calcLength(Points1st.back(), Points2nd.front());  // 计算1st尾到2nd头的距离。
				if (len > 10000) // 10m范围内不做处理。
				{
					std::reverse(m_1stLaneGroups.begin(), m_1stLaneGroups.end());  // 从尾部删除。
					delLaneGroupbylength(Points2nd.front(), m_1stLaneGroups);	   // 删除1st的尾，同2nd的头对齐
					std::reverse(m_1stLaneGroups.begin(), m_1stLaneGroups.end());
				}
			}
			else //需要删除2nd的头
			{
				getClosestSeg(Points2nd.front(), Points1st, ita, itb);// 确定2nd头和1st边的长短关系。
				if (*itb == Points1st.back())  // 2nd头长于1st的尾。 需要删除2nd的头，接近1st的尾
				{
					float len = calcLength(Points2nd.front(), Points1st.back());  // 计算2nd头到1st的尾距离。
					if (len > 10000) // 10m范围内不做处理。
					{
						delLaneGroupbylength(Points1st.back(), m_2ndLaneGroups); // 删除2nd的头，同1st的尾对齐
					}
				}
				else
				{
					//
					assert(0);
				}

			}

		}

#if 0

		std::vector<MapPoint3D64>::iterator ita, itb;
		// 1st头的处理	
		getClosestSeg(Points1st.front(), Points2nd, ita, itb);// 确定1st头和2nd边的长短关系。
		if(*ita== Points2nd.front())  // 1st头长于2nd的头。 需要删除1st的头，接近2nd的头
		{
			float len = calcLength(Points1st.front(),Points2nd.front());  // 计算1st头到2nd头的距离。
			if (len > 10000) // 控制在10m范围左右
			{
				delLaneGroupbylength(Points2nd.front(), m_1stLaneGroups);
			}

		}
		else if (*itb == Points2nd.back())  // 1st头长于2nd的尾， 需要删除1st的头. 接近2nd的尾。
		{
			float len = calcLength(Points1st.front(), Points2nd.back());
			if (len > 10000)
			{				
				delLaneGroupbylength(Points2nd.back(), m_1stLaneGroups);
			}
		}
		else  // 1st头对应的2nd的尾
		{
			// 2nd尾的处理
			getClosestSeg(Points2nd.back(), Points1st, ita, itb);// 确定2nd尾和1st边的长短关系。
			if (*ita == Points1st.front())  // 2nd尾巴长于1st的头。 需要删除2nd的尾，接近1st的头
			{
				float len = calcLength(Points2nd.back(), Points1st.front());  // 计算2nd尾到1st的头距离。
				if (len > 10000) // 控制在10m范围左右
				{
					std::reverse(m_2ndLaneGroups.begin(), m_2ndLaneGroups.end());
					delLaneGroupbylength(Points1st.front(), m_2ndLaneGroups);
					std::reverse(m_2ndLaneGroups.begin(), m_2ndLaneGroups.end());
				}

			}
			else if (*itb == Points1st.back())  // 2nd的尾长于1st的尾。 需要删除2nd的尾. 接近1st的尾。
			{
				float len = calcLength(Points2nd.back(), Points1st.back()); // 计算2nd尾到1st的尾距离。
				if (len > 10000)
				{
					std::reverse(m_2ndLaneGroups.begin(), m_2ndLaneGroups.end()); // 从尾部删除。
					delLaneGroupbylength(Points1st.back(), m_2ndLaneGroups);
					std::reverse(m_2ndLaneGroups.begin(), m_2ndLaneGroups.end());
				}
			}

		}

		// 1st尾的处理	
		getClosestSeg(Points1st.back(), Points2nd, ita, itb);// 确定1st尾和2nd边的长短关系。
		if (*ita == Points2nd.front())  // 1st尾巴长于2nd的头。 需要删除1st的尾，接近2nd的头
		{
			float len = calcLength(Points1st.back(), Points2nd.front());  // 计算1st尾到2nd的头距离。
			if (len > 10000) // 控制在10m范围左右
			{
				std::reverse(m_1stLaneGroups.begin(),m_1stLaneGroups.end());  // 从尾部删除。
				delLaneGroupbylength(Points2nd.front(), m_1stLaneGroups);
				std::reverse(m_1stLaneGroups.begin(), m_1stLaneGroups.end());
			}

		}
		else if (*itb == Points2nd.back())  // 1st尾长于2nd的尾， 需要删除1st的头. 接近2nd的尾。
		{
			float len = calcLength(Points1st.front(), Points2nd.back()); // 计算1st尾到2nd的尾距离。
			if (len > 10000)
			{
				std::reverse(m_1stLaneGroups.begin(), m_1stLaneGroups.end()); // 从尾部删除。
				delLaneGroupbylength(Points2nd.back(), m_1stLaneGroups);
				std::reverse(m_1stLaneGroups.begin(), m_1stLaneGroups.end());
			}
		}
		else // 1st尾对应2nd的头
		{
			// 2nd头的处理		
			getClosestSeg(Points2nd.front(), Points1st, ita, itb);// 确定2nd头和1st边的长短关系。
			if (*ita == Points1st.front())  // 2nd头长于1st的头。 需要删除2nd的头，接近1st的头
			{
				float len = calcLength(Points2nd.front(), Points1st.front());  // 计算2nd头到1st头的距离。
				if (len > 10000) // 控制在10m范围左右
				{
					delLaneGroupbylength(Points1st.front(), m_2ndLaneGroups);

				}

			}
			else if (*itb == Points1st.back())  // 2nd头长于1st的尾， 需要删除2nd的头. 接近1st的尾。
			{
				float len = calcLength(Points2nd.front(), Points1st.back());// 计算2nd头到1st尾的距离。
				if (len > 10000)
				{
					delLaneGroupbylength(Points1st.back(), m_2ndLaneGroups);// 计算2nd头到1st尾的距离。
				}
			}
		}

#endif

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
				line1st.insert(itb, grapedPt);
				line1st.erase(itb, line2nd.end());
			//	line1st.push_back(grapedPt);
			}
			else
			{
				assert(0); // 不能投上
				return false;
			}
		}

		return true;
	}

#if 0
	bool GreenbeltUrbanCompiler::isSegmentsCross(std::vector <MapPoint3D64>& points1st, std::vector <MapPoint3D64>& points2nd)
	{

		for (int i = 1; i < points1st.size(); i++)
		{
			segment_2t line1stSeg(POINT_T(points1st[i - 1],POINT_T(points1st[i]);
			for (int j = 1; j < points2nd.size(); j++)
			{
				segment_2t line2ndSeg(POINT_T(points2nd[j - 1], POINT_T(points2nd[j]);
				if (bg::intersects(line1stSeg, line2ndSeg))
				{
					return true;
				}
			}
		}

		return false;
	}
#endif


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

	void GreenbeltUrbanCompiler::PrintLaneGroup(std::vector<HadLaneGroup*> LaneGroups, std::string mark)
	{
		for (auto laneGroup : LaneGroups)
		{
			PrintLaneGroup(laneGroup, mark);
		}
	}

#endif

}