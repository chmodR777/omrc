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

		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227961664017207553)) // 20169163 压盖
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (225231298001051141)) // 20169163 压盖
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227955322690932993)) // 20169163 车道边
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (225249596268814085)) // 20169163 隔离带
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227960749105287425)) // 20169185 
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227939496864714753)) // 20169185 大圆转弯
			if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227964848517353473)) // 20169185 前小后大，后有宽度截断	
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227951732421234945)) // 20169160 补细面 
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (235397476841375235)) // 20169122
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (236998667245392641)) // 20169122			// 
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (236985947552951041)) // 20169122	
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (224930309251479555)) // 20169122				
		//	if (
		//	    (*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (225621966049452293)// 20169187		
		//	|| (*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227942684338622465)// 20169187
		//	||(*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227955252629278977)
		//	|| (*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (225639467978270472)// 20169187
		//		)
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


				std::vector<PartPoints> PartPointslist;
				if (alignPoints(Points1st, Points2nd, PartPointslist)==false)
				{
#ifdef __DEBUG_GREENBELTURBAN__					
					PrintLaneGroup(pGroup, "inLaneGroup Distacne>25:"); //debug
					PrintPoints(Points1st, "Points1st Distacne>25"); //debug
					PrintPoints(Points2nd, "Points2nd Distacne>25"); //debug
#endif

					continue;
				}
								
				for (int i= 0;i <PartPointslist.size();i++)
				{
					Points1st = PartPointslist[i].points1;
					Points2nd = PartPointslist[i].points2;


					if (isExistIntersect(Points1st, Points2nd))
					{
						continue;
					}

				//	if (isNeedGreenbeltSurface(Points1st, Points2nd))
				//	{
				//		makeGreenbeltSurfaceData(Points1st, Points2nd, pTile);
				//	}
				//	else
					{
						makeGreenbeltData(Points1st, Points2nd, pTile);
					}				

				}

				m_VisitLaneGroups.insert(m_VisitLaneGroups.end(), m_1stLaneGroups.begin(), m_1stLaneGroups.end());
				m_VisitLaneGroups.insert(m_VisitLaneGroups.end(), m_2ndLaneGroups.begin(), m_2ndLaneGroups.end());

#ifdef __DEBUG_GREENBELTURBAN__	
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

			//得到普通路绿化带的lanegroup
			if(ableGreenbeltLaneGroup(pGroup)) // 
			{
				laneGroups.push_back(pGroup);

#ifdef __DEBUG_GREENBELTURBAN__	    
				bool isRet1 = isProDataLevel(pGroup);
				bool isRet2 = isMultiDigitized(pGroup);
				std::string str = "isProDataLevel[" + std::to_string(isRet1) + "],isMultiDigitized[" + std::to_string(isRet2) + "]," + "inIntersection[" + std::to_string(pGroup->inIntersection) + "]";
				PrintLaneGroup(pGroup,"Yes greenbelt:"+ str);
#endif
			}
			else
			{

#ifdef __DEBUG_GREENBELTURBAN__	    
				bool isRet1 = isProDataLevel(pGroup);
				bool isRet2 = isMultiDigitized(pGroup);				
				std::string str = "isProDataLevel[" + std::to_string(isRet1) + "],isMultiDigitized[" + std::to_string(isRet2) + "]," + "inIntersection[" + std::to_string(pGroup->inIntersection) + "]";
				PrintLaneGroup(pGroup, "No greenbelt:"+ str);
#endif
			}
		}

		return true;
	}

	// 抓得对向道路的LaneGroup
	bool GreenbeltUrbanCompiler::grabOtherLaneGroup(HadGrid* const pGrid,HadLaneGroup* inLaneGroup, HadLaneGroup*& outNearLaneGroup)
	{
		std::vector<grablaneGroupInfo> grabLaneGroups;

		std::vector<HadLaneGroup*> NearbyLaneGroups = SpatialSeacher::seachNearby2d(pGrid, inLaneGroup, 1e4);

		getNearbyLaneGropuByAngle(inLaneGroup, NearbyLaneGroups); // 按照一定角度取得周边的lanegroups

//		getNearbyLaneGropuByDirection(inLaneGroup, NearbyLaneGroups); // 按照一定方向取得周边的lanegroups

		getNearbyLaneGropuByDistance(inLaneGroup, NearbyLaneGroups, outNearLaneGroup, grabLaneGroups);

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
#ifdef __DEBUG_GREENBELTURBAN__   // LaneGroup 没有数据
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
	
	bool GreenbeltUrbanCompiler::getNearbyLaneGropuByDirection(HadLaneGroup* inLaneGroup, std::vector<HadLaneGroup*>& inoutNearbyLaneGroups)
	{
		std::vector<HadLaneGroup*>  inlaneGropus = inoutNearbyLaneGroups;
		std::vector<HadLaneGroup*>  outlaneGropus;

		std::vector<MapPoint3D64> inPoints;		
		LineString3d line;
		getLaneGroupLocationLine(inLaneGroup, line);
		inPoints.insert(inPoints.end(), line.vertexes.begin(), line.vertexes.end());

		bool inLaneDriveDirection = false;		
		if (inLaneGroup->roadBoundaries.size())
		{
			inLaneDriveDirection = directionEqual(inLaneGroup->roadBoundaries[0], inLaneGroup, 3);			
		}
		else
		{
			inLaneDriveDirection = directionEqual(inLaneGroup->laneBoundaries[0], inLaneGroup, 3);
		}

		for (auto obj:inlaneGropus)
		{
			std::vector<MapPoint3D64> Points;			
			LineString3d line;
			getLaneGroupLocationLine(obj, line);
			Points.insert(Points.end(), line.vertexes.begin(), line.vertexes.end());

			bool DriveDirection = false;
			if (obj->roadBoundaries.size())
			{
				DriveDirection = directionEqual(obj->roadBoundaries[0], obj, 3);
			}
			else
			{
				DriveDirection = directionEqual(obj->laneBoundaries[0], obj, 3);
			}

			if (inLaneDriveDirection == DriveDirection && !isSameDirection(inPoints, Points))
			{
				outlaneGropus.push_back(obj);
			}
		}

		inoutNearbyLaneGroups = outlaneGropus;

		return true;
	}

	// 得到距离（投影）最短的outNearLaneGroup
	bool GreenbeltUrbanCompiler::getNearbyLaneGropuByDistance(HadLaneGroup* inLaneGroup, std::vector<HadLaneGroup*>& inNearbyLaneGroups, HadLaneGroup*& outNearLaneGroup, std::vector<grablaneGroupInfo>& grabLaneGroups)
	{	


		//std::map<HadLaneGroup*, double> laneGroupDistance;
		
		std::vector<HadLaneGroup*>  inlaneGropus = inNearbyLaneGroups;

		auto getLinkGroupDistance = [&](HadLaneGroup* inLaneGroup, HadLaneGroup* currentLaneGroup)
		{
			LineString3d inLine;
			getLaneGroupLocationLine(inLaneGroup, inLine);

			int inlinesize = inLine.vertexes.size();
			point_t  sourcePoint = POINT_T(inLine.vertexes[inlinesize / 2]);


			LineString3d currentLine;			
			getLaneGroupLocationLine(currentLaneGroup, currentLine);

			double distance = DBL_MAX;
			for (int i = 1; i < currentLine.vertexes.size(); i++)
			{
				grablaneGroupInfo st;
				point_t p1 = POINT_T(currentLine.vertexes[i - 1]);
				point_t p2 = POINT_T(currentLine.vertexes[i]);
				point_t grapedPt;
				if (GRAP_POINT(sourcePoint, segment_t(p1, p2), grapedPt, 300))  // 找到最近的一个lanegroup
				{					
					distance = bg::distance(sourcePoint, grapedPt);
					st.distance = distance;					
					st.hight = abs(sourcePoint.get<2>() - grapedPt.get<2>());
					st.laneGroup = currentLaneGroup;
					grabLaneGroups.push_back(st);
				}
			}

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

		// 把周边抓到的lanegroup按照距离排序
		std::sort(grabLaneGroups.begin(), grabLaneGroups.end(), [](grablaneGroupInfo a, grablaneGroupInfo b) {  return a.distance < b.distance; });

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
#if 0
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
#endif

		std::vector<MapPoint3D64> Points1st;
		getPointsByLaneGroups(in1stLaneGroups, Points1st);

		std::vector<MapPoint3D64> Points2nd;
		getPointsByLaneGroups(in2ndLaneGroups, Points2nd);
				
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

		return true;
	}


	bool GreenbeltUrbanCompiler::alignPoints(std::vector <MapPoint3D64>& points1st, std::vector <MapPoint3D64>& points2nd, std::vector<PartPoints>& partPointslist)
	{
		if (points1st.size() < 2 || points2nd.size() < 2)
		{
			printf("======points1st.size(%d), points2nd.size(%d)\n", points1st.size(), points2nd.size());
			return false;
		}

		if (!isSameDirection(points1st, points2nd)) // 变成同向的
		{
			std::reverse(points1st.begin(), points1st.end());
		}

		linestring_t  line1st = LINESTRING_T(points1st);
		linestring_t  line2nd = LINESTRING_T(points2nd);

		std::vector <MapPoint3D64> points1;
		std::vector <MapPoint3D64> points2;

		std::vector<PartPoints> Pointslist;

		//	auto alignPointsbyDistance = [&](std::vector <MapPoint3D64>& points1st, std::vector <MapPoint3D64>& points2nd)
		//	{
		int lastGrapIndex = -1;
		for (int i = 0; i < points1st.size(); i++)
		{
			point_t grapedPt;
			points1.push_back(points1st[i]);
			point_t startPoint1st = POINT_T(points1st[i]);
			for (int j = 1; j < points2nd.size(); j++)
			{
				segment_t seg(POINT_T(points2nd[j - 1]), POINT_T(points2nd[j]));
				if (GRAP_POINT(startPoint1st, seg, grapedPt, 300))
				{
					double length = bg::distance(startPoint1st, grapedPt);
					if (length < 25000)
					{
						lastGrapIndex = j;    //  1st最后投影2nd的位置
						points2.assign(points2nd.begin(), points2nd.begin() + j);// 把每次1st能投影到的2nd都进行保存。相当于2nd跟着1st同步动作。
					}
					else
					{
						// 遇到前方有大转弯的情况。投影有可能投到了前方转弯处，而不是对面。暂时考虑用距离（(25000*3)）,和高度差约束在对面。
						double dheght = abs(grapedPt.get<2>() - startPoint1st.get<2>());
						if (points2.size() >= 2 && length < (25000 * 3) && (dheght < 300))  // 有成对的小于阀值，保留这一对作为分隔。
						{
							points2.assign(points2nd.begin(), points2nd.begin() + j);
							PartPoints part;
							part.points1 = points1;
							part.points2 = points2;
							Pointslist.push_back(part);
							points1.clear();
							points2.clear();
						}
					}
				}
			}

			if (i == points1st.size() - 1 && lastGrapIndex < points2nd.size()) // 1st遍历完。2nd还有剩余, 直接把2剩余的点补上
			{
				points2.insert(points2.end(), points2nd.begin() + lastGrapIndex, points2nd.end());
			}
		}

		if (lastGrapIndex == -1)  // 没有过任何小于阀值的投影。说明1st和2nd的宽度都大于阀值。
		{
			return false;
		}

		if (Pointslist.empty()) // 没有过投影记录。
		{
			PartPoints part;
			part.points1 = points1;
			part.points2 = points2;
			partPointslist.push_back(part);
		}
		else
		{
			partPointslist = Pointslist;
		}

		return true;
}

	bool GreenbeltUrbanCompiler::isSameDirection(std::vector<MapPoint3D64>& points1st, std::vector<MapPoint3D64>& points2nd)
	{
		float x = points1st.front().pos.lat - points1st.back().pos.lat;
		float y = points1st.front().pos.lon - points1st.back().pos.lon;

		float x1 = points2nd.front().pos.lat - points2nd.back().pos.lat;
		float y1 = points2nd.front().pos.lon - points2nd.back().pos.lon;

		float dot_product = x * x1 + y * y1; // 
		return dot_product > 0 ? true : false;  // 计算点积  正值方向相同，负值方向相反
	};

	
	bool GreenbeltUrbanCompiler::isExistIntersect(std::vector <MapPoint3D64>& points1st, std::vector <MapPoint3D64>& points2nd)
	{

		//1 判断两条边是否相交
		linestring_2t line2t1st = LINESTRING_2T(points1st);
		linestring_2t line2t2nd = LINESTRING_2T(points2nd);
		if (bg::intersects(line2t1st, line2t2nd)) // 两条边有相交不做隔离带
		{
#ifdef __DEBUG_GREENBELTURBAN__
			PrintPoints(points1st, "Points1st and Points1st intersects:"); //debug
			PrintPoints(points1st, "Points2nd and Points1st intersects:"); //debug
#endif
			return true;			
		}

#if 0  // 其他相交的处理

		// 判断是否存在其他边（lanegroup）同两条边相交
		float length1st = calcLength(points1st);
		float length2nd = calcLength(points2nd);
		float length = length1st > length2nd ? length1st : length2nd;
		std::vector<HadLaneGroup*> NearbyLaneGroups = SpatialSeacher::seachNearby2d(m_pGrid, m_1stLaneGroups[m_1stLaneGroups.size()/2], length); // 得到一定范围内

		points1st.insert(points1st.end(), points2nd.begin(),points2nd.end()); // 之前做过isSameDirection的处理。所以头尾相连
		ring_2t ring2t = RING_2T(points1st);
		for(auto obj : NearbyLaneGroups)
		{			
			bool bRetFind1st = std::find(m_1stLaneGroups.begin(), m_1stLaneGroups.end(), obj) != m_1stLaneGroups.end(); // m_1stLaneGroups中是否存在
			bool bRetNext2nd = std::find(m_2ndLaneGroups.begin(), m_2ndLaneGroups.end(), obj) != m_2ndLaneGroups.end(); // m_2ndLaneGroups中是否存在
			if (bRetFind1st || bRetNext2nd) // 排除隔离带本身的lanegroup
			{
				continue;
			}
			std::vector<MapPoint3D64> pts;
			MapPoint3D64 min = obj->extent.min;
			pts.push_back(min);
			MapPoint3D64 max = obj->extent.max;
			pts.push_back(max);
			box_2t box = BOX_2T(pts);

			if (bg::intersects(ring2t, box))
			{
#ifdef __DEBUG_GREENBELTURBAN__
				PrintPoints(points1st, "Points1st and other intersects:"); //debug
				PrintLaneGroup(obj,"1st,2nd and other intersects:"); //debug
				PrintPoints(points1st, "Points2nd and other intersects:"); //debug
#endif
				
//				return true;
			}

		}

#endif  // 其他相交的处理

		return false;
	}

	bool GreenbeltUrbanCompiler::isNeedGreenbeltSurface(std::vector<MapPoint3D64>& points1st, std::vector<MapPoint3D64>& points2nd)
	{
		//if (isSameDirection(points1st, points2nd))  // 需要方向为逆
		//{
		//	std::reverse(points1st.begin(), points1st.end());
		//}

		// 得到所有的点
		std::vector<MapPoint3D64> allPoints = points1st;
		allPoints.insert(allPoints.end(), points2nd.begin(), points2nd.end()); // 之前做过isSameDirection的处理。所以头尾相连

		std::vector<std::vector<MapPoint3D64>> allLines;
		for (auto& rdsLineInfo : compilerData.m_rdsLines)
		{
			allLines.push_back(rdsLineInfo._originPoints);
		}

		int index = allPoints.size();
		for (auto point : allPoints)
		{
			point_t startPoint1st = POINT_T(point);
			for (size_t i = 0; i < allLines.size(); ++i)
			{
				bool isFindOk = false;
				for (size_t j = 0; j < allLines[i].size() - 1; ++j)
				{
					point_t grapedPt;
					std::vector<size_t> tmpSize;
					segment_t tmpSeg(POINT_T(allLines[i][j]), POINT_T(allLines[i][j + 1]));
					if (GRAP_POINT(startPoint1st, tmpSeg, grapedPt, 300))
					{
						double length = bg::distance(startPoint1st, grapedPt);
						if (length < 1000) //  1m内有边线
						{
							index--;
							isFindOk = true;
							break;
						}
					}

				}
				if (isFindOk)
				{
					break;
				}
			}
		}

		if (index == 0)
		{
			return true;
		}

		return false;
	}


	bool GreenbeltUrbanCompiler::makeGreenbeltData(std::vector<MapPoint3D64>& points1st, std::vector<MapPoint3D64>& points2nd, RdsTile* pTile)
	{

		// 普通隔离带，两条边线序要相反
		if (isSameDirection(points1st, points2nd))
		{
			std::reverse(points1st.begin(), points1st.end());
		}

		//	RdsGreenbelt* pGreenbelt = (RdsGreenbelt*)createObject(pTile, EntityType::RDS_GREENBELT); // debug 复用高速道路
		RdsGreenbeltUrban* pGreenbelt = (RdsGreenbeltUrban*)createObject(pTile, EntityType::RDS_GREENBELT_URBAN);
		OMDB::LineString3d location = { points1st };
		RDS::LineString3d line;
		convert(location, line);
		pGreenbelt->contour.lines.push_back(line);

		OMDB::LineString3d nearbyLocation = { points2nd };
		RDS::LineString3d nearbyLine;
		convert(nearbyLocation, nearbyLine);
		pGreenbelt->contour.lines.push_back(nearbyLine);

#ifdef __DEBUG_GREENBELTURBAN__
		PrintPoints(points1st, "points1st Data"); //debug
		PrintPoints(points2nd, "points2nd Data"); //debug
#endif
		return true;
	}


	bool GreenbeltUrbanCompiler::makeGreenbeltSurfaceData(std::vector<MapPoint3D64>& points1st, std::vector<MapPoint3D64>& points2nd, RdsTile* pTile)
	{

#if 1  //  =============复用La补面start=======

		// 复用的lA补面逻辑。线序要相同。
		if (!isSameDirection(points1st, points2nd))
		{
			std::reverse(points1st.begin(), points1st.end());
		}

		RdsRoad* pRoad = (RdsRoad*)createObject(pTile, EntityType::RDS_ROAD);
		OMDB::LineString3d location = { points1st };
		RDS::LineString3d line;
		convert(location, line);

		OMDB::LineString3d nearbyLocation = { points2nd };
		RDS::LineString3d nearbyLine;
		convert(nearbyLocation, nearbyLine);

		pRoad->contour.lines.resize(2);
		pRoad->contour.lines[0] = line;
		pRoad->contour.lines[1] = nearbyLine;
		pRoad->roadType = RdsRoad::RoadType::LA;
		m_1stLaneGroups.insert(m_1stLaneGroups.end(), m_2ndLaneGroups.begin(), m_2ndLaneGroups.end());
		for (auto laneGroup : m_1stLaneGroups) {
			pRoad->groups.push_back(laneGroup);
			RdsGroup* pRdsGroup = queryGroup(laneGroup->originId, pTile);
			if (pRdsGroup)
				pRdsGroup->objects.push_back(pRoad);
		}
#endif  //  =============复用La补面end=======

#ifdef __DEBUG_GREENBELTURBAN__
		PrintPoints(points1st, "points1st surfaceData"); //debug
		PrintPoints(points2nd, "points2nd surfaceData"); //debug
#endif

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