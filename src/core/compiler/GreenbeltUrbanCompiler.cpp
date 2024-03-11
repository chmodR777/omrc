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
	m_nearby(),
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
		m_nearby = nearby;
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
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227955358795501825)) // 20169163 压盖
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (225231298001051141)) // 20169163 压盖
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (225249596268814085)) // 20169163 正常情况
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227955326230925569)) // 20169163 点离的近包含在pologn
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227955322690932993)) // 20169163 车道边
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (225249596268814085)) // 20169163 隔离带
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (225256394329629957)) // 20169163 隔离带切分分断了		
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227959044674359553)) // 20169187 有圆弧缺失
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227939496864714753)) // 20169185 大圆转弯
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227964848517353473)) // 20169185 前小后大，后有宽度截断	
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (225551005602228489)) // 20169185 有空缺
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (246958940999857153)) // 20169141 Intersection有空缺
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (237014671174340865)) // 20169141 有空缺 挑头
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (236991894753061633)) // 20169141 Intersection有空缺,空缺有相交
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227951732421234945)) // 20169160 补细面 
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (235397476841375235)) // 20169122 全宽
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (236998667245392641)) // 20169122			// 
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (236985947552951041)) // 20169122	
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (224930309251479555)) // 20169122		
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227951800645783809)) // 20169160 	
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227936518917263361)) // 20169193 补面异常	
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (236993604712076033)) // 20169149 未显示出隔离带 
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227934782504767489)) // 20169192		// 
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227960745552711937)) // 20169185 去掉交叉后有直线
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (235351744830717961)) // 20169117  Intersection处理索引问题	
		//	if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (242597925608704011)) // 20169117  Intersection处理索引问题
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

				// 为了解决隔离带有三角型缺口问题。expandConnectionLaneGroups把一些满足条件的Intersection进行了连接。
				// 这里的处理是把Intersection的lanegroup和对向langroup组成隔离带的polygon，如果polygon中存在MiddleLaneGroup就删除Intersection的lanegroup。
				// 需要放到alignLaneGroupsbylength之前，要不有问题。
				deleteInvalidIntersectionLaneGroup(m_1stLaneGroups, m_2ndLaneGroups);

				alignLaneGroupsbylength(p1stLaneGroups, p2ndLaneGroups);
#ifdef __DEBUG_GREENBELTURBAN__
				PrintLaneGroup(m_1stLaneGroups, "Align 1st");
				PrintLaneGroup(m_2ndLaneGroups, "Align 2nd");
#endif
			


				if (existMiddleLaneGroup(m_1stLaneGroups, m_2ndLaneGroups))
				{
					continue;
				}

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
	
#ifdef __DEBUG_GREENBELTURBAN__	
				PrintInfo("Cut count:" + std::to_string(PartPointslist.size())); //debug
#endif

				for (int i= 0;i <PartPointslist.size();i++)
				{
					Points1st = PartPointslist[i].points1;
					Points2nd = PartPointslist[i].points2;
					double AverageWidth = PartPointslist[i].AverageWidth;

					if (Points1st.empty() || Points2nd.empty())
					{
						continue;
					}

					if (isExistIntersect(Points1st, Points2nd))  // 有相交
					{
						continue;
					}

					// 制作挑头侧隔离带
					makeUturnGreenbelt(Points1st, Points2nd, pTile);


#if 0   // 补面开关
					if (AverageWidth < 3000 && isNeedGreenbeltSurface(Points1st, Points2nd)) // 平均宽度小于3m，并且有车道边线的进行补面。
					{
#ifdef __DEBUG_GREENBELTURBAN__	
						PrintInfo("AverageWidth:" + std::to_string(AverageWidth)); //debug
#endif

						makeGreenbeltSurfaceData(Points1st, Points2nd, pTile);					}
					else
					{
						makeGreenbeltData(Points1st, Points2nd, pTile);
					}
#else
					makeGreenbeltData(Points1st, Points2nd, pTile);
#endif

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
			// 行车方向相同的
			if (inLaneDriveDirection == DriveDirection)
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
		std::sort(grabLaneGroups.begin(), grabLaneGroups.end(), [](grablaneGroupInfo &a, grablaneGroupInfo &b) {  return a.distance < b.distance; });

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

		auto getLinkGroupAngle = [&, this](HadLaneGroup* inLaneGroup, HadLaneGroup* currentLaneGroup)
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


		std::vector<HadLaneGroup*> preLaneGroups;
		std::vector<HadLaneGroup*> nextLaneGroups;
		HadLaneGroup* currentLaneGroup = inLaneGroup;

		auto availableLaneGroup = [&,this](HadLaneGroup* inLaneGroup)
		{
			bool bRetPreFind = std::find(preLaneGroups.begin(), preLaneGroups.end(),inLaneGroup) != preLaneGroups.end(); // pre中是否存在
			bool bRetNextFind = std::find(nextLaneGroups.begin(), nextLaneGroups.end(), inLaneGroup) != nextLaneGroups.end(); // next中是否存在
			// 一面边是Intersection，一边不是Intersection，就会出现一个三角形缺口。为了对应该case。这里把满足条件的Intersection也当作隔离带的一部分。
			//if ((inLaneGroup->inIntersection != 0)  // 路口不在接续
			  if(
				 bRetPreFind			// pre 已经存在，不再接续
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
				if (currentLaneGroup->inIntersection != 0) // 相对比较直的路口.并且只有这一个接续，这样的保存。不在接续.  为了解决三角形缺口问题。
				{
					HadLaneGroup*  lastLaneGroup = preLaneGroups.empty() ? inLaneGroup : preLaneGroups.back();					
					uint16 uiAngle = getLinkGroupAngle(lastLaneGroup, currentLaneGroup);
					uint16 uiConnectNum = lastLaneGroup->laneBoundaries.size() > 0 ? lastLaneGroup->laneBoundaries[0]->previous.size() : 2;
					uiConnectNum = lastLaneGroup->roadBoundaries.size() > 0 ? lastLaneGroup->roadBoundaries[0]->previous.size() : 2;					
					if (uiAngle<15 && uiConnectNum==1)  // 
					{
						preLaneGroups.push_back(currentLaneGroup);	
					}
					break;
				}
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
				if (currentLaneGroup->inIntersection != 0) // 相对比较直的路口.并且只有这一个接续，这样的保存。不在接续.
				{
					HadLaneGroup* lastLaneGroup = nextLaneGroups.empty() ? inLaneGroup : nextLaneGroups.back();
					uint16 uiAngle = getLinkGroupAngle(lastLaneGroup, currentLaneGroup);
					uint16 uiConnectNum = lastLaneGroup->laneBoundaries.size() > 0 ? lastLaneGroup->laneBoundaries[0]->next.size() : 2;
					uiConnectNum = lastLaneGroup->roadBoundaries.size() > 0 ? lastLaneGroup->roadBoundaries[0]->next.size() : 2;
					if (uiAngle < 15 && uiConnectNum == 1)  // 
					{
						nextLaneGroups.push_back(currentLaneGroup);						
					}
					break;
				}
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
	bool GreenbeltUrbanCompiler::getPointsByLaneGroups(std::vector<HadLaneGroup*>& inLaneGroups, std::vector<MapPoint3D64>& Points)
	{
		std::vector<MapPoint3D64>  currentPoints;
		for (auto obj : inLaneGroups)
		{
			LineString3d line;
			getLaneGroupLocationLine(obj, line);
			currentPoints.insert(currentPoints.end(), line.vertexes.begin(), line.vertexes.end());
		}

		if (currentPoints.empty())
		{
			return false;
		}
		
		Points.clear();
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

		std::vector<PartPoints> Pointslist;
		
		auto getClosestSeginfo = [&](const MapPoint3D64& point,
			std::vector<MapPoint3D64>& line,
			int &iIndex1,
			int &iIndex2,
			double& width,
			int &high,
			MapPoint3D64& grapPt
			)
		{

			struct grapPtinfo
			{
				int Index1;
				int Index2;
				double width;
				int high;
				MapPoint3D64 grapPoint;
			};

			std::vector<grapPtinfo>  grapPtlist;

			point_t grapedPt;
			point_t startPoint = POINT_T(point);
			for (int index = 1; index < line.size(); index++)
			{
				segment_t seg(POINT_T(line[index -1]), POINT_T(line[index]));
				if (GRAP_POINT(startPoint, seg, grapedPt, 300))
				{
					grapPtinfo pt;
					pt.Index1 = index-1;
					pt.Index2 = index;
					pt.width = bg::distance(startPoint, grapedPt);  // 最短距离的距离
					pt.high = abs(startPoint.get<2>() - grapedPt.get<2>())/10;  // 高度差.同MapPoint3D64中z的量级
					pt.grapPoint = MapPoint3D64_make(grapedPt.get<0>(), grapedPt.get<1>(), grapedPt.get<2>());
					pt.grapPoint.z /= 10;  //  POINT_T()中*10.  这里需要再/10
					grapPtlist.push_back(pt);  // 大转弯，有可能投到多个。
				}
			}

			if (grapPtlist.size() == 0)
			{
#ifdef __DEBUG_GREENBELTURBAN__
				std::vector<MapPoint3D64> lanepoints;
				lanepoints.push_back(point);
				PrintPoints(lanepoints, "[Fail]Points :GRAP_POINT"); //debug
			//	PrintPoints(lanepoints, "[Fail]Points :GRAP_POINT:index1[" + std::to_string(iIndex1) + "],index2[" + std::to_string(iIndex2) + "]"); //debug
			//	PrintPoints(line, "[Fail]Points :GRAP_POINT"); //debug
#endif	
				return false;
			}
			else if (grapPtlist.size() > 2)
			{
				//大转弯，有可能投到多个。取最短距离的那个
				std::sort(grapPtlist.begin(), grapPtlist.end(), [](grapPtinfo &a, grapPtinfo &b) {  return a.width < b.width; });
			}

			iIndex1 = grapPtlist.front().Index1;
			iIndex2 = grapPtlist.front().Index2;
			width = grapPtlist.front().width;
			high = grapPtlist.front().high;
			grapPt = grapPtlist.front().grapPoint;

			return true;


		};

		struct PointsPartIndex
		{
			int startIndex;
			int endIndex;
			PointsPartIndex() :startIndex(-1), endIndex(-1) {}
		};

		struct Points1st2ndPart
		{
			PointsPartIndex point1stPart;
			PointsPartIndex point2ndPart;
			double AverageWidth;
			Points1st2ndPart():point1stPart(), point2ndPart(), AverageWidth(0){}
		};

		std::vector<Points1st2ndPart> Points1st2ndPartList;
		PointsPartIndex points1stPart;
		PointsPartIndex points2ndPart;

		enum  CUTSTATUS {
			NONE = 0,
			NOTCUT, // 小于阀值
			CUT,   //  大于阀值，开始切割
		};

		double dTotalWidth = 0;
		int iWidthConunt = 0;

		auto makepart = [&]()
		{
			if((points1stPart.endIndex - points1stPart.startIndex >=1)
				&& (points2ndPart.endIndex - points2ndPart.startIndex >=1))
			{
					Points1st2ndPart part;
					part.point1stPart = points1stPart;
					part.point2ndPart = points2ndPart;
					part.AverageWidth = iWidthConunt?(dTotalWidth / iWidthConunt):0;
					Points1st2ndPartList.push_back(part);
					dTotalWidth = 0; // 制作了一次隔离带后，下一个隔离带从新统计。
					iWidthConunt = 0;
			}
		};

		CUTSTATUS currentState = NONE;
		for (int indexPoints1st = 0; indexPoints1st < points1st.size(); indexPoints1st++)
		{
			int indexPoints2ndStart;
			int indexPoints2ndEnd;
			double width;
			int deltaHigh;
			MapPoint3D64 grapPoint;
			
	//		points1.push_back(points1st[i]);
			if (getClosestSeginfo(points1st[indexPoints1st], points2nd, indexPoints2ndStart, indexPoints2ndEnd, width, deltaHigh, grapPoint))
			{
				if (width < 25000 && deltaHigh< 200)  // 宽大于25m，高度差大于200（参照同高速）进行切分，不进行显示
				{
					if (currentState == NONE)
					{
						points1stPart.startIndex = 0;
						points1stPart.endIndex = indexPoints1st;

						points2ndPart.startIndex = 0;
						points2ndPart.endIndex = indexPoints2ndEnd;
					
					}
					else if(currentState == NOTCUT)
					{

						points1stPart.endIndex = indexPoints1st;
						points2ndPart.endIndex = indexPoints2ndEnd;
						
					}
					else
					{
						points1stPart.startIndex = indexPoints1st;
						points2ndPart.startIndex = indexPoints2ndEnd;
					}

					dTotalWidth += width; // 累计宽度，用于得到平均宽度
					iWidthConunt +=1 ;
					currentState = NOTCUT;
				}
				else
				{
					if (currentState == NONE)
					{
						points1stPart.startIndex = indexPoints1st;
						points2ndPart.startIndex = indexPoints2ndStart;
					}
					else if (currentState == NOTCUT)
					{
						makepart();

						points1stPart.startIndex = indexPoints1st;
						points2ndPart.startIndex = indexPoints2ndStart;   // 可以考虑补点
					
					}
					else  //CUTSTATUS = CUT;
					{
						points1stPart.startIndex = indexPoints1st;
						points2ndPart.startIndex = indexPoints2ndStart;

					}
					currentState = CUT;

				}
				// 两边尾部的处理  
				if (indexPoints1st == points1st.size() - 1) // 1到尾了
				{

					if (indexPoints2ndEnd != points2nd.size() - 1)
					{
						if (currentState == NOTCUT)
						{
							points1stPart.endIndex = indexPoints1st;
							points2ndPart.endIndex = points2nd.size() - 1;;
						}
						else
						{
							points1stPart.endIndex = indexPoints1st; // 后面的不要了
							points2ndPart.endIndex = indexPoints2ndEnd;
						}
					}
					makepart();
					break;
				}

				if (indexPoints2ndEnd == points2nd.size() - 1) // 2到尾了
				{
					if (indexPoints1st != points1st.size() - 1)
					{
						if (currentState == NOTCUT)
						{
							points1stPart.endIndex = points1st.size() - 1;
							points2ndPart.endIndex = indexPoints2ndEnd;
						}
						else
						{
							points1stPart.endIndex = indexPoints1st; // 后面的不要了
							points2ndPart.endIndex = indexPoints2ndEnd;
						}
					}
					makepart();
					break;
				}

			}
			else //最后一个点长于对边，有投不上的情况。
			{
				if (indexPoints1st == points1st.size() - 1 || indexPoints2ndEnd == points2nd.size() - 1) // 2到尾了
				{
					if (currentState == NOTCUT) // 如果之前没有没有切分。认为当前满足做隔离带需求。
					{
						points1stPart.endIndex = points1st.size() - 1;
						points2ndPart.endIndex = points2nd.size() - 1;
						makepart();
						break;
					}
				}
			}
		}


		if (Points1st2ndPartList.size()>0) // 
		{
			
			for (auto obj : Points1st2ndPartList)
			{
				PartPoints part;
				std::vector <MapPoint3D64> points1;
				std::vector <MapPoint3D64> points2;
				points1.assign(points1st.begin()+obj.point1stPart.startIndex, points1st.begin() + obj.point1stPart.endIndex+1 );
				points2.assign(points2nd.begin()+obj.point2ndPart.startIndex, points2nd.begin() + obj.point2ndPart.endIndex+1);
				part.points1 = points1;
				part.points2 = points2;
				part.AverageWidth = obj.AverageWidth;
				partPointslist.push_back(part);
			}
		}
		else
		{
			// assert(0);
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
		allPoints.insert(allPoints.end(), points2nd.begin(), points2nd.end()); //

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


	bool GreenbeltUrbanCompiler::existMiddleLaneGroup(std::vector<HadLaneGroup*>& in1stLaneGroups, std::vector<HadLaneGroup*>& in2ndLaneGroups)
	{
		auto makePolygon = [&](std::vector<MapPoint3D64>& boundaryVertexes, std::vector<MapPoint3D64>& nearbyBoundaryVertexes, Polygon3d& polygon)
		{
			polygon.vertexes.clear();
			MapPoint3D64 prevVertex = {};
			for (auto& vertex : boundaryVertexes) {
				if (!floatEqual(vertex.pos.distanceSquare(prevVertex.pos), 0)) {
					polygon.vertexes.push_back(vertex);
					prevVertex = vertex;
				}
			}
			for (auto& vertex : nearbyBoundaryVertexes) {
				if (!floatEqual(vertex.pos.distanceSquare(prevVertex.pos), 0)) {
					polygon.vertexes.push_back(vertex);
					prevVertex = vertex;
				}
			}

			// 闭环
			MapPoint3D64& startPt = polygon.vertexes.front();
			MapPoint3D64& endPt = polygon.vertexes.back();
			if (!floatEqual(startPt.pos.distanceSquare(endPt.pos), 0)) {
				polygon.vertexes.push_back(startPt);
			}
		};
#if 0
		auto makeBoundingBox2d = [](MapPoint3D64& point, MapPoint3D64& nearbyPoint, BoundingBox2d& bbox)
		{
			std::vector<int64> vx;
			std::vector<int64> vy;
			vx.push_back(point.pos.lon);
			vy.push_back(point.pos.lat);
			if (nearbyPoint.pos.lon != point.pos.lon) {
				vx.push_back(nearbyPoint.pos.lon);
			}
			else {
				vx.push_back(nearbyPoint.pos.lon + 10);
			}
			if (nearbyPoint.pos.lat != point.pos.lat) {
				vy.push_back(nearbyPoint.pos.lat);
			}
			else {
				vy.push_back(nearbyPoint.pos.lat + 10);
			}
			std::sort(vx.begin(), vx.end());
			std::sort(vy.begin(), vy.end());

			//	BoundingBox2d bbox;
			bbox.min.lon = vx[0];
			bbox.min.lat = vy[0];
			bbox.max.lon = *vx.rbegin();
			bbox.max.lat = *vy.rbegin();

			return true;
		};


#endif

		auto grapPoint = [](MapPoint3D64& point, std::vector<MapPoint3D64>& nearbyPoints, MapPoint3D64& nearbyGrappedPt, size_t& si, size_t& ei) -> bool {
			if (GrapPointAlgorithm::grapPoint(point, nearbyPoints, nearbyGrappedPt, si, ei) &&
				point.pos.distance(nearbyGrappedPt.pos) < 25000 &&
				fabs(point.z - nearbyGrappedPt.z) < 200) {
				return true;
			}
			return false;
		};


		std::vector<MapPoint3D64> points1st;		
		getPointsByLaneGroups(in1stLaneGroups, points1st);

		std::vector<MapPoint3D64> points2nd;
		getPointsByLaneGroups(in2ndLaneGroups, points2nd);

		Polygon3d polygon;
		makePolygon(points1st, points2nd, polygon);


		MapPoint3D64 point = points1st.front();
		MapPoint3D64 nearbyPoint = points2nd.back();
		if (!isSameDirection(points1st, points2nd))
		{
			nearbyPoint = points2nd.front();
		}
				
		BoundingBox2d bbox;
//		makeBoundingBox2d(point, nearbyPoint, bbox);


	//	std::vector<HadLaneGroup*> pNearbyLinkGroups = SpatialSeacher::seachNearby(m_nearby, bbox);
		int len1 = static_cast<int>(calcLength(points1st));
		int len2 = static_cast<int>(calcLength(points2nd));
		std::vector<HadLaneGroup*> NearbyLaneGroups = SpatialSeacher::seachNearby2d(m_pGrid, in1stLaneGroups[in1stLaneGroups.size()/2], (len1> len2? len1: len2)*1.5);

		MapPoint3D64 grappedPt = {};
		size_t si = 0, ei = 0;
		for (auto laneGroup : NearbyLaneGroups)
		{
			// 过滤掉本身
			bool bRet1stFind = std::find(in1stLaneGroups.begin(), in1stLaneGroups.end(), laneGroup) != in1stLaneGroups.end(); // 1st中是否存在
			bool bRet2ndFind = std::find(in2ndLaneGroups.begin(), in2ndLaneGroups.end(), laneGroup) != in2ndLaneGroups.end(); // 2nd中是否存在
			if (bRet1stFind || bRet2ndFind)
			{
				continue;
			}

			for (auto roadBoundary : laneGroup->roadBoundaries)
			{
				std::vector<MapPoint3D64> middleVertexes = roadBoundary->location.vertexes;
				for (auto& pt : middleVertexes) {
				//	if (GrapPointAlgorithm::isPointInPolygon(pt, polygon.vertexes)) //不准。20169163，227955326230925569
					point_2t p2t = POINT_2T(pt);
					polygon_2t polygon2t = POLYGON_2T(polygon.vertexes);
					if(bg::within(p2t, polygon2t))
					{
						// 判断高程差
						if (grapPoint(pt, points1st, grappedPt, si, ei)) {
							
							// 线上有相等重复点不算。
							bool bRet1stFind = std::find(points1st.begin(), points1st.end(), pt) != points1st.end(); // 1st中是否存在
							bool bRet2ndFind = std::find(points2nd.begin(), points2nd.end(), pt) != points2nd.end(); // 2nd中是否存在
							if (bRet1stFind || bRet2ndFind)
							{
								continue;
							}

#ifdef __DEBUG_GREENBELTURBAN__
							PrintPoints(points1st, "points1st existMiddleLaneGroup "); //debug
							PrintPoints(middleVertexes, "pointsMiddle existMiddleLaneGroup "); //debug							
							PrintPoints(points2nd, "points2nd existMiddleLaneGroup"); //debug
#endif
							return true;
						}
					}
				}
			}
		}

		return false;
	}


	bool GreenbeltUrbanCompiler::makeUturnGreenbelt(std::vector <MapPoint3D64>& points1st, std::vector <MapPoint3D64>& points2nd,RdsTile* pTile)
	{
		if (!isSameDirection(points1st, points2nd)) // 变成同方向的
		{
			std::reverse(points1st.begin(), points1st.end());
		}

		auto isConnectUturn = [](MapPoint3D64 &srcPoint, MapPoint3D64 &destPoint)
		{
			point_t startPoint = POINT_T(srcPoint);
			point_t endPoint = POINT_T(destPoint);
			double distance = bg::distance(startPoint, endPoint);
			if (distance < 300) // 0.3m
			{
				return true;
			}
			return false;
		};

		std::vector<std::vector<MapPoint3D64>> allLines;
		for (auto rdsLineInfo : compilerData.greenbeltLines)
		{
			allLines.push_back(rdsLineInfo.vertexes);
		}
	
		std::vector <MapPoint3D64> points1;
		std::vector <MapPoint3D64> points2;
		std::vector<MapPoint3D64> UtrunPoints;
		for (auto line : allLines)
		{
			UtrunPoints = line;

			if (isConnectUturn(points1st.front(), UtrunPoints.front()) || // 1st头-头相连
				isConnectUturn(points1st.front(), UtrunPoints.back()) // 1st头-尾相连				
				)
			{
				if (isConnectUturn(points2nd.front(), UtrunPoints.front()) || // 2nd头-头相连
					isConnectUturn(points2nd.front(), UtrunPoints.back()) // 2nd头-尾相连				
					)
				{
					points1.push_back(points1st.front());
					points1.push_back(points2nd.front());
					points2 = UtrunPoints;
				}

				if (points1.size() > 0 && points2.size() > 0)
				{
					makeGreenbeltData(points1, points2, pTile);
				}
			}
			if (isConnectUturn(points1st.back(), UtrunPoints.front()) || // 尾-头相连
				isConnectUturn(points1st.back(), UtrunPoints.back())  // 尾-尾相连 
				)
			{
				if (isConnectUturn(points2nd.back(), UtrunPoints.front()) || // 尾-头相连
					isConnectUturn(points2nd.back(), UtrunPoints.back())  // 尾-尾相连 
					)
				{
					points1.push_back(points1st.back());
					points1.push_back(points2nd.back());
					points2 = UtrunPoints;
				}

				if (points1.size()>0 && points2.size() > 0)
				{
					makeGreenbeltData(points1, points2, pTile);
				}
			}

#if 0
			if (isConnectUturn(points1st.front(), UtrunPoints.front())) // 头-头相连
			{
				points1st.insert(points1st.begin(), UtrunPoints.begin(), UtrunPoints.end());
			}
			else if(isConnectUturn(points1st.front(), UtrunPoints.back())) // 头-尾相连
			{
				std::reverse(UtrunPoints.begin(), UtrunPoints.end());
				points1st.insert(points1st.begin(), UtrunPoints.begin(), UtrunPoints.end());
			}
			else if (isConnectUturn(points1st.back(), UtrunPoints.front())) // 尾-头相连
			{
				points1st.insert(points1st.end(), UtrunPoints.begin(), UtrunPoints.end());
			}
			else if (isConnectUturn(points1st.back(), UtrunPoints.back()))  // 尾-尾相连 
			{
				std::reverse(UtrunPoints.begin(), UtrunPoints.end());
				points1st.insert(points1st.end(), UtrunPoints.begin(), UtrunPoints.end());
			}
#endif
		};
		return true;
	}


	bool GreenbeltUrbanCompiler::insertPointAtFraction(MapPoint3D64& start, MapPoint3D64& end, double fraction, MapPoint3D64& outpoint)
	{
		float length = calcLength(start, end);

		if (length > 0)
		{
			// 计算新点的坐标  
			double newX = start.pos.lon + fraction * length * (end.pos.lon - start.pos.lon) / length;
			double newY = start.pos.lat + fraction * length * (end.pos.lat - start.pos.lat) / length;

			outpoint.pos.lon = newX;
			outpoint.pos.lat = newY;
			outpoint.z = end.z;
			return true;
		}
		
		return false;
	}

	//
	bool GreenbeltUrbanCompiler::deleteInvalidIntersectionLaneGroup(std::vector<HadLaneGroup*>& in1stLaneGroups, std::vector<HadLaneGroup*>& in2ndLaneGroups)
	{

		auto isDeleteInvalidIntersection = [&, this](HadLaneGroup* intersectionLaneGroup, std::vector<HadLaneGroup*>& in2ndLaneGroups)
		{
			if (intersectionLaneGroup->inIntersection == 0)
			{
				return false;
			}

			LineString3d line;
			getLaneGroupLocationLine(intersectionLaneGroup, line);
			MapPoint3D64 in1stMiddlePoint;
			in1stMiddlePoint.pos.lat= (line.vertexes.front().pos.lat + line.vertexes.back().pos.lat) / 2;
			in1stMiddlePoint.pos.lon = (line.vertexes.front().pos.lon + line.vertexes.back().pos.lon) / 2;
			in1stMiddlePoint.z = (line.vertexes.front().z + line.vertexes.back().z) / 2;

			getLaneGroupLocationLine(in2ndLaneGroups.front(), line);
			MapPoint3D64 in2ndStartPoint;
			in2ndStartPoint = line.vertexes.front();

			getLaneGroupLocationLine(in2ndLaneGroups.back(), line);
			MapPoint3D64 in2ndEndPoint;
			in2ndEndPoint = line.vertexes.back();

			float fstartlen = calcLength(in1stMiddlePoint, in2ndStartPoint);
			float fendlen = calcLength(in1stMiddlePoint, in2ndEndPoint);

			//int iotherIndex = 0;
			//getNearestOtherLaneGroup(intersectionLaneGroup, in2ndLaneGroups, iotherIndex);

			std::vector<HadLaneGroup*> laneGrops1st;
			laneGrops1st.push_back(intersectionLaneGroup);


			std::vector<HadLaneGroup*> laneGrops2nd;			
			if (fstartlen > fendlen) // 使用距离最短的头或者尾
			{
				laneGrops2nd.push_back(in2ndLaneGroups.back());
			}
			else
			{
				laneGrops2nd.push_back(in2ndLaneGroups.front());
			}

			if (existMiddleLaneGroup(laneGrops1st, laneGrops2nd))
			{
				return true;
			}
			return false;
		};

		// expandConnectionLaneGroups中，追加完IntersectionLaneGroup后，就会停止，所以，IntersectionLaneGroup只会出现在头或者尾部。	
		// check 1st lanegroups
		if (in1stLaneGroups.front()->inIntersection !=0)
		{
			if (isDeleteInvalidIntersection(in1stLaneGroups.front(), in2ndLaneGroups))
			{
				in1stLaneGroups.erase(in1stLaneGroups.begin());
			}
		}
		
		if (in1stLaneGroups.back()->inIntersection != 0)
		{
			if (isDeleteInvalidIntersection(in1stLaneGroups.back(), in2ndLaneGroups))
			{
				in1stLaneGroups.pop_back();
			}
		}

		// check 2nd lanegroups 
		if (in2ndLaneGroups.front()->inIntersection != 0)
		{
			if (isDeleteInvalidIntersection(in2ndLaneGroups.front(), in1stLaneGroups))
			{
				in2ndLaneGroups.erase(in2ndLaneGroups.begin());
			}
		}

		if (in2ndLaneGroups.back()->inIntersection != 0)
		{
			if (isDeleteInvalidIntersection(in2ndLaneGroups.back(), in1stLaneGroups))
			{
				in2ndLaneGroups.pop_back();
			}
		}

		return true;

	}

	bool GreenbeltUrbanCompiler::getNearestOtherLaneGroup(HadLaneGroup* in1stLaneGroup, std::vector<HadLaneGroup*>& in2ndLaneGroups, int& outLangGroupIndex)
	{

		LineString3d line;
		getLaneGroupLocationLine(in1stLaneGroup, line);
		MapPoint3D64 in1stMiddlePoint;
		in1stMiddlePoint.pos.lat= (line.vertexes.front().pos.lat + line.vertexes.back().pos.lat) / 2;
		in1stMiddlePoint.pos.lon = (line.vertexes.front().pos.lon + line.vertexes.back().pos.lon) / 2;
		in1stMiddlePoint.z = (line.vertexes.front().z + line.vertexes.back().z) / 2;


		std::vector<MapPoint3D64> in2ndPoints;
		getPointsByLaneGroups(in2ndLaneGroups, in2ndPoints);

		std::vector<MapPoint3D64>::iterator ita, itb;
		getClosestSeg(in1stMiddlePoint, in2ndPoints, ita, itb);//

		for (int index = 0; index < in2ndLaneGroups.size(); index++)
		{
			LineString3d line;
			getLaneGroupLocationLine(in2ndLaneGroups[index], line);
	//		bool ret1 = std::find(line.vertexes.begin(), line.vertexes.end(), *ita) != line.vertexes.end();
			bool ret2 = std::find(line.vertexes.begin(), line.vertexes.end(), *itb) != line.vertexes.end();
			if(ret2)
			{
				outLangGroupIndex = index;
				return true;
			}		
		}

		return false;

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

	void GreenbeltUrbanCompiler::PrintInfo(std::string info)
	{
		info += "\n";
		m_debugofs << info.c_str();
	}
#endif

}