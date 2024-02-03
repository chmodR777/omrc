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
	m_AllLaneGroups(),
	m_debugofs()
	{

	};

	// 编译开始
	void GreenbeltUrbanCompiler::compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile)
	{

		m_debugofs = std::ofstream(std::to_string(pGrid->getId())+".txt", std::ios_base::out | std::ios_base::trunc);
		printf("---------------compile::start()  pGrid[m_meshId=%d]----------------------", pGrid->getId());

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

			printf("=====compile::start() laneGroups(laneGroups.roadBoundaries[0].originId[%lld])=====\n",
				obj->roadBoundaries.size()?(*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId:0);
			
		
			std::vector<HadLaneGroup*> pNearbyLinkGroups;
	//		if ((*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)obj->roadBoundaries[0])))).originId == (227961643662250241))
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
				PrintLaneGroup(pGroup, "inLaneGroup");
				PrintLaneGroup(otherLaneGroup, "grabLaneGroup");
				printf("grabOtherLaneGroup (  ) otherLaneGroup(laneGroups.roadBoundaries[0].originId[%lld])\n",
					(*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)otherLaneGroup->roadBoundaries[0])))).originId);

				expandConnectionLaneGroups(otherLaneGroup, nearbyGrids, pNearbyLinkGroups);  // 通过抓取到的LaneGroup，做出另一条隔离带
				if (pNearbyLinkGroups.empty())
				{					
					continue;
				}
				m_2ndLaneGroups = pNearbyLinkGroups;
								
				std::vector<MapPoint3D64> Points1st;
				getPointsByLaneGroups(m_1stLaneGroups,Points1st);

				std::vector <MapPoint3D64> Points2nd;
				getPointsByLaneGroups(m_2ndLaneGroups,Points2nd);

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

				//PrintPoints(Points2nd, "point.txt"); debug

				m_AllLaneGroups.insert(m_AllLaneGroups.end(), m_1stLaneGroups.begin(), m_1stLaneGroups.end());
				m_AllLaneGroups.insert(m_AllLaneGroups.end(), m_2ndLaneGroups.begin(), m_2ndLaneGroups.end());
			}			

		}
		    
		m_debugofs.close();
	}

	//  得到能制作普通隔离带的道路的LaneGroups
	bool GreenbeltUrbanCompiler::getGreenbeltLaneGroups(HadGrid* const pGrid, std::vector<HadLaneGroup*>& laneGroups)
	{		
		for (auto obj : pGrid->query(ElementType::HAD_LANE_GROUP))
		{
			HadLaneGroup* pGroup = (HadLaneGroup*)obj;

			//PrintLaneGroup(pGroup); //debug

			//普通路绿化带
			if((isProDataLevel(pGroup)||isMultiDigitized(pGroup))
				&&(pGroup->inIntersection ==0))  // 带有Intersection的不做隔离带			
			{
				laneGroups.push_back(pGroup);
				PrintLaneGroup(pGroup,"greenbelt:");
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
			if (getLinkGroupAngle(inLaneGroup, laneGroup) < 30)
			{
				if(isInConnectionLaneGroups(laneGroup)==false)
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

				PrintLaneGroup(currentLaneGroup, "Prev->");
				printf("expandConnectionLaneGroups(previous ) currentLaneGroup(laneGroups.roadBoundaries[0].originId[%lld])\n",
					(*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)currentLaneGroup->roadBoundaries[0])))).originId);
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
				PrintLaneGroup(currentLaneGroup, "Next->");
				printf("expandConnectionLaneGroups( next ) currentLaneGroup(laneGroups.roadBoundaries[0].originId[%lld])\n",
					(*((OMDB::HadElement*)&(*((OMDB::HadSkeleton*)currentLaneGroup->roadBoundaries[0])))).originId);

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


	void GreenbeltUrbanCompiler::PrintPoints(std::vector<MapPoint3D64> lanepoints, std::string filename)
	{
		
		std::string temp = "lon,lat\n";

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

}