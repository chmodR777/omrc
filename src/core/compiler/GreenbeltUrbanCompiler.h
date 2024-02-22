#pragma once
#include "Compiler.h"
#include <fstream>


//#define __DEBUG_GREENBELTURBAN__   // 打印生成制作隔离带过程的文件。

namespace OMDB
{
	/// <summary>
	/// 城市绿化带
	/// </summary>
	class GreenbeltUrbanCompiler : public Compiler
	{

	public:
		GreenbeltUrbanCompiler(CompilerData& data);
	
	protected:
		virtual void compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile) override;


	private:
		bool getGreenbeltLaneGroups(HadGrid* const pGrid, std::vector<HadLaneGroup*>& laneGroups);
		
		bool grabOtherLaneGroup(HadGrid* const pGrid,HadLaneGroup* inLaneGroup, HadLaneGroup*& outNearLaneGroup);

		bool getNearbyLaneGropuByAngle(HadLaneGroup* inLaneGroup, std::vector<HadLaneGroup*>& inoutNearbyLaneGroups);

		bool getNearbyLaneGropuByDistance(HadLaneGroup* inLaneGroup, std::vector<HadLaneGroup*>& inNearbyLaneGroups, HadLaneGroup*& outNearLaneGroup);
				
		bool expandConnectionLaneGroups(HadLaneGroup* inLaneGroup, const std::vector<HadGrid*>& nearby, std::vector<HadLaneGroup*>& outConnectionLaneGroups);

		bool getPointsByLaneGroups(std::vector<HadLaneGroup*> inLaneGroups, std::vector<MapPoint3D64>& Points);

		bool ableGreenbeltLaneGroup(HadLaneGroup* inLaneGroup);

		bool alignLaneGroupsbylength(std::vector<HadLaneGroup*> & in1stLaneGroups, std::vector<HadLaneGroup*>& in2ndLaneGroups);

		bool alignPoints(std::vector <MapPoint3D64>& points1st, std::vector <MapPoint3D64>& points2nd);

#ifdef __DEBUG_GREENBELTURBAN__
		//debug
		void PrintPoints(std::vector<MapPoint3D64> lanepoints, std::string mark = "");
		void PrintLaneGroup(HadLaneGroup* inLaneGroup, std::string mark = "");
		void PrintLaneGroup(std::vector<HadLaneGroup*> LaneGroups, std::string mark = "");
#endif
	
		

	private:
		HadGrid* m_pGrid;
		std::vector<HadLaneGroup*>  m_1stLaneGroups;
		std::vector< HadLaneGroup*> m_2ndLaneGroups;
		std::vector< HadLaneGroup*> m_VisitLaneGroups;	

#ifdef __DEBUG_GREENBELTURBAN__
		//debug
		std::ofstream m_debugofs;
#endif

	};


}

