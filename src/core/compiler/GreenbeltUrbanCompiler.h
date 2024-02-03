#pragma once
#include "Compiler.h"
#include <fstream>

namespace OMDB
{
	/// <summary>
	/// 城市绿化带
	/// </summary>
#if 0
	class GreenbeltUrbanCompiler : public Compiler
	{
	public:
		GreenbeltUrbanCompiler(CompilerData& data) :Compiler(data) {};
		GreenbeltUrbanCompiler(){};

		void startGreenbeltUrbanCompile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile);

	protected:

		virtual void compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile) override;

	};
#endif

	class GreenbeltUrbanCompiler : public Compiler
	{
	public:
	//	GreenbeltUrbanCompiler(CompilerData& data) :Compiler(data) {};
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

		//debug
		void PrintPoints(std::vector<MapPoint3D64> lanepoints, std::string filename = "Greenbelt_points");
		void PrintLaneGroup(HadLaneGroup* inLaneGroup, std::string mark = "");

	private:
		std::vector<HadLaneGroup*>  m_1stLaneGroups;
		std::vector< HadLaneGroup*> m_2ndLaneGroups;
		std::vector< HadLaneGroup*> m_AllLaneGroups;

		//debug
		std::ofstream m_debugofs;
	};


}

