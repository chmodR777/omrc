#pragma once
#include "Compiler.h"
namespace OMDB
{

    struct OverlayLaneGroup
    {
        HadLaneGroup* upperLG;
        MapPoint3D64 upperCrossPt;
        MapPoint3D64 lowerCrossPt;
    };
    
    using PtrOverlayLaneGroup = std::shared_ptr<OverlayLaneGroup>;

    class TransparentCompiler : public Compiler
    {
    public:
        TransparentCompiler(CompilerData& data) :Compiler(data) {};
    protected:
        virtual void compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile) override;

        void getCenterLine(HadLaneGroup* pGroup, LineString3d& line);
       
    private:
        void overlayLaneGroup();
        void expandUpperRoad(bool bforward, int length);
        void expandLowerRoad(bool bforward, int length);
        void writeRDS();
        ring_2t getRect(MapPoint3D64 start, MapPoint3D64 end, int distance);
        void bfsLaneGroup(HadLaneGroup* lg, bool bForward, float searchDistance, std::vector<HadLaneGroup*>& lgOut);
    private:
        // 原始压盖关系
        std::unordered_map < HadLaneGroup*, std::vector<PtrOverlayLaneGroup>> _overLapRel;
        // 原始压盖+延展后的关系关系
        std::unordered_map < HadLaneGroup*, std::set<HadLaneGroup*>> _overLapRelExpand;

        HadGrid* _pGrid;
        std::vector<HadGrid*> _nearby;
        RdsTile* _pTile;

    };
}

