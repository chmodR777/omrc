#pragma once
#include <vector>
#include "Compiler.h"
#include "navi_rds/rds.h"
#include "algorithm/line_projection_triangle_surface.h"

using namespace RDS;
namespace OMDB
{
    class LineCompiler : public Compiler
    {
	public:
        LineCompiler(CompilerData& data) :Compiler(data) {};
    protected:
        virtual void compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile) override;

    private:
        void compileStopLocation(HadGrid* const pGrid, const std::vector<Triangle>& surfaceTriangles, const rtree_type_2box& surfaceRTree2T, RdsTile* pTile);
		void clipOutOfRoadFace(const std::vector<Triangle>& surfaceTriangles, const rtree_type_2box& surfaceRTree2T, std::vector<MapPoint3D64>& outPts);

        void compileLaneBoundary(HadGrid* const pGrid, const std::vector<Triangle>& surfaceTriangles, const rtree_type_2box& surfaceRTree2T, RdsTile* pTile);
        void groupByLaneBoundaryRange(HadLaneBoundary* const pLaneBoundary, std::map<int64, std::vector<HadPartAttribute*>> & groupAttributes);
        void groupByMarkSeqNum(std::vector<HadPartAttribute*>& attributes, std::map<int64, std::vector<HadPartAttribute*>>& groupAttributes);
        LineString3d createSubLaneBoundary(LineString3d& location, MultiPoint3d& points);
        void adjustLaneBoundaryOffset(LineString3d& location, int offset, Polygon3d& polygon);
        void saveRdsLine(LineString3d& location);
        forceinline int markingValue(int value) 
        {
            if (value != DB_HAD_APPLY_PA_REFERENCE && value != DB_HAD_NOT_APPLY_PA_REFERENCE) {
                return value;
            }
            return 0;
        }

        //根据百分比截取线段 startOffset,endOffset[0,1]
        void truncateLineSegment(const std::vector<MapPoint3D64>& line, double startOffset, double endOffset, std::vector<MapPoint3D64>& outline);
        double _getLength(const std::vector<MapPoint3D64>& line, std::vector<double>& sections);
        void mergeTurnwaitingPAs(const std::vector<HadLaneTurnwaiting*>& turnwaitinPA, std::vector<RangeF>& out);
    };
}
