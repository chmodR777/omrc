#include "stdafx.h"
#include "LaneGenerator.h"
#include "algorithm/grap_point_algorithm.h"
#include <algorithm>
namespace OMDB
{
    void LaneGenerator::generate(DbMesh* const pMesh)
    {
		UNREFERENCED_PARAMETER(pMesh);
    }

    void LaneGenerator::generateRelation(DbMesh* const pMesh, std::vector<DbMesh*>* nearby)
    {
		UNREFERENCED_PARAMETER(pMesh);
		UNREFERENCED_PARAMETER(nearby);
    }
}
