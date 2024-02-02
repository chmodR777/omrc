#include "stdafx.h"
#include "LaneBoundaryGenerator.h"
#include "algorithm/grap_point_algorithm.h"
#include <algorithm>
namespace OMDB
{
    void LaneBoundaryGenerator::generate(DbMesh* const pMesh)
    {
		UNREFERENCED_PARAMETER(pMesh);
    }

    void LaneBoundaryGenerator::generateRelation(DbMesh* const pMesh, std::vector<DbMesh*>* nearby)
    {
		UNREFERENCED_PARAMETER(pMesh);
		UNREFERENCED_PARAMETER(nearby);
    }
}
