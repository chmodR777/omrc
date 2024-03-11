#pragma once
#include <vector>
#include "../record/DbRecord.h"
#include "../record/DbMesh.h"
namespace OMDB
{
	/// <summary>
	/// 空间搜索类,当前只支持AABB包围盒重叠判断
	/// </summary>
	class RTreeSeacher
	{
	public:
		static std::vector<DbLink*> seachNearby3d(const std::vector<DbMesh*>& meshes, DbLink* pLink, MapPoint3D64 point, int32 posTolerance = 10, int32 zTolerance = 10);
		static std::vector<DbLink*> seachNearby2d(const std::vector<DbMesh*>& meshes, DbLink* pLink, MapPoint3D64 point, int32 tolerance = 10);
		static std::vector<DbLink*> seachNearby(const std::vector<DbMesh*>& meshes, const BoundingBox3d& input, int32 posTolerance = 10, int32 zTolerance = 10);
		static std::vector<DbLink*> seachNearby(const std::vector<DbMesh*>& meshes, const BoundingBox2d& input, int32 tolerance = 10);

		static std::vector<DbLink*> seachNearby3d(DbMesh* pMesh, DbLink* pLink, MapPoint3D64 point, int32 posTolerance = 10, int32 zTolerance = 10);
		static std::vector<DbLink*> seachNearby2d(DbMesh* pMesh, DbLink* pLink, MapPoint3D64 point, int32 tolerance = 10);
		static std::vector<DbLink*> seachNearby(DbMesh* pMesh, const BoundingBox3d& input, int32 posTolerance = 10, int32 zTolerance = 10);
		static std::vector<DbLink*> seachNearby(DbMesh* pMesh, const BoundingBox2d& input, int32 tolerance = 10);

		static std::vector<DbRdLinkLanePa*> seachNearby3dGroup(const std::vector<DbMesh*>& meshes, DbRdLinkLanePa* pLanePa, MapPoint3D64 point, int32 posTolerance = 10, int32 zTolerance = 10);
		static std::vector<DbRdLinkLanePa*> seachNearby2dGroup(const std::vector<DbMesh*>& meshes, DbRdLinkLanePa* pLanePa, MapPoint3D64 point, int32 tolerance = 10);
		static std::vector<DbRdLinkLanePa*> seachNearbyGroup(const std::vector<DbMesh*>& meshes, const BoundingBox3d& input, int32 posTolerance = 10, int32 zTolerance = 10);
		static std::vector<DbRdLinkLanePa*> seachNearbyGroup(const std::vector<DbMesh*>& meshes, const BoundingBox2d& input, int32 tolerance = 10);

		static std::vector<DbRdLinkLanePa*> seachNearby3dGroup(DbMesh* pMesh, DbRdLinkLanePa* pLanePa, MapPoint3D64 point, int32 posTolerance = 10, int32 zTolerance = 10);
		static std::vector<DbRdLinkLanePa*> seachNearby2dGroup(DbMesh* pMesh, DbRdLinkLanePa* pLanePa, MapPoint3D64 point, int32 tolerance = 10);
		static std::vector<DbRdLinkLanePa*> seachNearbyGroup(DbMesh* pMesh, const BoundingBox3d& input, int32 posTolerance = 10, int32 zTolerance = 10);
		static std::vector<DbRdLinkLanePa*> seachNearbyGroup(DbMesh* pMesh, const BoundingBox2d& input, int32 tolerance = 10);

		/**
		 * @brief 判断线段(segmentStart, segmentEnd)是否和Mesh中的link存在相交
		 * @param pMesh 
		 * @param segmentStart 
		 * @param segmentEnd 
		 * @return 
		*/
		static bool intersectsWithLinks2d(DbMesh* pMesh, const MapPoint64& segmentStart, const MapPoint64& segmentEnd);
		static bool intersectsWithLinks2d(std::vector<DbMesh*> meshes, const MapPoint64& segmentStart, const MapPoint64& segmentEnd);
	};
}

