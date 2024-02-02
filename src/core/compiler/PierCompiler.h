#pragma once
#include "Compiler.h"
#include <list>
#include "geometry/map_point3d64.h"
#include "math3d/vector3.h"
#include "had/had_point3d.h"

namespace OMDB
{
    class PierCompiler : public Compiler
    {
	public:
		PierCompiler(CompilerData& data) :Compiler(data) {};
	private:
		static const int LANE_WIDTH_DEFAULT = 375;        //车道默认宽度。单位：厘米。

    protected:
		virtual void compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile) override;

	private:
		struct _PierTemp
		{
			MapPoint3D64 pt;
			double angle;
			//int64 lgid;		//所在的车道组ID。
			HadLaneGroup* lg;	//所在车道组指针（指向的数据地址生命周期为进程processor数据后直到死亡）
		};

	private:
		void _initNdsGird(NdsGridId gridId);

		void _pierResampleCallback(Vector3 point, Vector3 dir);
		static void _pierResampleCallbackWrapper(Vector3 point, Vector3 dir, void* userdata) {
			((PierCompiler*)userdata)->_pierResampleCallback(point, dir);
		}

		bool _recursiveForwardLaneGroup(HadLaneGroup* pGroup, std::list<HadLaneGroup*>& listCombineLaneGroup);
		bool _recursiveBackwardLaneGroup(HadLaneGroup* pGroup, std::list<HadLaneGroup*>& listCombineLaneGroup);

		Vector3 _relFromMapPoint3D64(MapPoint3D64 had)
		{
			return vec3(
				(had.pos.lon - m_baseHadPoint3D.pos.lon) * m_sampleScale.x,
				(had.pos.lat - m_baseHadPoint3D.pos.lat) * m_sampleScale.y,
				(had.z - m_baseHadPoint3D.z) * m_sampleScale.z);
		}

		MapPoint3D64 _mapPoint3D64FromRel(MapPoint3D64 rel)
		{
			return MapPoint3D64_make(
				(int64)(rel.pos.lon * m_sampleScaleInv.x) + m_baseHadPoint3D.pos.lon,
				(int64)(rel.pos.lat * m_sampleScaleInv.y) + m_baseHadPoint3D.pos.lat,
				(int32)(rel.z * m_sampleScaleInv.z) + m_baseHadPoint3D.z
			);
		}

		bool _calTangentialCutWithBoundary(MapPoint3D64 pt, float dir, HadLaneBoundary* laneBoundary, MapPoint3D64& rstPt);

		void _makePierBoundingBox(const NdsPoint& ptCenter, int radius, BoundingBox2d& pierBoundingBox);

		size_t _makeLaneGroupPolygon(HadLaneGroup* pGroup, MapPoint3D64*& polygon);
		size_t _makeBoundingBoxPolygon(BoundingBox2d boundingBox, MapPoint3D64*& polygon);
		bool _hasIntersect(MapPoint3D64* points1, size_t pointCount1, MapPoint3D64* points2, size_t pointCount2);

		bool _collisionDetection(HadGrid* pGrid, HadLaneGroup* pGroup, MapPoint3D64 ptCenter, BoundingBox2d& pierBoundingBox);
		float _combineLaneGroupRresample(std::list<HadLaneGroup*>& listCombineLaneGroup, float interval, float start);
		void _amendResamplePt();
		MapPoint3D64 _nearestEndPoint(const MapPoint3D64& mp, const std::vector<MapPoint3D64>& vertexes);

	private:
		MapPoint3D64 m_baseHadPoint3D;		//当前图幅内左上基点。临时存储。
		Vector3 m_sampleScale;
		Vector3 m_sampleScaleInv;

		std::set< int64 > m_setGroupIdInMesh;	

		//int64	m_curLgid;		//当前正在处理的车道组ID。
		HadLaneGroup* m_pCurLaneGroup;	//当前在处理的车道组指针（指向的数据地址生命周期为进程processor数据后直到死亡）
		std::vector<_PierTemp>  m_vctResampleResult;

    };
}

