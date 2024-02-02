#pragma once

#include "geometry/map_point64.h"
#include "geometry/map_point3d64.h"
#include "math3d/vector2.h"
#include "math3d/vector3.h"
#include "math3d/nds_point_3d.h"

#include "cq_types_basic.h"
#include "base/nds.h"
#include "cqstl/vector.h"

namespace hadc
{
	class HadCoordinateConverter
	{
	public:
		void setGrid(NdsGridId gridId);

		const cqstd::vector<Vector3>& mapPoints2GridEx(const MapPoint64* points, int n);
		const cqstd::vector<Vector3>& mapPoint3Ds2GridEx(const MapPoint3D64* points, int n);
		const cqstd::vector<Vector3>& mapPoint3Ds2GridExWithHeightOffset(const MapPoint3D64* points, int n, float zOffset/*unit meters*/);
		const cqstd::vector<Vector3>& ndsPoints2GridEx(const NdsPoint* points, int n);
		const cqstd::vector<Vector3>& ndsPoint3Ds2GridEx(const NdsPoint3D* points, int n);

		Rect tileRectInGridEx() { return m_tileRectInGridEx; }
		RectF tileRectFInGridEx() { return m_tileRectFInGridEx; }
		RectF tileRectFMarsInGridEx() { return m_tileRectFMarsInGridEx; }

	private:
		void _calcTileRect();

		void transformPointsGrid3D2GridEx(const Vector3* rawGridPoint3Ds, int n, int inputStride, Vector3* pointsOut, int outputStride);
		void transformPointsGrid2GridEx(const Vector2* rawGridPoints, int n, int inputStride, Vector3* pointsOut, int outputStride);

		NdsGridId m_gridId;

		NdsPoint m_cornerSW;
		float m_scaleX;
		float m_scaleY;
		cqstd::vector<Vector3> m_gridExPointBuffer;
		cqstd::vector<Vector2> m_gridPointBuffer;
		cqstd::vector<Vector3> m_gridPoint3DBuffer;
		Rect m_tileRectInGridEx;
		RectF m_tileRectFInGridEx;
		RectF m_tileRectFMarsInGridEx;
	};

}
