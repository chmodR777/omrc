#include "stdafx.h"
#include "had_coordinate_converter.h"
#include "map-render/glmap/glmap_constants.h"

namespace hadc
{
	void HadCoordinateConverter::setGrid(NdsGridId gridId)
	{
		m_gridId = gridId;

		NdsRect ndsRect;
		NdsGridId_getNdsRect(m_gridId, &ndsRect);

		m_scaleX = glmap::MAX_GRID_COORD / ndsRect.width();
		m_scaleY = glmap::MAX_GRID_COORD / ndsRect.height();
		m_cornerSW = NdsPoint_make(ndsRect.left, ndsRect.top);

		_calcTileRect();

		NdsRect bbox84;
		NdsGridId_getNdsRect(m_gridId, &bbox84);

		Point corners84[4] =
		{
			Point_fromNdsPoint(NdsPoint_make(bbox84.left, bbox84.top)),
			Point_fromNdsPoint(NdsPoint_make(bbox84.left, bbox84.bottom)),
			Point_fromNdsPoint(NdsPoint_make(bbox84.right, bbox84.bottom)),
			Point_fromNdsPoint(NdsPoint_make(bbox84.right, bbox84.top)),
		};

		Point corners02[4];
		for (int i = 0; i < 4; i++)
			Math_wgsToMars(&corners84[i], &corners02[i]);

		NdsPoint corners02Nds[4];
		for (int i = 0; i < 4; i++)
			corners02Nds[i] = NdsPoint_makeWithPoint(corners02[i]);

		const cqstd::vector<Vector3>& gridPointsEx = ndsPoints2GridEx(corners02Nds, 4);

		m_tileRectFMarsInGridEx.left = m_tileRectFMarsInGridEx.top = FLT_MAX;
		m_tileRectFMarsInGridEx.right = m_tileRectFMarsInGridEx.bottom = -FLT_MAX;

		Rect_combinePoint(&m_tileRectFMarsInGridEx, &gridPointsEx[0]);
		Rect_combinePoint(&m_tileRectFMarsInGridEx, &gridPointsEx[1]);
		Rect_combinePoint(&m_tileRectFMarsInGridEx, &gridPointsEx[2]);
		Rect_combinePoint(&m_tileRectFMarsInGridEx, &gridPointsEx[3]);
	}

	const cqstd::vector<Vector3>& HadCoordinateConverter::mapPoints2GridEx(const MapPoint64* points, int n)
	{
		m_gridPointBuffer.clear();

		for (int i = 0; i < n; i++)
		{
			NdsPoint pointNds = points[i].toNdsPoint();
			m_gridPointBuffer.resize(m_gridPointBuffer.size() + 1);
			m_gridPointBuffer.back().x = (pointNds.x - m_cornerSW.x) * m_scaleX;
			m_gridPointBuffer.back().y = (pointNds.y - m_cornerSW.y) * m_scaleY;
		}

		m_gridExPointBuffer.resize(m_gridPointBuffer.size());
		transformPointsGrid2GridEx(m_gridPointBuffer.begin(), (int)m_gridPointBuffer.size(), sizeof(Vector2), m_gridExPointBuffer.begin(), sizeof(Vector3));

		return m_gridExPointBuffer;
	}

	const cqstd::vector<Vector3>& HadCoordinateConverter::mapPoint3Ds2GridEx(const MapPoint3D64* points, int n)
	{
		return mapPoint3Ds2GridExWithHeightOffset(points, n, 0);
	}

	static NdsPoint3D _mapPoint3DtoNdsPoint3D(const MapPoint3D64& point, float offset)
	{
		NdsPoint3D nds;

		nds.point = point.pos.toNdsPoint();
		nds.height = (point.z / 100.0f + offset) * NDS_LAT_UNIT_PER_METER;

		return nds;
	}

	const cqstd::vector<Vector3>& HadCoordinateConverter::mapPoint3Ds2GridExWithHeightOffset(const MapPoint3D64* points, int n, float zOffset)
	{
		m_gridPoint3DBuffer.clear();

		for (int i = 0; i < n; i++)
		{
			NdsPoint3D pointNds = _mapPoint3DtoNdsPoint3D(points[i], zOffset);
			m_gridPoint3DBuffer.resize(m_gridPoint3DBuffer.size() + 1);
			m_gridPoint3DBuffer.back().x = (pointNds.x - m_cornerSW.x) * m_scaleX;
			m_gridPoint3DBuffer.back().y = (pointNds.y - m_cornerSW.y) * m_scaleY;
			m_gridPoint3DBuffer.back().z = pointNds.height / Nds_getGridSize(13) * glmap::MAX_GRID_COORD;
		}

		m_gridExPointBuffer.resize(m_gridPoint3DBuffer.size());
		transformPointsGrid3D2GridEx(m_gridPoint3DBuffer.begin(), (int)m_gridPoint3DBuffer.size(), sizeof(Vector3), m_gridExPointBuffer.begin(), sizeof(Vector3));

		return m_gridExPointBuffer;
	}

	const cqstd::vector<Vector3>& HadCoordinateConverter::ndsPoints2GridEx(const NdsPoint* points, int n)
	{
		m_gridPoint3DBuffer.clear();

		for (int i = 0; i < n; i++)
		{
			m_gridPoint3DBuffer.resize(m_gridPoint3DBuffer.size() + 1);
			m_gridPoint3DBuffer.back().x = (points[i].x - m_cornerSW.x) * m_scaleX;
			m_gridPoint3DBuffer.back().y = (points[i].y - m_cornerSW.y) * m_scaleY;
			m_gridPoint3DBuffer.back().z = 0;
		}

		m_gridExPointBuffer.resize(m_gridPoint3DBuffer.size());
		transformPointsGrid3D2GridEx(m_gridPoint3DBuffer.begin(), (int)m_gridPoint3DBuffer.size(), sizeof(Vector3), m_gridExPointBuffer.begin(), sizeof(Vector3));

		return m_gridExPointBuffer;
	}

	const cqstd::vector<Vector3>& HadCoordinateConverter::ndsPoint3Ds2GridEx(const NdsPoint3D* points, int n)
	{
		m_gridPoint3DBuffer.clear();

		for (int i = 0; i < n; i++)
		{
			m_gridPoint3DBuffer.resize(m_gridPoint3DBuffer.size() + 1);
			m_gridPoint3DBuffer.back().x = (points[i].x - m_cornerSW.x) * m_scaleX;
			m_gridPoint3DBuffer.back().y = (points[i].y - m_cornerSW.y) * m_scaleY;
			m_gridPoint3DBuffer.back().z = points[i].height / 100 * METER_PER_NDS_LAT_UNIT * NDS_LAT_UNIT_PER_METER / Nds_getGridSize(13) * glmap::MAX_GRID_COORD;
		}

		m_gridExPointBuffer.resize(m_gridPoint3DBuffer.size());
		transformPointsGrid3D2GridEx(m_gridPoint3DBuffer.begin(), (int)m_gridPoint3DBuffer.size(), sizeof(Vector3), m_gridExPointBuffer.begin(), sizeof(Vector3));

		return m_gridExPointBuffer;
	}

	void HadCoordinateConverter::_calcTileRect()
	{
		static const Vector2 corners[4] = { { 0, 0 }, { 0, 16000 }, { 16000, 0 }, { 16000, 16000 } };

		m_tileRectFInGridEx.left = m_tileRectFInGridEx.top = FLT_MAX;
		m_tileRectFInGridEx.right = m_tileRectFInGridEx.bottom = -FLT_MAX;

		Vector3 rectCornerBuffer[4];
		transformPointsGrid2GridEx(corners, element_of(corners), sizeof(Vector2), rectCornerBuffer, sizeof(Vector3));
		Rect_combinePoint(&m_tileRectFInGridEx, &rectCornerBuffer[0]);
		Rect_combinePoint(&m_tileRectFInGridEx, &rectCornerBuffer[1]);
		Rect_combinePoint(&m_tileRectFInGridEx, &rectCornerBuffer[2]);
		Rect_combinePoint(&m_tileRectFInGridEx, &rectCornerBuffer[3]);

		RectF_toRect(&m_tileRectFInGridEx, &m_tileRectInGridEx);
	}

	void HadCoordinateConverter::transformPointsGrid3D2GridEx(
		const Vector3* rawGridPoint3Ds, int n, int inputStride, Vector3* pointsOut, int outputStride)
	{
		// CameraKind_plane was enough
		for (int i = 0; i < n; i++)
		{
			pointsOut->x = rawGridPoint3Ds->x;
			pointsOut->y = rawGridPoint3Ds->y;
			pointsOut->z = rawGridPoint3Ds->z;

			rawGridPoint3Ds = (Vector3*)(((uint8*)rawGridPoint3Ds) + inputStride);
			pointsOut = (Vector3*)(((uint8*)pointsOut) + outputStride);
		}
	}

	void HadCoordinateConverter::transformPointsGrid2GridEx(
		const Vector2* rawGridPoints, int n, int inputStride, Vector3* pointsOut, int outputStride)
	{
		// CameraKind_plane was enough
		// Simply add z=0
		for (int i = 0; i < n; i++)
		{
			pointsOut->x = rawGridPoints->x;
			pointsOut->y = rawGridPoints->y;
			pointsOut->z = 0;

			rawGridPoints = (Vector2*)(((uint8*)rawGridPoints) + inputStride);
			pointsOut = (Vector3*)(((uint8*)pointsOut) + outputStride);
		}
	}
}
