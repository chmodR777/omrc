#pragma once

#include "map_point64.h"

class HadStreamWriter : public ByteStreamWriter
{
public:
	void writePolyline2D(const MapPoint64* points, size_t pointCount);

	template<class Point3DArrayType>
	void writePolyline3D(Point3DArrayType points, size_t pointCount)
	{
		size_t xBitWidth = 0;
		size_t yBitWidth = 0;
		size_t zBitWidth = 0;

		for (size_t idx = 0; idx + 1 < pointCount; ++idx)
		{
			size_t bitWidth;

			bitWidth = Util_getBitWidthOfInt((int32)(points[idx + 1].pos.lon - points[idx].pos.lon));
			xBitWidth = cq_max(xBitWidth, bitWidth);
			bitWidth = Util_getBitWidthOfInt((int32)(points[idx + 1].pos.lat - points[idx].pos.lat));
			yBitWidth = cq_max(yBitWidth, bitWidth);
			bitWidth = Util_getBitWidthOfInt((int32)(points[idx + 1].z - points[idx].z));
			zBitWidth = cq_max(zBitWidth, bitWidth);
		}

		writeVarUint32((uint32)pointCount);
		if (pointCount == 0)
			return;

		writeUintN64(points[0].pos.lon, 36);
		writeUintN64(points[0].pos.lat, 36);
		writeVarInt32(points[0].z);
		if (pointCount == 1)
			return;

		writeUintN((uint32)(xBitWidth - 1), 6);
		writeUintN((uint32)(yBitWidth - 1), 6);
		writeUintN((uint32)(zBitWidth - 1), 5);
		for (size_t idx = 0; idx + 1 < pointCount; ++idx)
		{
			writeIntN((int32)(points[idx + 1].pos.lon - points[idx].pos.lon), xBitWidth);
			writeIntN((int32)(points[idx + 1].pos.lat - points[idx].pos.lat), yBitWidth);
			writeIntN((int32)(points[idx + 1].z - points[idx].z), zBitWidth);
		}
	}

	void writeFeatureLocalIndexList(const cqstd::vector<uint16>& indices);

	void writeStringAsUtf8(const cqWCHAR* str);

private:
	cqstd::vector<cqCHAR> m_stringBuffer;
};
