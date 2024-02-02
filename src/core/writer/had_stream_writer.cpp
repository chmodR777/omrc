#include "stdafx.h"
#include "had_stream_writer.h"

void HadStreamWriter::writePolyline2D(const MapPoint64* points, size_t pointCount)
{
	size_t xBitWidth = 0;
	size_t yBitWidth = 0;

	for (size_t idx = 0; idx + 1 < pointCount; ++idx)
	{
		size_t bitWidth;

		bitWidth = Util_getBitWidthOfInt((int32)(points[idx + 1].lon - points[idx].lon));
		xBitWidth = cq_max(xBitWidth, bitWidth);
		bitWidth = Util_getBitWidthOfInt((int32)(points[idx + 1].lat - points[idx].lat));
		yBitWidth = cq_max(yBitWidth, bitWidth);
	}

	writeVarUint32((uint32)pointCount);
	if (pointCount == 0)
		return;

	writeUintN64(points[0].lon, 36);
	writeUintN64(points[0].lat, 36);
	if (pointCount == 1)
		return;

	writeUintN((uint32)(xBitWidth - 1), 6);
	writeUintN((uint32)(yBitWidth - 1), 6);
	for (size_t idx = 0; idx + 1 < pointCount; ++idx)
	{
		writeIntN((int32)(points[idx + 1].lon - points[idx].lon), xBitWidth);
		writeIntN((int32)(points[idx + 1].lat - points[idx].lat), yBitWidth);
	}
}

void HadStreamWriter::writeFeatureLocalIndexList(const cqstd::vector<uint16>& indices)
{
	alignToBits(8);
	writeVarUint32((uint32)indices.size());
	for (cqstd::vector<uint16>::iterator it = indices.begin(); it != indices.end(); it++)
	{
		writeUint16(*it);
	}
}

void HadStreamWriter::writeStringAsUtf8(const cqWCHAR* str)
{
	size_t len = cq_encodeUtf8(str, SIZE_T_MAX, NULL, 0);
	m_stringBuffer.resize(len);
	cq_encodeUtf8(str, SIZE_T_MAX, m_stringBuffer.begin(), len);

	if (len <= 1)
	{
		writeVarUint32(0);
	}
	else
	{
		writeVarUint32((uint32)len - 1);
		writeBytes(m_stringBuffer.begin(), len - 1);
	}
}
