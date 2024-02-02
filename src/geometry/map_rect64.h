#pragma once

#include "map_point64.h"

/*
	高精度经纬度矩形，单位：1/10^8 度
*/
struct MapRect64
{
	int64 left, top, right, bottom;

	void setAsNegtiveMinimum()
	{
		left = top = INT64_MAX;
		right = bottom = INT64_MIN;
	}

	bool isValid() { return left <= right && top <= bottom; }

	MapPoint64 center()
	{
		return MapPoint64::make(left + (right - left) / 2, top + (bottom - top) / 2);
	}

	void combinePoint(MapPoint64 pt)
	{
		if (left > pt.lon)
			left = pt.lon;
		if (right < pt.lon)
			right = pt.lon;
		if (top > pt.lat)
			top = pt.lat;
		if (bottom < pt.lat)
			bottom = pt.lat;
	}

	void fromNdsRect(NdsRect ndsRect)
	{
		NdsPoint leftTop = NdsPoint_make(ndsRect.left, ndsRect.top);
		NdsPoint rightBottom = NdsPoint_make(ndsRect.right, ndsRect.bottom);
		MapPoint64 hadLeftTop;
		hadLeftTop.fromNdsPoint(leftTop);
		MapPoint64 hadRightBottom;
		hadRightBottom.fromNdsPoint(rightBottom);
		left = hadLeftTop.lon;
		top = hadLeftTop.lat;
		right = hadRightBottom.lon;
		bottom = hadRightBottom.lat;
	}

	NdsRect toNdsRect() const
	{
		MapPoint64 leftTop = MapPoint64_make(left, top);
		MapPoint64 rightBottom = MapPoint64_make(right, bottom);
		NdsPoint ndsLeftTop = leftTop.toNdsPoint();
		NdsPoint ndsRightBottom = rightBottom.toNdsPoint();
		NdsRect ndsRect;
		NdsRect_set(&ndsRect, ndsLeftTop.x, ndsLeftTop.y, ndsRightBottom.x, ndsRightBottom.y);
		return ndsRect;
	}

	void fromRect(Rect rect)
	{
		left = rect.left * 1000;
		top = rect.top * 1000;
		right = rect.right * 1000;
		bottom = rect.bottom * 1000;
	}

	Rect toRect() const
	{
		Rect rct = Rect_make(
			(int)(left / 1000),
			(int)(top / 1000),
			(int)((right + 500) / 1000),
			(int)((bottom + 500) / 1000));
		return rct;
	}

	forceinline void expand(int64 expandSize) { left -= expandSize; top -= expandSize; right += expandSize; bottom += expandSize; }
	forceinline bool PointInRect(MapPoint64 r) { return left < r.lon&& top < r.lat&& right > r.lon&& bottom > r.lat; }
	forceinline bool intersectsWithRect(MapRect64 r) { return left < r.right && top < r.bottom && right > r.left && bottom > r.top; }
};

forceinline MapRect64 MapRect64_make(int64 left, int64 top, int64 right, int64 bottom)
{
	MapRect64 rct;
	rct.left = left;
	rct.top = top;
	rct.right = right;
	rct.bottom = bottom;
	return rct;
}
