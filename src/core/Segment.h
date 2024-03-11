#pragma once
#include "cq_types_basic.h"

namespace OMDB
{
#define INVALID_INDEX -1

/**
注意,我们这里定义的SegmentId和算路rp文件的SegmentId完全没关系,
此处我们定义这个SegmentId是为了拓扑生成能分开link direct等于1时正向和反向拓扑,
算路rp文件的SegmentId是库中所有link排序之后顺序生产的Id
*/
typedef uint64 SegmentId;	///< 无向路段的 ID
#define INVALID_SEGMENT_ID	(SegmentId)(-1)

typedef uint64 DSegmentId;	///< 有向路段的 ID
#define INVALID_DSEGMENT_ID	(DSegmentId)(-1)

#define DSegmentId_setSegmentId(o, seg) ((o) = ((o) & 0x01 | (seg) << 1))
#define DSegmentId_setDir(o, dir) ((o) = (((o) & (DSegmentId)(-2)) | ((dir) ? 1 : 0)))
#define DSegmentId_set(o, segId, dir) ((o) = (segId) << 1 | ((dir) ? 1 : 0))
#define DSegmentId_invert(o) ((o) ^= 1)

/**
	@brief Get a reversed DSegmentId.
	@param [in] dsegId
		The original DSegmentId.
	@return
		A reversed DSegmentId of dsegId.
		dsegId should be reversible, or the return value will be infeasible.
*/
#define DSegmentId_getReversed(o) ((o) ^ 1)

/**
	@brief Get the corresponding SegmentId of a DSegmentId
	@param [in] dsegId
		The DSegmentId.
	@return
		The SegmentId correspond to dsegId.
*/
#define DSegmentId_getSegmentId(o) ((o) >> 1)

/**
	@brief Get the corresponding DSegmentId of a DbLink
	@param [in] DbLink uuid
		The uuid.
	@return
		The DSegmentId correspond to DbLink.
*/
forceinline DSegmentId DSegmentId_getDSegmentId(int64 uuid) {
	DSegmentId dSegmentId = 0x01;
	DSegmentId_setSegmentId(dSegmentId, uuid);
	return dSegmentId;
}

/**
	@brief Get the direction of a DSegmentId
	@param [in] dsegId
		The DSegmentId
	@return
		TRUE:  From Node1 to Node2 of the corresponding segment.
		FALSE: From Node2 to Node1 of the corresponding segment.
*/
#define DSegmentId_getDir(o) ((o) & 0x01)

/**	@brief 获取一条有向路段反向后的有向路段的 ID。
	在我们的实现中，DSegmentId 的高 63 位是对应路段的 SegmentId，最低 1 位是方向标志位，1 表示正向，0 表示反向。
*/
#define DSegmentId_invert(o) ((o) ^= 1)
}

