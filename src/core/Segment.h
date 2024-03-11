#pragma once
#include "cq_types_basic.h"

namespace OMDB
{
#define INVALID_INDEX -1

/**
ע��,�������ﶨ���SegmentId����·rp�ļ���SegmentId��ȫû��ϵ,
�˴����Ƕ������SegmentId��Ϊ�����������ֿܷ�link direct����1ʱ����ͷ�������,
��·rp�ļ���SegmentId�ǿ�������link����֮��˳��������Id
*/
typedef uint64 SegmentId;	///< ����·�ε� ID
#define INVALID_SEGMENT_ID	(SegmentId)(-1)

typedef uint64 DSegmentId;	///< ����·�ε� ID
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

/**	@brief ��ȡһ������·�η���������·�ε� ID��
	�����ǵ�ʵ���У�DSegmentId �ĸ� 63 λ�Ƕ�Ӧ·�ε� SegmentId����� 1 λ�Ƿ����־λ��1 ��ʾ����0 ��ʾ����
*/
#define DSegmentId_invert(o) ((o) ^= 1)
}

