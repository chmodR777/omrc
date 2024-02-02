#pragma once

#if !defined(_CRT_SECURE_NO_WARNINGS)
#	define _CRT_SECURE_NO_WARNINGS
#endif

#include <clocale>

#include "windows.h"

#include "gtest/gtest.h"

#include "nc_string.h"
#include "cqstl/hashmap.h"

#include "c_algorithm_dec.h"
#include "c_algorithm_def.h"

#include "dalr/dalr.h"

#include "util/byte_stream_writer.h"
#include "util/util.h"
#include "util/batched_allocator.h"
#include "util/ini_file.h"
#include "util/time_span.h"

#include "cpputil/nds_db.h"
#include "cpputil/arg_parser.h"
#include "cpputil/cli_command_center.h"
#include "cpputil/console_util.h"
#include "cpputil/cq_algorithm.h"
#include "cpputil/midmif_reader.h"
#include "cpputil/polyline_tile_splitter.h"
#include "tool_kit/safe_batched_allocator.h"
#include "geometry/map_point3d64.h"
#define EXE_NAME "omrc"


static const char* changelog =
R"(
* 0.1.0    2018-03-06 Initial draft version

)";
static const char* g_changelog =
R"(
* 1.0.0    2023-6-9
             * Support omdb V1.5
)";

/*
	@note
		�������İ汾���ڿ���������ʹ�� L"0.{1}.{2}" �������ʽ�汾��Ϊ L"0.{1}.0" �����ݡ�
		��ʽ�汾����ʱ���汾�ű�Ϊ L"1.0.0"��
*/
extern const cqCHAR* g_exeVersion;

/**
	@brief �ļ���ʽ�汾��
	@details
		��ʽ��ƿ��������еİ汾��ʹ�� L"0.?.0" ����ʽ��
		����ʽ��ʽ����������Ʒ��������ʱ���˰汾����Ҫ��Ϊ L"1.0.0"��
		�����ʽ���ò������������Ե��޸�(�������⣡)������Ҫ�����˰汾�š�
*/
const cqWCHAR FILE_FORMAT_VERSION[] = L"2.1.0";

forceinline uint32 NdsGridId_toMeshId(NdsGridId gridId) { return (uint32)gridId & (~0x20000000); }
forceinline NdsGridId MeshId_toNdsGridId(uint32 meshId) { return NdsGridId(0x20000000 | meshId); }
