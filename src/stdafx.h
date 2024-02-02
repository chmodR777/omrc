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
		编译器的版本号在开发过程中使用 L"0.{1}.{2}" 编译出格式版本号为 L"0.{1}.0" 的数据。
		正式版本就绪时，版本号变为 L"1.0.0"。
*/
extern const cqCHAR* g_exeVersion;

/**
	@brief 文件格式版本号
	@details
		格式设计开发过程中的版本号使用 L"0.?.0" 的形式。
		当格式正式定下来，产品即将上线时，此版本号需要改为 L"1.0.0"。
		如果格式不得不发生不兼容性的修改(尽量避免！)，则需要升级此版本号。
*/
const cqWCHAR FILE_FORMAT_VERSION[] = L"2.1.0";

forceinline uint32 NdsGridId_toMeshId(NdsGridId gridId) { return (uint32)gridId & (~0x20000000); }
forceinline NdsGridId MeshId_toNdsGridId(uint32 meshId) { return NdsGridId(0x20000000 | meshId); }
