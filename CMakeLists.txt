cmake_minimum_required(VERSION 2.6)
set (PROJECT_NAME "hadc")
project(${PROJECT_NAME})

set (BASE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set (EXECUTABLE_OUTPUT_PATH "${BASE_PATH}/lib")

MACRO(USE_PCH_CPP pch)
	IF(MSVC)
		GET_FILENAME_COMPONENT(pchPath ${pch} PATH)
		GET_FILENAME_COMPONENT(pchName ${pch} NAME_WE)
		SET(pchSourceName "${pchPath}/${pchName}.cpp")
		#MESSAGE(${pchSourceName})
		FILE (GLOB filesUsePch "${pchPath}/*.cpp")
		LIST (REMOVE_ITEM filesUsePch ${pchSourceName})
		set_source_files_properties(${pchSourceName} PROPERTIES COMPILE_FLAGS "/Yc${pchName}.h")
		set_source_files_properties(${filesUsePch} PROPERTIES COMPILE_FLAGS "/Yu${pchName}.h") 
	ENDIF(MSVC)
ENDMACRO(USE_PCH_CPP)

IF (${CMAKE_GENERATOR} MATCHES ".*Visual Studio.*")
	SET (CMAKE_CXX_FLAGS_DEBUG "/MT /O2 /EHa /Zi")
	SET (CMAKE_CXX_FLAGS_RELEASE "/MT /O2 /EHa /Zi")
	SET (CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /DEBUG /OPT:REF /OPT:ICF")

	add_definitions(
		-DAMD64
		-DUNICODE
		-D_UNICODE
		-D_SILENCE_TR1_NAMESPACE_DEPRECATION_WARNING
	)
ENDIF ()

include_directories(
	${BASE_PATH}/../navicore-x64-std_debug_2019/include
	${BASE_PATH}/../navicore-x64-std_debug_2019/include/protobuf
	${BASE_PATH}/../gtest/include
	${BASE_PATH}/../cpputil/include
	${BASE_PATH}/../navi_rds/include
	${BASE_PATH}/../sqlite3/include
	${BASE_PATH}/src
	${BASE_PATH}/3rd-party
	${BASE_PATH}/3rd-party/gsl/include
	${BASE_PATH}/3rd-party/boost/include
	${BASE_PATH}/3rd-party/libspatialite/include

)

link_directories(
	${BASE_PATH}/../sqlite3/lib
	${BASE_PATH}/../navi_rds/lib
	${BASE_PATH}/../navicore-x64-std_debug_2019/lib/Debug
	${BASE_PATH}/3rd-party/gsl/lib
	${BASE_PATH}/3rd-party/boost/lib
	${BASE_PATH}/3rd-party/libspatialite/lib
)


file(GLOB_RECURSE SRC "src/*.cpp" "../gtest/src/*.cc" 
	"../cpputil/src/arg_parser.cpp"
	"../cpputil/src/console_util.cpp"
	"../cpputil/src/cq_string.cpp"
	"../cpputil/src/file_util.cpp"
	"../cpputil/src/line_parser.cpp"
	"../cpputil/src/csv_field_parser.cpp"
	"../cpputil/src/midmif_reader.cpp"
	"../cpputil/src/csv_reader.cpp"
	"../cpputil/src/csv_field_parser.cpp"
	"../cpputil/src/polyline_tile_splitter.cpp"
	"../cpputil/src/process_util.cpp"
	"../cpputil/src/ini.cpp"
	"../cpputil/src/cli_command_center.cpp"
	"../cpputil/src/nds_db.cpp"
	"../cpputil/src/nds_cache.cpp"
	"../cpputil/src/concurrent_task_runner.cpp"
	)

USE_PCH_CPP(${CMAKE_CURRENT_SOURCE_DIR}/src/stdafx.h)
add_executable(${PROJECT_NAME} ${SRC})
target_link_libraries(${PROJECT_NAME}
	${BASE_PATH}/3rd-party/libspatialite/lib/spatialite_x64.lib
	${BASE_PATH}/3rd-party/boost/lib/libboost_filesystem-vc142-mt-s-x64-1_81.lib
)

