#include "stdafx.h"
#include "RdsDatabaseWriter.h"
#include "RdsWriterFactory.h"
#include <fstream>
#include "cq_stdlib.h"
#include "util/byte_stream_writer.h"
#include <algorithm>
#include <assert.h>
#include <thread>
#include <mutex>
#include <queue>
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "../CompileSetting.h"
#include "../framework/Spinlock.h"
#include "MapboxWriter.h"

namespace OMDB
{
	std::string RdsDatabaseWriter::base64EncodedGridList(std::vector<uint32>& gridIdList)
	{
		std::sort(gridIdList.begin(), gridIdList.end());

		struct NdsGridIdRange
		{
			uint32 startId;
			uint32 count;
		} range;
		cqstd::vector<NdsGridIdRange> rangeList;

		for (size_t i = 0; i < gridIdList.size(); i++)
		{
			if (i == 0)
			{
				range.startId = gridIdList.front();
				range.count = 1;
				rangeList.push_back(range);
				continue;
			}
			NdsGridIdRange& lastRange = rangeList.back();
			CQ_ASSERT(gridIdList[i] >= lastRange.startId + lastRange.count);

			if (gridIdList[i] == lastRange.startId + lastRange.count)
			{
				lastRange.count++;
			}
			else
			{
				range.startId = gridIdList[i];
				range.count = 1;
				rangeList.push_back(range);
			}
		}

		uint32 rangeCount = (uint32)rangeList.size();
		ByteStreamWriter writer;
		writer.writeVarUint32(rangeCount);
		writer.writeUint32(rangeList[0].startId);
		writer.writeVarUint32(rangeList[0].count);
		for (uint32 i = 1; i < rangeCount; i++)
		{
			writer.writeVarUint32(rangeList[i].startId - rangeList[i - 1].startId - rangeList[i - 1].count);
			writer.writeVarUint32(rangeList[i].count);
		}
		printInfo("Binary GridList data size: %u bytes.", writer.lengthInBytes());

		Base64Env env;
		Base64Env_construct(&env);
		size_t encodedDataSize = Base64Env_encode(&env, writer.bytes(), writer.lengthInBytes(), NULL, 0);
		if (encodedDataSize == SIZE_T_MAX)
		{
			return std::string();
		}

		char* encodedData = (char*)malloc(encodedDataSize);
		Base64Env_encode(&env, writer.bytes(), writer.lengthInBytes(), encodedData, encodedDataSize);
		std::string encodedGridListStr = (const char*)encodedData;

		free(encodedData);
		return encodedGridListStr;
	}

	void RdsDatabaseWriter::writeBinary(const char* path, RDS::RdsDatabase* pDatabase)
	{
		std::ifstream f(path);
		if (!f.good())
		{
			remove(path);
		}

		f.open(path, std::ios::_Nocreate);

		sqlite3* pDb = nullptr;
		int status = sqlite3_open_v2(path, &pDb, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, NULL);
		if (status != SQLITE_OK)
		{
			sqlite3_close(pDb);
			return;
		}
		sqlite3_exec(pDb, "PRAGMA synchronous = OFF", 0, 0, 0);

		writeBinaryHeader(pDb, pDatabase);
		writeBinaryMesh(pDb, pDatabase);

		sqlite3_close(pDb);
	}

	void RdsDatabaseWriter::writeSpatialite(const std::string& directory, RDS::RdsDatabase* pDatabase)
	{
		if (!boost::filesystem::exists(directory))
		{
			boost::filesystem::create_directory(directory);
		}
		// 初始化任务队列
		auto tiles = pDatabase->getTiles();
		std::queue < RDS::RdsTile* > tasks;
		for_each(tiles.begin(), tiles.end(), [&](RDS::RdsTile* pTile)->void {tasks.push(pTile); });
		std::vector<std::string> mvtFiles;

		// 定义线程函数
// 		std::mutex mutex;
		Spinlock mutex;
		auto process = [&] {
			while (true)
			{
				RDS::RdsTile* pTile = nullptr;
				{
					std::lock_guard<Spinlock> lock(mutex);
					if (tasks.empty())
						break;

					pTile = tasks.front();
					tasks.pop();
				}
				if (pTile != nullptr)
				{
					boost::filesystem::path output(directory + "\\" + std::to_string(MeshId_toNdsGridId(pTile->meshId)) + ".rds");
					if (boost::filesystem::exists(output))
					{
						boost::filesystem::remove(output);
					}
					RDS::Writer* pWriter = RDS::RdsWriterFactory::instance()->createWriter(RDS::FormatType::SPATIALITE);
					pWriter->Write(output.string().c_str(), pTile);
					delete pTile;
				}
			}

		};

		pDatabase->getTiles().clear();

		// 初始化线程
		CompileSetting* pSetting = CompileSetting::instance();
		unsigned int threadNumber = std::thread::hardware_concurrency();
		if (pSetting->threadNumber > 0)
		{
			threadNumber = min(threadNumber, pSetting->threadNumber);
		}
		// 		printInfo("threadNumber = %d", threadNumber);

		auto start = std::chrono::steady_clock::now();
		std::vector<std::thread> workers;
		workers.resize(threadNumber);
		for (unsigned int i = 0; i < threadNumber; i++)
		{
			workers[i] = std::move(std::thread(process));
		}

		for_each(workers.begin(), workers.end(), [](std::thread& thread)->void {
			if (thread.joinable())
			{
				thread.join();
			}
			});

		double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
		printInfo("写出Spatialite耗时：%f",seconds);
	}

	void RdsDatabaseWriter::writeBinaryHeader(sqlite3* pDb, RDS::RdsDatabase* pDatabase)
	{
		std::vector<RDS::RdsTile*> tiles = pDatabase->getTiles();
		std::vector<uint32> gridIdList;
		for_each(tiles.begin(), tiles.end(), [&](RDS::RdsTile* pTile)->void {gridIdList.push_back(MeshId_toNdsGridId(pTile->meshId)); });
		std::string encodedGridList = base64EncodedGridList(gridIdList);
		boost::posix_time::ptime currentTime = boost::posix_time::second_clock::local_time();
		std::string currentTimeStr = boost::posix_time::to_iso_extended_string(currentTime);
		std::map<std::string, std::string> headers =
		{
			{"compilerVersion","1.0"},
			{"compileTime", currentTimeStr},
			{"formatType","binary"},
			{"formatVersion","2.1.0"},
			{"dataVersion", CompileSetting::instance()->dataVersion},
			{"gridShifted","1"},
			{"gridList",encodedGridList}
		};
		const char* sql = "CREATE TABLE __Had_meta__ ("
			"key				  TEXT                 NOT NULL,"
			"value			  TEXT);";

		char* msg;
		int status = sqlite3_exec(pDb, sql, NULL, 0, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "BEGIN";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}
		sqlite3_stmt* stmt;
		sql = "INSERT INTO __Had_meta__ (key, value) VALUES (?,?)";
		status = sqlite3_prepare_v2(pDb, sql, strlen(sql), &stmt, NULL);
		if (status != SQLITE_OK)
			return;

		for (auto& pair : headers)
		{
			sqlite3_reset(stmt);
			sqlite3_clear_bindings(stmt);
			sqlite3_bind_text(stmt, 1, pair.first.c_str(), pair.first.length(), nullptr);
			sqlite3_bind_text(stmt, 2, pair.second.c_str(),pair.second.length(), nullptr);
			status = sqlite3_step(stmt);
		}

		sqlite3_finalize(stmt);
		sql = "COMMIT";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}
		
	}

	void RdsDatabaseWriter::writeBinaryMesh(sqlite3* pDb, RDS::RdsDatabase* pDatabase)
	{
		RDS::Writer* pWriter = RDS::RdsWriterFactory::instance()->createWriter(RDS::FormatType::BINARY);
		const char* sql = "CREATE TABLE Had ("
			"id				  INTEGER                 NOT NULL,"
			"data			  BLOB);";

		char* msg;
		int status = sqlite3_exec(pDb, sql, NULL, 0, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "BEGIN";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}
		sqlite3_stmt* stmt;
		sql = "INSERT INTO Had (id, data) VALUES (?,?)";
		status = sqlite3_prepare_v2(pDb, sql, strlen(sql), &stmt, NULL);
		if (status != SQLITE_OK)
			return;

// 		auto start0 = std::chrono::steady_clock::now();
		for (auto& pTile : pDatabase->getTiles())
		{
			void* pData = nullptr;
			unsigned long long size;
// 			auto start = std::chrono::steady_clock::now();
			pWriter->Write(pTile, pData,size);
// 			double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
// 			printInfo("写出二进制耗时：%f", seconds);
			if (pData == nullptr)
				continue;
			
			sqlite3_reset(stmt);
			sqlite3_clear_bindings(stmt);
			sqlite3_bind_int64(stmt, 1, MeshId_toNdsGridId(pTile->meshId));
			sqlite3_bind_blob(stmt, 2, pData, size, nullptr);
			status = sqlite3_step(stmt);

			if (pData != nullptr)
			{
				free(pData);
			}
			if (status != SQLITE_DONE && status != SQLITE_ROW)
			{
				break;
			}
		}
		sqlite3_finalize(stmt);
		sql = "COMMIT";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}
// 		double seconds0 = std::chrono::duration<double>(std::chrono::steady_clock::now() - start0).count();
// 		printInfo("总耗时：%f", seconds0);
	}

}