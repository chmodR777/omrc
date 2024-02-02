#include "stdafx.h"
#include "OmrpCommand.h"
#include "boost/filesystem.hpp"
#include "core/loader/db_table_record_iterator.h"
#include "../framework/FileManager.h"
#include "../loader/DbLinkLoader.h"
#include "../loader/DbLaneLoader.h"
#include "../loader/DbRoadBoundaryLoader.h"
#include "../loader/DbLaneMarkLoader.h"
#include "../loader/DbObjectLoader.h"
#include "spatialite.h"
#include "../loader/DbIntersectionLoader.h"
#include "../loader/DbTollGateLoader.h"
#include "cq_types_basic.h"
#include <queue>
#include "../record/MeshWriter.h"
#include "../record/DbMesh.h"
#include <fstream>
#include "tool_kit/call_stack_walker.h"

namespace OMDB
{
    OmrpCommand::OmrpCommand()
    {
    }

	OmrpCommand::OmrpCommand(const OmrpCommand& command)
	{
		command;
	}

	OmrpCommand::~OmrpCommand()
    {

    }

	const cqWCHAR* OmrpCommand::name()
	{
		return L"omrp";
	}

	const cqWCHAR* OmrpCommand::shortDescription()
	{
		return L"render preprocess one map data";
	}

	void OmrpCommand::printHelp()
	{
		printf(
			R"(Synopsis:
			  %s compile [OPTIONS]
			Options:
			  --source-dir <DIRECTORY>
				  Specify the directory which contains source data.
				  It will override the `sourceDir` setting in omdb.ini
			  --output-dir <DIRECTORY>
				  Specify the directory of output.
				  It will override the `outputDir` setting in omdb.ini
			  --thread-number <thread number>
				  Specify the concurrency thread number.
				  It will override the `threadNumber` setting in omdb.ini
			)",
			EXE_NAME);
	}

	bool OmrpCommand::parseArgs(ArgParser* parser)
	{
		ToString(parser->getArg(L"source-dir"), m_options.sourceDir);
		ToString(parser->getArg(L"output-dir"), m_options.outputDir);

		const cqWCHAR* s = parser->getArg(L"thread-number");
		if (s != nullptr)
			m_options.threadNumber = cq_wtoi(s);

		return !parser->printUnknownArgs();
	}
	int OmrpCommand::exec()
	{
		CompileSetting* pSetting = CompileSetting::instance();
		beginTitle("%s Version: %s", EXE_NAME, g_exeVersion);

		IniFile iniFile;
		beginSubtitle("Loading omrp.ini");
		if (!iniFile.load(L"omrp.ini")) {
			printError("Failed to open `omrp.ini`.");
			return false;
		}
		if (!pSetting->load(&iniFile))
			return false;
		endSubtitle();

		if (!pSetting->applyCompilerOptionsAndCheck(&m_options)) {
			return false;
		}
		pSetting->printSettings();
		FileManager::instance()->initialize();

		std::vector<uint32> ids = FileManager::instance()->idList();
		size_t capacity = 1000;
		if (pSetting->capacityNumber > 0)
			capacity = pSetting->capacityNumber;
		size_t size = ids.size();
		size_t number = size / capacity + (size % capacity == 0 ? 0 : 1);

		std::sort(ids.begin(), ids.end());
		// 初始化线程
		unsigned int threadNumber = std::thread::hardware_concurrency();
		if (pSetting->threadNumber > 0)
			threadNumber = min(pSetting->threadNumber, threadNumber);

		//sqlite3多线程支持问题，暂时采用单线程加载
		threadNumber = 1;

		std::string directory = pSetting->outputDir;
		if (!boost::filesystem::exists(directory))
		{
			boost::filesystem::create_directory(directory);
		}

		printInfo("----------------------------------------------------");

		// 输出数据
		std::string path(pSetting->outputDir + "\\china_mesh.omrp");
		if (boost::filesystem::exists(path))
		{
			boost::filesystem::remove(path);
		}

		printInfo("开始预处理，网格数量：%d", ids.size());
		auto begin = std::chrono::steady_clock::now();
		int progress = 0;
		for (int i = 0; i < number; i++)
		{
			std::vector<uint32> v;
			size_t start = i * capacity;
			size_t end = (i + 1) * capacity;
			if (end < size)
			{
				v.assign(ids.begin() + start, ids.begin() + end);
			}
			else
			{
				v.assign(ids.begin() + start, ids.end());
			}
			std::map<uint32, MeshBlob> data;
			std::queue <uint32 > tasks;
			for_each(v.begin(), v.end(), [&](uint32 id)->void {tasks.push(id); data[id] = MeshBlob(); });
			// 定义load线程函数
			Spinlock spinlock;
			auto threadFun = [&] {
				while (true)
				{
					uint32 id;
					try
					{
						{
							std::lock_guard<Spinlock> lock(spinlock);
							if (tasks.empty())
								break;

							int npg = (int)((v.size() - tasks.size() + i * capacity) * 1.0 / size * 100.0);
							if (npg > progress)
							{
								printInfo("[%d%%] pre-process omdb.", npg);
								progress = npg;
							}
							id = tasks.front();
							tasks.pop();
						}

						DbMesh* pMesh = new DbMesh();
						load(id, pMesh);
						if (pSetting->isGenerateHdData ||
							pMesh->query(RecordType::DB_HAD_LANE_MARK_LINK).size() > 0 ||
							pMesh->query(RecordType::DB_HAD_ROAD_BOUNDARY_LINK).size() > 0)
						{
							MeshWriter writer;
							writer.write(pMesh, data[id].data, data[id].size);
						}
						else
						{
							data.erase(id);
						}
						delete pMesh;
					}
					catch (CallStackWalker& walker)
					{
						walker.showCallstack();
						walker.showExceptionInfo();
					}
					catch (std::exception& ex)
					{
						printError("Catch C++ exception : %s", ex.what());
						printError("mesh_id: %d", id);
					}
				}
			};

			std::vector<std::thread> workers;
			workers.resize(threadNumber);

			for (unsigned int i = 0; i < threadNumber; i++)
			{
				workers[i] = std::move(std::thread(threadFun));
			}
			for_each(workers.begin(), workers.end(), [](std::thread& thread)->void {
				if (thread.joinable())
				{
					thread.join();
				}
				});
			writeDb(path.c_str(), data);
		}

		printInfo("[%d%%] pre-process omdb.", 100);
		printInfo("输出文件地址：%s", path.c_str());
		printInfo("预处理数据结束!");
		double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
		printInfo("总耗时：%f", seconds);
		return 0;
	}

	OmrpCommand OmrpCommand::g_sCommand;
	void OmrpCommand::load(uint32 id, DbMesh* pMesh)
	{
		NcString* path = FileManager::instance()->queryPath(id);
		if (path == nullptr)
			return;

		pMesh->setId(id);
		sqlite3* pSqlite3;
		std::vector<char> utf8PathBuf;
		utf8PathBuf.resize(path->convertToUtf8CString(nullptr, 0));
		path->convertToUtf8CString(utf8PathBuf.data(), (int)utf8PathBuf.size());
		if (SQLITE_OK != sqlite3_open_v2(utf8PathBuf.data(), &pSqlite3, SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX | SQLITE_OPEN_SHAREDCACHE, nullptr))
		{
			sqlite3_close(pSqlite3);
		}
		hadc::SqliteDbGuard dbGuard(pSqlite3);
		void* pConnection = spatialite_alloc_connection();
		spatialite_init_ex(pSqlite3, pConnection, 0);
		{
			if (!DbLinkLoader().checkLaneBoundary(pSqlite3))
				return;
			DbLinkLoader().Load(pSqlite3, pMesh);
			DbLaneLoader().Load(pSqlite3, pMesh);
			DbRoadBoundaryLoader().Load(pSqlite3, pMesh);
			DbLaneMarkLoader().Load(pSqlite3, pMesh);
			DbObjectLoader().Load(pSqlite3, pMesh);
			DbIntersectionLoader().Load(pSqlite3, pMesh);
			DbTollGateLoader().Load(pSqlite3, pMesh);
		}
		spatialite_cleanup_ex(pConnection);
		spatialite_shutdown();
		return;
	}
	void OmrpCommand::writeDb(const char* path, const std::map<uint32, MeshBlob>& data)
	{
		std::ifstream f(path);
		f.open(path, std::ios::_Nocreate);
		sqlite3* pDb = nullptr;
		int status = sqlite3_open_v2(path, &pDb, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, NULL);
		if (status != SQLITE_OK)
		{
			sqlite3_close(pDb);
			return;
		}
		sqlite3_exec(pDb, "PRAGMA synchronous = OFF", 0, 0, 0);

		const char* sql = "CREATE TABLE IF NOT EXISTS mesh ("
			"id				  INTEGER                 NOT NULL,"
			"district		  TEXT,"
			"data			  BLOB);";

		char* msg;
		status = sqlite3_exec(pDb, sql, NULL, 0, &msg);
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
		sql = "INSERT INTO mesh (id, data,district) VALUES (?,?,?)";
		status = sqlite3_prepare_v2(pDb, sql, strlen(sql), &stmt, NULL);
		if (status != SQLITE_OK)
			return;

		for (auto& pair : data)
		{
			sqlite3_reset(stmt);
			NcString* path = FileManager::instance()->queryPath(pair.first);
			int pos = nc_max(path->locationOfCharacterBackwards(L'/'), path->locationOfCharacterBackwards(L'\\'));

			NcString* sub = path->substringTo(pos);
			pos = nc_max(sub->locationOfCharacterBackwards(L'/'), sub->locationOfCharacterBackwards(L'\\'));

			NcString* directoryName = sub->substringFrom(pos+1);
			sqlite3_clear_bindings(stmt);
			sqlite3_bind_int64(stmt, 1, pair.first);
			sqlite3_bind_blob(stmt, 2, pair.second.data, pair.second.size, nullptr);

			char cs[255];
			WideCharToMultiByte(CP_ACP, 0, directoryName->cstr(), -1,cs, sizeof(cs), NULL, NULL);

			int length = strlen(cs);
			sqlite3_bind_text(stmt, 3,cs,length+1,nullptr);

			status = sqlite3_step(stmt);

			if (pair.second.data != nullptr)
			{
				free(pair.second.data);
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
		}

		sqlite3_close(pDb);

	}


	OmrpCommand* OmrpCommand::instance()
	{
		return &g_sCommand;
	}
}
