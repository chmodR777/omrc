#include "stdafx.h"
#include "OmrpTestCommand.h"
#include "boost/filesystem.hpp"
#include "spatialite.h"
#include "core/loader/db_table_record_iterator.h"
#include "../framework/FileManager.h"
#include "../loader/DbLinkLoader.h"
#include "../loader/DbLaneLoader.h"
#include "../loader/DbRoadBoundaryLoader.h"
#include "../loader/DbLaneMarkLoader.h"
#include "../loader/DbObjectLoader.h"
#include "../loader/DbIntersectionLoader.h"
#include "../loader/DbTollGateLoader.h"
#include "cq_types_basic.h"
#include "../record/MeshReader.h"
#include "../record/DbMesh.h"
#include <fstream>
#include "tool_kit/call_stack_walker.h"
#include <queue>
#include "../record/DbRecord.h"

namespace OMDB
{
    OmrpTestCommand::OmrpTestCommand()
    {
    }

	OmrpTestCommand::OmrpTestCommand(const OmrpTestCommand& command)
	{
		command;
	}

	OmrpTestCommand::~OmrpTestCommand()
    {

    }

	const cqWCHAR* OmrpTestCommand::name()
	{
		return L"omrpTest";
	}

	const cqWCHAR* OmrpTestCommand::shortDescription()
	{
		return L"render preprocess one map data";
	}

	void OmrpTestCommand::printHelp()
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

	bool OmrpTestCommand::parseArgs(ArgParser* parser)
	{
		ToString(parser->getArg(L"source-dir"), m_options.sourceDir);
		ToString(parser->getArg(L"output-dir"), m_options.outputDir);

		const cqWCHAR* s = parser->getArg(L"thread-number");
		if (s != nullptr)
			m_options.threadNumber = cq_wtoi(s);

		return !parser->printUnknownArgs();
	}
	int OmrpTestCommand::exec()
	{
		CompileSetting* pSetting = CompileSetting::instance();
		beginTitle("%s Version: %s", EXE_NAME, g_exeVersion);

		IniFile iniFile;
		beginSubtitle("Loading omdb.ini");
		if (!iniFile.load(L"omdb.ini")) {
			printError("Failed to open `omdb.ini`.");
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
		std::map<uint32, DbMesh*> meshes01;
		loadOmdb(ids, meshes01);
		std::map<uint32, DbMesh*> meshes02;
		loadOmrp(ids, meshes02);
		for (auto iter = meshes01.begin();iter != meshes01.end();iter++)
		{
			DbMesh* pFirst = iter->second;
			if (meshes02.find(pFirst->getId()) == meshes02.end())
			{
				printError("omrp不存在网格：%d",pFirst->getId());
			}
			else
			{
				DbMesh* pSecond = meshes02[pFirst->getId()];
// 				if (pSecond->memoryUsage() != pFirst->memoryUsage())
				{
					printError("omrp数据错误：%d", pFirst->getId());
					{
// 						for (auto pRecord : pSecond->query(RecordType::DB_HAD_LINK))
// 						{
// 							DbLink* pLinkFirst = (DbLink*)pRecord;
// 							DbLink* pLinkSecond = (DbLink*)pFirst->query(pLinkFirst->uuid,RecordType::DB_HAD_LINK);
// 							if (!equalLink(pLinkFirst, pLinkSecond))
// 							{
// 								printError("omrp数据错误：meshid = %d,Linkid =%ld", pFirst->getId(),pLinkFirst->uuid);
// 							}
// 						}

// 						for (auto& pair : pFirst->m_mapBatchedAllocator)
// 						{
// 							if (pair.second.memoryUsage() != pSecond->m_mapBatchedAllocator[pair.first].memoryUsage())
// 							{
// 								printError("omrp数据错误：meshid = %d,recordType =%d", pFirst->getId(), pair.first);
// 							}
// 						}
					}
				}
			}
		}

		for (auto& pair : meshes01)
		{
			delete pair.second;
		}

		for (auto& pair : meshes02)
		{
			delete pair.second;
		}

		return 0;
	}

	OmrpTestCommand OmrpTestCommand::g_sCommand;
	void OmrpTestCommand::loadOmdb(const std::vector<uint32>& ids, std::map<uint32, DbMesh*>& meshes)
	{
		// 初始化线程
		CompileSetting* pSetting = CompileSetting::instance();
		unsigned int threadNumber = std::thread::hardware_concurrency();
		if (pSetting->threadNumber > 0)
			threadNumber = min(pSetting->threadNumber, threadNumber);
		std::queue <uint32 > tasks;
		for_each(ids.begin(), ids.end(), [&](uint32 id)->void {tasks.push(id); });
		// 定义load线程函数
		Spinlock spinlock;
		auto threadFun = [&] {
			while (true)
			{
				uint32 id;
				{
					{
						std::lock_guard<Spinlock> lock(spinlock);
						if (tasks.empty())
							break;
						id = tasks.front();
						tasks.pop();
					}
					DbMesh* pMesh = new DbMesh();
					loadOmdb(id, pMesh);
					meshes[id] = pMesh;
				}
			}
		};

		// load
		auto start = std::chrono::steady_clock::now();
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
	}
	void OmrpTestCommand::loadOmdb(uint32 id, DbMesh* pMesh)
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
	void OmrpTestCommand::loadOmrp(const std::vector<uint32>& ids, std::map<uint32, DbMesh*>& meshes)
	{
		const char* path = "./working/china_mesh.omrp";
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
		std::string s("SELECT id,data FROM mesh WHERE id IN (");
		for (auto pair : ids)
		{
			s += std::to_string(pair);
			s += ",";
		}
		if (!ids.empty())
		{
			s = s.substr(0, s.size() - 1);
		}
		s += ")";


		std::map<uint32, MeshBlob> data;
		const char* sql = s.c_str();
		DbRecordIterator recordIterator = DbRecordIterator(pDb);
		recordIterator.resetWithSqlStr(sql);
		while (recordIterator.hasNextRecord())
		{
			sqlite3_stmt* stmt = recordIterator.record();
			uint32 id = sqlite3_column_int64(stmt, 0);
			void* blob = (void*)sqlite3_column_blob(stmt, 1);
			int size = sqlite3_column_bytes(stmt, 1);
			MeshBlob mb;
			mb.data = (unsigned char*)malloc(size);
			memcpy(mb.data, blob, size);
			mb.size = size;
			data.emplace(id, mb);
		}
		sqlite3_close(pDb);

		// 初始化线程
		CompileSetting* pSetting = CompileSetting::instance();
		unsigned int threadNumber = std::thread::hardware_concurrency();
		if (pSetting->threadNumber > 0)
			threadNumber = min(pSetting->threadNumber, threadNumber);
		std::queue <uint32 > tasks;
		for_each(data.begin(), data.end(), [&](const std::pair<uint32, MeshBlob>& pair)->void {tasks.push(pair.first); });
		MeshReader reader;

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
						id = tasks.front();
						tasks.pop();
					}

					MeshBlob blob = data[id];
					DbMesh* pMesh = new DbMesh();
					pMesh->setId(id);
					reader.read(blob.data, blob.size, pMesh);
					free(blob.data);
					meshes[id] = pMesh;
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

		// load
		auto start = std::chrono::steady_clock::now();
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
	}


	OmrpTestCommand* OmrpTestCommand::instance()
	{
		return &g_sCommand;
	}
}
