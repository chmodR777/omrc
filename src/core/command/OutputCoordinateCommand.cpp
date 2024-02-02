#include "stdafx.h"
#include "OutputCoordinateCommand.h"
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
    OutputCoordinateCommnad::OutputCoordinateCommnad()
    {
    }

	OutputCoordinateCommnad::OutputCoordinateCommnad(const OutputCoordinateCommnad& command)
	{
		command;
	}

	OutputCoordinateCommnad::~OutputCoordinateCommnad()
    {

    }

	const cqWCHAR* OutputCoordinateCommnad::name()
	{
		return L"opcd";
	}

	const cqWCHAR* OutputCoordinateCommnad::shortDescription()
	{
		return L"render preprocess one map data";
	}

	void OutputCoordinateCommnad::printHelp()
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

	bool OutputCoordinateCommnad::parseArgs(ArgParser* parser)
	{
		ToString(parser->getArg(L"source-dir"), m_options.sourceDir);
		ToString(parser->getArg(L"output-dir"), m_options.outputDir);

		const cqWCHAR* s = parser->getArg(L"thread-number");
		if (s != nullptr)
			m_options.threadNumber = cq_wtoi(s);

		return !parser->printUnknownArgs();
	}
	int OutputCoordinateCommnad::exec()
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
		std::string filePath = "speedlimitboard.txt";
		if (boost::filesystem::exists(filePath))
		{
			boost::filesystem::remove(filePath);
		}
		size_t capacity = 1000;
		size_t size = ids.size();
		size_t number = size / capacity + (size % capacity == 0 ? 0 : 1);
		std::sort(ids.begin(), ids.end());

		printInfo("开始统计限速标牌坐标，网格数量：%d", ids.size());
		int progress = 0;
		for (int i = 0; i < number; i++)
		{
			std::ofstream fout(filePath,std::ios_base::app);
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
			std::map<uint32, DbMesh*> meshes;
			std::map<uint32, std::string> corrdinates;
			loadOmrp(v, meshes);

			for (int j = 0;j < v.size();j++)
			{
				int npg = (int)(( j + i * capacity) * 1.0 / size * 100.0);
				if (npg > progress)
				{
					printInfo("[%d%%] compile omdb.", npg);
					progress = npg;
				}
				// 统计限速标牌位置信息
				{
					if (meshes.find(v[j]) == meshes.end())
					{
						continue;
					}
					DbMesh* pMesh = meshes[v[j]];
					std::string s;

					for (auto& e : pMesh->query(RecordType::DB_HAD_OBJECT_TRAFFIC_SIGN))
					{
						DbTrafficSign* psi = (DbTrafficSign*)e;
						int signType = psi->signType;
						int kind1 = signType / 100;
						if (kind1 < 2 || kind1 > 4)
							continue;

						int kind2 = signType % 100;
						if (kind2 <= 1)
							continue;

						if (kind1 == 2 || kind1 == 3 || kind2 == 4)
						{
							s += ("(" + std::to_string(psi->geometry.vertexes[0].pos.lon / 1000) + "," + std::to_string(psi->geometry.vertexes[0].pos.lat / 1000) + ")");
						}
					}
					if (!s.empty())
					{
						corrdinates[pMesh->getId()] = s;
					}
					delete pMesh;
				}
			}
			for (auto& pair : corrdinates)
			{
				fout << pair.first << "|" << pair.second << std::endl;
			}
			fout.flush();
			fout.close();
		}
		return 0;
	}

	OutputCoordinateCommnad OutputCoordinateCommnad::g_sCommand;
	void OutputCoordinateCommnad::loadOmrp(const std::vector<uint32>& ids, std::map<uint32, DbMesh*>& meshes)
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


	OutputCoordinateCommnad* OutputCoordinateCommnad::instance()
	{
		return &g_sCommand;
	}
}
