#include "stdafx.h"
#include "MeshCache.h"
#include "FileManager.h"
#include "spatialite.h"
#include "../loader/DbLinkLoader.h"
#include "../loader/DbLaneLoader.h"
#include "../loader/DbRoadBoundaryLoader.h"
#include "../loader/DbLaneMarkLoader.h"
#include "../loader/DbObjectLoader.h"
#include "../loader/DbIntersectionLoader.h"
#include "../loader/DbTollGateLoader.h"
#include <fstream>
#include <algorithm>
#include <mutex>
#include "../CompileSetting.h"
#include "TopoBuilder.h"
#include "../record/MeshReader.h"
#include "boost/filesystem/string_file.hpp"
#include "tool_kit/call_stack_walker.h"
namespace OMDB
{
	class SqliteDbGuard
	{
	public:
		SqliteDbGuard(sqlite3* db) : m_db(db) {}
		~SqliteDbGuard()
		{
			if (SQLITE_OK != sqlite3_close(m_db))
				printError("Close Database failed: %s", sqlite3_errmsg(m_db));
		}
	private:
		sqlite3* m_db;
	};

	MeshCache::~MeshCache()
	{
		for (auto& pair : m_data)
		{
			delete pair.second;
		}

		m_data.clear();
	}
	void MeshCache::searchNearBy(uint32 id, std::vector<uint32>& nearby)
	{
		for (int row = -1; row <= 1; row++) {
			for (int col = -1; col <= 1; col++) {
				NdsGridId gridId = NdsGridId_shift(MeshId_toNdsGridId(id), row, col);
				int32 meshId = NdsGridId_toMeshId(gridId);
				if (meshId != id) {
					nearby.push_back(meshId);
				}
			}
		}
	}

	void MeshCache::loadOmdb(const std::vector<uint32>& ids)
	{
		for (auto& id : ids)
		{
			NcString* path = FileManager::instance()->queryPath(id);
			if (path == nullptr)
				continue;

			std::vector<uint32> nearby;
			searchNearBy(id, nearby);
			for (auto iter = nearby.begin(); iter != nearby.end(); iter++)
			{
				if (FileManager::instance()->queryPath(*iter) == nullptr)
				{
					nearby.erase(iter);
					iter--;
					continue;
				}
				m_data.emplace(*iter, nullptr);
			}
			m_mapNearby[id] = nearby;
			m_data.emplace(id, nullptr);
		}

		// 初始化线程
		CompileSetting* pSetting = CompileSetting::instance();
		unsigned int threadNumber = std::thread::hardware_concurrency();
		if (pSetting->threadNumber > 0)
			threadNumber = min(pSetting->threadNumber, threadNumber);

		//由于sqlite3的多线程支持问题，此处暂时采用单线程
		threadNumber = 1;
		std::queue <uint32 > tasks;
		for_each(m_data.begin(), m_data.end(), [&](const std::pair<uint32, DbMesh*>& pair)->void {tasks.push(pair.first); });
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
					load(id, pMesh);
					m_data[id] = pMesh;
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

	void MeshCache::loadOmrp(const std::vector<uint32>& ids)
	{
		std::vector<uint32>& total = FileManager::instance()->idList();
		for (auto& id : ids)
		{
			if (std::find(total.begin(),total.end(),id) == total.end())
			{
				continue;
			}

			std::vector<uint32> nearby;
			searchNearBy(id, nearby);
			for (auto iter = nearby.begin(); iter != nearby.end(); iter++)
			{
				if (std::find(total.begin(), total.end(), *iter) == total.end())
				{
					nearby.erase(iter);
					iter--;
					continue;
				}
				m_data.emplace(*iter, nullptr);
			}
			m_mapNearby[id] = nearby;
			m_data.emplace(id, nullptr);
		}

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
		for (auto pair : m_data)
		{
			s += std::to_string(pair.first);
			s += ",";
		}
		if (!m_data.empty())
		{
			s = s.substr(0,s.size() -1);
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
			if (size != 0)
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
					m_data[id] = pMesh;
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

	void MeshCache::query(uint32 meshId, DbMesh*& center)
	{
		center = m_data[meshId];
	}

	void MeshCache::query(uint32 meshId, DbMesh*& center, std::vector<DbMesh*>& nearby)
	{
		center = m_data[meshId];
		std::vector<uint32> nearbyIds = m_mapNearby[meshId];
		for (auto id : nearbyIds)
		{
			nearby.push_back(m_data[id]);
		}
	}

	void MeshCache::reset(const std::vector<uint32>& ids)
	{
		for (auto& p : m_data)
		{
			delete p.second;
		}
		m_data.clear();
		m_mapNearby.clear();

		std::string path = "./working/china_mesh.omrp";
		if (boost::filesystem::exists(path))
		{
			loadOmrp(ids);
		}
		else
		{
			loadOmdb(ids);
		}
	}
	void MeshCache::load(uint32 id, DbMesh* pMesh)
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
		SqliteDbGuard dbGuard(pSqlite3);
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
}