#include "stdafx.h"
#include "FileManager.h"
#include "spatialite.h"
#include "../CompileSetting.h"
#include "boost/filesystem/string_file.hpp"
#include "../loader/DbRecordIterator.h"
#include "../loader/DbTollGateLoader.h"
namespace OMDB
{
	FileManager::FileManager()
	{

	}

	FileManager FileManager::g_manager;

	void FileManager::initializePatchData()
	{
		initializeTollPatchData();
	}

	void FileManager::initializeTollPatchData()
	{
		std::string path = "./working/HAD_LANE_LINK_TOLL_23m9.sqlite";
		if (boost::filesystem::exists(path))
		{
			sqlite3* pDb = nullptr;
			int status = sqlite3_open_v2(path.c_str(), &pDb, SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX | SQLITE_OPEN_SHAREDCACHE, nullptr);
			if (status != SQLITE_OK)
			{
				sqlite3_close(pDb);
				return;
			}
			void* pConnection = spatialite_alloc_connection();
			spatialite_init_ex(pDb, pConnection, 0);
			{
				DbTollGateLoader().LoadLinkCardTypes(pDb);
			}
			spatialite_cleanup_ex(pConnection);
			spatialite_shutdown();
			sqlite3_close(pDb);
			return;
		}
	}

	FileManager* FileManager::instance()
	{
		return &g_manager;
	}

	void FileManager::initialize()
	{
		if (CompileSetting::instance()->omrpDir != "")
		{
			std::string path = CompileSetting::instance()->omrpDir + "/china_mesh.omrp";
			if (boost::filesystem::exists(path))
			{
				std::ifstream f(path);
				f.open(path, std::ios::_Nocreate);
				sqlite3* pDb = nullptr;
				int status = sqlite3_open_v2(path.c_str(), &pDb, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, NULL);
				if (status != SQLITE_OK)
				{
					sqlite3_close(pDb);
					return;
				}
				sqlite3_exec(pDb, "PRAGMA synchronous = OFF", 0, 0, 0);
				std::string s("SELECT id FROM mesh");

				const char* sql = s.c_str();
				DbRecordIterator recordIterator = DbRecordIterator(pDb);
				recordIterator.resetWithSqlStr(sql);
				while (recordIterator.hasNextRecord())
				{
					sqlite3_stmt* stmt = recordIterator.record();
					m_meshIdList.push_back(sqlite3_column_int(stmt, 0));
				}
				sqlite3_close(pDb);
			}
			else
			{
				printError("Not Get Omrp File From omdb.ini File Settting");
			}
		}
		else
		{
			CompileSetting* pSetting = CompileSetting::instance();
			NcArray<NcString>* fileNames = FileSys_findFilesInPathWithPattern(NcString::stringWithUtf8CString(pSetting->sourceDir.c_str()), true, _S("*.omdb"));
			for (int i = 0; i < fileNames->size(); i++)
			{
				NcString* path = fileNames->objectAtIndex(i);
				int startPos = nc_max(path->locationOfCharacterBackwards(L'/'), path->locationOfCharacterBackwards(L'\\'));
				int endPos = path->locationOfCharacterBackwards(L'.');
				NcString* gridStr = path->substringFromTo(startPos + 1, endPos);
				if (gridStr->equals(L"METADATA"))
				{
					continue;
				}
				uint32 meshId = cq_wtoi(gridStr->cstr());
				m_meshIdList.push_back(meshId);
				m_mapFilePath.emplace(meshId, path);
			}
		}

		initializePatchData();
	}

	NcString* FileManager::queryPath(uint32 meshId)
	{
		if (m_mapFilePath.find(meshId) == m_mapFilePath.end())
		{
			return nullptr;
		}
		return m_mapFilePath[meshId];
	}

	std::vector<uint32>& FileManager::idList()
	{
		return m_meshIdList;
	}

}
