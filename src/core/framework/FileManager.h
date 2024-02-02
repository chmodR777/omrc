#pragma once
#include <map>
namespace OMDB
{
	class FileManager
	{
	private:
		FileManager();
		std::vector<uint32> m_meshIdList;
		std::map<uint32, NcString*> m_mapFilePath;
		static FileManager g_manager;

		// 初始化补丁数据
		void initializePatchData();
		// 初始化收费站补丁数据
		void initializeTollPatchData();
	public:
		static FileManager* instance();
		void initialize();
		NcString* queryPath(uint32 meshId);
		std::vector<uint32>& idList();

	};
}

