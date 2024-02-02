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

		// ��ʼ����������
		void initializePatchData();
		// ��ʼ���շ�վ��������
		void initializeTollPatchData();
	public:
		static FileManager* instance();
		void initialize();
		NcString* queryPath(uint32 meshId);
		std::vector<uint32>& idList();

	};
}

