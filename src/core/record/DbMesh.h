#pragma once
#include "stdafx.h"
#include "DbRecord.h"
#include "framework/FastTable.h"
#include "framework/Spinlock.h"
#include "tool_kit/boost_geometry_utils.h"
#include <unordered_map>
namespace OMDB
{
	struct MeshBlob
	{
		unsigned char* data{nullptr};
		size_t size{0};
	};

	struct dbLinkInfo
	{
		DbLink* _link;
		box_2t _linkBox2T;
		linestring_2t _linkPoints2T;
	};

	class DbMesh : public DbRecord
	{
	private:
		int32									m_meshId = 0;
		static std::vector<DbRecord*>				INVALID;
		std::map<RecordType, FastTable<int64, DbRecord*>> m_tbTable;
		BatchedAllocator m_batchedAllocator;
		std::vector<DbRecord*> m_data;

		// 路网,抓路索引(给generator用)
		std::vector<box_2t> m_linkBoxes;
		std::vector<dbLinkInfo> m_links;
		std::shared_ptr<rtree_type_2box> m_linkRtree;
	public:
		DbMesh();
		~DbMesh();
		DbRecord* alloc(RecordType objectType);
		void insert(int64 id,DbRecord* record);
		DbRecord* query(int64 id, RecordType recordType);
		std::vector<DbRecord*>& query(RecordType record);
		bool erase(int64 id, RecordType recordType);
		FastTable<int64, DbRecord*>& queryTable(RecordType record);
		void setId(int32 id);
		int32 getId();
		int64 memoryUsage() { return m_batchedAllocator.memoryUsage(); }
// 		std::map<RecordType, BatchedAllocator> m_mapBatchedAllocator;

		dbLinkInfo& queryLinkInfo(int64 id);
		void insertLinkInfo(dbLinkInfo& info);
		std::vector<dbLinkInfo>& getLinkInfos();
		std::shared_ptr<rtree_type_2box> getLinkRtree();
	};
}