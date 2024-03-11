#pragma once
#include "stdafx.h"
#include "DbRecord.h"
#include "framework/FastTable.h"
#include "framework/Spinlock.h"
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

	struct lanePaInfo
	{
		DbRdLinkLanePa* _lanePa;

		// 左右边界都是顺方向,如果道路边界坐标改变以下值也需更新
		linestring_t _leftLine;
		linestring_t _rightLine;
		box_2t _lanePaBox2T;
	};

	class DbMesh : public DbRecord
	{
	private:
		int32									m_meshId = 0;
		BoundingBox2d							m_extent{};
		static std::vector<DbRecord*>				INVALID;
		std::map<RecordType, FastTable<int64, DbRecord*>> m_tbTable;
		BatchedAllocator m_batchedAllocator;
		std::vector<DbRecord*> m_data;

		// 路网,抓路索引(给generator用)
		std::vector<box_2t> m_linkBoxes;
		std::vector<dbLinkInfo> m_links;
		std::map<int64, size_t> m_linkIndexes;
		std::shared_ptr<rtree_type_2box> m_linkRtree;

		// 车道组,抓路面索引(给generator用)
		std::vector<box_2t> m_lanePaBoxes;
		std::vector<lanePaInfo> m_lanePas;
		std::map<int64, size_t> m_lanePaIndexes;
		std::shared_ptr<rtree_type_2box> m_lanePaRtree;

		// 上下行道路（ZGenerator生成，RoadBoundaryGenerator使用），
		// vector of divided road, divided road = [left path, right path], path = vector of dir link, dir link = (DbLink, direction)
		std::vector<std::array<std::vector<std::pair<DbLink*, bool>>, 2>> m_dividedRoads;
	public:
		DbMesh();
		~DbMesh();
		DbRecord* alloc(RecordType objectType);
		void insert(int64 id,DbRecord* record);
		DbRecord* query(int64 id, RecordType recordType);
		std::vector<DbRecord*>& query(RecordType record);
		FastTable<int64, DbRecord*>& queryTable(RecordType record);
		void setId(int32 id);
		int32 getId();
		const BoundingBox2d& getBoundingbox2d() { return m_extent; }
		int64 memoryUsage() { return m_batchedAllocator.memoryUsage(); }
// 		std::map<RecordType, BatchedAllocator> m_mapBatchedAllocator;

		dbLinkInfo& queryLinkInfo(int64 id);
		void insertLinkInfo(dbLinkInfo& info);
		std::vector<dbLinkInfo>& getLinkInfos();
		std::shared_ptr<rtree_type_2box> getLinkRtree();

		lanePaInfo& queryLanePaInfo(int64 id);
		void insertLanePaInfo(lanePaInfo& info);
		std::vector<lanePaInfo>& getLanePaInfos();
		std::shared_ptr<rtree_type_2box> getLanePaRtree();

		void setDividedRoads(std::vector<std::array<std::vector<std::pair<DbLink*, bool>>, 2>>&& dividedRoads) { m_dividedRoads = std::move(dividedRoads); }
		std::vector<std::array<std::vector<std::pair<DbLink*, bool>>, 2>>& getDividedRoads() { return m_dividedRoads; }
	};
}