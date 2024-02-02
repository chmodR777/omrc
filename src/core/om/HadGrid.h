#pragma once
#include "HadElement.h"
#include "../framework/FastTable.h"
#include "util/batched_allocator.h"
#include <list>
namespace OMDB
{
    class HadGrid
    {
    private:
		int32	m_meshId = 0;
		BoundingBox2d m_extent{};
		BatchedAllocator m_batchedAllocator;
		std::vector<HadElement*> m_data;
		std::map<ElementType, FastTable<int64, HadElement*>> m_tbTable;

		static std::vector<HadElement*> INVALID;

	public:
		HadGrid();
		~HadGrid();
		HadElement* alloc(ElementType objectType);
		void insert(int64 id, HadElement* pObject);
		HadElement* query(int64 id, ElementType objectType);
		std::vector<HadElement*>& query(ElementType objectType);
		void setId(int32 id);
		int32 getId() const;
		const BoundingBox2d& getBoundingbox2d() { return m_extent; }
    };

}

