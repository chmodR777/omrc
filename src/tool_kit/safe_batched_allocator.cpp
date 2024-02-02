#include "stdafx.h"
#include "safe_batched_allocator.h"

void* SafeBatchedAllocator::allocMemory(size_t size)
{
	void* mem = nullptr;
	spinlock(&m_spinLock)
	{
		mem = m_ba.allocMemory(size);
	}
	return mem;
}

void SafeBatchedAllocator::freeAll()
{
	spinlock(&m_spinLock)
	{
		m_ba.freeAll();
	}
}

size_t SafeBatchedAllocator::memoryUsage()
{
	size_t usage = 0;
	spinlock(&m_spinLock)
	{
		usage = m_ba.memoryUsage();
	}
	return usage;
}

size_t SafeBatchedAllocator::systemMallocCount()
{
	size_t count = 0;
	spinlock(&m_spinLock)
	{
		count = m_ba.systemMallocCount();
	}
	return count;
}

void SafeBatchedAllocator::swap(SafeBatchedAllocator* ba)
{
	spinlock(&m_spinLock)
	{
		m_ba.swap(&ba->m_ba);
	}
}

void* SafeBatchedAllocator::storeBuffer(const void* buffer, size_t size)
{
	void* buf = nullptr;
	spinlock(&m_spinLock)
	{
		buf = m_ba.storeBuffer(buffer, size);
	}
	return buf;
}

cqWCHAR* SafeBatchedAllocator::storeString(const cqWCHAR* s)
{
	cqWCHAR* str = nullptr;
	spinlock(&m_spinLock)
	{
		str = m_ba.storeString(s);
	}
	return str;
}

cqWCHAR* SafeBatchedAllocator::storeUtf8AsString(const cqCHAR* s)
{
	cqWCHAR* str = nullptr;
	spinlock(&m_spinLock)
	{
		str = m_ba.storeUtf8AsString(s);
	}
	return str;
}

cqWCHAR* SafeBatchedAllocator::storeStringWithLength(const cqWCHAR* s, size_t len)
{
	cqWCHAR* str = nullptr;
	spinlock(&m_spinLock)
	{
		str = m_ba.storeStringWithLength(s, len);
	}
	return str;
}

cqCHAR* SafeBatchedAllocator::storeAnsiString(const cqCHAR* s)
{
	cqCHAR* str = nullptr;
	spinlock(&m_spinLock)
	{
		str = m_ba.storeAnsiString(s);
	}
	return str;
}

cqCHAR* SafeBatchedAllocator::storeAnsiStringWithLength(const cqCHAR* s, size_t len)
{
	cqCHAR* str = nullptr;
	spinlock(&m_spinLock)
	{
		str = m_ba.storeAnsiStringWithLength(s, len);
	}
	return str;
}