#pragma once

#include "util/batched_allocator.h"

/**
	@brief �̰߳�ȫ�汾��BatchedAllocator
*/
class SafeBatchedAllocator
{
public:
	SafeBatchedAllocator() : m_spinLock(0) {}
	SafeBatchedAllocator(size_t blockSize) : m_spinLock(0) { initWithBlockSize(blockSize); }

	void initWithBlockSize(size_t blockSize) { m_ba.initWithBlockSize(blockSize); }

	void* allocMemory(size_t size);

	template<typename T>
	T* allocArray(size_t size) { return (T*)allocMemory(sizeof(T) * size); }

	void freeAll();

	size_t memoryUsage();

	size_t systemMallocCount();

	void swap(SafeBatchedAllocator* ba);

	void* storeBuffer(const void* buffer, size_t size);
	cqWCHAR* storeString(const cqWCHAR* s);
	cqWCHAR* storeUtf8AsString(const cqCHAR* s);	///< ��һ�� UTF-8 ������ַ���ת��Ϊ���ַ����󱣴浽 MemPools ��
	cqWCHAR* storeStringWithLength(const cqWCHAR* s, size_t len); ///< �����ڴ����ڱ���ָ�����ȵĿ��ַ���
	cqCHAR* storeAnsiString(const cqCHAR* s);
	cqCHAR* storeAnsiStringWithLength(const cqCHAR* s, size_t len);

private:
	BatchedAllocator m_ba;
	volatile long m_spinLock;
};
