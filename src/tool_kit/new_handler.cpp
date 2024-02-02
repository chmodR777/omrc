#include "stdafx.h"
#include "tool_kit/new_handler.h"
#include "cq_memory_dbg.h"
#include "tool_kit/os_kit.h"

#ifdef _WIN32
#include <new.h>
#include <dbghelp.h>
#include <windows.h>
#else
#include <new>
#endif

namespace
{
#ifdef _WIN32
	int windowsNewHandler(size_t size)
	{
		printError("Failed to alloc memory(size:%I64u), call stack:", size);
		OsKit_printCurrentStackInfo();
		OsKit_printMemoryUsage();
		// 返回0表示不再继续尝试申请内存，后续会抛出异常
		return 0;
	}
#else
	void newHandler()
	{
		printError("Failed to alloc memory, call stack:");
		OsKit_printCurrentStackInfo();
		OsKit_printMemoryUsage();
		// 再次调用 operator new 仍然失败时会抛出 bad_alloc 异常
		std::set_new_handler(nullptr);
	}
#endif
}

void setNewHandler()
{
#ifdef _WIN32
	_set_new_handler(&windowsNewHandler);
#else
	std::set_new_handler(&newHandler);
#endif
}
