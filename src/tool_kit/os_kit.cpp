#include "stdafx.h"
#include "os_kit.h"
#include "psapi.h"

#ifdef _WIN32
#include <Windows.h>
#include <DbgHelp.h>
#endif

void OsKit_printMemoryUsage()
{
	HANDLE currentProcessHandle = GetCurrentProcess();
	PROCESS_MEMORY_COUNTERS pmc;

	beginSubtitle("Current Process Memory Usage");
	if (!GetProcessMemoryInfo(currentProcessHandle, &pmc, sizeof(pmc)))
	{
		printError("GetProcessMemoryInfo() returns FALSE and GetLastError() returns %d.", GetLastError());
		return;
	}

	printInfo("PageFaultCount            : %u", pmc.PageFaultCount);
	printInfo("PeakWorkingSetSize        : %Iu", pmc.PeakWorkingSetSize);
	printInfo("WorkingSetSize            : %Iu", pmc.WorkingSetSize);
	printInfo("QuotaPeakPagedPoolUsage   : %Iu", pmc.QuotaPeakPagedPoolUsage);
	printInfo("QuotaPagedPoolUsage       : %Iu", pmc.QuotaPagedPoolUsage);
	printInfo("QuotaPeakNonPagedPoolUsage: %Iu", pmc.QuotaPeakNonPagedPoolUsage);
	printInfo("QuotaNonPagedPoolUsage    : %Iu", pmc.QuotaNonPagedPoolUsage);
	printInfo("PagefileUsage             : %Iu", pmc.PagefileUsage);
	printInfo("PeakPagefileUsage         : %Iu", pmc.PeakPagefileUsage);

	endSubtitle();
}

void OsKit_printCurrentStackInfo()
{
#ifdef _WIN32
	static const int MAX_STACK_FRAMES = 50;

	void* pStack[MAX_STACK_FRAMES];

	HANDLE processHandle = GetCurrentProcess();
	SymInitialize(processHandle, NULL, TRUE);
	WORD frames = CaptureStackBackTrace(0, MAX_STACK_FRAMES, pStack, NULL);

	printError("Callstack :");
	for (WORD i = 0; i < frames; ++i)
	{
		DWORD64 address = (DWORD64)(pStack[i]);

		BYTE symbolBuffer[sizeof(PIMAGEHLP_SYMBOL) + sizeof(cqCHAR) * MAX_PATH] = { 0 };
		PIMAGEHLP_SYMBOL symbol = (PIMAGEHLP_SYMBOL)symbolBuffer;
		symbol->SizeOfStruct = sizeof(symbolBuffer);
		symbol->MaxNameLength = MAX_PATH;

		IMAGEHLP_LINE lineInfo = { sizeof(IMAGEHLP_LINE) };

		DWORD64 displayment = 0;
		DWORD lineInfoDisplayment = 0;

		const cqCHAR* fileName = "unknown file";
		const cqCHAR* funcName = "unknown function";
		int lineNum = 0;

		if (SymGetLineFromAddr(processHandle, address, &lineInfoDisplayment, &lineInfo))
		{
			fileName = lineInfo.FileName;
			lineNum = lineInfo.LineNumber;
		}

		if (SymGetSymFromAddr(processHandle, address, &displayment, symbol))
		{
			funcName = symbol->Name;
		}

		printError("\t%s(%d): in [%s]", fileName, lineNum, funcName);
	}
#else
#error "Unsupported platform!"
#endif
}
