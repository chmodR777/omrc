#include "stdafx.h"
#include "call_stack_walker.h"

#ifdef _WIN32

static const cqCHAR* _exceptionCodeToString(DWORD exceptionCode)
{
	switch (exceptionCode)
	{
	case EXCEPTION_ACCESS_VIOLATION:
		return "ACCESS_VIOLATION";
	case EXCEPTION_DATATYPE_MISALIGNMENT:
		return "DATATYPE_MISALIGNMENT";
	case EXCEPTION_BREAKPOINT:
		return "BREAKPOINT";
	case EXCEPTION_SINGLE_STEP:
		return "SINGLE_STEP";
	case EXCEPTION_ARRAY_BOUNDS_EXCEEDED:
		return "ARRAY_BOUNDS_EXCEEDED";
	case EXCEPTION_FLT_DENORMAL_OPERAND:
		return "FLT_DENORMAL_OPERAND";
	case EXCEPTION_FLT_DIVIDE_BY_ZERO:
		return "FLT_DIVIDE_BY_ZERO";
	case EXCEPTION_FLT_INEXACT_RESULT:
		return "FLT_INEXACT_RESULT";
	case EXCEPTION_FLT_INVALID_OPERATION:
		return "FLT_INVALID_OPERATION";
	case EXCEPTION_FLT_OVERFLOW:
		return "FLT_OVERFLOW";
	case EXCEPTION_FLT_STACK_CHECK:
		return "STACK_CHECK";
	case EXCEPTION_INT_DIVIDE_BY_ZERO:
		return "INT_DIVIDE_BY_ZERO";
	case EXCEPTION_INVALID_HANDLE:
		return "INVALID_HANDLE";
	case EXCEPTION_PRIV_INSTRUCTION:
		return "PRIV_INSTRUCTION";
	case EXCEPTION_IN_PAGE_ERROR:
		return "IN_PAGE_ERROR";
	case EXCEPTION_ILLEGAL_INSTRUCTION:
		return "ILLEGAL_INSTRUCTION";
	case EXCEPTION_NONCONTINUABLE_EXCEPTION:
		return "NONCONTINUABLE_EXCEPTION";
	case EXCEPTION_STACK_OVERFLOW:
		return "STACK_OVERFLOW";
	case EXCEPTION_INVALID_DISPOSITION:
		return "INVALID_DISPOSITION";
	case EXCEPTION_FLT_UNDERFLOW:
		return "FLT_UNDERFLOW";
	case EXCEPTION_INT_OVERFLOW:
		return "INT_OVERFLOW";
	default:
		return "UNKNOWN_EXCEPTION";
	}
}

CallStackWalker::CallStackWalker(HANDLE processHandle, const cqCHAR* symbolPath, PEXCEPTION_POINTERS ep)
{
	m_processHandle = processHandle;
	m_symbolPath = symbolPath;
	m_ep = ep;
}

void CallStackWalker::seTranslator(unsigned int /*ui*/, PEXCEPTION_POINTERS ep)
{
	CallStackWalker stackWalker(GetCurrentProcess(), NULL, ep);
	throw stackWalker;
}

LONG WINAPI CallStackWalker::unhandledExceptionFilter(PEXCEPTION_POINTERS ep)
{
	CallStackWalker stackWalker(GetCurrentProcess(), NULL, ep);
	stackWalker.showExceptionInfo();

	return EXCEPTION_CONTINUE_SEARCH;
}

void CallStackWalker::showExceptionInfo()
{
	printError("=============== Exception Occured ===============");
	printError("Code : %08x (%s)", m_ep->ExceptionRecord->ExceptionCode, _exceptionCodeToString(m_ep->ExceptionRecord->ExceptionCode));
	printError("Callstack :");

	showCallstack();
}

void CallStackWalker::showCallstack()
{
	SymInitialize(m_processHandle, m_symbolPath, TRUE);

	dumpCallstack();

	SymCleanup(m_processHandle);
}

void CallStackWalker::dumpCallstack()
{
	CONTEXT* context = m_ep->ContextRecord;

	STACKFRAME stackFrame;
	ZeroMemory(&stackFrame, sizeof(stackFrame));
	DWORD imageType = 0;

#ifdef _M_IX86
	imageType = IMAGE_FILE_MACHINE_I386;
	stackFrame.AddrPC.Offset = context->Eip;
	stackFrame.AddrPC.Mode = AddrModeFlat;
	stackFrame.AddrFrame.Offset = context->Ebp;
	stackFrame.AddrFrame.Mode = AddrModeFlat;
	stackFrame.AddrStack.Offset = context->Esp;
	stackFrame.AddrStack.Mode = AddrModeFlat;
#elif _M_X64
	imageType = IMAGE_FILE_MACHINE_AMD64;
	stackFrame.AddrPC.Offset = context->Rip;
	stackFrame.AddrPC.Mode = AddrModeFlat;
	stackFrame.AddrFrame.Offset = context->Rsp;
	stackFrame.AddrFrame.Mode = AddrModeFlat;
	stackFrame.AddrStack.Offset = context->Rsp;
	stackFrame.AddrStack.Mode = AddrModeFlat;
#elif _M_IA64
	imageType = IMAGE_FILE_MACHINE_IA64;
	stackFrame.AddrPC.Offset = context->StIIP;
	stackFrame.AddrPC.Mode = AddrModeFlat;
	stackFrame.AddrFrame.Offset = context->IntSp;
	stackFrame.AddrFrame.Mode = AddrModeFlat;
	stackFrame.AddrBStore.Offset = context->RsBSP;
	stackFrame.AddrBStore.Mode = AddrModeFlat;
	stackFrame.AddrStack.Offset = context->IntSp;
	stackFrame.AddrStack.Mode = AddrModeFlat;
#else
#error "Platform not supported!"
#endif

	for (;;)
	{
		if (!StackWalk64(imageType, m_processHandle, GetCurrentThread(), &stackFrame, context, NULL, SymFunctionTableAccess, SymGetModuleBase, 0))
			break;

		if (stackFrame.AddrFrame.Offset == 0)
			break;

		BYTE symbolBuffer[sizeof(IMAGEHLP_SYMBOL) + sizeof(cqCHAR) * MAX_PATH] = { 0 };
		PIMAGEHLP_SYMBOL symbol = (PIMAGEHLP_SYMBOL)symbolBuffer;
		symbol->SizeOfStruct = sizeof(symbolBuffer);
		symbol->MaxNameLength = MAX_PATH;

		IMAGEHLP_LINE lineInfo = { sizeof(IMAGEHLP_LINE) };

		DWORD64 displayment = 0;
		DWORD lineInfoDisplayment = 0;

		const cqCHAR* fileName = "unknown file";
		const cqCHAR* funcName = "unknown function";
		int lineNum = 0;

		if (SymGetLineFromAddr(m_processHandle, stackFrame.AddrPC.Offset, &lineInfoDisplayment, &lineInfo))
		{
			fileName = lineInfo.FileName;
			lineNum = lineInfo.LineNumber;
		}

		if (SymGetSymFromAddr(m_processHandle, stackFrame.AddrPC.Offset, &displayment, symbol))
		{
			funcName = symbol->Name;
		}

		printError("\t%s(%d): in [%s]", fileName, lineNum, funcName);
	}
}

#endif
