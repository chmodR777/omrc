#pragma once

#ifdef _WIN32
#include <dbghelp.h>
#include <stddef.h>

class CallStackWalker
{
public:
	CallStackWalker(HANDLE processHandle, const cqCHAR* symbolPath, PEXCEPTION_POINTERS ep);
	~CallStackWalker() {}

	static void seTranslator(unsigned int ui, PEXCEPTION_POINTERS ep);
	static LONG WINAPI unhandledExceptionFilter(PEXCEPTION_POINTERS ep);

	void showExceptionInfo();
	void showCallstack();
	void dumpCallstack();

private:
	HANDLE m_processHandle;
	const cqCHAR* m_symbolPath;
	PEXCEPTION_POINTERS m_ep;
};

forceinline void setDefaultExceptions() {
	_set_se_translator(CallStackWalker::seTranslator);
}

#endif
