#include "stdafx.h"
#include "navi_core_env.h"
#include "tool_kit/call_stack_walker.h"
#include "tool_kit/new_handler.h"
#include "core/CompileCommand.h"
#include "core/command/OmrpCommand.h"
#include "core/command/RdsDataCheckCommand.h"
#include "core/command/OmrpTestCommand.h"
#include "core/command/OutputCoordinateCommand.h"

const cqCHAR* g_exeVersion = nullptr;
std::string string_To_UTF8(const std::string& str)
{
	int nwLen = ::MultiByteToWideChar(CP_ACP, 0, str.c_str(), -1, NULL, 0);

	wchar_t* pwBuf = new wchar_t[nwLen + 1];//一定要加1，不然会出现尾巴 
	ZeroMemory(pwBuf, nwLen * 2 + 2);

	::MultiByteToWideChar(CP_ACP, 0, str.c_str(), str.length(), pwBuf, nwLen);

	int nLen = ::WideCharToMultiByte(CP_UTF8, 0, pwBuf, -1, NULL, NULL, NULL, NULL);

	char* pBuf = new char[nLen + 1];
	ZeroMemory(pBuf, nLen + 1);

	::WideCharToMultiByte(CP_UTF8, 0, pwBuf, nwLen, pBuf, nLen, NULL, NULL);

	std::string retStr(pBuf);

	delete[]pwBuf;
	delete[]pBuf;

	pwBuf = NULL;
	pBuf = NULL;

	return retStr;
}

void printHelp()
{
	printf(R"(%s version: %s
Compiled at %s %s
Usage: %s [--version] [--help] [--changelog]
            <command> [<args>]

Supported commands:
)", EXE_NAME, g_exeVersion, __DATE__, __TIME__, EXE_NAME);

	CliCommandCenter::instance()->printCommandList();
}

int wmain(int argc, wchar_t* argv[])
{
	try
	{
		setDefaultExceptions();
// 		setNewHandler();

		setvbuf(stdout, NULL, _IONBF, 0);
		if (setlocale(LC_CTYPE, "chs") == NULL)
			return -1;

		ArgParser parser;
		parser.parse(argc, argv);

		NaviCoreEnv env;

		cq_logSetLevel(MapbarLogLevel_info);

		g_exeVersion = ControlUtil_versionFromChangelog(g_changelog);

		autoreleasepool
		{
			CliCommandCenter* cc = CliCommandCenter::instance();
			cc->addCommand(OMDB::CompileCommand::instance());
			cc->addCommand(OMDB::OmrpCommand::instance());
			cc->addCommand(OMDB::RdsDataCheckCommand::instance());
			cc->addCommand(OMDB::OutputCoordinateCommnad::instance());
 			cc->addCommand(OMDB::OmrpTestCommand::instance());
			cc->parseCommand(&parser);
			CliCommand* cmd = cc->choosenCommand();
			if (cmd == NULL)
			{
				if (cc->shouldPrintVersion())
					printf("%s\n", g_exeVersion);
				else if (cc->shouldPrintChangelog())
					printf("%s", g_changelog);
				else if (cc->shouldPrintHelp())
					printHelp();
				else
					return -1;

				return 0;
			}

			return cmd->exec();
		}
	}
	catch (CallStackWalker& walker)
	{
		walker.showExceptionInfo();
		return -1;
	}
	catch (const std::exception& e)
	{
		printError("Catch C++ exception: %s", e.what());
		return -1;
	}

	return 0;
}
