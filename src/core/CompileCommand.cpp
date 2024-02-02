#include "stdafx.h"
#include "CompileCommand.h"
#include "navi_rds/rds.h"
#include "writer/RdsWriterFactory.h"
#include "boost/filesystem.hpp"
#include "writer/RdsDatabaseWriter.h"
#include "framework/SpatialSeacher.h"
#include "DistributedCompiler.h"
#include "framework/FileManager.h"

namespace OMDB
{
    CompileCommand::CompileCommand()
    {
    }

	CompileCommand::CompileCommand(const CompileCommand& command)
	{
		command;
	}

	CompileCommand::~CompileCommand()
    {

    }

	const cqWCHAR* CompileCommand::name()
	{
		return L"omdb";
	}

	const cqWCHAR* CompileCommand::shortDescription()
	{
		return L"Compile one map data";
	}

	void CompileCommand::printHelp()
	{
		printf(
			R"(Synopsis:
			  %s compile [OPTIONS]
			Options:
			  --sourceDir <DIRECTORY>
				  Specify the directory which contains source data.
				  It will override the `sourceDir` setting in omdb.ini
			  --outputDir <DIRECTORY>
				  Specify the directory of output.
				  It will override the `outputDir` setting in omdb.ini
			  --threadNumber <thread number>
				  Specify the concurrency thread number.
				  It will override the `threadNumber` setting in omdb.ini
			)",
			EXE_NAME);
	}

	bool CompileCommand::parseArgs(ArgParser* parser)
	{
		ToString(parser->getArg(L"sourceDir"), m_options.sourceDir);
		ToString(parser->getArg(L"outputDir"), m_options.outputDir);

		const cqWCHAR* s = parser->getArg(L"threadNumber");
		if (s != nullptr)
			m_options.threadNumber = cq_wtoi(s);

		return !parser->printUnknownArgs();
	}
	int CompileCommand::exec()
	{
		CompileSetting* pSetting = CompileSetting::instance();
		beginTitle("%s Version: %s", EXE_NAME, g_exeVersion);

		IniFile iniFile;
		beginSubtitle("Loading omdb.ini");
		if (!iniFile.load(L"omdb.ini")) {
			printError("Failed to open `omdb.ini`.");
			return false;
		}
		if (!pSetting->load(&iniFile))
			return false;
		endSubtitle();

		if (!pSetting->applyCompilerOptionsAndCheck(&m_options)) {
			return false;
		}

		pSetting->printSettings();

		FileManager::instance()->initialize();
		DistributedCompiler compiler;
		compiler.execute();
		return 0;

	}

	CompileCommand CompileCommand::g_sCommand;

	CompileCommand* CompileCommand::instance()
	{
		return &g_sCommand;
	}
}
