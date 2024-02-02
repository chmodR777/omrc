#pragma once
#include <vector>
#include "loader/DataLoader.h"
#include "processor/Processor.h"
#include "compiler/Compiler.h"
#include "CompileSetting.h"
using namespace RDS;
namespace OMDB
{
    class CompileCommand : public CliCommand
    {
    private:
        CompileCommand();
        CompileCommand(const CompileCommand& command);
        ~CompileCommand();
        virtual const cqWCHAR* name() override;
		virtual const cqWCHAR* shortDescription() override;
		virtual void printHelp() override;
        virtual bool parseArgs(ArgParser* parser) override;

    public:
        static CompileCommand* instance();
    public:
		virtual int exec() override;

    private:
        CompilerOptions m_options;
		static CompileCommand g_sCommand;
    };
}

