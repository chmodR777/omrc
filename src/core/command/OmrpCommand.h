#pragma once
#include <vector>
#include "../loader/DataLoader.h"
#include "../processor/Processor.h"
#include "../CompileSetting.h"
namespace OMDB
{
    class OmrpCommand : public CliCommand
    {
    private:
        OmrpCommand();
        OmrpCommand(const OmrpCommand& command);
        ~OmrpCommand();
        virtual const cqWCHAR* name() override;
		virtual const cqWCHAR* shortDescription() override;
		virtual void printHelp() override;
        virtual bool parseArgs(ArgParser* parser) override;
        void load(uint32 id, DbMesh* pMesh);
        void writeDb(const char* path, const std::map<uint32, MeshBlob>& data);
    public:
        static OmrpCommand* instance();
    public:
		virtual int exec() override;

    private:
        CompilerOptions m_options;
		static OmrpCommand g_sCommand;
    };
}

