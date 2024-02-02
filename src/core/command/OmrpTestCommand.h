#pragma once
#include <vector>
#include "../loader/DataLoader.h"
#include "../CompileSetting.h"
namespace OMDB
{
    class OmrpTestCommand : public CliCommand
    {
    private:
        OmrpTestCommand();
        OmrpTestCommand(const OmrpTestCommand& command);
        ~OmrpTestCommand();
        virtual const cqWCHAR* name() override;
		virtual const cqWCHAR* shortDescription() override;
		virtual void printHelp() override;
        virtual bool parseArgs(ArgParser* parser) override;
        void loadOmdb(const std::vector<uint32>& ids, std::map<uint32, DbMesh*>& meshes);
		void loadOmrp(const std::vector<uint32>& ids, std::map<uint32, DbMesh*>& meshes);

        void loadOmdb(uint32 id, DbMesh* pMesh);

    public:
        static OmrpTestCommand* instance();
    public:
		virtual int exec() override;

    private:
        CompilerOptions m_options;
		static OmrpTestCommand g_sCommand;
    };
}

