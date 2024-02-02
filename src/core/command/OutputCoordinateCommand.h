#pragma once
#include <vector>
#include "../loader/DataLoader.h"
#include "../CompileSetting.h"
namespace OMDB
{
    class OutputCoordinateCommnad : public CliCommand
    {
    private:
        OutputCoordinateCommnad();
        OutputCoordinateCommnad(const OutputCoordinateCommnad& command);
        ~OutputCoordinateCommnad();
        virtual const cqWCHAR* name() override;
		virtual const cqWCHAR* shortDescription() override;
		virtual void printHelp() override;
        virtual bool parseArgs(ArgParser* parser) override;
		void loadOmrp(const std::vector<uint32>& ids, std::map<uint32, DbMesh*>& meshes);

    public:
        static OutputCoordinateCommnad* instance();
    public:
		virtual int exec() override;

    private:
        CompilerOptions m_options;
		static OutputCoordinateCommnad g_sCommand;
    };
}

