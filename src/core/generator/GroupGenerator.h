#pragma once
#include "Generator.h"
namespace OMDB
{
	class GroupGenerator : public Generator
	{
	public:
		GroupGenerator(GeneratorData& data) :Generator(data) {};
	protected:
		virtual void generate(DbMesh* const pMesh) override;
		virtual void generateRelation(DbMesh* const pMesh, std::vector<DbMesh*>* nearby) override;

		void fixLanePaError(DbLink* const pLink);
	public:
		static void generateGroup(DbMesh* const pMesh, DbLink* pLink, DbRdLinkLanePa* lanePa);
	};
}

