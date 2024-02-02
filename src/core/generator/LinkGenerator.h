#pragma once
#include "Generator.h"
namespace OMDB
{
	class LinkGenerator : public Generator
	{
	public:
		LinkGenerator(GeneratorData& data) :Generator(data) {};
	protected:
		virtual void generate(DbMesh* const pMesh) override;
		virtual void generateRelation(DbMesh* const pMesh, std::vector<DbMesh*>* nearby) override;

		void generateLanePas(DbMesh* const pMesh);
		void generateGroups(DbMesh* const pMesh);
	};
}

