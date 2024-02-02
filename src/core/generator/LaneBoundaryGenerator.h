#pragma once
#include "Generator.h"
namespace OMDB
{
	class LaneBoundaryGenerator : public Generator
	{
	public:
		LaneBoundaryGenerator(GeneratorData& data) :Generator(data) {};
	protected:
		virtual void generate(DbMesh* const pMesh) override;
		virtual void generateRelation(DbMesh* const pMesh, std::vector<DbMesh*>* nearby) override;

	};
}

