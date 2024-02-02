#pragma once
#include "Generator.h"
namespace OMDB
{
	class LaneGenerator : public Generator
	{
	public:
		LaneGenerator(GeneratorData& data) :Generator(data) {};
	protected:
		virtual void generate(DbMesh* const pMesh) override;
		virtual void generateRelation(DbMesh* const pMesh, std::vector<DbMesh*>* nearby) override;

	};
}

