#pragma once
#include "Generator.h"
namespace OMDB
{
	class RoadGenerator : public Generator
	{
	public:
		RoadGenerator(GeneratorData& data) :Generator(data) {};
	protected:
		virtual void generate(DbMesh* const pMesh) override;
		virtual void generateRelation(DbMesh* const pMesh, std::vector<DbMesh*>* nearby) override;

		void mergeLanePas();
		void mergeLanePa(DbRdLinkLanePa* pLanePa);

	private:
		DbMesh* m_mesh = nullptr;
		std::vector<DbMesh*>* m_nearbyMesh = nullptr;
	};
}

