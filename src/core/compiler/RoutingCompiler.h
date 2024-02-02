#pragma once
#include "Compiler.h"
#include <unordered_map>
#include "algorithm/line_projection_triangle_surface.h"

namespace OMDB
{
	class RoutingCompiler : public Compiler
	{
	public:
		RoutingCompiler(CompilerData& data) :Compiler(data) {};
	protected:
		virtual void compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile) override;

	protected:
		std::unordered_map<uint64, unsigned char> isTunnelLinkMap;
		std::unordered_map<uint64, unsigned char> isTunnelLinkPAMap;

		std::unordered_map<uint64, unsigned char> isLowerCameraLinkMap;

	};
}