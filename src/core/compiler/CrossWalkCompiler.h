 #pragma once
 #include "Compiler.h"
 namespace OMDB
 {
     class CrossWalkCompiler : public Compiler
     {
	 public:
         CrossWalkCompiler(CompilerData& data) :Compiler(data) {};
     protected:
         virtual void compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile) override;
     };
 }
 
