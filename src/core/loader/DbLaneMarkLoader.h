#pragma once
#include "DataLoader.h"
#include "DbRecordIterator.h"
namespace OMDB
{
    class DbLaneMarkLoader : public DataLoader
    {
    protected:
        virtual void    load() override;
        void            loadLaneMarkLink();
        void            loadLaneMarkMarkingGeo();
        void            loadLaneMarkBoundaryType();
        
        void            loadLaneMarkTraversal();
		void            loadLaneMarkNode();
		void            loadLaneMarkRel();
		void            loadLgMarkRel();
		void            loadLaneMarkPA();
		void            loadLaneMarkPAValue();
    private:
        DbRecordIterator recordIterator;
    };
}

