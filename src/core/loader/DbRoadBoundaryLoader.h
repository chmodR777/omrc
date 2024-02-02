#pragma once
#include "DataLoader.h"
#include "../record/DbRecord.h"
#include "DbRecordIterator.h"
namespace OMDB
{
    class DbRoadBoundaryLoader : public DataLoader
    {
    protected:
        virtual void    load() override;
        void            loadRoadBoundLink();
        void            loadRoadBoundBoundaryType();

        void            loadRoadBoundNode();
        void            loadLgRoadBoundRel();

    private:
        DbRecordIterator recordIterator;
    };


}

