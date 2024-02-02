#pragma once
#include "DataLoader.h"
#include "DbRecordIterator.h"
namespace OMDB
{
    class DbLaneLoader : public DataLoader
    {
    protected:
        virtual void    load() override;
        void            loadLaneLink();
        void            loadLaneLinkGeo();
        
        void            loadLaneLinkLg();
        void            loadLaneLinkCondition();
        void            loadLaneNode();
		void            loadLaneSpeedLimit();
        void            loadLaneLinkTurnwaiting();
    private:
        DbRecordIterator recordIterator;
    };
}

