#pragma once
#include "DataLoader.h"
#include "DbRecordIterator.h"
namespace OMDB
{
    class DbObjectLoader : public DataLoader
    {
    protected:
        virtual void    load() override;

        void loadBarrier();
        void loadWall();
        void loadText();
        void loadArrow();
        void loadCrossWalk();
        void loadFillArea();
        void loadTrafficSign();
        void loadStopLocation();
        void loadTrafficLights();
        void loadPole();

        void loadLgRel(const char* tableName, DbObjectType objectType);
        void loadLaneLinkRel(const char* tableName, DbObjectType objectType);
        void loadObjectRel();
    private:
        DbRecordIterator recordIterator;
    };


}

