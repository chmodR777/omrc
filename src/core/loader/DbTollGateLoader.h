#pragma once
#include "DataLoader.h"
#include "DbRecordIterator.h"
namespace OMDB
{
    class DbTollGateLoader : public DataLoader
    {
    public:
        void LoadLinkCardTypes(sqlite3* pDb);
        static std::map<int64, int> linkCardTypes;
    protected:
        virtual void    load() override;

        void loadTollGate();
        void loadTollGateType();
        void loadTollGatePassage();
        void loadTollGatePayMethod();
        void loadTollGatePayCardType();
        void loadTollGateName();

    private:
        DbRecordIterator recordIterator;
    };


}

