#ifndef RDS_WRITER_H
#define RDS_WRITER_H

#include "navi_rds/Entity.h"
namespace RDS
{
    class Writer
    {
    public:
        Writer();
        ~Writer();

    public:
        void Write(const char* path,RdsTile* tile);
        void Write(RdsTile* pTile,void*& pData,unsigned long long& size);
    protected:
        virtual void write(const char* path, RdsTile* tile);
        virtual void write(RdsTile* pTile, void*& pData, unsigned long long& size);
    };

}
#endif