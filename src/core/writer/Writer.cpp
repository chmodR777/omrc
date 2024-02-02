#include "stdafx.h"
#include "Writer.h"
namespace RDS
{

    Writer::Writer()
    {

    }

    Writer::~Writer()
    {

    }

    void Writer::Write(const char* path, RdsTile* tile)
    {
        write(path, tile);
    }

	void Writer::Write(RdsTile* pTile, void*& pData, unsigned long long& size)
	{
        write(pTile, pData, size);
	}

	void Writer::write(RdsTile* pTile, void*& pData, unsigned long long& size)
	{
        pTile;
        pData;
        size;
	}

	void Writer::write(const char* path, RdsTile* tile)
	{
        path;
        tile;
	}

}


