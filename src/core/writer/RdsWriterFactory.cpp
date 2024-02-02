#include "stdafx.h"
#include "RdsWriterFactory.h"
#include "SpatialiteWriter.h"
#include "MapboxWriter.h"
namespace RDS
{
	RdsWriterFactory::~RdsWriterFactory()
	{
		for (auto& pair : m_mapWriter)
		{
			if (pair.second != nullptr)
			{
				delete pair.second;
			}
		}
	}

	RdsWriterFactory::RdsWriterFactory()
	{
		m_mapWriter[FormatType::SPATIALITE] = new SpatialiteWriter();
		m_mapWriter[FormatType::BINARY] = new MapboxWriter();

	}

	Writer* RdsWriterFactory::createWriter(FormatType type)
	{
		if (m_mapWriter.find(type) != m_mapWriter.end())
		{
			return m_mapWriter[type];
		}
		return nullptr;
	}

	RdsWriterFactory* RdsWriterFactory::instance()
	{
		return &g_sInstanceFactory;
	}

	RdsWriterFactory RdsWriterFactory::g_sInstanceFactory;

}
