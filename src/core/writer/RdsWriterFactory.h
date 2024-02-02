#pragma once

#include "Writer.h"
#include <map>
namespace RDS
{
	class RdsWriterFactory
	{
	private:
		RdsWriterFactory();
		~RdsWriterFactory();
		RdsWriterFactory(const RdsWriterFactory& factory) = delete;
	public:
		Writer* createWriter(FormatType type);
		static RdsWriterFactory* instance();

	private:
		std::map<FormatType, Writer*> m_mapWriter;
		static RdsWriterFactory g_sInstanceFactory;
	};
}

