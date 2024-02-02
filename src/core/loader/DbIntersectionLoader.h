#pragma once
#include "DataLoader.h"
#include "DbRecordIterator.h"

namespace OMDB
{
	class DbIntersectionLoader : public DataLoader
	{
	protected:
		virtual void load() override;
		void loadIntersection();
		void loadIntersectionType();
		void loadIntersectionLG();
		void loadIntersectionLanePA();
		void loadIntersectionMesh();
		void loadIntersectionLink();
		void loadIntersectionNode();
		void loadIntersectionPoint();

	private:
		DbRecordIterator recordIterator;
	};
}


