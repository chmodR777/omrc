#pragma once
#include "stdafx.h"
#include "DirLinkPath.h"

namespace OMDB
{
namespace sd
{
	void DirLinkPath::Portion::reverse(double pathLength)
	{
		dirLink = dirLink.getReverseDirLink();
		accLength = pathLength - accLength;
	}

	std::size_t DirLinkPath::Portion::countVertexNum()
	{
		// 默认实现认为一个Portion完全包含一个DirLink
		return dirLink.link()->geometry.vertexes.size();
	}

	void DirLinkPath::addPortion(DirLink& dirLink)
	{
		Portion portion;
		portion.dirLink = dirLink;
		portion.accLength = m_portions.empty() ? dirLink.calLength() : m_portions.back().accLength + dirLink.calLength();
		addPortion(portion);
	}

	void DirLinkPath::reverse()
	{

		if (!m_portions.empty())
		{
			double pathLength = m_portions.back().accLength;
			std::reverse(m_portions.begin(), m_portions.end());
			for (auto& portion : m_portions)
				portion.reverse(pathLength);
		}
	}

	void DirLinkPath::appendPath(DirLinkPath& path)
	{
		double originPathLength = m_portions.empty() ? 0.0 : m_portions.back().accLength;
		for (Portion portion : path.m_portions)
		{
			portion.accLength += originPathLength;
			m_portions.push_back(portion);
		}
	}

	void DirLinkPath::accessVertices(VertexAccessor& accessor)
	{
		double accLength{ 0.0 };
		std::size_t vertexIndex{ 0 };
		std::size_t totalVertexNum = calTotalVertexNum();

		for (auto& portion : m_portions)
		{
			if (portion.dirLink.forward())
				DbLinkVertexAccessor(*portion.dirLink.link()).accessAllVerticesForward(vertexIndex, accLength, totalVertexNum, accessor);
			else
				DbLinkVertexAccessor(*portion.dirLink.link()).accessAllVerticesBackward(vertexIndex, accLength, totalVertexNum, accessor);
		}
	}

	std::size_t DirLinkPath::calTotalVertexNum()
	{
		std::size_t totalVertexNum{ 0 };

		for (auto& portion : m_portions)
		{
			totalVertexNum += portion.countVertexNum();
		}

		return totalVertexNum;
	}

	void DirLinkPath::DbLinkVertexAccessor::accessVerticesOrdered(int indexFrom, int indexTo, std::size_t& vertexIndex, double& accLength, const std::size_t totalVertexNum, DirLinkPath::VertexAccessor& accessor)
	{
		assert(indexFrom >= 0 && indexFrom < link->geometry.vertexes.size());
		assert(indexTo >= 0 && indexTo < link->geometry.vertexes.size());

		if (indexFrom < indexTo)
		{
			MapPoint3D64* prevPoint{ nullptr };

			for (int i = indexFrom; i <= indexTo; ++i)
			{
				MapPoint3D64& point = m_link.geometry.vertexes[i];
				if (prevPoint != nullptr)
				{
					accLength += MapPoint64::geodistance(point.pos, prevPoint->pos);
				}
				prevPoint = &point;

				accessor.access(point, DirLinkPath::VertexAccessor::AccessInfo{ vertexIndex++, totalVertexNum, accLength });
			}
		}
		else
		{
			MapPoint3D64* prevPoint{ nullptr };

			for (int i = indexFrom; i >= indexTo; --i)
			{
				MapPoint3D64& point = m_link.geometry.vertexes[i];
				if (prevPoint != nullptr)
				{
					accLength += MapPoint64::geodistance(point.pos, prevPoint->pos);

				}
				prevPoint = &point;

				accessor.access(point, DirLinkPath::VertexAccessor::AccessInfo{ vertexIndex++, totalVertexNum, accLength });
			}
		}
	}

}
}