#pragma once

#include "stdafx.h"
#include "DirLink.h"
#include "core/generator/Generator.h"
#include "algorithm/map_point_math.h"

#include <cmath>

namespace OMDB
{
namespace sd
{

	DirLink::DirLink() : m_linkuuid(0), m_forward(true), m_dsegID(INVALID_DSEGMENT_ID), m_link(nullptr)
	{
	}

	DirLink::DirLink(DbLink* link, bool forward) :
		m_linkuuid(link->uuid), m_forward(forward), m_dsegID(static_cast<DSegmentId>(forward)), m_link(link)
	{
		DSegmentId_setSegmentId(m_dsegID, link->uuid);
		if (link->direct == 3)
			m_dsegID = DSegmentId_getReversed(m_dsegID);
	}

	double DirLink::startHeadingAngle()
	{
		MapPoint64 origin = getPointRefInDir(1).pos;
		MapPoint64 point = getPointRefInDir(0).pos;

		return map_pt_math::twoPointAngle(origin, point);
	}

	double DirLink::endHeadingAngle()
	{
		MapPoint64 origin = getPointRefInDir(m_link->geometry.vertexes.size() - 2).pos;
		MapPoint64 point = getPointRefInDir(m_link->geometry.vertexes.size() - 1).pos;

		return map_pt_math::twoPointAngle(origin, point);
	}

	double DirLink::getStartHeight()
	{
		if (m_forward)
			return m_link->geometry.vertexes.front().z;
		else
			return m_link->geometry.vertexes.back().z;
	}

	double DirLink::getEndHeight()
	{
		if (m_forward)
			return m_link->geometry.vertexes.back().z;
		else
			return m_link->geometry.vertexes.front().z;
	}

	double DirLink::calLength() const
	{
		int linkVertexNum = static_cast<int>(m_link->geometry.vertexes.size());
		double len = 0.0;
		for (int i = 0; i < linkVertexNum - 1; ++i)
		{
			auto& point = m_link->geometry.vertexes.at(i);
			auto& nextPoint = m_link->geometry.vertexes.at(i + 1);
			len += MapPoint64::geodistance(point.pos, nextPoint.pos);
		}
		return len;
	}

	double DirLink::calPortionLength(int indexFrom, int indexTo) const
	{
		int linkVertexNum = static_cast<int>(m_link->geometry.vertexes.size());
		if (indexFrom < 0
			|| indexFrom >= linkVertexNum
			|| indexTo < 0 || indexFrom >= linkVertexNum)
		{
			assert(false); // TODO: 使用其它提示方式来表明这是一个错误。
			return -1.0;
		}

		double len = 0.0;
		for (int i = indexFrom; i < indexTo; ++i)
		{
			auto& point = m_link->geometry.vertexes.at(i);
			auto& nextPoint = m_link->geometry.vertexes.at(i + 1);
			len += MapPoint64::geodistance(point.pos, nextPoint.pos);
		}

		for (int i = indexFrom; i > indexTo; --i)
		{
			auto& point = m_link->geometry.vertexes.at(i);
			auto& nextPoint = m_link->geometry.vertexes.at(i - 1);
			len += MapPoint64::geodistance(point.pos, nextPoint.pos);
		}

		return len;
	}

	std::vector<DirLink> DirLink::getEndDirectOutlinks()
	{
		const std::vector<DbLink*>* dSegsOut{ nullptr };

		DSegmentId tSegmentId = DSegmentId_getDSegmentId(m_link->uuid);
		if (m_link->direct != 1) {
			if (m_dsegID == tSegmentId) { // 正向
				if (m_link->direct == 2) {
					dSegsOut = &m_link->endNodePtr->links;
				}
				if (m_link->direct == 3) {
					dSegsOut = &m_link->startNodePtr->links;
				}
			}
			if (m_dsegID == DSegmentId_getReversed(tSegmentId)) { // 反向
				if (m_link->direct == 2) {
					dSegsOut = &m_link->startNodePtr->links;
				}
				if (m_link->direct == 3) {
					dSegsOut = &m_link->endNodePtr->links;
				}
			}
		}
		else {
			// 假定1的默认方向为正向
			if (m_dsegID == tSegmentId) { // 正向
				dSegsOut = &m_link->endNodePtr->links;
			}

			if (m_dsegID == DSegmentId_getReversed(tSegmentId)) { // 反向
				dSegsOut = &m_link->startNodePtr->links;
			}
		}

		// 生成OutDirLink
		if (dSegsOut)
		{
			std::vector<DirLink> outDirLinks;
			outDirLinks.reserve(dSegsOut->size());

			for (DbLink* outlink : *dSegsOut)
			{
				DSegmentId dsegID = Generator::getNextDSegId(m_link, m_dsegID, outlink);
				bool forward = Generator::getDSegmentDir(outlink, dsegID);
				outDirLinks.emplace_back(outlink, forward);
			}
			return outDirLinks;
		}
		else
		{
			// should never reach here
			return std::vector<DirLink>{};
		}
	}

	std::vector<int64> DirLink::getEndDirectOutlinksUUID()
	{
		std::vector<DirLink> outlinks = getEndDirectOutlinks();
		std::vector<int64> outlinksUUID(outlinks.size());
		for (std::size_t i = 0; i < outlinks.size(); ++i)
		{
			outlinksUUID[i] = outlinks[i].link()->uuid;
		}
		return outlinksUUID;
	}

	std::vector<DirLink> DirLink::getEndDirectOutlinksExceptSelf()
	{
		// 从所有oultinks中移除当前link
		std::vector<DirLink> outlinks = this->getEndDirectOutlinks();
		auto selfIter = std::find_if(outlinks.begin(), outlinks.end(), [this](DirLink& link) { return this->rawLinkEqual(link); });
		if (selfIter != outlinks.end())
		{
			outlinks.erase(selfIter);
		}
		return outlinks;
	}

	std::vector<int64> DirLink::getEndDirectOutlinksExceptSelfUUID()
	{
		std::vector<DirLink> outlinks = getEndDirectOutlinksExceptSelf();
		std::vector<int64> outlinksUUID(outlinks.size());
		for (std::size_t i = 0; i < outlinks.size(); ++i)
		{
			outlinksUUID[i] = outlinks[i].link()->uuid;
		}
		return outlinksUUID;
	}

}
}