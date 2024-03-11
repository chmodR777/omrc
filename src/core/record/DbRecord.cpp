#include "stdafx.h"
#include "DbRecord.h"
#include "generator/Generator.h"
#include "algorithm/map_point_math.h"
#include <algorithm>

namespace OMDB
{

	DbDirectLink DbDirectLink::getDirectLink(DbLink* pLink, int direct)
	{
		if (direct != 2 && direct != 3) 
		{
			// 20169163 link 84208767637592940 direct = -99;
			if (pLink->direct != 1 && pLink->direct != -99)
			{
				throw std::invalid_argument("direct error");
			}
		}

		DbDirectLink directLink;
		directLink.link = pLink;
		directLink.direct = direct;
		directLink.dsegId = DSegmentId_getDSegmentId(pLink->uuid);
		if (pLink->direct == 3)
		{
			directLink.dsegId = DSegmentId_getReversed(directLink.dsegId);
		}
		return directLink;
	}

	int64 DbDirectLink::getStartNode()
	{
		return Generator::getDSegmentStartNode(link, dsegId);
	}

	int64 DbDirectLink::getEndNode()
	{
		return Generator::getDSegmentEndNode(link, dsegId);
	}

	std::vector<DbRdLinkLanePa*> DbDirectLink::getLanePas()
	{
		auto getRelLinkPair = [&](const DbRdLinkLanePa* lanePa)->std::pair<int64, DbRdLinkLanePa::DbRelLink> {
			std::pair<int64, DbHadLinkLanePa::DbRelLink> relLinkPair;
			for (auto& pair : lanePa->relLinks) {
				if (pair.second.relLinkid == link->uuid) {
					relLinkPair = pair;
					break;
				}
			}
			return relLinkPair;
		};

		std::vector<DbRdLinkLanePa*> grps;
		for (auto lanePa : link->lanePas) {
			for (auto& relLinkPair : lanePa->relLinks) {
				if (relLinkPair.second.relLinkid == link->uuid) {
					if (relLinkPair.second.directType == direct) {
						if (std::find(grps.begin(), grps.end(), lanePa) == grps.end()) {
							grps.push_back(lanePa);
							break;
						}
					}
				}
			}
		}

		std::sort(grps.begin(), grps.end(),
			[&](const DbRdLinkLanePa* first, const DbRdLinkLanePa* second)->bool {
				auto relLinkFirst = getRelLinkPair(first);
				auto relLInkSecond = getRelLinkPair(second);
				if (relLinkFirst.second.startOffset != relLInkSecond.second.startOffset)
					return relLinkFirst.second.startOffset < relLInkSecond.second.startOffset;
				return relLinkFirst.second.endOffset < relLInkSecond.second.endOffset;
			}
		);
		return grps;
	}

	std::vector<DbSkeleton*> DbDirectLink::getPrevious()
	{
		return Generator::getDSegmentPreviousLink(link, dsegId);
	}

	std::vector<DbSkeleton*> DbDirectLink::getNext()
	{
		return Generator::getDSegmentNextLink(link, dsegId);
	}


	void DbRdLinkLanePa::updateBoundingBox()
	{
		std::vector<int64> vx;
		std::vector<int64> vy;
		std::vector<int32> vz;
		for (auto& rb : roadBoundaries)
		{
			for_each(rb->geometry.vertexes.begin(), rb->geometry.vertexes.end(),
				[&](const MapPoint3D64& point)->void {
					vx.push_back(point.pos.lon);
					vy.push_back(point.pos.lat);
					vz.push_back(point.z);
				}
			);
		}

		if (!vx.empty())
		{
			std::sort(vx.begin(), vx.end());
			std::sort(vy.begin(), vy.end());
			std::sort(vz.begin(), vz.end());

			extent.min.pos.lon = vx[0];
			extent.min.pos.lat = vy[0];
			extent.min.z = vz[0];

			extent.max.pos.lon = *vx.rbegin();
			extent.max.pos.lat = *vy.rbegin();
			extent.max.z = *vz.rbegin();
		}
	}

}

