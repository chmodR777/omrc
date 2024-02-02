#include "stdafx.h"
#include "RdsDataCheckCommand.h"
#include <fstream>
#include <queue>
#include "boost/filesystem.hpp"
#include "spatialite.h"
#include "cq_types_basic.h"
#include "tool_kit/call_stack_walker.h"
#include "sqlite3/sqlite3.h"
#include "Geometry.h"


using namespace RDS;

namespace OMDB
{



	RdsDataCheckCommand::RdsDataCheckCommand()
    {
    }

	RdsDataCheckCommand::RdsDataCheckCommand(const RdsDataCheckCommand& command)
	{
		command;
	}

	RdsDataCheckCommand::~RdsDataCheckCommand()
    {

    }

	const cqWCHAR* RdsDataCheckCommand::name()
	{
		return L"RdsCheck";
	}

	const cqWCHAR* RdsDataCheckCommand::shortDescription()
	{
		return L"check rds data";
	}

	void RdsDataCheckCommand::printHelp()
	{
		printf(
			R"(Synopsis:
			  %s compile [OPTIONS]
			Options:
			  --chinaRefMapFile <DIRECTORY>
			)",
			EXE_NAME);
	}

	bool RdsDataCheckCommand::parseArgs(ArgParser* parser)
	{
		ToString(parser->getArg(L"chinaRefMapFile"), m_options.sourceDir);

		const cqWCHAR* s = parser->getArg(L"thread_number");
		if (s != nullptr)
			m_options.threadNumber = cq_wtoi(s);

		return !parser->printUnknownArgs();
	}
	int RdsDataCheckCommand::exec()
	{
		CompileSetting* pSetting = CompileSetting::instance();
		IniFile iniFile;
		beginTitle("%s Version: %s", EXE_NAME, g_exeVersion);
		beginSubtitle("Loading omdb.ini");
		if (!iniFile.load(L"omdb.ini")) {
			printError("Failed to open `omdb.ini`.");
			return false;
		}
		if (!pSetting->load(&iniFile))
			return false;
		endSubtitle();
		if (!pSetting->applyCompilerOptionsAndCheck(&m_options)) {
			return false;
		}
		pSetting->printSettings();

		sqlite3* pSqlite3;
		std::string path = pSetting->chinaRefMapFile + "//" + "china_refmap.db";
		if (SQLITE_OK != sqlite3_open_v2(path.c_str(), &pSqlite3, SQLITE_OPEN_READONLY | SQLITE_OPEN_NOMUTEX | SQLITE_OPEN_SHAREDCACHE, nullptr)){
			sqlite3_close(pSqlite3);
			return false;
		}
		RDS::Reader* pReader = RDS::RdsReaderFactory::instance()->createReader(RDS::FormatType::BINARY);
		std::vector<RDS::RdsObject*> roads;
		//const char* sql = "SELECT * FROM Had WHERE id = ?";
		const char* sql = "SELECT id,data FROM Had";
		sqlite3_stmt* stmt;
		int status = sqlite3_prepare_v2(pSqlite3, sql, strlen(sql), &stmt, NULL);
		if (status != SQLITE_OK)
			return false;

		int count = 0;
		while (SQLITE_ROW == sqlite3_step(stmt))
		{
			int64 tmpMeshId = sqlite3_column_int64(stmt, 0);
			const unsigned char* pData = (const unsigned char*)sqlite3_column_blob(stmt, 1);
			int size = sqlite3_column_bytes(stmt, 1);
			if (size > 0)
			{
				RDS::RdsTile* pTile = pReader->Read((void*)pData, size);
				checkRdsTile(pTile);
				delete pTile;
			}
			count++;
			if (count % 5000 == 0)
			{
				std::cout << std::endl;
				std::cout << "count num: " << count << std::endl << std::endl;
			}
		}

		sqlite3_finalize(stmt);
		std::cout << "count num: " << count << std::endl << std::endl;

		return 0;
	}

	void RdsDataCheckCommand::checkRdsTile(RDS::RdsTile* pTile)
	{
		//checkGuardrail(pTile);
		checkRoadCavity(pTile);
		return;
	}

	void RdsDataCheckCommand::checkGuardrail(RDS::RdsTile* pTile)
	{
		auto convertLine = [&](const RDS::LineString3d& src, OMDB::LineString3d& dst)
		{
			dst.vertexes.resize(src.vertexes.size());
			memcpy(dst.vertexes.data(), src.vertexes.data(), sizeof(Point3d) * src.vertexes.size());
		};

		std::vector<RdsObject*> pRdsGuardrail = pTile->query(RDS::EntityType::RDS_GUARDRAIL);
		for (auto g : pRdsGuardrail)
		{
			RdsGuardrail* tmpGuardrail = (RdsGuardrail*)g;
			OMDB::LineString3d tmpGuardrailLine;
			convertLine(tmpGuardrail->location, tmpGuardrailLine);
			linestring_t originLine = LINESTRING_T(tmpGuardrailLine.vertexes);
			if (originLine.size() > 3)
			{
				for (size_t i = 1; i < originLine.size() - 1; ++i)
				{
					point_t pa = originLine.at(i - 1);
					point_t pb = originLine.at(i);
					point_t pc = originLine.at(i + 1);
					vector_t va = V3_N(S3_V3(pa, pb));
					vector_t vb = V3_N(S3_V3(pb, pc));
					if (bg::dot_product(va, vb) < -0.1)
					{
						if (bg::distance(pa, pb) > 5000 && bg::distance(pb, pc) > 5000)
							std::cout << "(" << std::to_string(pa.get<0>() / 1000) << ", " << std::to_string(pa.get<1>() / 1000) << ")" << std::endl;
					}
				}
			}
		}
	}

	void RdsDataCheckCommand::checkRoadCavity(RDS::RdsTile* pTile)
	{
		auto convertLine = [&](const RDS::LineString3d& src, OMDB::LineString3d& dst)
		{
			dst.vertexes.resize(src.vertexes.size());
			memcpy(dst.vertexes.data(), src.vertexes.data(), sizeof(Point3d) * src.vertexes.size());
		};

		auto convertPolygon = [&](const RDS::Polygon3d & src, OMDB::Polygon3d & dst)
		{
			dst.vertexes.resize(src.vertexes.size());
			memcpy(dst.vertexes.data(), src.vertexes.data(), sizeof(Point3d) * src.vertexes.size());
		};

		auto convertMultiLine = [&](const RDS::MultiLineString3d& src, OMDB::MultiLineString3d& dst)
		{
			int size = src.lines.size();
			dst.lines.resize(size);
			for (int i = 0; i < size; i++)
			{
				convertLine(src.lines[i], dst.lines[i]);
			}
		};

		RdsCheck::RoadGeometries tileRoadGeometries;
		std::vector<RdsObject*> pRdsRoad = pTile->query(RDS::EntityType::RDS_ROAD);
		for (auto g : pRdsRoad)
		{
			RdsRoad* tmpRoad = (RdsRoad*)g;
			OMDB::MultiLineString3d originRoadLines;
			convertMultiLine(tmpRoad->contour, originRoadLines);
			std::vector<linestring_t> tmpRoadLines;
			for (auto l : originRoadLines.lines)
			{
				tmpRoadLines.push_back(LINESTRING_T(l.vertexes));
			}

			if (tmpRoadLines.empty())
				continue;

			if (tmpRoadLines.size() == 1 && !tmpRoadLines.front().empty())
			{
				std::cout << "road line size equal 1 ---> " << "mesh id: " << pTile->meshId << "  " << "(" <<
					std::to_string(tmpRoadLines.front().front().get<0>() / 1000) << ", " <<
					std::to_string(tmpRoadLines.front().front().get<1>() / 1000) << ")" << std::endl;
				continue;
			}

			if (tmpRoadLines.size() > 2 && !tmpRoadLines.front().empty())
			{
				std::cout << "road line size greater than 2 ---> " << "mesh id: " << pTile->meshId << "  " << "(" <<
					std::to_string(tmpRoadLines.front().front().get<0>() / 1000) << ", " <<
					std::to_string(tmpRoadLines.front().front().get<1>() / 1000) << ")" << std::endl;
			}

			auto& leftRoadLine = tmpRoadLines.at(0);
			auto& rightRoadLine = tmpRoadLines.at(1);
			if (leftRoadLine.size() > 1)
			{
				if (!bg::equals(leftRoadLine.front(), rightRoadLine.front()))
				{
					tileRoadGeometries.stopLines.push_back(segment_t(leftRoadLine.front(), rightRoadLine.front()));
					tileRoadGeometries.stopLine2ds.push_back(segment_2t(P3_P2(leftRoadLine.front()), P3_P2(rightRoadLine.front())));
				}
				if (!bg::equals(leftRoadLine.back(), rightRoadLine.back()))
				{
					tileRoadGeometries.stopLines.push_back(segment_t(leftRoadLine.back(), rightRoadLine.back()));
					tileRoadGeometries.stopLine2ds.push_back(segment_2t(P3_P2(leftRoadLine.back()), P3_P2(rightRoadLine.back())));
				}
			}

			bg::reverse(tmpRoadLines.at(1));
			ring_t tmpRoadRing;
			tmpRoadRing.assign(tmpRoadLines.at(1).begin(), tmpRoadLines.at(1).end());
			tmpRoadRing.insert(tmpRoadRing.end(), tmpRoadLines.at(0).begin(), tmpRoadLines.at(0).end());
			bg::unique(tmpRoadRing);
			tileRoadGeometries.rings.push_back(tmpRoadRing);
			tileRoadGeometries.ids.push_back(tmpRoad->id);
		}
		for (auto& r : tileRoadGeometries.rings)
		{
			box_2t b2;
			auto r2 = R3_R2(r);
			bg::envelope(r2, b2);
			tileRoadGeometries.ring2ds.push_back(r2);
			tileRoadGeometries.ringBox2ds.push_back(b2);
		}
		for (auto& r : tileRoadGeometries.stopLine2ds)
		{
			box_2t b2;
			bg::envelope(r, b2);
			tileRoadGeometries.stopLineBox2ds.push_back(b2);
		}
		auto tileRoadRingsRtree = RdsCheck::makeRTree2ForRings(tileRoadGeometries);
		auto tileRoadSegmentsRtree = RdsCheck::makeRTree2ForSegments(tileRoadGeometries);

		std::vector<RdsObject*> pRdsDiversion = pTile->query(RDS::EntityType::RDS_DIVERSION);
		std::vector<ring_t> tileDiversionRings;
		for (auto g : pRdsDiversion)
		{
			RdsDiversion* tmpDiversion = (RdsDiversion*)g;
			OMDB::Polygon3d	originDiversionPolygon;
			convertPolygon(tmpDiversion->contour, originDiversionPolygon);
			ring_t tmpDiversionRing = RING_T(originDiversionPolygon.vertexes);
			if (tmpDiversionRing.empty())
				continue;
			bg::unique(tmpDiversionRing);
			tileDiversionRings.push_back(tmpDiversionRing);
		}

		for (size_t i = 0; i < tileRoadGeometries.stopLines.size(); ++i)
		{
			auto& tmpSegment = tileRoadGeometries.stopLines.at(i);
			std::vector<int> resultSegmentIds;
			std::vector<segment_t> resultSegments;
			tileRoadSegmentsRtree->query(bgi::intersects(tileRoadGeometries.stopLineBox2ds.at(i)),
				boost::make_function_output_iterator([&](size_t const& id) {
					if (std::addressof(tileRoadGeometries.stopLine2ds.at(id)) == std::addressof(tileRoadGeometries.stopLine2ds.at(i)))
						return;
					auto& tmpResultSegment = tileRoadGeometries.stopLines.at(id);
					point_t m1, m2;
					bg::centroid(tmpSegment, m1);
					bg::centroid(tmpResultSegment, m2);
					double tmpMidValue = bg::distance(m1, m2);
					if (tmpMidValue > 0.0 && tmpMidValue < 100.0)
					{
						resultSegmentIds.push_back(id);
						resultSegments.push_back(tileRoadGeometries.stopLines.at(id));

						std::cout << "车道组之间错位 ---> " << "mesh id: " << pTile->meshId << "  " << "(" <<
							std::to_string(resultSegments.back().first.get<0>() / 1000) << ", " <<
							std::to_string(resultSegments.back().first.get<1>() / 1000) << ")" << std::endl;
					}
					}));
		}
		
		//for (size_t i = 0; i < tileRoadGeometries.ringBox2ds.size(); ++i)
		//{
		//	auto& tmpRing2d = tileRoadGeometries.ring2ds.at(i);
		//	std::vector<int> resultRingIds;
		//	std::vector<ring_2t> resultRings;
		//	tileRoadRingsRtree->query(bgi::intersects(tileRoadGeometries.ringBox2ds.at(i)),
		//		boost::make_function_output_iterator([&](size_t const& id) {
		//			if (tileRoadGeometries.ids.at(id) == tileRoadGeometries.ids.at(i))
		//				return;
		//			std::vector<ring_2t> results;
		//			bg::intersection(tmpRing2d, tileRoadGeometries.ring2ds.at(id), results);
		//			if (!results.empty())
		//			{
		//				resultRingIds.push_back(id);
		//				resultRings.push_back(tileRoadGeometries.ring2ds.at(id));
		//			}
		//			}));
		//	if (!resultRings.empty() && !resultRings.front().empty())
		//	{
		//		std::cout << "车道组相交的情况 ---> " << "mesh id: " << pTile->meshId << "  " << "(" <<
		//			std::to_string(resultRings.front().front().get<0>() / 1000) << ", " <<
		//			std::to_string(resultRings.front().front().get<1>() / 1000) << ")" << std::endl;
		//	}
		//}

		return;
	}

	RdsDataCheckCommand RdsDataCheckCommand::g_sCommand;


	RdsDataCheckCommand* RdsDataCheckCommand::instance()
	{
		return &g_sCommand;
	}
}
