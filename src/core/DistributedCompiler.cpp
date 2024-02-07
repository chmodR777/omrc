#include "stdafx.h"
#include "DistributedCompiler.h"

#include "generator/ZGenerator.h"
#include "generator/LinkGenerator.h"
#include "generator/GroupGenerator.h"
#include "generator/RoadBoundaryGenerator.h"
#include "generator/IntersectionGenerator.h"
#include "generator/LaneBoundaryGenerator.h"
#include "generator/LaneGenerator.h"
#include "processor/LinkProcessor.h"
#include "processor/LinkGroupProcessor.h"
#include "processor/LinkGroupAssociationProcessor.h"
#include "processor/LaneProcessor.h"
#include "processor/LaneBoundaryProcessor.h"
#include "processor/RoadBoundaryProcessor.h"
#include "processor/ObjectProcessor.h"
#include "processor/IntersectionProcessor.h"
#include "processor/TollGateProcessor.h"
#include "compiler/GroupCompiler.h"
#include "compiler/RoadCompiler.h"
#include "compiler/LineCompiler.h"
#include "compiler/GuardrailCompiler.h"
#include "compiler/MarkingCompiler.h"
//#include "compiler/Sign3dCompiler.h"
#include "compiler/SpeedLimitBoardCompiler.h"
#include "compiler/CrossWalkCompiler.h"
#include "compiler/DiversionCompiler.h"
#include "compiler/GreenbeltCompiler.h"
#include "compiler/GreenbeltUrbanCompiler.h"
#include "compiler/PierCompiler.h"
#include "compiler/TextCompiler.h"
#include "compiler/TollGateCompiler.h"
#include "compiler/IntersectionCompiler.h"
#include "compiler/GradeSeparationCompiler.h"
#include "compiler/GroupSemiTransparentCompiler.h"
#include "compiler/RoutingCompiler.h"
#include "compiler/TunnelCompiler.h"
#include "framework/FileManager.h"
#include "framework/MeshCache.h"
#include <boost/filesystem.hpp>
#include "writer/Writer.h"
#include "writer/RdsWriterFactory.h"
#include "writer/RdsDatabaseWriter.h"
#include "CompileSetting.h"
#include <mutex>
#include <boost/filesystem.hpp>
#include "tool_kit/call_stack_walker.h"
#include "framework/TopoBuilder.h"
#include "framework/SpatialSeacher.h"
#include "boost/filesystem/fstream.hpp"
#include <boost/date_time/posix_time/posix_time.hpp>
#include "compiler/TrafficLightsCompiler.h"

#include "compiler/TransparentCompiler.h"

#include "compiler/Compiler.h"

//#define USE_SPATIALITE
// #define DEBUG_DATA_SIZE
namespace OMDB
{
	void DistributedCompiler::execute()
	{
		MeshCache cache;
		std::vector<uint32> ids = FileManager::instance()->idList();
// 		ids.clear();
// 		std::ifstream infile;
// 		infile.open("id.txt", std::ios::in);
// 		if (!infile.is_open())
// 		{
// 			std::cout << "读取文件失败" << std::endl;
// 			return;
// 		}
// 		std::string s;
// 		while (getline(infile, s))
// 		{
// 			infile >> s;
// 			ids.push_back(atoi(s.c_str()));
// 		}
// 		infile.close();
		std::sort(ids.begin(), ids.end());

// 		std::vector<uint32> v1;
// 		size_t length = ids.size() / 64;
// 		v1.assign(ids.begin() + length*25, ids.begin() + length*26);
// 		ids.clear();
// 		ids.assign(v1.begin(), v1.end());

		//ids.clear();
		//ids.push_back(20596316);

		CompileSetting* pSetting = CompileSetting::instance();
		if (pSetting->writeSpatialiteFileMeshId != 0)
		{
			ids.clear();
			ids.push_back(pSetting->writeSpatialiteFileMeshId);
		}
		size_t capacity = 1000;
		if(pSetting->capacityNumber > 0)
		{
			capacity = pSetting->capacityNumber;
		}
		size_t size = ids.size();
		size_t number = size / capacity + (size%capacity == 0 ? 0: 1);
		// 初始化线程
		unsigned int threadNumber = std::thread::hardware_concurrency();
		if (pSetting->threadNumber > 0)
		{
			threadNumber = min(pSetting->threadNumber, threadNumber);
		}
		std::string directory = pSetting->outputDir;
		if (!boost::filesystem::exists(directory))
		{
			boost::filesystem::create_directory(directory);
		}

		std::string path(directory + "\\china_refmap.db");
		if (!pSetting->isWriteSpatialite)
		{
			if (boost::filesystem::exists(path))
			{
				boost::filesystem::remove(path);
			}
		}

		printInfo("----------------------------------------------------");
		printInfo("开始编译，网格数量：%d",ids.size());
		auto begin = std::chrono::steady_clock::now();

#ifdef DEBUG_DATA_SIZE
		std::map<uint32, std::map<uint32, Blob>> total;
		for (int m = 0;m < 10;m++)
#endif
		{
			int progress = 0;
			for (int i = 0; i < number; i++)
			{
				std::vector<uint32> v;
				size_t start = i * capacity;
				size_t end = (i + 1) * capacity;
				if (end < size)
				{
					v.assign(ids.begin() + start, ids.begin() + end);
				}
				else
				{
					v.assign(ids.begin() + start, ids.end());
				}
				cache.reset(v);

				std::map<uint32, Blob> data;
				for_each(v.begin(), v.end(), [&](uint32 id)->void {data[id] = Blob(); });

				std::queue <uint32 > tasks;
				for (auto tmpId : v)
				{
					DbMesh* tmpMesh;
					cache.query(tmpId, tmpMesh);
					if (tmpMesh == nullptr)
						continue;
					if (!pSetting->isGenerateHdData &&
						tmpMesh->query(RecordType::DB_HAD_LANE_MARK_LINK).empty() &&
						tmpMesh->query(RecordType::DB_HAD_ROAD_BOUNDARY_LINK).empty())
						continue;
					tasks.push(tmpId);
				}
				Spinlock spinlock;
				auto threadFun = [&] {
					while (true)
					{
						uint32 id;
						try
						{
							{
								std::lock_guard<Spinlock> lock(spinlock);
								if (tasks.empty())
									break;

								int npg = (int)((v.size() - tasks.size() + i * capacity) * 1.0 / size * 100.0);
								if (npg > progress)
								{
									printInfo("[%d%%] compile omdb.", npg);
									progress = npg;
								}
								id = tasks.front();

								// printInfo("meshid : %d", id);
								tasks.pop();
							}
							DbMesh* pCenterMesh;
							std::vector<DbMesh*> pNearbyMesh;
							std::map<DbMesh*, HadGrid*> nearbyGridMap;
							cache.query(id, pCenterMesh, pNearbyMesh);
							if (pCenterMesh == nullptr)
								continue;

							// generate
							if (pSetting->isGenerateHdData)
							{
								generate(pCenterMesh, &pNearbyMesh);
							}

							// process db to grid
							for (auto& pMesh : pNearbyMesh)
							{
								if (pMesh != nullptr)
								{
									HadGrid* nearbyGrid = new HadGrid();
									process(pMesh, nearbyGrid);
									nearbyGridMap.emplace(pMesh, nearbyGrid);
								}
							}

							HadGrid* pCenterGrid = new HadGrid();
							process(pCenterMesh, pCenterGrid);

							// process grid's relation
							for (auto& pMesh : pNearbyMesh)
							{
								if (pMesh != nullptr)
								{
									std::vector<HadGrid*> nearby;
									nearby.push_back(pCenterGrid);
									for (auto& pair : nearbyGridMap) {
										if (pMesh != pair.first) {
											nearby.push_back(pair.second);
										}
									}
									HadGrid* nearbyGrid = nearbyGridMap[pMesh];
									processRelation(pMesh, nearbyGrid, &nearby);
								}
							}

							std::vector<HadGrid*> nearby;
							for_each(nearbyGridMap.begin(), nearbyGridMap.end(), [&](auto& p)->void {nearby.push_back(p.second); });
							processRelation(pCenterMesh, pCenterGrid, &nearby);

							RdsTile* pTile = new RdsTile();
							compile(pCenterGrid, nearby, pTile);

							delete pCenterGrid;
							for (auto p : nearby)
							{
								delete p;
							}

							if (pSetting->isWriteSpatialite)
							{
								boost::filesystem::path output(directory + "\\" + std::to_string(MeshId_toNdsGridId(pTile->meshId)) + ".rds");
								if (boost::filesystem::exists(output))
									boost::filesystem::remove(output);
								RDS::Writer* pWriter = RDS::RdsWriterFactory::instance()->createWriter(RDS::FormatType::SPATIALITE);
								pWriter->Write(output.string().c_str(), pTile);
							}
							else
							{
								RDS::Writer* pWriter = RDS::RdsWriterFactory::instance()->createWriter(RDS::FormatType::BINARY);
								pWriter->Write(pTile, data[id].data, data[id].size);
							}

#ifdef DEBUG_DATA_SIZE
							total[m] = data;
#endif
							delete pTile;
						}
						catch (CallStackWalker& walker)
						{
							walker.showCallstack();
							walker.showExceptionInfo();
						}
						catch (std::exception& ex)
						{
							printError("Catch C++ exception : %s", ex.what());
							printError("mesh_id: %d", id);
						}
					}
				};

				std::vector<std::thread> workers;
				workers.resize(threadNumber);

				for (unsigned int i = 0; i < threadNumber; i++)
				{
					workers[i] = std::move(std::thread(threadFun));
				}
				for_each(workers.begin(), workers.end(), [](std::thread& thread)->void {
					if (thread.joinable())
					{
						thread.join();
					}
					});

				if (!pSetting->isWriteSpatialite)
				{
					writeBinaryMesh(path.c_str(), data);
					data.clear();
				}
			}

			printInfo("[%d%%] compile omdb.", 100);

			if (!pSetting->isWriteSpatialite)
			{
				writeBinaryHeader(path.c_str(), ids);
				printInfo("输出文件地址：%s", path.c_str());
			}
		}

#ifdef DEBUG_DATA_SIZE
		std::ofstream txt("mesh.txt");
		std::map<uint32, Blob> dt = total[0];
		for (auto& pair : dt)
		{
			txt << pair.first << "|" << pair.second.size;
			for (int i = 1; i < 10; i++)
			{
				std::map<uint32, Blob>& tp = total[i];
				txt << "," << tp[pair.first].size;
			}
			txt << std::endl;
		}
		txt.flush();
		txt.close();
#endif

		printInfo("编译数据结束!");
		double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
		printInfo("总耗时：%f", seconds);
	}

	void DistributedCompiler::compile(HadGrid* pGrid,std::vector<HadGrid*>& nearby, RDS::RdsTile* pTile)
	{
		pTile->meshId = MeshId_toNdsGridId(pGrid->getId());
		pTile->latitude = pGrid->getBoundingbox2d().min.lat;
		pTile->longitude = pGrid->getBoundingbox2d().min.lon;
		CompilerData compilerData;
		GroupCompiler(compilerData).Compile(pGrid,nearby, pTile);
		IntersectionCompiler(compilerData).Compile(pGrid, nearby, pTile);
		RoadCompiler(compilerData).Compile(pGrid, nearby, pTile);
		LineCompiler(compilerData).Compile(pGrid, nearby, pTile);
 		DiversionCompiler(compilerData).Compile(pGrid, nearby,pTile);
		GuardrailCompiler(compilerData).Compile(pGrid, nearby, pTile);
		MarkingCompiler(compilerData).Compile(pGrid, nearby, pTile);
 		SpeedLimitBoardCompiler(compilerData).Compile(pGrid, nearby, pTile);	//顺序依赖GuardrailCompiler()完成。
		CrossWalkCompiler(compilerData).Compile(pGrid, nearby, pTile);
		GreenbeltCompiler(compilerData).Compile(pGrid, nearby, pTile);
		GreenbeltUrbanCompiler(compilerData).Compile(pGrid, nearby, pTile);
		PierCompiler(compilerData).Compile(pGrid, nearby, pTile);
		TextCompiler(compilerData).Compile(pGrid, nearby, pTile);
		TollGateCompiler(compilerData).Compile(pGrid, nearby, pTile);
		TunnelCompiler(compilerData).Compile(pGrid, nearby, pTile);
// 		GradeSeparationCompiler(compilerData).Compile(pGrid, nearby, pTile);
		TrafficLightsCompiler(compilerData).Compile(pGrid, nearby, pTile);
		if (CompileSetting::instance()->isOutputRoutingData)
		{
			RoutingCompiler(compilerData).Compile(pGrid, nearby, pTile);
		}
		GroupSemiTransparentCompiler(compilerData).Compile(pGrid, nearby, pTile);
		if (CompileSetting::instance()->isCompileTransparency)
		{
			TransparentCompiler(compilerData).Compile(pGrid, nearby, pTile);
		}
		
	}

	void DistributedCompiler::generate(DbMesh* pMesh, std::vector<DbMesh*>* nearby)
	{
		std::vector<DbMesh*> meshes;
		for_each((*nearby).begin(), (*nearby).end(), [&](DbMesh* m)->void { meshes.push_back(m); });
		meshes.push_back(pMesh);
		auto generate = [](DbMesh* pMesh) {
			if (pMesh != nullptr)
			{
				GeneratorData generatorData;
				LinkGenerator(generatorData).Generate(pMesh);
				ZGenerator(generatorData).Generate(pMesh);
				GroupGenerator(generatorData).Generate(pMesh);
				RoadBoundaryGenerator(generatorData).Generate(pMesh);
				IntersectionGenerator(generatorData).Generate(pMesh);
				LaneBoundaryGenerator(generatorData).Generate(pMesh);
				LaneGenerator(generatorData).Generate(pMesh);
			}
		};
		for (auto& mesh : meshes)
		{
			generate(mesh);
		}

		GeneratorData generatorData;
		LinkGenerator(generatorData).GenerateRelation(pMesh, nearby);
		ZGenerator(generatorData).GenerateRelation(pMesh, nearby);
		GroupGenerator(generatorData).GenerateRelation(pMesh, nearby);
		RoadBoundaryGenerator(generatorData).GenerateRelation(pMesh, nearby);
		IntersectionGenerator(generatorData).GenerateRelation(pMesh, nearby);
		LaneBoundaryGenerator(generatorData).GenerateRelation(pMesh, nearby);
		LaneGenerator(generatorData).GenerateRelation(pMesh, nearby);
	}

	void DistributedCompiler::process(DbMesh* pMesh, HadGrid* pGrid)
	{
		pGrid->setId(pMesh->getId());
		LinkProcessor().Process(pMesh, pGrid);
		LinkGroupProcessor().Process(pMesh, pGrid);
		LaneProcessor().Process(pMesh, pGrid);
		ObjectProcessor().Process(pMesh, pGrid);
		LaneBoundaryProcessor().Process(pMesh, pGrid);
		RoadBoundaryProcessor().Process(pMesh, pGrid);
		IntersectionProcessor().Process(pMesh, pGrid);
		TollGateProcessor().Process(pMesh, pGrid);

		// 建立topo关系
		TopoBuilder::buildTopo(pGrid);
	}

	void DistributedCompiler::processRelation(DbMesh* pMesh, HadGrid* pGrid, std::vector<HadGrid*>* nearby)
	{
		LinkProcessor().ProcessRelation(pMesh, pGrid, nearby);
		LinkGroupProcessor().ProcessRelation(pMesh, pGrid, nearby);
		LaneProcessor().ProcessRelation(pMesh, pGrid, nearby);
		ObjectProcessor().ProcessRelation(pMesh, pGrid, nearby);
		LaneBoundaryProcessor().ProcessRelation(pMesh, pGrid, nearby);
		RoadBoundaryProcessor().ProcessRelation(pMesh, pGrid, nearby);
		IntersectionProcessor().ProcessRelation(pMesh, pGrid, nearby);
		TollGateProcessor().ProcessRelation(pMesh, pGrid, nearby);

		auto makeBoundingbox = [](HadLaneGroup* pGroup) -> void
		{
			std::vector<int64> vx;
			std::vector<int64> vy;
			std::vector<int32> vz;
			if (pGroup->roadBoundaries.empty())
			{
				if (!pGroup->laneBoundaries.empty())
				{
					size_t size = pGroup->laneBoundaries.size();
					for_each(pGroup->laneBoundaries[0]->location.vertexes.begin(), pGroup->laneBoundaries[0]->location.vertexes.end(),
						[&](const MapPoint3D64& point)->void {
							vx.push_back(point.pos.lon);
							vy.push_back(point.pos.lat);
							vz.push_back(point.z);
						}
					);

					for_each(pGroup->laneBoundaries[size - 1]->location.vertexes.begin(), pGroup->laneBoundaries[size - 1]->location.vertexes.end(),
						[&](const MapPoint3D64& point)->void {
							vx.push_back(point.pos.lon);
							vy.push_back(point.pos.lat);
							vz.push_back(point.z);
						}
					);
				}
			}
			else
			{
				for (auto& rb : pGroup->roadBoundaries)
				{
					for_each(rb->location.vertexes.begin(), rb->location.vertexes.end(),
						[&](const MapPoint3D64& point)->void {
							vx.push_back(point.pos.lon);
							vy.push_back(point.pos.lat);
							vz.push_back(point.z);
						}
					);
				}
			}

			if (!vx.empty())
			{
				std::sort(vx.begin(), vx.end());
				std::sort(vy.begin(), vy.end());
				std::sort(vz.begin(), vz.end());

				pGroup->extent.min.pos.lon = vx[0];
				pGroup->extent.min.pos.lat = vy[0];
				pGroup->extent.min.z = vz[0];

				pGroup->extent.max.pos.lon = *vx.rbegin();
				pGroup->extent.max.pos.lat = *vy.rbegin();
				pGroup->extent.max.z = *vz.rbegin();
			}

		};
		for (auto& pElement : pGrid->query(ElementType::HAD_LANE_GROUP))
		{
			HadLaneGroup* pGroup = (HadLaneGroup*)pElement;
			makeBoundingbox(pGroup);
		}

		// 建立跨网格topo关系
		TopoBuilder::buildTopoCrossGrid(pGrid, nearby);

		// LA关系依赖SpatialSeacher,最后执行
		LinkGroupAssociationProcessor().ProcessRelation(pMesh, pGrid, nearby);
	}

	void DistributedCompiler::writeBinaryHeader(const char* path, const std::vector<uint32>& ids)
	{
		std::ifstream f(path);
		f.open(path, std::ios::_Nocreate);
		sqlite3* pDb = nullptr;
		int status = sqlite3_open_v2(path, &pDb, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, NULL);
		if (status != SQLITE_OK)
		{
			sqlite3_close(pDb);
			return;
		}
		sqlite3_exec(pDb, "PRAGMA synchronous = OFF", 0, 0, 0);
		std::vector<uint32> gridIdList;
		for_each(ids.begin(), ids.end(), [&](uint32 id)->void {gridIdList.push_back(MeshId_toNdsGridId(id)); });
		std::string encodedGridList = RdsDatabaseWriter::base64EncodedGridList(gridIdList);
		boost::posix_time::ptime currentTime = boost::posix_time::second_clock::local_time();
		std::string currentTimeStr = boost::posix_time::to_iso_extended_string(currentTime);
		std::map<std::string, std::string> headers =
		{
			{"compilerVersion","1.0"},
			{"compileTime", currentTimeStr},
			{"formatType","binary"},
			{"formatVersion","2.1.0"},
			{"dataVersion", CompileSetting::instance()->dataVersion},
			{"gridShifted","1"},
			{"gridList",encodedGridList}
		};
		const char* sql = "CREATE TABLE IF NOT EXISTS __Had_meta__ ("
			"key				  TEXT                 NOT NULL,"
			"value			  TEXT);";

		char* msg;
		status = sqlite3_exec(pDb, sql, NULL, 0, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "BEGIN";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}
		sqlite3_stmt* stmt;
		sql = "INSERT INTO __Had_meta__ (key, value) VALUES (?,?)";
		status = sqlite3_prepare_v2(pDb, sql, strlen(sql), &stmt, NULL);
		if (status != SQLITE_OK)
			return;

		for (auto& pair : headers)
		{
			sqlite3_reset(stmt);
			sqlite3_clear_bindings(stmt);
			sqlite3_bind_text(stmt, 1, pair.first.c_str(), pair.first.length(), nullptr);
			sqlite3_bind_text(stmt, 2, pair.second.c_str(), pair.second.length(), nullptr);
			status = sqlite3_step(stmt);
		}

		sqlite3_finalize(stmt);
		sql = "COMMIT";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}
		sqlite3_close(pDb);
	}

	void DistributedCompiler::writeBinaryMesh(const char* path, const std::map<uint32,Blob>& data)
	{
		std::ifstream f(path);
		f.open(path, std::ios::_Nocreate);
		sqlite3* pDb = nullptr;
		int status = sqlite3_open_v2(path, &pDb, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, NULL);
		if (status != SQLITE_OK)
		{
			sqlite3_close(pDb);
			return;
		}
		sqlite3_exec(pDb, "PRAGMA synchronous = OFF", 0, 0, 0);

		const char* sql = "CREATE TABLE IF NOT EXISTS Had ("
			"id				  INTEGER                 NOT NULL,"
			"data			  BLOB);";

		char* msg;
		status = sqlite3_exec(pDb, sql, NULL, 0, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}

		sql = "BEGIN";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}
		sqlite3_stmt* stmt;
		sql = "INSERT INTO Had (id, data) VALUES (?,?)";
		status = sqlite3_prepare_v2(pDb, sql, strlen(sql), &stmt, NULL);
		if (status != SQLITE_OK)
			return;
		for (auto& pair : data)
		{
			if (pair.second.size == 0)
			{
				if (pair.second.data != nullptr)
				{
					free(pair.second.data);
				}
				continue;
			}

			sqlite3_reset(stmt);
			sqlite3_clear_bindings(stmt);
			sqlite3_bind_int64(stmt, 1, MeshId_toNdsGridId(pair.first));
			sqlite3_bind_blob(stmt, 2, pair.second.data, pair.second.size, nullptr);
			status = sqlite3_step(stmt);

			if (pair.second.data != nullptr)
			{
				free(pair.second.data);
			}
			if (status != SQLITE_DONE && status != SQLITE_ROW)
			{
				break;
			}
		}
		sqlite3_finalize(stmt);
		sql = "COMMIT";
		status = sqlite3_exec(pDb, sql, NULL, NULL, &msg);
		if (status != SQLITE_OK)
		{
			sqlite3_free(msg);
			return;
		}
		sqlite3_close(pDb);
	}


}
