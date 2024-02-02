#include "stdafx.h"
#include <fstream>
#include "MapboxWriter.h"
#include <boost/filesystem.hpp>

namespace RDS
{
	void MapboxWriter::write(RdsTile* pTile, void*& pData, unsigned long long& size)
	{
		Tile tile;
		Tile* pMapboxTile = &tile;
		pMapboxTile->set_id(pTile->meshId);
		writeRoad(pTile, pMapboxTile);
		writeText(pTile, pMapboxTile);
		writeToll(pTile, pMapboxTile);
		writeTunnel(pTile, pMapboxTile);
		writeIntersection(pTile, pMapboxTile);
		writeGuardrail(pTile, pMapboxTile);
		writeMarking(pTile, pMapboxTile);
		writeLine(pTile, pMapboxTile);
		writeDiversion(pTile, pMapboxTile);
		writeSign3d(pTile, pMapboxTile);
		writeSpeedLimitBoard(pTile, pMapboxTile);
		writePier(pTile, pMapboxTile);
		writeGreenbelt(pTile, pMapboxTile);
		writeCrossWalk(pTile, pMapboxTile);
		writeGroup(pTile, pMapboxTile);
		writeTrafficLight(pTile, pMapboxTile);
		writeBridgeTransparency(pTile, pMapboxTile);
		// 序列化为字节流
		size = pMapboxTile->ByteSizeLong();
		pData = malloc(size);
		pMapboxTile->SerializeToArray(pData, pMapboxTile->ByteSizeLong());
	}

	void MapboxWriter::write(const char* path, RdsTile* tile)
	{
		if (boost::filesystem::exists(path))
			boost::filesystem::remove(path);
		std::ofstream outStream(path, std::ios::trunc | std::ios::binary);
		if (outStream.fail())
			return;
		void* pData;
		unsigned long long size;
		write(tile, pData, size);
		outStream.write(reinterpret_cast<char*>(pData), size);
		outStream.close();
	}

	void MapboxWriter::writeRoad(RdsTile* pTile, Tile* pMapboxTile)
	{
		Tile_Layer* pLayer = pMapboxTile->add_layers();
		std::vector<RdsObject*>& pRoads = pTile->query(EntityType::RDS_ROAD);
		pLayer->set_name((int)RDS::EntityType::RDS_ROAD);
		pLayer->set_version(RdsRoad::version);
		for (auto p : pRoads)
		{
			RdsRoad* pRoad = (RdsRoad*)p;
			Tile_Feature* pFeature = pLayer->add_features();
			pFeature->set_type(Tile_GeomType_LINESTRING);
			createMultiLineStrng(pFeature, pRoad->contour.lines.data(), pRoad->contour.lines.size());
		}
	}

	void MapboxWriter::writeText(RdsTile* pTile, Tile* pMapboxTile)
	{
		std::vector<RdsObject*>& pTexts = pTile->query(EntityType::RDS_TEXT);
		Tile_Layer* pLayer = pMapboxTile->add_layers();
		pLayer->set_name((int)RDS::EntityType::RDS_TEXT);
		pLayer->set_version(RdsText::version);

		// keys
		pLayer->add_keys((int)RDS::AttributeType::TYPE);
		pLayer->add_keys((int)RDS::AttributeType::COLOR);
		pLayer->add_keys((int)RDS::AttributeType::TEXT);
		pLayer->add_keys((int)RDS::AttributeType::SPEED_LIMIT_TYPE);

		// values
		std::map<int32, uint32> values;
		std::map<std::string, uint32> text_values;

		for_each(pTexts.begin(), pTexts.end(), [&](RdsObject* pObject) -> void {
			RdsText* pText = (RdsText*)pObject;
			values[(int32)pText->type] = 0;
			values[(int32)pText->color] = 0;
			text_values[pText->text] = 0;

			values[(int32)pText->speedLimitType] = 0;
			});

		for (std::map<int32,uint32>::iterator pIter = values.begin(); pIter != values.end();pIter++)
		{
			pIter->second = pLayer->values().size();
			pLayer->add_values()->set_sint_value(pIter->first);
		}

		for (std::map<std::string,uint32>::iterator pIter = text_values.begin(); pIter != text_values.end(); pIter++)
		{
			pIter->second = pLayer->values().size();
			pLayer->add_values()->set_string_value(pIter->first);
		}

		for (auto p : pTexts)
		{
			RdsText* pText = (RdsText*)p;
			Tile_Feature* pFeature = pLayer->add_features();
			pFeature->set_type(Tile_GeomType_POLYGON);
			pFeature->add_tags(0);
			pFeature->add_tags(values[(int32)pText->type]);
			pFeature->add_tags(1);
			pFeature->add_tags(values[(int32)pText->color]);
			pFeature->add_tags(2);
			pFeature->add_tags(text_values[pText->text]);

			pFeature->add_tags(3);
			pFeature->add_tags(values[(int32)pText->speedLimitType]);
			createPolygon(pFeature, pText->contour);
		}
	}

	void MapboxWriter::writeToll(RdsTile* pTile, Tile* pMapboxTile)
	{
		std::vector<RdsObject*>& pTolls = pTile->query(EntityType::RDS_TOLL);
		Tile_Layer* pLayer = pMapboxTile->add_layers();
		pLayer->set_name((int)RDS::EntityType::RDS_TOLL);
		pLayer->set_version(RdsText::version);

		// keys
		pLayer->add_keys((int)RDS::AttributeType::ANGLE);
		pLayer->add_keys((int)RDS::AttributeType::NAME);
		pLayer->add_keys((int)RDS::AttributeType::TYPELIST);

		// values
		std::map<int32, uint32> values;
		std::map<std::string, uint32> s_values;

		for_each(pTolls.begin(), pTolls.end(), [&](RdsObject* pObject) -> void {
			RdsToll* pToll = (RdsToll*)pObject;
			values[pToll->angle] = 0;

			// 类型列表使用字符流存储，每字节(8bit)表示对应的类型，目前协议使用了3bit。
			std::string s;
			size_t size = pToll->typeList.size();
			s.resize(size);
			for (size_t i = 0; i < size; i++)
				s[i] = (char)(int8)pToll->typeList[i];
			s_values[s] = 0;
			s_values[pToll->name] = 0;

			});

		for (std::map<int32, uint32>::iterator pIter = values.begin(); pIter != values.end(); pIter++)
		{
			pIter->second = pLayer->values().size();
			pLayer->add_values()->set_sint_value(pIter->first);
		}

		for (std::map<std::string, uint32>::iterator pIter = s_values.begin(); pIter != s_values.end(); pIter++)
		{
			pIter->second = pLayer->values().size();
			pLayer->add_values()->set_string_value(pIter->first);
		}

		for (auto p : pTolls)
		{
			RdsToll* pToll = (RdsToll*)p;
			Tile_Feature* pFeature = pLayer->add_features();
			pFeature->set_type(Tile_GeomType_POINT);
			pFeature->add_tags(0);
			pFeature->add_tags(values[(int32)pToll->angle]);
			pFeature->add_tags(1);
			pFeature->add_tags(s_values[pToll->name]);
			pFeature->add_tags(2);
			std::string s;
			size_t size = pToll->typeList.size();
			s.resize(size);
			for (size_t i = 0; i < size; i++)
				s[i] = (char)(int8)pToll->typeList[i];
			pFeature->add_tags(s_values[s]);

			createMultiPoint(pFeature, pToll->positions.postions.data(),pToll->positions.postions.size());
		}
	}


	void MapboxWriter::writeTunnel(RdsTile* pTile, Tile* pMapboxTile)
	{
		std::vector<RdsObject*>& pTunnels = pTile->query(EntityType::RDS_TUNNEL);
		Tile_Layer* pLayer = pMapboxTile->add_layers();
		pLayer->set_name((int)RDS::EntityType::RDS_TUNNEL);
		pLayer->set_version(RdsTunnel::version);

		// keys
		pLayer->add_keys((int)RDS::AttributeType::HEIGHT);
		pLayer->add_keys((int)RDS::AttributeType::TUNNELTYPE);
		pLayer->add_keys((int)RDS::AttributeType::THICKNESS);
		pLayer->add_keys((int)RDS::AttributeType::HEIGHT_RANGE_LIST);
		// values
		std::map<int32, uint32> values;
		std::map<std::string, uint32> s_values;

		for_each(pTunnels.begin(), pTunnels.end(), [&](RdsObject* pObject) -> void {
			RdsTunnel* pTunnel = (RdsTunnel*)pObject;

			values[pTunnel->height] = 0;
			values[(int32)pTunnel->tunnelType] = 0;
			values[(int32)pTunnel->thickness] = 0;

			// 类型列表使用字符流存储，每字节(8bit)表示对应的类型，目前协议使用了3bit。
			std::string s, sBody;
			uint32 heightInfoCount = 0;
			for (size_t i = 0; i < pTunnel->heightInfos.size(); i++) {
				std::vector<RdsTunnel::HeightInfo>& heighInfo = pTunnel->heightInfos[i];
				for (size_t j = 0; j < heighInfo.size(); j++)
				{
					char szBuffer[128]{ 0 };
					cq_sprintf(szBuffer, "%d,%d,%d,%d#", i, heighInfo[j].startIndex, heighInfo[j].endIndex, heighInfo[j].height);
					sBody.append(szBuffer);
					heightInfoCount++;
				}
			}
			if (sBody.size())
			{
				char szBuffer[128]{ 0 };
				cq_sprintf(szBuffer, "%d,%d#", pTunnel->heightInfos.size(), heightInfoCount);
				s.append(szBuffer).append(sBody);
				s_values[s] = 0;
			}
			
		

			});
		for (std::map<int32, uint32>::iterator pIter = values.begin(); pIter != values.end(); pIter++)
		{
			pIter->second = pLayer->values().size();
			pLayer->add_values()->set_sint_value(pIter->first);
		}
		for (std::map<std::string, uint32>::iterator pIter = s_values.begin(); pIter != s_values.end(); pIter++)
		{
			pIter->second = pLayer->values().size();
			pLayer->add_values()->set_string_value(pIter->first);
		}

		for (auto p : pTunnels)
		{
			RdsTunnel* pTunnel = (RdsTunnel*)p;
			Tile_Feature* pFeature = pLayer->add_features();
			pFeature->set_type(Tile_GeomType_POINT);
			pFeature->add_tags(0);
			pFeature->add_tags(values[pTunnel->height]);
			pFeature->add_tags(1);
			pFeature->add_tags(values[(int32)pTunnel->tunnelType]);
			pFeature->add_tags(2);
			pFeature->add_tags(values[(int32)pTunnel->thickness]);
			

			// 类型列表使用字符流存储，每字节(8bit)表示对应的类型，目前协议使用了3bit。
			std::string s,sBody;
			uint32 heightInfoCount = 0;
			for (size_t i = 0; i < pTunnel->heightInfos.size(); i++) {
				std::vector<RdsTunnel::HeightInfo>& heighInfo = pTunnel->heightInfos[i];
				for (size_t j = 0; j < heighInfo.size(); j++)
				{
					char szBuffer[128]{ 0 };
					cq_sprintf(szBuffer, "%d,%d,%d,%d#", i, heighInfo[j].startIndex, heighInfo[j].endIndex, heighInfo[j].height);
					sBody.append(szBuffer);
					heightInfoCount++;
				}
			}

			if (sBody.size())
			{
				char szBuffer[128]{ 0 };
				cq_sprintf(szBuffer, "%d,%d#", pTunnel->heightInfos.size(), heightInfoCount);
				s.append(szBuffer).append(sBody);

				pFeature->add_tags(3);
				pFeature->add_tags(s_values[s]);
			}
		
			createMultiLineStrng(pFeature, pTunnel->contour.lines.data(), pTunnel->contour.lines.size());
		}
	}

	void MapboxWriter::writeIntersection(RdsTile* pTile, Tile* pMapboxTile)
	{
		std::vector<RdsObject*>& pIntersections = pTile->query(EntityType::RDS_INTERSECTION);
		Tile_Layer* pLayer = pMapboxTile->add_layers();
		pLayer->set_name((int)RDS::EntityType::RDS_INTERSECTION);
		pLayer->set_version(RdsIntersection::version);

		for (auto p : pIntersections)
		{
			RdsIntersection* pIntersection = (RdsIntersection*)p;
			Tile_Feature* pFeature = pLayer->add_features();
			pFeature->set_type(Tile_GeomType_POINT);
			createPolygon(pFeature, pIntersection->contour);
		}
	}

	void MapboxWriter::writeGuardrail(RdsTile* pTile, Tile* pMapboxTile)
	{
		std::vector<RdsObject*>& pGuardrails = pTile->query(EntityType::RDS_GUARDRAIL);
		Tile_Layer* pLayer = pMapboxTile->add_layers();
		pLayer->set_name((int)RDS::EntityType::RDS_GUARDRAIL);
		pLayer->set_version(RdsGuardrail::version);

		// keys
		pLayer->add_keys((int)RDS::AttributeType::RAILTYPE);

		// values
		std::map<int32, uint32> values;

		for_each(pGuardrails.begin(), pGuardrails.end(), [&](RdsObject* pObject) -> void {
			RdsGuardrail* pGuardrail = (RdsGuardrail*)pObject;
			values[(int32)pGuardrail->railType] = 0;
			});

		for (std::map<int32, uint32>::iterator pIter =  values.begin(); pIter != values.end(); pIter++)
		{
			pIter->second = pLayer->values().size();
			pLayer->add_values()->set_sint_value(pIter->first);
		}

		for (auto p : pGuardrails)
		{
			RdsGuardrail* pGuardrail = (RdsGuardrail*)p;
			Tile_Feature* pFeature = pLayer->add_features();
			pFeature->set_type(Tile_GeomType_LINESTRING);
			pFeature->add_tags(0);
			pFeature->add_tags(values[(int32)pGuardrail->railType]);
			createLineString(pFeature, pGuardrail->location);
		}
	}

	void MapboxWriter::writeMarking(RdsTile* pTile, Tile* pMapboxTile)
	{
		std::vector<RdsObject*>& pMarkings = pTile->query(EntityType::RDS_MARKING);
		Tile_Layer* pLayer = pMapboxTile->add_layers();
		pLayer->set_name((int)RDS::EntityType::RDS_MARKING);
		pLayer->set_version(RdsMarking::version);

		// keys
		pLayer->add_keys((int)RDS::AttributeType::COLOR);
		pLayer->add_keys((int)RDS::AttributeType::MARKINGTYPE);

		// values
		std::map<int32, uint32> values;

		for_each(pMarkings.begin(), pMarkings.end(), [&](RdsObject* pObject) -> void {
			RdsMarking* pMarking = (RdsMarking*)pObject;
			values[(int32)pMarking->color] = 0;
			values[(int32)pMarking->markingType] = 0;
			});

		for (std::map<int32, uint32>::iterator pIter = values.begin(); pIter != values.end(); pIter++)
		{
			pIter->second = pLayer->values().size();
			pLayer->add_values()->set_sint_value(pIter->first);
		}

		for (auto p : pMarkings)
		{
			RdsMarking* pMarking = (RdsMarking*)p;
			Tile_Feature* pFeature = pLayer->add_features();
			pFeature->set_type(Tile_GeomType_POLYGON);
			pFeature->add_tags(0);
			pFeature->add_tags(values[(int32)pMarking->color]);
			pFeature->add_tags(1);
			pFeature->add_tags(values[(int32)pMarking->markingType]);
			createPolygon(pFeature, pMarking->contour);
		}
	}

	void MapboxWriter::writeLine(RdsTile* pTile, Tile* pMapboxTile)
	{
		std::vector<RdsObject*>& pLines = pTile->query(EntityType::RDS_LINE);
		Tile_Layer* pLayer = pMapboxTile->add_layers();
		pLayer->set_name((int)RDS::EntityType::RDS_LINE);
		pLayer->set_version(RdsLine::version);

		// keys
		pLayer->add_keys((int)RDS::AttributeType::WIDTH);
		pLayer->add_keys((int)RDS::AttributeType::COLOR);
		pLayer->add_keys((int)RDS::AttributeType::LINETYPE);
		pLayer->add_keys((int)RDS::AttributeType::SIDE);

		// values
		std::map<int32, uint32> values;

		for_each(pLines.begin(), pLines.end(), [&](RdsObject* pObject) -> void {
			RdsLine* pLine = (RdsLine*)pObject;
			values[(int32)pLine->color] = 0;
			values[(int32)pLine->width] = 0;
			values[(int32)pLine->lineType] = 0;
			values[(int32)pLine->side] = 0;
			});

		for (std::map<int32, uint32>::iterator pIter = values.begin(); pIter != values.end(); pIter++)
		{
			pIter->second = pLayer->values().size();
			pLayer->add_values()->set_sint_value(pIter->first);
		}

		for (auto p : pLines)
		{
			RdsLine* pLine = (RdsLine*)p;
			Tile_Feature* pFeature = pLayer->add_features();
			pFeature->set_type(Tile_GeomType_LINESTRING);
			pFeature->add_tags(0);
			pFeature->add_tags(values[(int32)pLine->width]);
			pFeature->add_tags(1);
			pFeature->add_tags(values[(int32)pLine->color]);
			pFeature->add_tags(2);
			pFeature->add_tags(values[(int32)pLine->lineType]);
			pFeature->add_tags(3);
			pFeature->add_tags(values[(int32)pLine->side]);
			createLineString(pFeature, pLine->location);
		}
	}

	void MapboxWriter::writeDiversion(RdsTile* pTile, Tile* pMapboxTile)
	{
		std::vector<RdsObject*>& pDiversions = pTile->query(EntityType::RDS_DIVERSION);
		Tile_Layer* pLayer = pMapboxTile->add_layers();
		pLayer->set_name((int)RDS::EntityType::RDS_DIVERSION);
		pLayer->set_version(RdsDiversion::version);

		// keys
		pLayer->add_keys((int)RDS::AttributeType::ANGLE);
		pLayer->add_keys((int)RDS::AttributeType::DIVERSIONTYPE);

		// values
		std::map<int32, uint32> values;

		for_each(pDiversions.begin(), pDiversions.end(), [&](RdsObject* pObject) -> void {
			RdsDiversion* pLine = (RdsDiversion*)pObject;
			values[(int32)pLine->angle] = 0;
			values[(int32)pLine->diversionType] = 0;

			});

		for (std::map<int32, uint32>::iterator pIter = values.begin(); pIter != values.end(); pIter++)
		{
			pIter->second = pLayer->values().size();
			pLayer->add_values()->set_sint_value(pIter->first);
		}

		for (auto p : pDiversions)
		{
			RdsDiversion* pDiversion = (RdsDiversion*)p;
			Tile_Feature* pFeature = pLayer->add_features();
			pFeature->set_type(Tile_GeomType_LINESTRING);
			pFeature->add_tags(0);
			pFeature->add_tags(values[(int32)pDiversion->angle]);
			pFeature->add_tags(1);
			pFeature->add_tags(values[(int32)pDiversion->diversionType]);
			LineString3d polygon;
			polygon.vertexes.assign(pDiversion->contour.vertexes.begin(),pDiversion->contour.vertexes.end());
			createLineString(pFeature, polygon);
			createLineString(pFeature, pDiversion->centerRefereceLine);
			createTriIndex(pFeature, pDiversion->polyTriIndex);
		}
	}

	void MapboxWriter::writeSign3d(RdsTile* pTile, Tile* pMapboxTile)
	{
		std::vector<RdsObject*>& pSign3ds = pTile->query(EntityType::RDS_SIGN3D);
		Tile_Layer* pLayer = pMapboxTile->add_layers();
		pLayer->set_name((int)RDS::EntityType::RDS_SIGN3D);
		pLayer->set_version(RdsSign3d::version);

		// keys
		pLayer->add_keys((int)RDS::AttributeType::ANGLE);
		pLayer->add_keys((int)RDS::AttributeType::SIGNTYPE);
		pLayer->add_keys((int)RDS::AttributeType::CONTENT);

		// values
		std::map<int32, uint32> values;
		std::map<std::string, uint32> s_values;

		for_each(pSign3ds.begin(), pSign3ds.end(), [&](RdsObject* pObject) -> void {
			RdsSign3d* pSign3d = (RdsSign3d*)pObject;
			values[(int32)pSign3d->angle] = 0;
			values[(int32)pSign3d->signType] = 0;
			s_values[pSign3d->content] = 0;
			});

		for (std::map<int32, uint32>::iterator pIter = values.begin(); pIter != values.end(); pIter++)
		{
			pIter->second = pLayer->values().size();
			pLayer->add_values()->set_sint_value(pIter->first);
		}

		for (std::map<std::string, uint32>::iterator pIter = s_values.begin(); pIter != s_values.end(); pIter++)
		{
			pIter->second = pLayer->values().size();
			pLayer->add_values()->set_string_value(pIter->first);
		}

		for (auto p : pSign3ds)
		{
			RdsSign3d* pSign3d = (RdsSign3d*)p;
			Tile_Feature* pFeature = pLayer->add_features();
			pFeature->set_type(Tile_GeomType_POINT);
			pFeature->add_tags(0);
			pFeature->add_tags(values[(int32)pSign3d->angle]);
			pFeature->add_tags(1);
			pFeature->add_tags(s_values[pSign3d->content]);
			pFeature->add_tags(2);
			pFeature->add_tags(values[(int32)pSign3d->signType]);
			createPoint(pFeature, pSign3d->position);
		}
	}

	void MapboxWriter::writeSpeedLimitBoard(RdsTile* pTile, Tile* pMapboxTile)
	{
		std::vector<RdsObject*>& pObjecs = pTile->query(EntityType::RDS_SPEEDLIMITBOARD);
		Tile_Layer* pLayer = pMapboxTile->add_layers();
		pLayer->set_name((int)RDS::EntityType::RDS_SPEEDLIMITBOARD);
		pLayer->set_version(RdsSpeedLimitBoard::version);

		// keys
		pLayer->add_keys((int)RDS::AttributeType::ABOVE);
		pLayer->add_keys((int)RDS::AttributeType::BLOW);
		pLayer->add_keys((int)RDS::AttributeType::TYPE);

		// values
		std::map<int32, uint32> values;

		for_each(pObjecs.begin(), pObjecs.end(), [&](RdsObject* pObject) -> void {
			RdsSpeedLimitBoard* pBoard = (RdsSpeedLimitBoard*)pObject;
			values[(int32)pBoard->above] = 0;
			values[(int32)pBoard->blow] = 0;
			values[(int32)pBoard->type] = 0;
			});

		for (std::map<int32, uint32>::iterator pIter = values.begin(); pIter != values.end(); pIter++)
		{
			pIter->second = pLayer->values().size();
			pLayer->add_values()->set_sint_value(pIter->first);
		}

		for (auto p : pObjecs)
		{
			RdsSpeedLimitBoard* pBoard = (RdsSpeedLimitBoard*)p;
			Tile_Feature* pFeature = pLayer->add_features();
			pFeature->set_type(Tile_GeomType_POINT);
			pFeature->add_tags(0);
			pFeature->add_tags(values[(int32)pBoard->above]);
			pFeature->add_tags(1);
			pFeature->add_tags(values[(int32)pBoard->blow]);
			pFeature->add_tags(2);
			pFeature->add_tags(values[(int32)pBoard->type]);
			createPoint(pFeature, pBoard->position);
		}
	}

	void MapboxWriter::writePier(RdsTile* pTile, Tile* pMapboxTile)
	{
		std::vector<RdsObject*>& pPiers = pTile->query(EntityType::RDS_PIER);
		Tile_Layer* pLayer = pMapboxTile->add_layers();
		pLayer->set_name((int)RDS::EntityType::RDS_PIER);
		pLayer->set_version(RdsPier::version);

		// keys
		pLayer->add_keys((int)RDS::AttributeType::ANGLE);
		pLayer->add_keys((int)RDS::AttributeType::WIDTH);

		// values
		std::map<int32, uint32> values;

		for_each(pPiers.begin(), pPiers.end(), [&](RdsObject* pObject) -> void {
			RdsPier* pPier = (RdsPier*)pObject;
			values[pPier->angle] = 0;
			values[pPier->width] = 0;
			});

		for (std::map<int32, uint32>::iterator pIter = values.begin(); pIter != values.end(); pIter++)
		{
			pIter->second = pLayer->values().size();
			pLayer->add_values()->set_sint_value(pIter->first);
		}

		for (auto p : pPiers)
		{
			RdsPier* pPier = (RdsPier*)p;
			Tile_Feature* pFeature = pLayer->add_features();
			pFeature->set_type(Tile_GeomType_POINT);
			pFeature->add_tags(0);
			pFeature->add_tags(values[pPier->angle]);
			pFeature->add_tags(1);
			pFeature->add_tags(values[pPier->width]);
			createPoint(pFeature, pPier->position);
		}
	}

	void MapboxWriter::writeGreenbelt(RdsTile* pTile, Tile* pMapboxTile)
	{
		std::vector<RdsObject*>& pGreenbelts = pTile->query(EntityType::RDS_GREENBELT);
		Tile_Layer* pLayer = pMapboxTile->add_layers();
		pLayer->set_name((int)RDS::EntityType::RDS_GREENBELT);
		pLayer->set_version(RdsGreenbelt::version);

		for (auto p : pGreenbelts)
		{
			RdsGreenbelt* pGreenbelt = (RdsGreenbelt*)p;
			Tile_Feature* pFeature = pLayer->add_features();
			pFeature->set_type(Tile_GeomType_LINESTRING);
			createMultiLineStrng(pFeature, pGreenbelt->contour.lines.data(), pGreenbelt->contour.lines.size());
		}
	}

	void MapboxWriter::writeCrossWalk(RdsTile* pTile, Tile* pMapboxTile)
	{
		std::vector<RdsObject*>& pCrossWalks = pTile->query(EntityType::RDS_CROSSWALK);
		Tile_Layer* pLayer = pMapboxTile->add_layers();
		pLayer->set_name((int)RDS::EntityType::RDS_CROSSWALK);
		pLayer->set_version(RdsCrossWalk::version);

		// keys
		pLayer->add_keys((int)RDS::AttributeType::ANGLE);

		// values
		std::map<int32, uint32> values;

		for_each(pCrossWalks.begin(), pCrossWalks.end(), [&](RdsObject* pObject) -> void {
			RdsCrossWalk* pCrosswalk = (RdsCrossWalk*)pObject;
			values[(int32)pCrosswalk->zebraHeading] = 0;
			});

		for (std::map<int32, uint32>::iterator pIter = values.begin(); pIter != values.end(); pIter++)
		{
			pIter->second = pLayer->values().size();
			pLayer->add_values()->set_sint_value(pIter->first);
		}

		for (auto p : pCrossWalks)
		{
			RdsCrossWalk* tmpCrosswalk = (RdsCrossWalk*)p;
			Tile_Feature* pFeature = pLayer->add_features();
			pFeature->set_type(Tile_GeomType_POLYGON);
			pFeature->add_tags(0);
			pFeature->add_tags(values[(int32)tmpCrosswalk->zebraHeading]);
			createPolygon(pFeature, tmpCrosswalk->contour);
		}
	}

	void MapboxWriter::writeGroup(RdsTile* pTile, Tile* pMapboxTile)
	{
		std::vector<RdsObject*>& pGroups = pTile->query(EntityType::RDS_GROUP);
		Tile_Layer* pLayer = pMapboxTile->add_layers();
		pLayer->set_name((int)RDS::EntityType::RDS_GROUP);
		pLayer->set_version(RdsGroup::version);

		// keys
		pLayer->add_keys((int)RDS::AttributeType::ORIGINID);
		pLayer->add_keys((int)RDS::AttributeType::LINKID);
		pLayer->add_keys((int)RDS::AttributeType::ANGLE);
		pLayer->add_keys((int)RDS::AttributeType::SIZE);
		pLayer->add_keys((int)RDS::AttributeType::LENGTH);
		pLayer->add_keys((int)RDS::AttributeType::CREATED_TUNNEL_TAG);

		// values
		std::map<int32, uint32> values;
		for_each(pGroups.begin(), pGroups.end(), [&](RdsObject* pObject) -> void {
			RdsGroup* pGroup = (RdsGroup*)pObject;
			values[pGroup->originId] = 0;
			values[pGroup->linkId] = 0;
			values[pGroup->angle] = 0;
			values[pGroup->objects.size()] = 0;
			values[pGroup->semiTransparentLength] = 0;
			values[pGroup->isCreatedTunnel?1:0] = 0;
			});

		for (std::map<int32, uint32>::iterator pIter = values.begin(); pIter != values.end(); pIter++)
		{
			pIter->second = pLayer->values().size();
			pLayer->add_values()->set_sint_value(pIter->first);
		}

		for (auto p : pGroups)
		{
			RdsGroup* pGroup = (RdsGroup*)p;
			Tile_Feature* pFeature = pLayer->add_features();
			pFeature->set_type(Tile_GeomType_UNKNOWN);
			pFeature->add_tags(0);
			pFeature->add_tags(values[pGroup->originId]);
			pFeature->add_tags(1);
			pFeature->add_tags(values[pGroup->linkId]);
			pFeature->add_tags(2);
			pFeature->add_tags(values[pGroup->angle]);
			pFeature->add_tags(3);
			pFeature->add_tags(values[pGroup->objects.size()]);
			pFeature->add_tags(4);
			pFeature->add_tags(values[pGroup->semiTransparentLength]);
			pFeature->add_tags(5);
			pFeature->add_tags(values[pGroup->isCreatedTunnel]);

			for (size_t i = 0;i < pGroup->objects.size();i++)
			{
				pFeature->add_geometry(pGroup->objects[i]->id);
			}

			createMultiPoint(pFeature, pGroup->semiTransparentPoints.postions.data(), pGroup->semiTransparentPoints.postions.size());
		}
	}


	void MapboxWriter::writeTrafficLight(RdsTile* pTile, Tile* pMapboxTile)
	{
		std::vector<RdsObject*>& pTrafficLights = pTile->query(EntityType::RDS_TRAFFICLIGHT);
		Tile_Layer* pLayer = pMapboxTile->add_layers();
		pLayer->set_name((int)RDS::EntityType::RDS_TRAFFICLIGHT);
		pLayer->set_version(RdsTrafficLight::version);

		// keys
		pLayer->add_keys((int)RDS::AttributeType::TYPE);
		pLayer->add_keys((int)RDS::AttributeType::ANGLE);
		pLayer->add_keys((int)RDS::AttributeType::HEIGHT);
		pLayer->add_keys((int)RDS::AttributeType::LENGTH);
		pLayer->add_keys((int)RDS::AttributeType::SIZE);	//灯箱大小
	
		

		// values
		std::map<int32, uint32> values;
		
		for_each(pTrafficLights.begin(), pTrafficLights.end(), [&](RdsObject* pObject) -> void {
			RdsTrafficLight* pLight = (RdsTrafficLight*)pObject;
			values[(int32)pLight->type] = 0;
			values[(int32)pLight->barHeading] = 0;
			values[(int32)pLight->maxHeight] = 0;
			values[(int32)pLight->maxBarLenght] = 0;
			values[(int32)pLight->lightBoxs.size()] = 0;
		
			});

		for (std::map<int32, uint32>::iterator pIter = values.begin(); pIter != values.end(); pIter++)
		{
			pIter->second = pLayer->values().size();
			pLayer->add_values()->set_sint_value(pIter->first);
		}



		for (auto p : pTrafficLights)
		{
			RdsTrafficLight* pLight = (RdsTrafficLight*)p;
			Tile_Feature* pFeature = pLayer->add_features();
			pFeature->set_type(Tile_GeomType_POINT);
			pFeature->add_tags(0);
			pFeature->add_tags(values[(int32)pLight->type]);
			pFeature->add_tags(1);
			pFeature->add_tags(values[(int32)pLight->barHeading]);
			pFeature->add_tags(2);
			pFeature->add_tags(values[(int32)pLight->maxHeight]);
			pFeature->add_tags(3);
			pFeature->add_tags(values[(int32)pLight->maxBarLenght]);
			pFeature->add_tags(4);
			pFeature->add_tags(values[(int32)pLight->lightBoxs.size()]);
		
			size_t size = pLight->lightBoxs.size();
			for (size_t i = 0; i < size; i++)
			{
				uint64 lightAttr = pLight->lightBoxs[i].GetContent();
				pLight->lightBoxs[i].SetContent(lightAttr);
				uint64 lightAttrT = pLight->lightBoxs[i].GetContent();
				CQ_ASSERT(lightAttrT == lightAttr);
 				pFeature->add_geometry(lightAttr);
			}
			
			createPoint(pFeature, pLight->position);
		}

	}

	void MapboxWriter::writeBridgeTransparency(RdsTile* pTile, Tile* pMapboxTile)
	{
        std::vector<RdsObject*>& pTransparencyrels = pTile->query(EntityType::RDS_TRANSPARENCYREL);
        Tile_Layer* pLayer = pMapboxTile->add_layers();
        pLayer->set_name((int)RDS::EntityType::RDS_TRANSPARENCYREL);
        pLayer->set_version(RdsTransparencyRel::version);

        // keys
        pLayer->add_keys((int)RDS::AttributeType::ORIGINID);
		pLayer->add_keys((int)RDS::AttributeType::SIZE);

        // values
        std::map<int64, uint32> values;

        for_each(pTransparencyrels.begin(), pTransparencyrels.end(), [&](RdsObject* pObject) -> void {
			RdsTransparencyRel* pTrans = (RdsTransparencyRel*)pObject;
            values[(int64)pTrans->first] = 0;
			values[(int64)pTrans->second.size()] = 0;
            });

        for (std::map<int64, uint32>::iterator pIter = values.begin(); pIter != values.end(); pIter++)
        {
            pIter->second = pLayer->values().size();
            pLayer->add_values()->set_sint_value(pIter->first);
        }



        for (auto p : pTransparencyrels)
        {
			RdsTransparencyRel* pTrans = (RdsTransparencyRel*)p;
            Tile_Feature* pFeature = pLayer->add_features();
            pFeature->set_type(Tile_GeomType_POINT);
            pFeature->add_tags(0);
            pFeature->add_tags(values[(int64)pTrans->first]);
            pFeature->add_tags(1);
            pFeature->add_tags(values[(int32)pTrans->second.size()]);

            size_t size = pTrans->second.size();
            for (size_t i = 0; i < size; i++)
            {
                pFeature->add_geometry(pTrans->second[i]);
            }

        }
	}


	RDS::uint32 MapboxWriter::createCommandInteger(CommandType type, uint32 count)
	{
		return  ((unsigned char)type & 0x7) | (count << 3);
	}

	uint64 MapboxWriter::createParameterInteger(int64 coordinate)
	{
		return (coordinate << 1) ^ (coordinate >> 63);
	}

	void MapboxWriter::offsetPoint(const RDS::Point3d& origin, RDS::Point3d& point)
	{
		point.x -= origin.x;
		point.y -= origin.y;
		point.z -= origin.z;
	}

	void MapboxWriter::offsetPoints(const RDS::Point3d& origin, RDS::Point3d* points, unsigned int count)
	{
		for (size_t i = 0; i < count; i++)
		{
			RDS::Point3d& point = points[i];
			offsetPoint(origin, point);
		}
	}

	void MapboxWriter::offsetLineString(const RDS::Point3d& origin, RDS::LineString3d& line)
	{
		offsetPoints(origin, line.vertexes.data(), line.vertexes.size());
	}

	void MapboxWriter::offsetMultiLineString(const RDS::Point3d& origin, RDS::LineString3d* lines, unsigned int count)
	{
		for (size_t i = 0; i < count; i++)
		{
			RDS::LineString3d& line = lines[i];
			offsetLineString(origin, line);
		}
	}

	void MapboxWriter::createPoint(Tile_Feature* pFeature, const RDS::Point3d& point)
	{
		pFeature->add_geometry(createCommandInteger(CommandType::MoveTo, 1));
		pFeature->add_geometry(createParameterInteger(point.x));
		pFeature->add_geometry(createParameterInteger(point.y));
		pFeature->add_geometry(createParameterInteger(point.z));
	}

	void MapboxWriter::createMultiPoint(Tile_Feature* pFeature, RDS::Point3d* points, uint32 count)
	{
		if (count == 0)
			return;
		
		createPoint(pFeature, points[0]);
		for (unsigned int i = 0;i+1 < count;i++)
		{
			RDS::Point3d point = points[i+1];
			offsetPoint(points[i], point);
			createPoint(pFeature, point);
		}
	}

	void MapboxWriter::createLineString(Tile_Feature* pFeature, const RDS::LineString3d& line)
	{
		if (line.vertexes.empty())
			return;
		
		pFeature->add_geometry(createCommandInteger(CommandType::MoveTo, 1));

		RDS::Point3d firstPoint = line.vertexes[0];
		pFeature->add_geometry(createParameterInteger(firstPoint.x));
		pFeature->add_geometry(createParameterInteger(firstPoint.y));
		pFeature->add_geometry(createParameterInteger(firstPoint.z));

		pFeature->add_geometry(createCommandInteger(CommandType::LineTo, line.vertexes.size() - 1));
		for (unsigned int i = 0; i + 1 < line.vertexes.size(); i++)
		{
			RDS::Point3d point =  line.vertexes[i+1];
			offsetPoint(line.vertexes[i], point);
			pFeature->add_geometry(createParameterInteger(point.x));
			pFeature->add_geometry(createParameterInteger(point.y));
			pFeature->add_geometry(createParameterInteger(point.z));
		}
	}

	void MapboxWriter::createMultiLineStrng(Tile_Feature* pFeature, RDS::LineString3d* lines, uint32 count)
	{
		for (unsigned int i = 0; i < count;i++)
		{
			createLineString(pFeature, lines[i]);
		}
	}

	void MapboxWriter::createPolygon(Tile_Feature* pFeature, const RDS::Polygon3d& polygon)
	{
		if (polygon.vertexes.empty())
			return;
		
		pFeature->add_geometry(createCommandInteger(CommandType::MoveTo, 1));

		RDS::Point3d firstPoint = polygon.vertexes[0];
		pFeature->add_geometry(createParameterInteger(firstPoint.x));
		pFeature->add_geometry(createParameterInteger(firstPoint.y));
		pFeature->add_geometry(createParameterInteger(firstPoint.z));

		pFeature->add_geometry(createCommandInteger(CommandType::LineTo, polygon.vertexes.size() - 1));
		for (unsigned int i = 0; i + 1 < polygon.vertexes.size(); i++)
		{
			RDS::Point3d point = polygon.vertexes[i+1];
			offsetPoint(polygon.vertexes[i], point);
			pFeature->add_geometry(createParameterInteger(point.x));
			pFeature->add_geometry(createParameterInteger(point.y));
			pFeature->add_geometry(createParameterInteger(point.z));
		}

		pFeature->add_geometry(createCommandInteger(CommandType::ClosePath, 1));
	}

	void MapboxWriter::createTriIndex(Tile_Feature* pFeature, const std::vector<uint16>& triangleIndex)
	{
		RDS::PolyTriIndex polyTriIndex;
		auto nSize = triangleIndex.size() / 3;
		for (auto tIdx = 0; tIdx < nSize; tIdx++) {
			RDS::TriIndex triIndex;
			triIndex.index0 = triangleIndex[tIdx * 3 + 0];
			triIndex.index1 = triangleIndex[tIdx * 3 + 1];
			triIndex.index2 = triangleIndex[tIdx * 3 + 2];
			polyTriIndex.indexes.push_back(triIndex);
		}

		auto& polyTriIndexes = polyTriIndex.indexes;
		if (!polyTriIndexes.empty())
		{
			LineString3d poly2TriIndex;
			poly2TriIndex.vertexes.resize(polyTriIndexes.size());
			memcpy(poly2TriIndex.vertexes.data(), polyTriIndexes.data(), sizeof(TriIndex) * polyTriIndexes.size());
			createLineString(pFeature, poly2TriIndex);
		}
	}

}

