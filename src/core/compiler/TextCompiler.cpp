#include "stdafx.h"
#include "TextCompiler.h"
#include "algorithm/grap_point_algorithm.h"

#ifdef OUTPUT_DEBUG_INFO
#include<fstream>
#include<iostream>
#include<sstream>
#endif

#define IS_SPEED_LIMIT(value) (value % 10 == 0 && value > 30 && value < 130)
#define IS_TIME_LIMIT(value) (value > 0 && value < 24)

namespace OMDB
{


	void TextCompiler::compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile)
	{
		//跨网格
		std::vector<std::vector<MapPoint3D64>> allLines;
		for (auto& rdsLineInfo : compilerData.m_rdsLines)
		{
			allLines.push_back(rdsLineInfo._originPoints);
		}
		for (auto& tmpTextObj : pGrid->query(ElementType::HAD_OBJECT_TEXT))
		{
			HadText* hadText = (HadText*)tmpTextObj;
			processCrossGrid(pGrid, hadText, allLines);
		}
		if (allLines.empty())
			return;

		compileTexts(pGrid, nearby, pTile);
		modifyTextGeometry(pTile, allLines);
	}

	void TextCompiler::processCrossGrid(HadGrid* const pGrid, HadText* hadText, std::vector<std::vector<MapPoint3D64>>& allLines)
	{
		if (hadText == nullptr)
		{
			return;
		}

		bool isCrossGird = false;
		for each (HadLane * hadlane  in hadText->refLanes)
		{
			if (hadlane->linkGroup == nullptr)
				continue;

			if (pGrid != hadlane->owner)
			{
				OMDB::LineString3d left_HadLine;
				LineString3d leftSide = hadlane->leftBoundary->location;
				if (directionEqual(hadlane->leftBoundary, hadlane->linkGroup, 3))
				{
					std::reverse(leftSide.vertexes.begin(), leftSide.vertexes.end());
				}
				LineString3d rightSide = hadlane->rightBoundary->location;
				if (directionEqual(hadlane->rightBoundary, hadlane->linkGroup, 3))
				{
					std::reverse(rightSide.vertexes.begin(), rightSide.vertexes.end());
				}
				left_HadLine.vertexes.insert(left_HadLine.vertexes.end(), leftSide.vertexes.begin(), leftSide.vertexes.end());
				if (left_HadLine.vertexes.size() < 2)
					continue;
				allLines.push_back(left_HadLine.vertexes);

				OMDB::LineString3d right_HadLine;
				right_HadLine.vertexes.insert(right_HadLine.vertexes.end(), rightSide.vertexes.begin(), rightSide.vertexes.end());
				if (right_HadLine.vertexes.size() < 2)
					continue;
				allLines.push_back(right_HadLine.vertexes);
			}

		}
	}

	std::vector<int> TextCompiler::getNearbyObjects(
        HadText* currentText,
		const std::vector<HadText*>& texts,
		int nearbyNum,
		int64 minValue,
        int64 maxValue,
		bool& isParallel)
	{
		std::vector<int> tmpIds(0);
		auto tmpVertexes = currentText->polygon.vertexes;
		if (tmpVertexes.size() < 4)
			return tmpIds;
		coordinatesTransform.convert(tmpVertexes.data(), tmpVertexes.size());
		auto tmpCenterVertex = getTextPolyCenter(currentText->polygon.vertexes);
		Vector2 tmpDir = vec2(tmpVertexes[3].pos.lon - tmpVertexes[0].pos.lon, tmpVertexes[3].pos.lat - tmpVertexes[0].pos.lat);
		tmpDir.normalize();
		std::vector<point_t> tmpPts;
		for (const auto& tmpText : texts)
			tmpPts.push_back(POINT_T(getTextPolyCenter(tmpText->polygon.vertexes)));
		parameters param;
		index_getter originInd(tmpPts);
		rtree_type rtree(boost::irange<std::size_t>(0lu, tmpPts.size()), param, originInd);
		rtree.query(bgi::nearest(POINT_T(tmpCenterVertex), nearbyNum),
			boost::make_function_output_iterator([&](size_t const& id) {
				double tmpDistance = std::abs(bg::distance(POINT_T(tmpCenterVertex), tmpPts[id]));
				auto currentPoint = getTextPolyCenter(texts[id]->polygon.vertexes);
				Vector2 currentDir = vec2(currentPoint.pos.lon - tmpCenterVertex.pos.lon, currentPoint.pos.lat - tmpCenterVertex.pos.lat);
				float verDistance = std::abs(dot(tmpDir, currentDir));
				if (tmpDistance < maxValue && tmpDistance > minValue)
				{
					if (verDistance < minValue * 0.5)
					{
						isParallel = true;
					}
					//if (IS_SPEED_LIMIT(std::atoi(texts[id]->content.data())))
					if (IS_SPEED_LIMIT(_wtoi(texts[id]->content.data())))
					{
						tmpIds.push_back(id);
					}
				}
				}));

		//有些相等的也加进来了
		for each (auto tempId in tmpIds)
		{
			auto nearbyText = texts[tempId];
			if (_wtoi(currentText->content.data()) == _wtoi(nearbyText->content.data()))
			{
				tmpIds.erase(std::remove(tmpIds.begin(), tmpIds.end(), tempId), tmpIds.end());
			}
		}
	
		return tmpIds;
	}

	void TextCompiler::getNearbyLanes(HadText* currentText, std::vector<HadLane*>& nearbyLanes)
	{
		int withinRefLanes = 0;
		if (currentText == nullptr)
		{
			return;
		}
		if (currentText->refLanes.size() <= 0)
		{
			return;
		}
		auto& tmpVertexes = currentText->polygon.vertexes;
		if (tmpVertexes.size() < 4)
		{
			return;
		}

		// 公交与多成员车道可能跨车道,直接返回
		std::wstring tmpContent = currentText->content;
		boost::algorithm::trim(tmpContent);
		auto regconf = boost::regex::perl | boost::regex::icase;
		auto isBusContent = boost::regex_search(tmpContent.data(), boost::wregex(L"^公$", regconf));
		auto isHovContent = boost::regex_search(tmpContent.data(), boost::wregex(L"^多$", regconf));
		if (isBusContent || isHovContent)
		{
			return;
		}

		int count;
		int totalNum = 4;
		auto tmpCenterVertex = getTextPolyCenter(currentText->polygon.vertexes);
		point_t pos = POINT_T(tmpCenterVertex);
		MapPoint3D64 tmpPos = MapPoint3D64_make(pos.get<0>(), pos.get<1>(), pos.get<2>());
		coordinatesTransform.invert(&tmpPos, 1);
		tmpPos.z /= 10;
		point_t tmpPosPoint = POINT_T(tmpPos);
		point_t nearyPosition = {};
		double minDistance = DBL_MAX;
		for (HadLane* hadlane : currentText->refLanes)
		{
			if (hadlane->linkGroup == nullptr)
				continue;

			std::vector<MapPoint3D64> points;
			LineString3d leftSide = hadlane->leftBoundary->location;
			if (directionEqual(hadlane->leftBoundary, hadlane->linkGroup, 3))
			{
				std::reverse(leftSide.vertexes.begin(), leftSide.vertexes.end());
			}
			LineString3d rightSide = hadlane->rightBoundary->location;
			if (directionEqual(hadlane->rightBoundary, hadlane->linkGroup, 3))
			{
				std::reverse(rightSide.vertexes.begin(), rightSide.vertexes.end());
			}
			points.insert(points.end(), leftSide.vertexes.begin(), leftSide.vertexes.end());
			points.insert(points.end(), rightSide.vertexes.rbegin(), rightSide.vertexes.rend());
			points.push_back(leftSide.vertexes.at(0));
			polygon_t tmpLine = POLYGON_T(points);
			if (bg::within(tmpPosPoint, tmpLine))
			{
				OMDB::LineString3d left_HadLine, right_HadLine;
				left_HadLine.vertexes.insert(left_HadLine.vertexes.end(), leftSide.vertexes.begin(), leftSide.vertexes.end());
				if (left_HadLine.vertexes.size() < 2)
					continue;

				right_HadLine.vertexes.insert(right_HadLine.vertexes.end(), rightSide.vertexes.begin(), rightSide.vertexes.end());
				if (right_HadLine.vertexes.size() < 2)
					continue;

				withinRefLanes++;
				size_t si = 0, ei = 0;
				MapPoint3D64 grappedPt{ 0 };
				MapPoint3D64 grappedPt1{ 0 };
				coordinatesTransform.convert(left_HadLine.vertexes.data(), left_HadLine.vertexes.size());
				coordinatesTransform.convert(right_HadLine.vertexes.data(), right_HadLine.vertexes.size());
				MapPoint3D64 centerPoint = MapPoint3D64_make(pos.get<0>(), pos.get<1>(), pos.get<2>());
				bool bGrap = GrapPointAlgorithm::grapOrMatchNearestPoint(centerPoint, left_HadLine.vertexes, grappedPt, si, ei, 10.0);
				bool bGrap1 = GrapPointAlgorithm::grapOrMatchNearestPoint(centerPoint, right_HadLine.vertexes, grappedPt1, si, ei, 10.0);

				point_t sp_m;
				point_t sp_1 = POINT_T(grappedPt);
				point_t sp_2 = POINT_T(grappedPt1);
				bg::divide_value(sp_1, 2.0);
				bg::divide_value(sp_2, 2.0);
				bg::convert(sp_1, sp_m);
				bg::add_point(sp_m, sp_2);
				double distance = bg::distance(pos, sp_m);
				if (distance < minDistance) {
					count = 0;
					nearbyLanes.push_back(hadlane);
					getNextLanes(hadlane, nearbyLanes, totalNum, count);
					count = 0;
					getPreviousLanes(hadlane, nearbyLanes, totalNum, count);
					nearyPosition = sp_m;//取左右两边中心点
					minDistance = distance;
				}
			}

		}
		if (withinRefLanes != 1) {
			nearbyLanes.clear();
		}
	}

	void TextCompiler::getNextLanes(const HadLane* lane, std::vector<HadLane*>& lanes, const int& totalNum, int& count)
	{
		if (!lane->next.empty())
		{
			for (const auto& nextLane : lane->next)
			{
				HadLane* tmpNextLane = (HadLane*)nextLane;
				if (std::find(lanes.begin(), lanes.end(), tmpNextLane) == lanes.end())
				{
					lanes.push_back(tmpNextLane);
				}
				count++;
				if (count < totalNum)
				{
					getNextLanes(tmpNextLane, lanes, totalNum, count);
				}
			}
		}
	}

	void TextCompiler::getPreviousLanes(const HadLane* lane, std::vector<HadLane*>& lanes, const int& totalNum, int& count)
	{
		if (!lane->previous.empty())
		{
			for (const auto& prevLane : lane->previous)
			{
				HadLane* tmpPrevLane = (HadLane*)prevLane;
				if (std::find(lanes.begin(), lanes.end(), tmpPrevLane) == lanes.end())
				{
					lanes.insert(lanes.begin(), tmpPrevLane);
				}
				count++;
				if (count < totalNum)
				{
					getPreviousLanes(tmpPrevLane, lanes, totalNum, count);
				}
			}
		}
	}

	std::vector<int> TextCompiler::getTimeNearbyObjects(
		HadText* currentText,
		const std::vector<HadText*>& texts)
	{
		std::vector<int> tmpIds(0);
		auto tmpVertexes = currentText->polygon.vertexes;
		tmpVertexes.resize(4);
		if (tmpVertexes.size() < 4)
			return tmpIds;
		auto tmpCenterVertex = getTextPolyCenter(currentText->polygon.vertexes);
		coordinatesTransform.convert(tmpVertexes.data(), tmpVertexes.size());
		Vector2 tmpDir = vec2(tmpVertexes[3].pos.lon - tmpVertexes[0].pos.lon, tmpVertexes[3].pos.lat - tmpVertexes[0].pos.lat);
		tmpDir.normalize();
		std::vector<point_t> tmpPts;
		for (const auto& tmpText : texts)
			tmpPts.push_back(POINT_T(getTextPolyCenter(tmpText->polygon.vertexes)));
		parameters param;
		index_getter originInd(tmpPts);
		rtree_type rtree(boost::irange<std::size_t>(0lu, tmpPts.size()), param, originInd);
		rtree.query(bgi::nearest(POINT_T(tmpCenterVertex), 10),
			boost::make_function_output_iterator([&](size_t const& id) {
				double tmpDistance = std::abs(bg::distance(POINT_T(tmpCenterVertex), tmpPts[id]));
				auto currentPoint = getTextPolyCenter(texts[id]->polygon.vertexes);
				Vector2 currentDir = vec2(currentPoint.pos.lon - tmpCenterVertex.pos.lon, currentPoint.pos.lat - tmpCenterVertex.pos.lat);
				double dirDis = dot(currentDir, tmpDir);
				//printInfo("id = %lld,dirDis = %g ,originId = %lld", id, dirDis,texts[id]->originId);
				if ((dirDis > 0 && std::abs(dirDis) < 50000) ||
					(dirDis < 0 && std::abs(dirDis) < 20000))
				{
					tmpIds.push_back(id);
				}
				}));
		return tmpIds;
	}

	bool TextCompiler::sortTimeNearbyObjects(
		HadText* baseText,
		std::vector<HadText*>& texts,
		std::vector<HadText*>& preTexts,
		std::vector<HadText*>& nextTexts)
	{
		if (texts.empty() && baseText != nullptr)
			return false;
		auto currentText = baseText;
		auto tmpVertexes = currentText->polygon.vertexes;
		tmpVertexes.resize(4);
		if (tmpVertexes.size() < 4)
			return false;
		auto tmpCenterVertex = getTextPolyCenter(currentText->polygon.vertexes);
		coordinatesTransform.convert(tmpVertexes.data(), tmpVertexes.size());
		Vector2 tmpDir = vec2(tmpVertexes[3].pos.lon - tmpVertexes[0].pos.lon, tmpVertexes[3].pos.lat - tmpVertexes[0].pos.lat);
		tmpDir.normalize();

		std::vector<point_t> tmpPts;
		for (const auto& tmpText : texts)
			tmpPts.push_back(POINT_T(getTextPolyCenter(tmpText->polygon.vertexes)));

		for (size_t i = 0; i < tmpPts.size(); ++i)
		{
			double tmpDistance = std::abs(bg::distance(POINT_T(tmpCenterVertex), tmpPts[i]));
			auto currentPoint = getTextPolyCenter(texts[i]->polygon.vertexes);
			Vector2 currentDir = vec2(currentPoint.pos.lon - tmpCenterVertex.pos.lon, currentPoint.pos.lat - tmpCenterVertex.pos.lat);
			double dirDis = dot(currentDir, tmpDir);
			if (dirDis < 0)
			{
				preTexts.push_back(texts[i]);
			}
			else
			{
				nextTexts.push_back(texts[i]);
			}
		}


		auto regconf = boost::regex::perl | boost::regex::icase;
		// 过滤掉“工作日7：00-9：00"(19927092)   工作日7：00-9：00(19927070)  19927071 "
		for (size_t i = 0; i < preTexts.size(); i++)
		{
			auto tmpContent = preTexts.at(i)->content;
			boost::algorithm::trim(tmpContent);
			if (boost::regex_search(tmpContent.data(), boost::wregex(L"^工作日\\s*\\d+\\s*:?：?\\s*\\d+\\s*\\-\\s*\\d+\\s*:?：?\\s*\\d+$", regconf)))
			{
				//printInfo("work time  (%lld,%lld)", preTexts.at(i)->postion.pos.lon / 1000, preTexts.at(i)->postion.pos.lat / 1000);
				return false;
			}
		}

		bool isBusOrHovAndTime = false;
		preTexts.erase(std::remove_if(preTexts.begin(), preTexts.end(), [&](const HadText* iItem) { 
			auto tmpContent = iItem->content;
			boost::algorithm::trim(tmpContent);
			if (boost::regex_search(tmpContent.data(), boost::wregex(L"^\\d+\\s*：\\s*\\d+\\s*\\-\\s*\\d+\\s*：\\s*\\d+$", regconf)) ||
				boost::regex_search(tmpContent.data(), boost::wregex(L"^工作日\\d+\\s*：\\s*\\d+\\s*\\-\\s*\\d+\\s*：\\s*\\d+$", regconf)) ||
				boost::regex_search(tmpContent.data(), boost::wregex(L"^\\d+\\s*:\\s*\\d+\\s*\\-\\s*\\d+\\s*:\\s*\\d+$", regconf)) ||
				boost::regex_search(tmpContent.data(), boost::wregex(L"^工作日\\s*\\d+\\s*:\\s*\\d+\\s*\\-\\s*\\d+\\s*:\\s*\\d+$", regconf)) ||
				boost::regex_search(tmpContent.data(), boost::wregex(L"^\\s*\\d+\\s*\\-\\s*\\d+\\s*$", regconf)) ||
				boost::regex_search(tmpContent.data(), boost::wregex(L"^\\s*工作日\\s*$", regconf)))
			{
				isBusOrHovAndTime = true;
				return false;
			}
			return true;
			}), preTexts.end());

		if (!isBusOrHovAndTime)
			return false;

		//20596606  (11668090,3989224) 同一位置有两个“7-9”
		if (preTexts.size() > 2)
		{
			auto sort_content_text = [](const HadText* s1, const HadText* s2)
			{
				return s1->content.compare(s2->content) == 0;
			};

			std::sort(preTexts.begin(), preTexts.end(), sort_content_text);
			auto last = std::unique(preTexts.begin(), preTexts.end(), sort_content_text);
			//去重，保留一个重复值
			preTexts.resize(distance(preTexts.begin(), last));

		}

		std::sort(preTexts.begin(), preTexts.end(), [&](HadText* a, HadText* b)->bool {
			auto aPoint = getTextPolyCenter(a->polygon.vertexes);
			auto bPoint = getTextPolyCenter(b->polygon.vertexes);
			Vector2 aDir = vec2(aPoint.pos.lon - tmpCenterVertex.pos.lon, aPoint.pos.lat - tmpCenterVertex.pos.lat);
			Vector2 bDir = vec2(bPoint.pos.lon - tmpCenterVertex.pos.lon, bPoint.pos.lat - tmpCenterVertex.pos.lat);
			return std::abs(dot(aDir, tmpDir)) < std::abs(dot(bDir, tmpDir)) ? true : false; });

		std::sort(nextTexts.begin(), nextTexts.end(), [&](HadText* a, HadText* b)->bool {
			auto aPoint = getTextPolyCenter(a->polygon.vertexes);
			auto bPoint = getTextPolyCenter(b->polygon.vertexes);
			Vector2 aDir = vec2(aPoint.pos.lon - tmpCenterVertex.pos.lon, aPoint.pos.lat - tmpCenterVertex.pos.lat);
			Vector2 bDir = vec2(bPoint.pos.lon - tmpCenterVertex.pos.lon, bPoint.pos.lat - tmpCenterVertex.pos.lat);
			return std::abs(dot(aDir, tmpDir)) < std::abs(dot(bDir, tmpDir)) ? true : false; });

		return true;
	}

	std::vector<MapPoint3D64> TextCompiler::getNearbyPoints(
		const MapPoint3D64& point,
		const std::vector<MapPoint3D64>& allPoints,
		int nearbyNum)
	{
		std::vector<point_t> tmpPts;
		for (const auto& tmpPoint : allPoints)
			tmpPts.push_back(POINT_T(tmpPoint));
		parameters param;
		index_getter originInd(tmpPts);
		rtree_type rtree(boost::irange<std::size_t>(0lu, tmpPts.size()), param, originInd);
		std::vector<MapPoint3D64> results;
		rtree.query(bgi::nearest(POINT_T(point), nearbyNum),
			boost::make_function_output_iterator([&](size_t const& id) {
				results.push_back(allPoints[id]);
				}));
		return results;
	}

	void TextCompiler::getNextLaneObjects(
		const HadLane* lane,
		std::vector<HadText*>& texts,
		const int& totalNum,
		int& count)
	{
		if (!lane->next.empty())
		{
			for (const auto& nextLane : lane->next)
			{
				HadLane* tmpNextLane = (HadLane*)nextLane;
				for (const auto& obj : tmpNextLane->objects)
				{
					if (obj->objectType == OMDB::ElementType::HAD_OBJECT_TEXT)
					{
						HadText* hadText = (HadText*)obj;
						std::vector<HadLane*> nearbyLanes;
						getNearbyLanes(hadText, nearbyLanes);
						if (!nearbyLanes.empty()) {
							if (std::find(nearbyLanes.begin(), nearbyLanes.end(), tmpNextLane) != nearbyLanes.end())
							{
								texts.push_back(hadText);
							}
						}
						else {
							texts.push_back(hadText);
						}
					}
				}
				count++;
				if (count < totalNum)
				{
					getNextLaneObjects(tmpNextLane, texts, totalNum, count);
				}
			}
		}
	}

	void TextCompiler::getPreviousLaneObjects(
		const HadLane* lane,
		std::vector<HadText*>& texts,
		const int& totalNum,
		int& count)
	{
		if (!lane->previous.empty())
		{
			for (const auto& prevLane : lane->previous)
			{
				HadLane* tmpPrevLane = (HadLane*)prevLane;
				for (const auto& obj : tmpPrevLane->objects)
				{
					if (obj->objectType == OMDB::ElementType::HAD_OBJECT_TEXT)
					{
						HadText* hadText = (HadText*)obj;
						std::vector<HadLane*> nearbyLanes;
						getNearbyLanes(hadText, nearbyLanes);
						if (!nearbyLanes.empty()) {
							if (std::find(nearbyLanes.begin(), nearbyLanes.end(), tmpPrevLane) != nearbyLanes.end())
							{
								texts.push_back(hadText);
							}
						}
						else {
							texts.push_back(hadText);
						}
					}
				}
				count++;
				if (count < totalNum)
				{
					getPreviousLaneObjects(tmpPrevLane, texts, totalNum, count);
				}
			}
		}
	}

	void TextCompiler::clearRepeatTexts(std::vector<HadText*>& texts)
	{
		if (texts.size() > 1)
		{
			std::sort(texts.begin(), texts.end(), [&](const HadText* a, const HadText* b)->bool { return a->originId < b->originId; });
			for (auto it = texts.begin(); it != texts.end() - 1;)
			{
				if ((*it)->originId == (*(it + 1))->originId)
					it = texts.erase(it);
				else
					it++;
			}
		}
	}

	void TextCompiler::breakSpeedLimitTextChains(std::vector<HadText*>& texts, std::vector<HadText*>& breakChainSpeedLimitTexts)
	{
		std::set<HadText*> speedLimitRangeTexts;
		for (auto breakChainSpeedLimitText : breakChainSpeedLimitTexts)
		{
			auto iter = std::find(texts.begin(), texts.end(), breakChainSpeedLimitText);
			if (iter != texts.end())
			{
				auto hadText = *iter;
				hadText->speedLimitType = HadText::SpeedLimitTextType::singleSpeed;
				speedLimitRangeTexts.emplace(hadText->speedLimitRangeText);
				hadText->speedLimitRangeText = nullptr;
				hadText->previousText = nullptr;
				hadText->nextText = nullptr;
			}
			else
			{
				breakChainSpeedLimitText->speedLimitType = HadText::SpeedLimitTextType::singleSpeed;
				speedLimitRangeTexts.emplace(breakChainSpeedLimitText->speedLimitRangeText);
				breakChainSpeedLimitText->speedLimitRangeText = nullptr;
				breakChainSpeedLimitText->previousText = nullptr;
				breakChainSpeedLimitText->nextText = nullptr;
				texts.push_back(breakChainSpeedLimitText);
			}
		}
		// 删除生成的限速连接文字
		for (auto speedLimitRangeText : speedLimitRangeTexts) {
			auto iter = std::find(texts.begin(), texts.end(), speedLimitRangeText);
			if (iter != texts.end())
			{
				texts.erase(iter);
			}
		}
	}

	void TextCompiler::clearRepeatTextsMulti(std::vector<std::vector<HadText*>>& texts)
	{

		if (texts.size() > 1)
		{
			for (auto it = texts.begin(); it != texts.end();)
			{
				if ((*it).empty())
					it = texts.erase(it);
				else
					it++;
			}
			std::sort(texts.begin(), texts.end(), [&](const std::vector<HadText*>& a, const std::vector<HadText*>& b)->bool { return a.back()->originId < b.back()->originId; });
			for (auto it = texts.begin(); it != texts.end() - 1;)
			{
				if ((*it).back()->originId == (*(it + 1)).back()->originId)
					it = texts.erase(it);
				else
					it++;
			}
		}
	}

	void TextCompiler::compileTexts(
		HadGrid* const pGrid,
		const std::vector<HadGrid*>& nearby,
		RdsTile* pTile)
	{
		UNREFERENCED_PARAMETER(pTile);
		UNREFERENCED_PARAMETER(nearby);
		m_Grid = pGrid;

		std::vector<HadText*> totalSpeedTexts;
		std::vector<std::vector<HadText*>> busTexts;
		std::vector<std::vector<HadText*>> hovTexts;
		std::vector<HadText*> speedLimitTexts;
		std::vector<HadText*> breakChainSpeedLimitTexts;
		auto regconf = boost::regex::perl | boost::regex::icase;
		int count;
		int totalNum = 4;
		for (auto& iItem : pGrid->query(ElementType::HAD_OBJECT_TEXT))
		{
			HadText* currentText = (HadText*)iItem;
			//if (currentText->originId == 84206667985765482)
			//		printInfo("");

			//判断是否为普通路
			if (CompileSetting::instance()->isNotCompileUrbanData)
			{
				if (!isProDataLevel(currentText->laneGroups))
					continue;
			}

			std::wstring tmpContent = currentText->content;
			boost::algorithm::trim(tmpContent);
			auto isBusContent = boost::regex_search(tmpContent.data(), boost::wregex(L"^公$", regconf));
			auto isHovContent = boost::regex_search(tmpContent.data(), boost::wregex(L"^多$", regconf));

			std::vector<HadText*> currentTexts;
			std::vector<HadLane*> currentLanes;
			getNearbyLanes(currentText, currentLanes);
			for (auto& jItem : currentText->refLanes)
			{
				HadLane* lane = (HadLane*)jItem;
				if (!currentLanes.empty()) {
					if (std::find(currentLanes.begin(), currentLanes.end(), lane) == currentLanes.end())
					{
						continue;
					}
				}
				for (const auto& obj : lane->objects)
				{
					if (obj->objectType == OMDB::ElementType::HAD_OBJECT_TEXT)
					{
						HadText* hadText = (HadText*)obj;
						std::vector<HadLane*> nearbyLanes;
						getNearbyLanes(hadText, nearbyLanes);
						if (!nearbyLanes.empty()) {
							if (std::find(nearbyLanes.begin(), nearbyLanes.end(), lane) != nearbyLanes.end())
							{
								currentTexts.push_back(hadText);
							}
						} else {
							currentTexts.push_back(hadText);
						}
					}
				}
				count = 0;
				getNextLaneObjects(lane, currentTexts, totalNum, count);
				count = 0;
				getPreviousLaneObjects(lane, currentTexts, totalNum, count);
				clearRepeatTexts(currentTexts);
			}

			if (boost::regex_search(tmpContent.data(), boost::wregex(L"^\\d+\\s*\\-\\s*\\d+$", regconf)))
			{
				std::vector<std::string> splitResults;
				boost::algorithm::split(splitResults, tmpContent, boost::algorithm::is_any_of("-"));
				if (splitResults.size() == 2)
				{
					if (IS_SPEED_LIMIT(std::atoi(splitResults[0].data())) &&
						IS_SPEED_LIMIT(std::atoi(splitResults[1].data())))
					{
						currentText->speedLimitType = HadText::SpeedLimitTextType::totalSpeed;
						totalSpeedTexts.push_back(currentText);
					}
				}
			}
			else if (isBusContent || isHovContent)
			{
				/*		if (currentText->originId == 170855069534077185)
						{
							printInfo("");
						}*/
				std::vector<int> tmpIds = getTimeNearbyObjects(currentText, currentTexts);
				std::vector<std::wstring> tmpBusStrings;
				std::vector<std::wstring> tmpHovStrings;
				std::vector<HadText*> tmpAllTexts;
				std::vector<HadText*> tmpBusTexts(4);
				std::vector<HadText*> tmpHovTexts(5);
				std::vector<bool> tmpBusAllOf(5, false);
				std::vector<bool> tmpHovAllOf(5, false);
				tmpBusTexts[0] = currentText;
				tmpHovTexts[0] = currentText;
				tmpBusAllOf[0] = true;
				tmpHovAllOf[0] = true;
				for (auto& id : tmpIds)
				{
					//bus
					if (isBusContent)
					{
						tmpBusStrings.push_back(currentTexts[id]->content);
						tmpAllTexts.push_back(currentTexts[id]);
						if (currentTexts[id]->content == L"公")
						{
						}
						else if (currentTexts[id]->content == L"交")
						{
							tmpBusTexts[1] = currentTexts[id];
							tmpBusAllOf[1] = true;
						}
						else if (currentTexts[id]->content == L"专")
						{
							tmpBusTexts[2] = currentTexts[id];
							tmpBusAllOf[2] = true;
						}
						else if (currentTexts[id]->content == L"用")
						{
							tmpBusTexts[3] = currentTexts[id];
							tmpBusAllOf[3] = true;
						}
						else if (currentTexts[id]->content == L"道")
						{
							tmpBusTexts[4] = currentTexts[id];
							tmpBusAllOf[4] = true;
						}
					
					}
						
					//hov
					if (isHovContent)
					{
						tmpHovStrings.push_back(currentTexts[id]->content);
						tmpAllTexts.push_back(currentTexts[id]);
						if (currentTexts[id]->content == L"多")
						{
						}
						else if (currentTexts[id]->content == L"乘")
						{
							tmpHovTexts[1] = currentTexts[id];
							tmpHovAllOf[1] = true;
						}
						else if (currentTexts[id]->content == L"员")
						{
							tmpHovTexts[2] = currentTexts[id];
							tmpHovAllOf[2] = true;
						}
						else if (currentTexts[id]->content == L"车" || currentTexts[id]->content == L"专")
						{
							tmpHovTexts[3] = currentTexts[id];
							tmpHovAllOf[3] = true;
						}
						else if (currentTexts[id]->content == L"道" || currentTexts[id]->content == L"用")
						{
							tmpHovTexts[4] = currentTexts[id];
							tmpHovAllOf[4] = true;
						}
					}

				}
					if (std::all_of(tmpBusAllOf.begin(), tmpBusAllOf.end(), [](bool tag) {return tag; }) 
						|| std::all_of(tmpBusAllOf.begin(), tmpBusAllOf.begin()+4, [](bool tag) {return tag; }))
				{
					std::vector<HadText*> preTexts;
					std::vector<HadText*> nextTexts;
					std::vector<HadText*> allTexts;
					if (sortTimeNearbyObjects(currentText, tmpAllTexts, preTexts, nextTexts))
					{
						if (preTexts.size() > 2)
							preTexts.resize(2);
						std::reverse(preTexts.begin(), preTexts.end());
						if (nextTexts.size() > 3)
							nextTexts.resize(3);
						allTexts.assign(preTexts.begin(), preTexts.end());
						allTexts.push_back(currentText);
						allTexts.insert(allTexts.end(), nextTexts.begin(), nextTexts.end());
						busTexts.push_back(allTexts);
					}
				}
				else if (std::all_of(tmpHovAllOf.begin(), tmpHovAllOf.end(), [](bool tag) {return tag; }))
				{
					std::vector<HadText*> preTexts;
					std::vector<HadText*> nextTexts;
					std::vector<HadText*> allTexts;
					if (sortTimeNearbyObjects(currentText, tmpAllTexts, preTexts, nextTexts))
					{
						if (preTexts.size() > 3)
							preTexts.resize(3);
						std::reverse(preTexts.begin(), preTexts.end());
						if (nextTexts.size() > 4)
							nextTexts.resize(4);
						allTexts.assign(preTexts.begin(), preTexts.end());
						allTexts.push_back(currentText);
						allTexts.insert(allTexts.end(), nextTexts.begin(), nextTexts.end());
						hovTexts.push_back(allTexts);
					}
				}
			}
			else
			{
				//int tmpTextValue = std::atoi(tmpContent.data());
				int tmpTextValue = _wtoi(tmpContent.data());
				std::vector<HadText*> tmpTexts;
				if (IS_SPEED_LIMIT(tmpTextValue))
				{
					bool isParallelText = false;
					std::vector<int> tmpIds = getNearbyObjects(currentText, currentTexts, 6, 1000, 50000, isParallelText);	// 同一条车道并排两个文字距离大概是 1700， min取1000是为了拿到这些数据
					if (isParallelText)
						continue;
					if (tmpIds.empty())
					{
						currentText->speedLimitType = HadText::SpeedLimitTextType::singleSpeed;
						speedLimitTexts.push_back(currentText);
					}
					else
					{
						if (tmpIds.size() == 1)
						{
							speedLimitTexts.push_back(currentText);
							auto nearbyText = currentTexts[tmpIds[0]];
							//if (std::atoi(currentText->content.data()) < std::atoi(nearbyText->content.data()))
							if (_wtoi(currentText->content.data()) < _wtoi(nearbyText->content.data()))
							{
								currentText->speedLimitType = HadText::SpeedLimitTextType::minSpeed;
								nearbyText->speedLimitType = HadText::SpeedLimitTextType::maxSpeed;
							}
							else if (_wtoi(currentText->content.data()) > _wtoi(nearbyText->content.data()))
							{
								currentText->speedLimitType = HadText::SpeedLimitTextType::maxSpeed;
								nearbyText->speedLimitType = HadText::SpeedLimitTextType::minSpeed;
							}
							HadText* speedLimitSymbolText;
							if (currentText->speedLimitRangeText == nullptr)
							{
								speedLimitSymbolText = (HadText*)pGrid->alloc(ElementType::HAD_OBJECT_TEXT);
								std::vector<MapPoint3D64> symbolPoly;
								speedLimitSymbolText->width = 0.0;
								speedLimitSymbolText->length = 0.0;
								speedLimitSymbolText->polygon.vertexes = symbolPoly;
								speedLimitSymbolText->objectType = OMDB::ElementType::HAD_OBJECT_TEXT;
								speedLimitSymbolText->color = OMDB::Color::WHITE;
								speedLimitSymbolText->content = L"-";
								speedLimitSymbolText->speedLimitType = HadText::SpeedLimitTextType::rangeSymbol;
								currentText->speedLimitRangeText = speedLimitSymbolText;
								nearbyText->speedLimitRangeText = speedLimitSymbolText;
								speedLimitTexts.push_back(speedLimitSymbolText);
							}
							else
							{
								speedLimitSymbolText = currentText->speedLimitRangeText;
							}

							//build text topo
							if (currentText->speedLimitType == HadText::SpeedLimitTextType::minSpeed && nearbyText->speedLimitType == HadText::SpeedLimitTextType::maxSpeed)
							{
								currentText->nextText = speedLimitSymbolText;
								speedLimitSymbolText->previousText = currentText;
								speedLimitSymbolText->nextText = nearbyText;
								nearbyText->previousText = speedLimitSymbolText;
							}
							else if (currentText->speedLimitType == HadText::SpeedLimitTextType::maxSpeed && nearbyText->speedLimitType == HadText::SpeedLimitTextType::minSpeed)
							{
								currentText->previousText = speedLimitSymbolText;
								speedLimitSymbolText->nextText = currentText;
								speedLimitSymbolText->previousText = nearbyText;
								nearbyText->nextText = speedLimitSymbolText;
							}
						}
						else if (tmpIds.size() == 2)
						{
							breakChainSpeedLimitTexts.push_back(currentText);
							for (auto tmpId : tmpIds)
							{
								auto nearbyText = currentTexts[tmpId];
								breakChainSpeedLimitTexts.push_back(nearbyText);
							}
						}
					}
				}
			}
		}

		//push text

		clearRepeatTexts(speedLimitTexts);
		breakSpeedLimitTextChains(speedLimitTexts, breakChainSpeedLimitTexts);
		for (auto& text : speedLimitTexts)
		{
			if (text->speedLimitType == HadText::SpeedLimitTextType::singleSpeed)
			{
				TextGroup tmpTextGroup;
				tmpTextGroup.texts = std::vector<HadText*>{ text };
				tmpTextGroup.type = RDS::RdsText::TextType::SPEED;
				if(pGrid->query(text->originId, OMDB::ElementType::HAD_OBJECT_TEXT) != nullptr)
					m_newTexts.push_back(tmpTextGroup);
			}
			else if (text->speedLimitType == HadText::SpeedLimitTextType::minSpeed)
			{
				std::vector<HadText*> tmpTexts;
				HadText* currentText;
				currentText = text;
				tmpTexts.push_back(currentText);
				int i = 0;
				while (!(currentText->nextText == nullptr) && i < 5)
				{
					tmpTexts.push_back(currentText->nextText);
					currentText = currentText->nextText;
					i++;
				}
				TextGroup tmpTextGroup;
				tmpTextGroup.texts = tmpTexts;
				tmpTextGroup.type = RDS::RdsText::TextType::SPEED;

				// 60 -100 跨网格问题，只有两个id 都不在当前
				if (!(pGrid->query(tmpTexts.back()->originId, OMDB::ElementType::HAD_OBJECT_TEXT) == nullptr
					&& pGrid->query(text->originId, OMDB::ElementType::HAD_OBJECT_TEXT) == nullptr))
					m_newTexts.push_back(tmpTextGroup);
			}
		}

		for (auto& text : totalSpeedTexts)
		{
			TextGroup tmpTextGroup;
			tmpTextGroup.texts = std::vector<HadText*>{ text };
			tmpTextGroup.type = RDS::RdsText::TextType::SPEED;
			if(pGrid->query(text->originId, OMDB::ElementType::HAD_OBJECT_TEXT) != nullptr)
				m_newTexts.push_back(tmpTextGroup);
		}

		clearRepeatTextsMulti(busTexts);
		clearRepeatTextsMulti(hovTexts);
		for (auto& item : busTexts)
		{
			TextGroup tmpTextGroup;
			tmpTextGroup.texts = item;
			tmpTextGroup.type = RDS::RdsText::TextType::BUS;
			if(pGrid->query(item.back()->originId, OMDB::ElementType::HAD_OBJECT_TEXT) != nullptr)
				m_newTexts.push_back(tmpTextGroup);
		}
		for (auto& item : hovTexts)
		{
			TextGroup tmpTextGroup;
			tmpTextGroup.texts = item;
			tmpTextGroup.type = RDS::RdsText::TextType::HOV;
			if(pGrid->query(item.back()->originId, OMDB::ElementType::HAD_OBJECT_TEXT) != nullptr)
				m_newTexts.push_back(tmpTextGroup);
		}

	}

	void TextCompiler::modifyTextGeometry(RdsTile* pTile, std::vector<std::vector<MapPoint3D64>>& allLines)
	{
		for (auto& tmpItem : allLines)
		{
			coordinatesTransform.convert(tmpItem.data(), tmpItem.size());
		}

		std::vector<segment_t> m_segments;
		std::vector<std::vector<size_t>> m_sizes;
		for (size_t i = 0; i < allLines.size(); ++i)
		{
			for (size_t j = 0; j < allLines[i].size() - 1; ++j)
			{
				std::vector<size_t> tmpSize;
				segment_t tmpSeg(POINT_T(allLines[i][j]), POINT_T(allLines[i][j + 1]));
				m_segments.push_back(tmpSeg);
				m_sizes.push_back(std::vector<size_t>{i, j});
			}
		}
		parameters param;
		index_getter_segment originInd(m_segments);
		rtree_type_segment	rtree(boost::irange<std::size_t>(0lu, m_segments.size()), param, originInd);
		for each(auto textGroup in m_newTexts)
		{
			if (textGroup.texts.empty())
			{
				continue;
			}
		
			
			HadText* hadText = textGroup.texts[0];
			auto polyPoints = hadText->polygon.vertexes;
			if (polyPoints.size() < 4)
				continue;
			coordinatesTransform.convert(polyPoints.data(), polyPoints.size());
			point_t p_1(0, 0, 0);
			linestring_t tmpLine = LINESTRING_T(polyPoints);
			for (size_t i = 0; i < 4; ++i)
				bg::add_point(p_1, tmpLine[i]);
			bg::divide_value(p_1, 4.0);

			point_t pos;
			Vector3 dir;
			double laneWidth;
			const int withinRefLanes = 1;
			MarkingCompiler::getObjectWidthAndDir(allLines, rtree, p_1, withinRefLanes, false, polyPoints, m_segments, m_sizes, pos, dir, laneWidth);
			laneWidth = 2500;
			std::vector<TextObj> textObjs;
			getTextsDis(getComboBoxType(textGroup.texts), textObjs);
			if (textObjs.size() != textGroup.texts.size())
				continue;

			std::vector<MapPoint3D64> polygon;
			std::vector<MapPoint3D64> polygon1;
			for (size_t i = 0; i < textObjs.size(); i++)
			{
				HadText* tmpHadText = textGroup.texts[i];
				if (tmpHadText->polygon.vertexes.size() < 4)
				{
					continue;
				}
				polygon.push_back(tmpHadText->polygon.vertexes.at(1));
				polygon.push_back(tmpHadText->polygon.vertexes.at(2));

				polygon1.push_back(tmpHadText->polygon.vertexes.at(0));
				polygon1.push_back(tmpHadText->polygon.vertexes.at(3));
			}
			for (size_t i = 0; i < textObjs.size(); i++)
			{
				TextObj obj = textObjs[i];
				HadText* tmpHadText = textGroup.texts[i];
	
				float w, h;
				getTextRate(obj.type, w, h);

				std::vector<MapPoint3D64> outPts;
				Vector3 upDir = vec3(0.0f, 0.0f, 1.0f);
				Vector3 lengthDir = dir * obj.dis * laneWidth;
				point_t lengthPoint(lengthDir.x, lengthDir.y, lengthDir.z);
				point_t basePoint = pos;
				add_point(basePoint, lengthPoint);


				////修复弯曲的道路，文字溢出路面问题
				point_t tmp_pos = basePoint;
				Vector3 tmp_dir = dir;
				if (textObjs.size() > 4)
				{
					getObjectPosAndDir(allLines, rtree, polygon, polygon1, basePoint, tmpHadText, m_segments, m_sizes, tmp_pos, tmp_dir);
				}

				MapPoint3D64 tmpPos = MapPoint3D64_make(tmp_pos.get<0>(), tmp_pos.get<1>(), tmp_pos.get<2>());
				MarkingCompiler::generateContourByPosAndDir(tmpPos, tmp_dir, upDir, w * laneWidth, h * laneWidth, outPts);

				coordinatesTransform.invert(outPts.data(), outPts.size());
				tmpHadText->polygon.vertexes.assign(outPts.begin(), outPts.end());
				addRdsText(pTile, tmpHadText, textGroup.type);
			}
		}
	}





	void TextCompiler::getObjectPosAndDir(
		const std::vector<std::vector<MapPoint3D64>>& allLines,
		const rtree_type_segment& rtree,
		std::vector<MapPoint3D64> polygon,
		std::vector<MapPoint3D64> polygon1,
		point_t pos,
		const HadText* hadText,
		const std::vector<segment_t>& segments,
		const std::vector<std::vector<size_t>>& sizes,
		point_t& tmp_pos,
		Vector3& dir)
	{
		if (hadText == nullptr 
			|| allLines.size() <= 0 
			|| segments.size() <= 0)
		{
			return;
		}

		if (polygon.size() <= 0 
			&& polygon1.size() <= 0)
		{
			return;
		}

		auto tmp_polyPoints = hadText->polygon.vertexes;
		if (tmp_polyPoints.size() < 4)
		{
			return;
		}

		coordinatesTransform.convert(tmp_polyPoints.data(), tmp_polyPoints.size());
		coordinatesTransform.convert(polygon.data(), polygon.size());
		coordinatesTransform.convert(polygon1.data(), polygon1.size());

		//投影，取得最近的投影信息。
		size_t si = 0, ei = 0;
		MapPoint3D64 grappedPt{ 0 };
		MapPoint3D64 grappedPt1{ 0 };
		MapPoint3D64 centerPoint = MapPoint3D64_make(pos.get<0>(), pos.get<1>(), pos.get<2>());
		bool bGrap = GrapPointAlgorithm::grapPoint(centerPoint, polygon.data(),polygon.size(), grappedPt, si, ei, 10.0);
		bool bGrap1 = GrapPointAlgorithm::grapPoint(centerPoint, polygon1.data(), polygon1.size(), grappedPt1, si, ei, 10.0);
		if (bGrap && bGrap1)
		{
			point_t sp_m;
			point_t sp_1 = POINT_T(grappedPt);
			point_t sp_2 = POINT_T(grappedPt1);
			bg::divide_value(sp_1, 2.0);
			bg::divide_value(sp_2, 2.0);
			bg::convert(sp_1, sp_m);
			bg::add_point(sp_m, sp_2);
			pos = sp_m;//取左右两边中心点
		}

		double tmp_laneWidth;
		const int withinRefLanes = 1;
		MarkingCompiler::getObjectWidthAndDir(allLines, rtree, pos, withinRefLanes, false, tmp_polyPoints, segments, sizes, tmp_pos, dir, tmp_laneWidth);

		// 校验坐标是否在面内
		Polygon3d originPolygon;
		MapPoint3D64 prevVertex = {};
		for_each(polygon.begin(), polygon.end(), [&](auto& vertex) {
			if (!floatEqual(vertex.pos.distanceSquare(prevVertex.pos), 0)) {
				originPolygon.vertexes.push_back(vertex);
				prevVertex = vertex;
			}
		});
		for_each(polygon1.rbegin(), polygon1.rend(), [&](auto& vertex) {
			if (!floatEqual(vertex.pos.distanceSquare(prevVertex.pos), 0)) {
				originPolygon.vertexes.push_back(vertex);
				prevVertex = vertex;
			}
		});

		// 闭环
		MapPoint3D64& startPt = originPolygon.vertexes.front();
		MapPoint3D64& endPt = originPolygon.vertexes.back();
		if (!floatEqual(startPt.pos.distanceSquare(endPt.pos), 0)) {
			originPolygon.vertexes.push_back(startPt);
		}
		centerPoint = MapPoint3D64_make(pos.get<0>(), pos.get<1>(), pos.get<2>());
		MapPoint3D64 newPoint = MapPoint3D64_make(tmp_pos.get<0>(), tmp_pos.get<1>(), tmp_pos.get<2>());
		if (!GrapPointAlgorithm::isPointInPolygon(newPoint, originPolygon.vertexes) && 
			GrapPointAlgorithm::isPointInPolygon(centerPoint, originPolygon.vertexes)) {
			tmp_pos = pos;
		}
	}


	void TextCompiler::addRdsText(
		RdsTile* pTile, 
		HadText* hadText, 
		RDS::RdsText::TextType type)
	{
		if (pTile == nullptr || hadText == nullptr)
		{
			return;
		}
		
		RdsText* pText = (RdsText*)createObject(pTile, EntityType::RDS_TEXT);
		pText->type = type;

		char szDest[1024]{0};
		WideCharToMultiByte(CP_UTF8, 0, hadText->content.c_str(), -1, szDest, 1024, NULL, NULL);
		pText->text = szDest;
		boost::algorithm::trim(pText->text);

		pText->color = (RdsText::TextColor)hadText->color;
		pText->speedLimitType = (RDS::RdsText::SpeedLimitTextType)hadText->speedLimitType;
		Polygon3d polygon = hadText->polygon;
		assert(polygon.vertexes.size() > 4);
		polygon.vertexes.resize(4);
		convert(polygon, pText->contour);
		for (auto laneGroup : hadText->laneGroups) {
			RdsGroup* pRdsGroup = queryGroup(laneGroup->originId, pTile);
			if (pRdsGroup)
				pRdsGroup->objects.push_back(pText);
		}
	}
	
	TextCompiler::ComboBoxType TextCompiler::getComboBoxType(
		const std::vector<HadText*>& texts)
	{
		ComboBoxType textType;
		size_t idx = texts[0]->content.find(L":");
		switch (texts.size())
		{
		case 5:
		{
			if (idx != std::string::npos)
			{
				textType = ComboBoxType::CBT_FTIME1_BUS;
			}
			else
			{
				textType = ComboBoxType::CBT_ITIME1_BUS;
			}	
		}
		break;
		case 6:
		{
			if (idx != std::string::npos)
			{
				textType = ComboBoxType::CBT_FTIME2_BUS;
			}
			else
			{
				textType = ComboBoxType::CBT_ITIME2_BUS;
			}
		}
		break;
		case 7:
		{
			if (idx != std::string::npos)
			{
				textType = ComboBoxType::CBT_FTIME1_HOV;
			}
			else
			{
				textType = ComboBoxType::CBT_ITIME1_HOV;
			}
		}
		break;
		case 8:
		{
			if (idx != std::string::npos)
			{
				textType = ComboBoxType::CBT_FTIME2_HOV;
			}
			else
			{
				textType = ComboBoxType::CBT_ITIME2_HOV;
			}
		}
		break;
		}

		//add speed limit type
		if (texts[0]->speedLimitType == HadText::SpeedLimitTextType::singleSpeed)
		{
			textType = ComboBoxType::CBT_SINGLE_SPEED_LIMIT;
		}
		else if (texts[0]->speedLimitType == HadText::SpeedLimitTextType::totalSpeed)
		{
			textType = ComboBoxType::CBT_TOTAL_SPEED_LIMIT;
		}
		else if (texts[0]->speedLimitType == HadText::SpeedLimitTextType::minSpeed && texts.size() == 3)
		{
			textType = ComboBoxType::CBT_RANGE_SPEED_LIMIT;
		}

		return textType;
	}

	void TextCompiler::getTextRate(TextDataType tType, float& fw, float& fh)
	{
		switch (tType)
		{
		case OMDB::TextCompiler::TextDataType::TCT_TIME_ONE2ONE:
		case OMDB::TextCompiler::TextDataType::TCT_TIME_ONE2TWO:
		case OMDB::TextCompiler::TextDataType::TCT_TIME_TWO2TWO:
		{
			fw = 0.8f;
			fh = 0.4f;
		}
		break;
		case OMDB::TextCompiler::TextDataType::TCT_TIME_ONE2ONE_L:
		case OMDB::TextCompiler::TextDataType::TCT_TIME_ONE2TWO_L:
		case OMDB::TextCompiler::TextDataType::TCT_TIME_TWO2TWO_L:
		{
			fw = 0.8f;
			fh = 0.4f;
		}
		break;
		case OMDB::TextCompiler::TextDataType::TCT_CHINESE_ONE:
		{
			fw = 0.5f;
			fh = 1.0f;
		}
		break;
		case OMDB::TextCompiler::TextDataType::TCT_CHINESE_TWO:
		case OMDB::TextCompiler::TextDataType::TCT_CHINESE_THREE:
		{
			fw = 0.8f;
			fh = 0.4f;
		}
		break;
		case OMDB::TextCompiler::TextDataType::TCT_SINGLE_SPEED_LIMIT:
		case OMDB::TextCompiler::TextDataType::TCT_MIN_SPEED_LIMIT:
		case OMDB::TextCompiler::TextDataType::TCT_MAX_SPEED_LIMIT:
		{
			fw = 0.6f;
			fh = 0.6f;
		}
		break;
		case OMDB::TextCompiler::TextDataType::TCT_RANGE_SYMBOL:
		{
			fw = 0.05f;
			fh = 0.5f;
		}
		break;
		case OMDB::TextCompiler::TextDataType::TCT_TOTAL_SPEED_LIMIT:
		{
			fw = 1.0f;
			fh = 0.6f;
		}
		break;
		default:
		{
			fw = 0.5f;
			fh = 0.6f;
		}
		break;
		}
	}

	float TextCompiler::getIntervalDisRate(int iType)
	{
		switch (iType)
		{
		case 0:
		{
			return 0.4f;
		}
		break;
		case 1:
		case 2:
		{
			return 0.8f;
		}
		break;
		}
		return 0.5f;
	}

	void TextCompiler::getTextsDis(ComboBoxType textType, std::vector<TextObj>& outTextObjs)
	{
		float dis1 = getIntervalDisRate(0);
		float dis2 = getIntervalDisRate(1);
		float dis3 = getIntervalDisRate(2);
		float dw1, dh1, dw2, dh2, dw3, dh3, dw4, dh4;
		getTextRate(TextDataType::TCT_TIME_ONE2ONE, dw1, dh1);
		getTextRate(TextDataType::TCT_TIME_ONE2ONE_L, dw2, dh2);
		getTextRate(TextDataType::TCT_CHINESE_ONE, dw3, dh3);
		getTextRate(TextDataType::TCT_CHINESE_THREE, dw4, dh4);
		switch (textType)
		{
		case OMDB::TextCompiler::ComboBoxType::CBT_ITIME1_BUS:
		case OMDB::TextCompiler::ComboBoxType::CBT_FTIME1_BUS:
		{
			TextObj obj;
			obj.dis = 0;
			obj.type = TextDataType::TCT_TIME_ONE2ONE;
			outTextObjs.push_back(obj);
			float len1 = dh1 * 0.5f + dh3 * 0.5f + dis2;
			obj.dis = len1;
			obj.type = TextDataType::TCT_CHINESE_ONE;
			outTextObjs.push_back(obj);
			for (size_t i = 1; i < 4; i++)
			{
				TextObj obj;
				obj.dis = len1 + i * (dis3 + dh3);
				obj.type = TextDataType::TCT_CHINESE_ONE;
				outTextObjs.push_back(obj);
			}
		}
		break;
		case OMDB::TextCompiler::ComboBoxType::CBT_ITIME2_BUS:
		case OMDB::TextCompiler::ComboBoxType::CBT_FTIME2_BUS:
		{
			TextObj obj;
			obj.dis = 0;
			obj.type = TextDataType::TCT_TIME_ONE2ONE;
			outTextObjs.push_back(obj);
			obj.dis = dis1 + dh1;
			obj.type = TextDataType::TCT_TIME_ONE2ONE;
			outTextObjs.push_back(obj);
			float len1 = dis1 + dh1 + dh1 * 0.5f + dh3 * 0.5f + dis2;
			obj.dis = len1;
			obj.type = TextDataType::TCT_CHINESE_ONE;
			outTextObjs.push_back(obj);
			for (size_t i = 1; i < 4; i++)
			{
				TextObj obj;
				obj.dis = len1 +  i * (dis3 +  dh3);
				obj.type = TextDataType::TCT_CHINESE_ONE;
				outTextObjs.push_back(obj);
			}
		}
		break;
		case OMDB::TextCompiler::ComboBoxType::CBT_ITIME1_HOV:
		case OMDB::TextCompiler::ComboBoxType::CBT_FTIME1_HOV:
		{
			TextObj obj;
			obj.dis = 0;
			obj.type = TextDataType::TCT_TIME_ONE2ONE;
			outTextObjs.push_back(obj);
			obj.dis = dis1 + (dh1 + dh4) * 0.5f;
			obj.type = TextDataType::TCT_CHINESE_THREE;
			outTextObjs.push_back(obj);
			float len1 =  dh1 * 0.5f + dis1 + dh4 + dis2 + dh3 * 0.5f;
			obj.dis = len1;
			obj.type = TextDataType::TCT_CHINESE_ONE;
			outTextObjs.push_back(obj);
			for (size_t i = 1; i < 5; i++)
			{
				TextObj obj;
				obj.dis = len1 + i * (dis3 + dh3);
				obj.type = TextDataType::TCT_CHINESE_ONE;
				outTextObjs.push_back(obj);
			}
		}
		break;
		case OMDB::TextCompiler::ComboBoxType::CBT_ITIME2_HOV:
		case OMDB::TextCompiler::ComboBoxType::CBT_FTIME2_HOV:
		{
			TextObj obj;
			obj.dis = 0;
			obj.type = TextDataType::TCT_TIME_ONE2ONE;
			outTextObjs.push_back(obj);
			obj.dis = dis1 + dh1;
			obj.type = TextDataType::TCT_TIME_ONE2ONE;
			outTextObjs.push_back(obj);
			obj.dis = dh1 * 0.5f + dis1 + dh1 + dis1 + dh4 * 0.5f;
			obj.type = TextDataType::TCT_CHINESE_THREE;
			outTextObjs.push_back(obj);
			float len1 = dh1 * 0.5f + dis1 + dh1 + dis1 + dh4 + dis2 + dh3 * 0.5f;
			obj.dis = len1;
			obj.type = TextDataType::TCT_CHINESE_ONE;
			outTextObjs.push_back(obj);
			for (size_t i = 1; i < 5; i++)
			{
				TextObj obj;
				obj.dis = len1 + i * (dis3 + dh3);
				obj.type = TextDataType::TCT_CHINESE_ONE;
				outTextObjs.push_back(obj);
			}
		}
		break;
		case OMDB::TextCompiler::ComboBoxType::CBT_SINGLE_SPEED_LIMIT:
		{
			TextObj obj;
			obj.dis = 0;
			obj.type = TextDataType::TCT_SINGLE_SPEED_LIMIT;
			outTextObjs.push_back(obj);
		}
		break;
		case OMDB::TextCompiler::ComboBoxType::CBT_TOTAL_SPEED_LIMIT:
		{
			TextObj obj;
			obj.dis = 0;
			obj.type = TextDataType::TCT_TOTAL_SPEED_LIMIT;
			outTextObjs.push_back(obj);
		}
		break;
		case OMDB::TextCompiler::ComboBoxType::CBT_RANGE_SPEED_LIMIT:
		{
			TextObj obj;
			float dis;
			float width, height;
			
			obj.dis = 0;
			obj.type = TextDataType::TCT_MIN_SPEED_LIMIT;
			outTextObjs.push_back(obj);

			getTextRate(TextDataType::TCT_MIN_SPEED_LIMIT, width, height);
			obj.dis += height / 2;
			obj.dis += getIntervalDisRate(1);
			getTextRate(TextDataType::TCT_RANGE_SYMBOL, width, height);
			obj.dis += height / 2;
			obj.type = TextDataType::TCT_RANGE_SYMBOL;
			outTextObjs.push_back(obj);

			getTextRate(TextDataType::TCT_RANGE_SYMBOL, width, height);
			obj.dis += height / 2;
			obj.dis += getIntervalDisRate(1);
			getTextRate(TextDataType::TCT_MAX_SPEED_LIMIT, width, height);
			obj.dis += height / 2;
			obj.type = TextDataType::TCT_MAX_SPEED_LIMIT;
			outTextObjs.push_back(obj);
		}
		break;
		default:
			break;
		}
	}

}

