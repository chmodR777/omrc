#include "stdafx.h"

#include "TollGateCompiler.h"
#include <algorithm>
#include "algorithm/grap_point_algorithm.h"
#include "map_point3d64_converter.h"
#include "math3d/vector_math.h"
#include "boost/filesystem/fstream.hpp"
#include <codecvt>
using namespace RDS;
namespace OMDB
{
	/// @brief 编译收费站
	/// @param pDatabase
	/// @param pRDSDatabase
	void TollGateCompiler::compile(HadGrid* const pGrid, const std::vector<HadGrid*>& nearby, RdsTile* pTile)
	{
		
		UNREFERENCED_PARAMETER(nearby);
		m_vctPostion.clear();

		auto dTolls = pGrid->query(ElementType::HAD_TOLLGATE);
		for (auto obj : pGrid->query(ElementType::HAD_TOLLGATE))
		{
			m_pTollGate = (HadTollGate*)obj;
			//printInfo("收费站图幅=%d，收费站id=%I64d，收费站坐标是(%I64d,%I64d)", pGrid->getId(), m_pTollGate->originId, m_pTollGate->position.pos.lon/1000, m_pTollGate->position.pos.lat/1000);



			m_pRdsToll = nullptr;	//每个新的RDS收费站先置空，如果确实有效，则再创建。
			MapPoint3D64 ptFoot = { 0 };	//收费站PA点在最左侧边界线上的投影点。
			bool bStart = false;	//收费站位置是否处在车道中心线的起始位置附近。
			HadLaneGroup* langGroup = nullptr;
			size_t nLaneSize = m_pTollGate->tollLanes.size();
			m_TollLanes.clear();
			bool isCreateToll = true;
			for (size_t i = 0; i < nLaneSize; i++)
			{
				auto tmpTollLane = m_pTollGate->tollLanes[i];
				m_pLane = (HadLane*)pGrid->query(tmpTollLane->laneLinkPid, ElementType::HAD_LANE);
				if (m_pLane == nullptr)
					continue;

				if (m_pLane->linkGroup == nullptr)	//目前原始数据上存在这种情况：有车道，但没有对应的车道组。
					continue;

				langGroup = m_pLane->linkGroup;

				//判断是否为普通路
				if (CompileSetting::instance()->isNotCompileUrbanData)
				{
					if (!isProDataLevel(langGroup))
					{
						isCreateToll = false;
						break;
					}
				}

				CQ_ASSERT(tmpTollLane->seqNum > 0);
				//CQ_ASSERT(m_pLane->linkGroup->laneBoundaries.size() > 1)
				CQ_ASSERT(m_pLane->leftBoundary != nullptr && m_pLane->rightBoundary != nullptr);

				if (i == 0)
				{
					MapPoint3D64 pt = _nearestLineStringEnd(m_pTollGate->position, m_pLane->location);
					int64 frontDis = _squaredHDistance(m_pTollGate->position, m_pLane->location.vertexes.front());
					int64 backDis = _squaredHDistance(m_pTollGate->position, m_pLane->location.vertexes.back());
					bStart = frontDis < backDis ? true : false;

					//收费站PA点投影到最左侧边界线上。(最右侧收费岛不画；PA点不一定在车道组范围内)
					HadLaneBoundary* pBaseBoundary = nullptr;
					//std::vector<MapPoint3D64> polyline = m_pLane->linkGroup->laneBoundaries[0]->location.vertexes;
					std::vector<MapPoint3D64> polyline = m_pLane->leftBoundary->location.vertexes;
					if (polyline.front() != m_pTollGate->position && polyline.back() != m_pTollGate->position)
						//pBaseBoundary = m_pLane->linkGroup->laneBoundaries[0];
						pBaseBoundary = m_pLane->leftBoundary;
					else
					{
						//pBaseBoundary = m_pLane->linkGroup->laneBoundaries[m_pLane->linkGroup->laneBoundaries.size() - 1];
						HadTollGate::PtrTollLane oTollLane = m_pTollGate->tollLanes[nLaneSize-1];
						HadLane* pLane = (HadLane*)pGrid->query(oTollLane->laneLinkPid, ElementType::HAD_LANE);
						CQ_ASSERT(pLane != nullptr);
						pBaseBoundary = pLane->rightBoundary;
					}

					MapPoint3D64 grapedFt;
					size_t si = 0, ei = 0;
					auto position = m_pTollGate->position;
					coordinatesTransform.convert(&position, 1);
					auto locationVertexes = pBaseBoundary->location.vertexes;
					coordinatesTransform.convert(locationVertexes.data(), locationVertexes.size());
					if (GrapPointAlgorithm::grapOrMatchNearestPoint(position, locationVertexes, grapedFt, si, ei, 100.0)) {
						coordinatesTransform.invert(&grapedFt, 1);
						ptFoot = grapedFt;
					}
					else
					{
						ptFoot = _calFootOfPerpendicular(pBaseBoundary->location.vertexes.front(), pBaseBoundary->location.vertexes.back(), m_pTollGate->position);
					}
				}

				MapPoint3D64 ptLaneEnd = { 0 };
				if (bStart)
					ptLaneEnd = m_pLane->location.vertexes.front();
				else
					ptLaneEnd = m_pLane->location.vertexes.back();

				//添加收费岛
				//_addRdsTollPassageway(m_oTollLane.seqNum-1, ptLaneEnd, ptFoot, pTile);
				_addRdsTollPassageway(m_pLane->leftBoundary, tmpTollLane, ptLaneEnd, ptFoot, pTile);
				m_TollLanes.push_back(tmpTollLane);

				//最右侧增加一个虚拟收费岛。
				if (i + 1 == nLaneSize)
				{
					HadTollGate::PtrTollLane hadTollLane = std::make_shared<HadTollGate::TollLane>();
					hadTollLane->laneLinkPid = tmpTollLane->laneLinkPid;
					hadTollLane->payType = tmpTollLane->payType;
					hadTollLane->seqNum = tmpTollLane->seqNum;
					hadTollLane->tollPoint = tmpTollLane->tollPoint;
					//_addRdsTollPassageway(m_oTollLane.seqNum, ptLaneEnd, ptFoot, pTile);
					_addRdsTollPassageway(m_pLane->rightBoundary, hadTollLane, ptLaneEnd, ptFoot, pTile);
					m_TollLanes.push_back(hadTollLane);
				}
			}

			if (!isCreateToll)
				continue;

			if (m_pRdsToll != nullptr)
			{
				std::vector<MapPoint3D64> tmpPoints;
				std::vector<MapPoint3D64> resultPoints;
				if (m_TollLanes.size() > 1)
				{
					for (size_t i = 0; i < m_TollLanes.size(); ++i)
					{
						auto tmpToll = m_TollLanes[i];
						tmpPoints.push_back(tmpToll->tollPoint);
					}
				}
				coordinatesTransform.convert(tmpPoints.data(), tmpPoints.size());
				linestring_t tmpLine = LINESTRING_T(tmpPoints);
				auto tmpDis = bg::length(tmpLine);
				double tollTmpNum = tmpDis / 3500.0;
				int tollNum = static_cast<int>(std::round(tollTmpNum));
				if (tollTmpNum < 0.9 || tollNum == 0)
					continue;
				double tmpValue = tmpDis / tollNum;
				vector_t tollNV = V3_N(S3_V3(tmpLine.back(), tmpLine.front()));
				point_t p = tmpLine.front();
				resultPoints.push_back(MapPoint3D64_make(p.get<0>(), p.get<1>(), p.get<2>() / 10));
				point_t prePoint = p;
				if (tollNum > 0)
				{
					for (size_t i = 0; i < tollNum - 1; ++i)
					{
						p = point_t(
							prePoint.get<0>() + tollNV.get<0>() * tmpValue,
							prePoint.get<1>() + tollNV.get<1>() * tmpValue,
							prePoint.get<2>() + tollNV.get<2>() * tmpValue);
						prePoint = p;
						resultPoints.push_back(MapPoint3D64_make(p.get<0>(), p.get<1>(), p.get<2>() / 10));
					}
				}
				p = tmpLine.back();
				resultPoints.push_back(MapPoint3D64_make(p.get<0>(), p.get<1>(), p.get<2>() / 10));
				coordinatesTransform.invert(resultPoints.data(), resultPoints.size());
				for (size_t i = 0; i < resultPoints.size(); ++i)
				{
					Point3d pt3d;
					convert(resultPoints.at(i), pt3d);
					m_pRdsToll->positions.postions.push_back(pt3d);
					if (i < m_TollLanes.size())
					{
						auto tmpToll = m_TollLanes.at(i);
						m_pRdsToll->typeList.push_back((RdsToll::LanePayType)tmpToll->payType);
					}
					else
					{
						auto tmpToll = m_TollLanes.back();
						m_pRdsToll->typeList.push_back((RdsToll::LanePayType)tmpToll->payType);
					}
				}
				_setRdsTollGate(langGroup);
				RdsGroup* pRdsGroup = queryGroup(langGroup->originId, pTile);
				if (pRdsGroup)
				{
					pRdsGroup->objects.push_back(m_pRdsToll);
				}
			}
		}


		
	}

	/// @note 因车道的左右边界线数据不全，故使用车道组内的边界线信息。
	void TollGateCompiler::_addRdsTollPassageway(
		const HadLaneBoundary* pBoundary, 
        const HadTollGate::PtrTollLane& pTollLane,
		MapPoint3D64 ptLaneEnd, 
		MapPoint3D64 ptFoot, 
		RdsTile* pTile)
	{
		if (m_pRdsToll == nullptr)
		{
			m_pRdsToll = (RdsToll*)createObject(pTile, EntityType::RDS_TOLL);
			m_pRdsToll->typeList.reserve(30);
		}

		//判定收费站位置处于边界线起点附近还是终点附近。处于终点附近为正向。
		MapPoint3D64 ptNearest = _nearestLineStringEnd(ptLaneEnd, pBoundary->location);
		std::vector<MapPoint3D64> polyline = pBoundary->location.vertexes;
		bool bForward = (ptNearest == polyline.back()) ? true : false;

		//对边界线最后一节线段所在直线做投影，计算投影点。
		MapPoint3D64 pt1 = { 0 }, pt2 = { 0 };
		if (bForward)
		{
			pt1 = polyline[polyline.size() - 2];
			pt2 = polyline[polyline.size() - 1];
		}
		else
		{
			pt1 = polyline[1];
			pt2 = polyline[0];
		}
		MapPoint3D64 ptGate = _calIntersection(pt1, pt2, m_pTollGate->position, ptFoot);
		ptGate.z += 20; // 抬高一点收费站,避免被路面压盖
		pTollLane->tollPoint = ptGate;
	}

	void TollGateCompiler::_setRdsTollGate(HadLaneGroup* pLaneGroup)
	{
		if (m_pRdsToll == nullptr)
			return;

		_recalPosition();
		std::vector<Point3d>& ptPiers = m_pRdsToll->positions.postions;

		CQ_ASSERT(ptPiers.size() > 1);
		MapPoint3D64 pt1 = {}, pt2 = {};
		convert(ptPiers.front(), pt1);
		convert(ptPiers.back(), pt2);
		Vector3d vDirection = calDirection(pt1, pt2);

		// 旧的计算角度方法,只是计算第四象限是不对的,只有在垂直或平行x轴时才对
		m_pRdsToll->angle = (RDS::int32)(atan2l(-vDirection.y, vDirection.x) * 1e6);

		// 尝试使用路面求方向
		LineString3d leftSide, rightSide;
		getLaneGroupBoundary(pLaneGroup, leftSide, rightSide);
		if (!leftSide.vertexes.empty() && !rightSide.vertexes.empty()) {
			size_t si = 0, ei = 0;
			MapPoint3D64 grappedPt{};
			bool grappedBoundaryAveragePt = false;
			if (boundaryAverage(leftSide) > 0.99)	// arccos(0.99) = 8.1 degree
			{
				if (GrapPointAlgorithm::grapOrMatchNearestPoint(pt1, leftSide.vertexes, grappedPt, si, ei, 10.0)) {
					pt1 = leftSide.vertexes[si];
					pt2 = leftSide.vertexes[ei];
					grappedBoundaryAveragePt = true;
				}
			} else if (boundaryAverage(rightSide) > 0.99)
			{
				if (GrapPointAlgorithm::grapOrMatchNearestPoint(pt1, rightSide.vertexes, grappedPt, si, ei, 10.0)) {
					pt1 = rightSide.vertexes[si];
					pt2 = rightSide.vertexes[ei];
					grappedBoundaryAveragePt = true;
				}
			}
			if (grappedBoundaryAveragePt) {
				// 有些计算的不全对,部分方向得加/减个M_PI
				//Vector3d hDirection = calDirection(pt2, pt1);
				//m_pRdsToll->angle = (RDS::int32)(atan2l(hDirection.y, hDirection.x) * 1e6);
			}
		}

		std::wstring optTollName;
		optimizeName(m_pTollGate->tollName, optTollName);
		char szDest[1024]{0};
		
		if(optTollName != L"")
			WideCharToMultiByte(CP_UTF8, 0, optTollName.c_str(), -1, szDest, 1024, NULL, NULL);
		else
			WideCharToMultiByte(CP_UTF8, 0, L"收费站", -1, szDest, 1024, NULL, NULL);
		for (size_t i = 0; i < strlen(szDest); i++)
			m_pRdsToll->name.push_back(szDest[i]);
	}

	double TollGateCompiler::boundaryAverage(const LineString3d& originLine)
	{
		if (originLine.vertexes.size() > 1)
		{
			point_t pStart = POINT_T(originLine.vertexes.front());
			point_t pEnd = POINT_T(originLine.vertexes.back());
			vector_t v = V3_N(S3_V3(pStart, pEnd));
			double vSum = 0.0;
			double totalLength = 0.0;
			for (size_t i = 1; i < originLine.vertexes.size(); ++i)
			{
				point_t pa = POINT_T(originLine.vertexes.at(i - 1));
				point_t pb = POINT_T(originLine.vertexes.at(i));
				vector_t tmpV = V3_N(S3_V3(pa, pb));
				vSum += bg::dot_product(v, tmpV);
				totalLength += bg::distance(pa, pb);
			}
			double vAverage = vSum / (originLine.vertexes.size() - 1);
			return totalLength > 5000 ? vAverage : DBL_MIN;
		}
		return DBL_MIN;
	}

	void TollGateCompiler::_recalPosition()
	{
		if (m_pRdsToll == nullptr)
			return;

		//只要一个收费通道（2个点）的收费站，不需要再次计算收费岛位置。
		if (m_pRdsToll->positions.postions.size() <= 2)
			return;

		//转换成以距离为单位的笛卡尔坐标系，坐标单位：米。
		std::vector<Point3d>& ptPiers = m_pRdsToll->positions.postions;
		MapPoint3D64 start = { 0 }, end = { 0 };
		convert(ptPiers.front(), start);
		convert(ptPiers.back(), end);

		MapPoint3D64Converter c;
		c.setBasePoint(start);
		c.convert(&start, 1);
		c.convert(&end, 1);

		Vector3 vStart = { (float)start.pos.lon, (float)start.pos.lat, (float)start.z };
		Vector3 vEnd = { (float)end.pos.lon, (float)end.pos.lat, (float)end.z };

		cqstd::vector<Vector3> relPoints;
		relPoints.push_back(vStart);
		relPoints.push_back(vEnd);

		//重采样
		float length = sqrtf(vEnd.x * vEnd.x + vEnd.y * vEnd.y);
		float interval = length / (m_pRdsToll->positions.postions.size() - 1);
		m_vctPostion.clear();
		Polyline_resample3D(relPoints.begin(), relPoints.size(), interval, 0, _resampleCallbackWrapper, this);

		//赋值
		for( int i=0; i< m_vctPostion.size(); i++)
		{
			MapPoint3D64 pt = { {(int64)m_vctPostion[i].x,(int64)m_vctPostion[i].y}, (int64)m_vctPostion[i].z };
			c.invert(&pt, 1);
			RDS::Point3d rdsPoint;
			convert(pt, rdsPoint);
			m_pRdsToll->positions.postions[i] = rdsPoint;
		}
	}
	
	void TollGateCompiler::_resampleCallback(Vector3 point, Vector3 dir)
	{
		UNUSED_VAR(dir);
		m_vctPostion.push_back(point);
	}

    void TollGateCompiler::optimizeName() {
       
		boost::filesystem::ifstream fileIn;
        boost::filesystem::ofstream fileOut;
		fileIn.open("收费站名称.txt", std::ios::app);
		fileOut.open("优化后收费站名称.txt", std::ios::app);

        if (fileIn.is_open()) {
            // 文件打开成功，可以进行读取操作
            std::string line;
            // 定义一个宽字符串转换器
            std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
            while (std::getline(fileIn, line)) {
                // 处理每一行数据
                // 将 std::string 转换为 std::wstring
                std::wstring wstr = converter.from_bytes(line);
				std::wstring wstrOpt;
				optimizeName(wstr, wstrOpt);


				std::string strOut = converter.to_bytes(wstr + L"\t" + wstrOpt + L"\n");
				fileOut << strOut;
		
               
            }
			fileIn.close();
        }

		fileOut.close();

    }


	void TollGateCompiler::optimizeName(const std::wstring& tollName, std::wstring& optimizeTollName)
    {
	
        auto regconf = boost::regex::perl | boost::regex::icase;

		optimizeTollName = tollName;
      
        //含()且含有”收费站 ->删除 ()以及()中内容,删除后缀“收费站”
		//常德北（太阳山）收费站 ->常德北
		if(boost::regex_search(optimizeTollName.data(), boost::wregex(L"（.*）.*收费站$", regconf)))
		{
			optimizeTollName = boost::regex_replace(optimizeTollName, boost::wregex(L"（.*）|收费站$"), L"");
		}
        //含字母、数字且含有“收费站”->删除字母,删除数字,删除后缀“收费站,注意数据中是全角字符
		//Ｇ１５０３牡丹江路收费站->牡丹江路
        else if (boost::regex_search(optimizeTollName.data(), boost::wregex(L"(?=.*[ａ-ｚＡ-Ｚ])(?=.*[０-９]).*收费站$", regconf)))
        {
			optimizeTollName = boost::regex_replace(optimizeTollName, boost::wregex(L"[ａ-ｚＡ-Ｚ０-９]|收费站$"), L"");
			
        }
        //全字母含空格 ->替换内容为“收费站,注意数据中是全角字符
        //Ｓｕｉｘｉａｎ　Ｗｅｓｔ　Ｔｏｌｌ　Ｇａｔｅ ->收费站
        else if (boost::regex_search(optimizeTollName.data(), boost::wregex(L"^[ａ-ｚＡ-Ｚ０-９]+$", regconf)))
        {
			optimizeTollName = L"收费站";
        }
		//含·且含有收费站->删除后缀“收费站”
		//越秀·鄢陵北收费站->越秀·鄙陵
        else if (boost::regex_search(optimizeTollName.data(), boost::wregex(L"·.*收费站$", regconf)))
        {
			optimizeTollName = boost::regex_replace(optimizeTollName, boost::wregex(L"收费站$"), L"");
	
        }
        //大于4个字符且含有“收费站”->删除后缀“收费站”
      // 汶川收费站->汶川
		  else if (boost::regex_search(optimizeTollName.data(), boost::wregex(L".{2,}收费站$", regconf)))
		  {
			optimizeTollName = boost::regex_replace(optimizeTollName, boost::wregex(L"收费站$"), L"");
		  }
		  // 大于4个字符且没有“收费站" ->替换内容为"收费站
		  //绕城高速成仁站D出口 ->收费站
		  else if (boost::regex_search(optimizeTollName.data(), boost::wregex(L"^(?!.*收费站).{4,}", regconf)))
		  {
			  optimizeTollName = L"收费站";
		  }
		if (optimizeTollName.length() >= 5)
		{
			optimizeTollName = L"收费站";
		}
	
    }

    MapPoint3D64 TollGateCompiler::_calFootOfPerpendicular(MapPoint3D64 pt1, MapPoint3D64 pt2, MapPoint3D64 pt3)
	{
		MapPoint3D64Converter c;
		c.setBasePoint(pt1);
		c.convert(&pt1, 1);
		c.convert(&pt2, 1);
		c.convert(&pt3, 1);

		int64 a = pt2.pos.lon - pt1.pos.lon;
		int64 b = pt2.pos.lat - pt1.pos.lat;
		double t = (a * (pt1.pos.lat - pt3.pos.lat) + b * (pt3.pos.lon - pt1.pos.lon));
		t = t / (a * a + b * b);//分两步计算，直接一步实现会由于精度问题导致结果出错

		MapPoint3D64 pt = { 0 };
		pt.pos.lon = -b * t + pt3.pos.lon;
		pt.pos.lat = a * t + pt3.pos.lat;
		if (pt.pos.distance(pt1.pos) < pt.pos.distance(pt2.pos)) {
			pt.z = pt1.z;
		} else {
			pt.z = pt2.z;
		}
		// pt.z = max(pt1.z, pt2.z);

		c.invert(&pt, 1);

		return pt;
	}

	MapPoint3D64 TollGateCompiler::_calIntersection(MapPoint3D64 pt1, MapPoint3D64 pt2, MapPoint3D64 pt3, MapPoint3D64 pt4)
	{
		MapPoint3D64Converter c;
		c.setBasePoint(pt1);
		c.convert(&pt1, 1);
		c.convert(&pt2, 1);
		c.convert(&pt3, 1);
		c.convert(&pt4, 1);

		double a1, a2, a3, b1, b2, b3;
		a1 = pt4.pos.lat - pt3.pos.lat;
		a2 = pt3.pos.lat - pt1.pos.lat;
		a3 = pt2.pos.lat - pt1.pos.lat;
		b1 = pt4.pos.lon - pt3.pos.lon;
		b2 = pt3.pos.lon - pt1.pos.lon;
		b3 = pt2.pos.lon - pt1.pos.lon;
		double t = (a3 * b2 - a2 * b3) / (a1 * b3 - a3 * b1);

		MapPoint3D64 pt;
		pt.pos.lon = (pt4.pos.lon - pt3.pos.lon) * t + pt3.pos.lon;
		pt.pos.lat = (pt4.pos.lat - pt3.pos.lat) * t + pt3.pos.lat;
		if (pt.pos.distance(pt1.pos) < pt.pos.distance(pt2.pos)) {
			pt.z = pt1.z;
		} else {
			pt.z = pt2.z;
		}
		// pt.z = max(pt1.z, pt2.z);

		c.invert(&pt, 1);

		return pt;
	}

}

