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
	/// @brief �����շ�վ
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
			//printInfo("�շ�վͼ��=%d���շ�վid=%I64d���շ�վ������(%I64d,%I64d)", pGrid->getId(), m_pTollGate->originId, m_pTollGate->position.pos.lon/1000, m_pTollGate->position.pos.lat/1000);



			m_pRdsToll = nullptr;	//ÿ���µ�RDS�շ�վ���ÿգ����ȷʵ��Ч�����ٴ�����
			MapPoint3D64 ptFoot = { 0 };	//�շ�վPA���������߽����ϵ�ͶӰ�㡣
			bool bStart = false;	//�շ�վλ���Ƿ��ڳ��������ߵ���ʼλ�ø�����
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

				if (m_pLane->linkGroup == nullptr)	//Ŀǰԭʼ�����ϴ�������������г�������û�ж�Ӧ�ĳ����顣
					continue;

				langGroup = m_pLane->linkGroup;

				//�ж��Ƿ�Ϊ��ͨ·
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

					//�շ�վPA��ͶӰ�������߽����ϡ�(���Ҳ��շѵ�������PA�㲻һ���ڳ����鷶Χ��)
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

				//����շѵ�
				//_addRdsTollPassageway(m_oTollLane.seqNum-1, ptLaneEnd, ptFoot, pTile);
				_addRdsTollPassageway(m_pLane->leftBoundary, tmpTollLane, ptLaneEnd, ptFoot, pTile);
				m_TollLanes.push_back(tmpTollLane);

				//���Ҳ�����һ�������շѵ���
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

	/// @note �򳵵������ұ߽������ݲ�ȫ����ʹ�ó������ڵı߽�����Ϣ��
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

		//�ж��շ�վλ�ô��ڱ߽�����㸽�������յ㸽���������յ㸽��Ϊ����
		MapPoint3D64 ptNearest = _nearestLineStringEnd(ptLaneEnd, pBoundary->location);
		std::vector<MapPoint3D64> polyline = pBoundary->location.vertexes;
		bool bForward = (ptNearest == polyline.back()) ? true : false;

		//�Ա߽������һ���߶�����ֱ����ͶӰ������ͶӰ�㡣
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
		ptGate.z += 20; // ̧��һ���շ�վ,���ⱻ·��ѹ��
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

		// �ɵļ���Ƕȷ���,ֻ�Ǽ�����������ǲ��Ե�,ֻ���ڴ�ֱ��ƽ��x��ʱ�Ŷ�
		m_pRdsToll->angle = (RDS::int32)(atan2l(-vDirection.y, vDirection.x) * 1e6);

		// ����ʹ��·������
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
				// ��Щ����Ĳ�ȫ��,���ַ���ü�/����M_PI
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
			WideCharToMultiByte(CP_UTF8, 0, L"�շ�վ", -1, szDest, 1024, NULL, NULL);
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

		//ֻҪһ���շ�ͨ����2���㣩���շ�վ������Ҫ�ٴμ����շѵ�λ�á�
		if (m_pRdsToll->positions.postions.size() <= 2)
			return;

		//ת�����Ծ���Ϊ��λ�ĵѿ�������ϵ�����굥λ���ס�
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

		//�ز���
		float length = sqrtf(vEnd.x * vEnd.x + vEnd.y * vEnd.y);
		float interval = length / (m_pRdsToll->positions.postions.size() - 1);
		m_vctPostion.clear();
		Polyline_resample3D(relPoints.begin(), relPoints.size(), interval, 0, _resampleCallbackWrapper, this);

		//��ֵ
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
		fileIn.open("�շ�վ����.txt", std::ios::app);
		fileOut.open("�Ż����շ�վ����.txt", std::ios::app);

        if (fileIn.is_open()) {
            // �ļ��򿪳ɹ������Խ��ж�ȡ����
            std::string line;
            // ����һ�����ַ���ת����
            std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
            while (std::getline(fileIn, line)) {
                // ����ÿһ������
                // �� std::string ת��Ϊ std::wstring
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
      
        //��()�Һ��С��շ�վ ->ɾ�� ()�Լ�()������,ɾ����׺���շ�վ��
		//���±���̫��ɽ���շ�վ ->���±�
		if(boost::regex_search(optimizeTollName.data(), boost::wregex(L"��.*��.*�շ�վ$", regconf)))
		{
			optimizeTollName = boost::regex_replace(optimizeTollName, boost::wregex(L"��.*��|�շ�վ$"), L"");
		}
        //����ĸ�������Һ��С��շ�վ��->ɾ����ĸ,ɾ������,ɾ����׺���շ�վ,ע����������ȫ���ַ�
		//�ǣ�������ĵ����·�շ�վ->ĵ����·
        else if (boost::regex_search(optimizeTollName.data(), boost::wregex(L"(?=.*[��-����-��])(?=.*[��-��]).*�շ�վ$", regconf)))
        {
			optimizeTollName = boost::regex_replace(optimizeTollName, boost::wregex(L"[��-����-�ڣ�-��]|�շ�վ$"), L"");
			
        }
        //ȫ��ĸ���ո� ->�滻����Ϊ���շ�վ,ע����������ȫ���ַ�
        //�ӣ��������ף������ԣ��졡�ǣ���� ->�շ�վ
        else if (boost::regex_search(optimizeTollName.data(), boost::wregex(L"^[��-����-�ڣ�-��]+$", regconf)))
        {
			optimizeTollName = L"�շ�վ";
        }
		//�����Һ����շ�վ->ɾ����׺���շ�վ��
		//Խ�㡤۳�걱�շ�վ->Խ�㡤����
        else if (boost::regex_search(optimizeTollName.data(), boost::wregex(L"��.*�շ�վ$", regconf)))
        {
			optimizeTollName = boost::regex_replace(optimizeTollName, boost::wregex(L"�շ�վ$"), L"");
	
        }
        //����4���ַ��Һ��С��շ�վ��->ɾ����׺���շ�վ��
      // �봨�շ�վ->�봨
		  else if (boost::regex_search(optimizeTollName.data(), boost::wregex(L".{2,}�շ�վ$", regconf)))
		  {
			optimizeTollName = boost::regex_replace(optimizeTollName, boost::wregex(L"�շ�վ$"), L"");
		  }
		  // ����4���ַ���û�С��շ�վ" ->�滻����Ϊ"�շ�վ
		  //�ƳǸ��ٳ���վD���� ->�շ�վ
		  else if (boost::regex_search(optimizeTollName.data(), boost::wregex(L"^(?!.*�շ�վ).{4,}", regconf)))
		  {
			  optimizeTollName = L"�շ�վ";
		  }
		if (optimizeTollName.length() >= 5)
		{
			optimizeTollName = L"�շ�վ";
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
		t = t / (a * a + b * b);//���������㣬ֱ��һ��ʵ�ֻ����ھ������⵼�½������

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

