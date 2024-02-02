#pragma once

#include "Compiler.h"
#include "MarkingCompiler.h"

namespace OMDB
{  
    class TextCompiler : public Compiler
    {
    public:
        TextCompiler(CompilerData& data) :Compiler(data) {};
        enum class ComboBoxType : uint8
        {
            UNKNOWN = 0,
            CBT_ITIME1_BUS = 1,
            CBT_ITIME2_BUS = 2,
            CBT_FTIME1_BUS = 3,
            CBT_FTIME2_BUS = 4,

            CBT_ITIME1_HOV = 5,
            CBT_ITIME2_HOV = 6,
            CBT_FTIME1_HOV = 7,
            CBT_FTIME2_HOV = 8,

            CBT_SINGLE_SPEED_LIMIT = 9,
            CBT_TOTAL_SPEED_LIMIT = 10,
            CBT_RANGE_SPEED_LIMIT = 11,
        };

        enum class TextDataType : uint8
        {
            TCT_TIME_ONE2ONE = 0,
            TCT_TIME_ONE2TWO = 1,
            TCT_TIME_TWO2TWO = 2,

            TCT_TIME_ONE2ONE_L = 3,
            TCT_TIME_ONE2TWO_L = 4,
            TCT_TIME_TWO2TWO_L = 5,

            TCT_CHINESE_ONE = 6,
            TCT_CHINESE_TWO = 7,
            TCT_CHINESE_THREE = 8,

            TCT_SINGLE_SPEED_LIMIT = 9,
            TCT_MIN_SPEED_LIMIT = 10,
            TCT_MAX_SPEED_LIMIT = 11,
            TCT_RANGE_SYMBOL = 12,
            TCT_TOTAL_SPEED_LIMIT = 13,

        };

        struct TextObj
        {
            float dis;
            TextDataType type;
        };

        struct TextGroup
        {
            std::vector<HadText*> texts;
            RDS::RdsText::TextType type;
        };

    protected:
        virtual void compile(
            HadGrid* const pGrid, 
            const std::vector<HadGrid*>& nearby, 
            RdsTile* pTile) override;

		void processCrossGrid(HadGrid* const pGrid,
			HadText* hadText,
			std::vector<std::vector<MapPoint3D64>>& allLines);

        void compileTexts(
            HadGrid* const pGrid, 
            const std::vector<HadGrid*>& nearby, 
            RdsTile* pTile);

        void modifyTextGeometry(
            RdsTile* pTile,
            std::vector<std::vector<MapPoint3D64>>& allLines);

        void addRdsText(
            RdsTile* pTile, 
            HadText* hadText, 
            RDS::RdsText::TextType type);

        void getTextRate(
            TextDataType tType, 
            float& fw, 
            float& fh);

        // 0 时间和时间，1 时间和文字， 2 文字和文字
        float getIntervalDisRate(
            int iType); 

        /** @note *nextIndexOut in [1, polyline.size() - 1] if succ. */
        void getTextsDis(
            ComboBoxType textType, 
            std::vector<TextObj>& outTextObjs);

        ComboBoxType getComboBoxType(
            const std::vector<HadText*>& texts);

        std::vector<int> getNearbyObjects(
            HadText* currentText,
            const std::vector<HadText*>& texts,
            int nearbyNum,
            int64 minValue,
            int64 maxValue,
            bool& isParallel);

        void getNearbyLanes(
            HadText* currentText,
            std::vector<HadLane*>& nearbyLanes);

		void getNextLanes(
			const HadLane* lane,
			std::vector<HadLane*>& lanes,
			const int& totalNum,
			int& count);

		void getPreviousLanes(
			const HadLane* lane,
			std::vector<HadLane*>& lanes,
			const int& totalNum,
			int& count);

        std::vector<int> getTimeNearbyObjects(
            HadText* currentText,
            const std::vector<HadText*>& texts);

        std::vector<MapPoint3D64> getNearbyPoints(
            const MapPoint3D64& point,
            const std::vector<MapPoint3D64>& allPoints,
            int nearbyNum);

        bool sortTimeNearbyObjects(
		    HadText* baseText,
            std::vector<HadText*>& texts,
            std::vector<HadText*>& preTexts,
            std::vector<HadText*>& nextTexts);

        void getNextLaneObjects(
            const HadLane* lane,
            std::vector<HadText*>& texts,
            const int& totalNum,
            int& count);

        void getPreviousLaneObjects(
            const HadLane* lane,
            std::vector<HadText*>& texts,
            const int& totalNum,
            int& count);

        void clearRepeatTexts(
            std::vector<HadText*>& texts);

		void breakSpeedLimitTextChains(
			std::vector<HadText*>& texts,
            std::vector<HadText*>& breakChainSpeedLimitTexts);

        void clearRepeatTextsMulti(
            std::vector<std::vector<HadText*>>& texts);


		void getObjectPosAndDir(
			const std::vector<std::vector<MapPoint3D64>>& allLines,
			const rtree_type_segment& rtree,
            std::vector<MapPoint3D64> polygon,
            std::vector<MapPoint3D64> polygon1,
            point_t pos,
			const HadText* hadText,
			const std::vector<segment_t>& segments,
            const std::vector<std::vector<size_t>>& sizes,
            point_t& tmp_pos,
            Vector3& dir);


        forceinline MapPoint3D64 getTextPolyCenter(
            std::vector<MapPoint3D64> points)
        {
            point_t p_1(0, 0, 0);
            if (!points.empty())
            {
                coordinatesTransform.convert(points.data(), points.size());
            }
            linestring_t tmpLine = LINESTRING_T(points);
            if (tmpLine.size() > 3)
            {
                for (size_t i = 0; i < 4; ++i)
                    bg::add_point(p_1, tmpLine[i]);
                bg::divide_value(p_1, 4.0);
            }
            return MapPoint3D64_make(p_1.get<0>(), p_1.get<1>(), p_1.get<2>() / 10);
        }

        std::vector<TextGroup> m_newTexts;
        HadGrid* m_Grid;
  
    };
}
 
