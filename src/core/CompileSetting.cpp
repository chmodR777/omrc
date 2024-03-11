#include "stdafx.h"
#include "CompileSetting.h"
namespace OMDB
{

	CompileSetting::CompileSetting()
		:threadNumber(4)
	{
	}

	OMDB::CompileSetting CompileSetting::g_sInstance;

	bool CompileSetting::load(const IniFile* ini)
	{
		const IniSection* defaultSection = ini->sectionWithName(NULL);

		const cqWCHAR* value = defaultSection->valueWithName(L"sourceDir");
		if (value == NULL) {
			printError("Failed to find `sourceDir` in omdb.ini");
			return false;
		}
		ToString(value, sourceDir);

		value = defaultSection->valueWithName(L"outputDir");
		if (value == NULL) {
			printError("Failed to find `outputDir` in omdb.ini");
			return false;
		}
		ToString(value, outputDir);

		value = defaultSection->valueWithName(L"omrpDir");
		ToString(value, omrpDir);

		value = defaultSection->valueWithName(L"threadNumber");
		if (value == NULL) {
			// default 4 thread
			threadNumber = 4;
		} else {
			threadNumber = cq_wtoi(value);
			if (threadNumber < 1) {
				printError("Invalid threadNumber(%S) in omdb.ini", value);
				return false;
			}
		}

		value = defaultSection->valueWithName(L"capacityNumber");
		if (value == NULL) {
			// default 1000 
			capacityNumber = 1000;
		}
		else {
			capacityNumber = cq_wtoi(value);
			if (capacityNumber < 1) {
				printError("Invalid capacityNumber(%S) in omdb.ini", value);
				return false;
			}
		}

		value = defaultSection->valueWithName(L"dataVersion");
		if (value == NULL) {
			dataVersion = "";
		}
		else {
			ToString(value, dataVersion);
		}

		value = defaultSection->valueWithName(L"chinaRefMapFile");
		if (value == NULL) {
			chinaRefMapFile = "";
		}
		ToString(value, chinaRefMapFile);

		//调整高度网格划分层数
		value = defaultSection->valueWithName(L"boundNumber");
		if (value == NULL) {
			boundNumber = 4;
		}
		else {
			boundNumber = cq_wtoi(value);
			if (boundNumber < 1) {
				printError("Invalid boundNumber(%S) in omdb.ini", value);
				return false;
			}
		}

		value = defaultSection->valueWithName(L"meshIdsForCompiling");
		ToString(value, meshIdsForCompiling);

		auto setOption = [&](bool& isAnything, const std::wstring& tmpString)-> void 
		{
			value = defaultSection->valueWithName(tmpString.data());
			if (value != NULL)
			{
				if (cq_wcscmp(value, L"false") == 0 || cq_wcscmp(value, L"FALSE") == 0)
					isAnything = false;
				else if (cq_wcscmp(value, L"true") == 0 || cq_wcscmp(value, L"TRUE") == 0)
					isAnything = true;
				else
					printError("Invalid (%S)(%S) in omdb.ini", tmpString.data(), value);
			}
		};

		setOption(isOutputRoutingData, L"isOutputRoutingData");
		setOption(isOutputHdRoutingData, L"isOutputHdRoutingData");
		setOption(isNotCompileUrbanData, L"isNotCompileUrbanData");
		setOption(isGenerateHdData, L"isGenerateHdData");
		setOption(isModifyHeightFromAbsToRel, L"isModifyHeightFromAbsToRel");
		setOption(isDaimlerShangHai, L"isDaimlerShangHai");
		setOption(isCompileTurnWaiting, L"isCompileTurnWaiting");
		setOption(isWriteSpatialite, L"isWriteSpatialite");
		setOption(isCompileNdsData, L"isCompileNdsData");
		return true;
	}

	bool CompileSetting::applyCompilerOptionsAndCheck(const CompilerOptions* options)
	{
		if (!options->sourceDir.empty()) {
			sourceDir = options->sourceDir;
		}

		if (!options->outputDir.empty()) {
			outputDir = options->outputDir;
		} else if (sourceDir.empty()){
			printError("Must specified `outputDir` in omdb.ini or `--outputDir` in command line arguments");
			return false;
		}

		if (!options->dataVersion.empty()) {
			dataVersion = options->dataVersion;
		}

		if (options->threadNumber) {
			int num = options->threadNumber;
			if (num < 1) {
				printWarning("Invalid --threadNumber argument: %S, use value from INI: %d",
					options->threadNumber, threadNumber);
			} else {
				threadNumber = num;
			}
		}

		if (options->capacityNumber) {
			int num = options->capacityNumber;
			if (num < 1) {
				printWarning("Invalid --capacityNumber argument: %S, use value from INI: %d",
					options->capacityNumber, capacityNumber);
			}
			else {
				capacityNumber = num;
			}
		}

		if (!options->isOutputRoutingData.empty())
		{
			if (options->isOutputRoutingData == "false" || options->isOutputRoutingData == "FALSE")
				isOutputRoutingData = false;
			else if (options->isOutputRoutingData == "true" || options->isOutputRoutingData == "TRUE")
				isOutputRoutingData = true;
			else
			{
				printWarning("Invalid --isOutputRoutingData argument: %S, use value from INI: %s",
					options->isOutputRoutingData, isOutputRoutingData ? "true" : "false");
			}
		}

		if (!options->isOutputHdRoutingData.empty())
		{
			if (options->isOutputHdRoutingData == "false" || options->isOutputHdRoutingData == "FALSE")
				isOutputHdRoutingData = false;
			else if (options->isOutputHdRoutingData == "true" || options->isOutputHdRoutingData == "TRUE")
				isOutputHdRoutingData = true;
			else
			{
				printWarning("Invalid --isOutputHdRoutingData argument: %S, use value from INI: %s",
					options->isOutputHdRoutingData, isOutputHdRoutingData ? "true" : "false");
			}
		}

		if (!options->chinaRefMapFile.empty()) {
			chinaRefMapFile = options->chinaRefMapFile;
		}
		else if (chinaRefMapFile.empty()) {
			chinaRefMapFile = "";
		}

		if (!options->isNotCompileUrbanData.empty())
		{
			if (options->isNotCompileUrbanData == "false" || options->isNotCompileUrbanData == "FALSE")
				isNotCompileUrbanData = false;
			else if (options->isNotCompileUrbanData == "true" || options->isNotCompileUrbanData == "TRUE")
				isNotCompileUrbanData = true;
			else
			{
				printWarning("Invalid --isNotCompileUrbanData argument: %S, use value from INI: %s",
					options->isNotCompileUrbanData, isNotCompileUrbanData ? "true" : "false");
			}
		}

		if (!options->isGenerateHdData.empty())
		{
			if (options->isGenerateHdData == "false" || options->isGenerateHdData == "FALSE")
				isGenerateHdData = false;
			else if (options->isGenerateHdData == "true" || options->isGenerateHdData == "TRUE")
				isGenerateHdData = true;
			else
			{
				printWarning("Invalid --isGenerateHdData argument: %S, use value from INI: %s",
					options->isGenerateHdData, isGenerateHdData ? "true" : "false");
			}
		}

		if (!options->isModifyHeightFromAbsToRel.empty())
		{
			if (options->isModifyHeightFromAbsToRel == "false" || options->isModifyHeightFromAbsToRel == "FALSE")
				isModifyHeightFromAbsToRel = false;
			else if (options->isModifyHeightFromAbsToRel == "true" || options->isModifyHeightFromAbsToRel == "TRUE")
				isModifyHeightFromAbsToRel = true;
			else
			{
				printWarning("Invalid --isModifyHeightFromAbsToRel argument: %S, use value from INI: %s",
					options->isModifyHeightFromAbsToRel, isModifyHeightFromAbsToRel ? "true" : "false");
			}
		}

		if (!options->isCompileTurnWaiting.empty())
		{
			if (options->isCompileTurnWaiting == "false" || options->isCompileTurnWaiting == "FALSE")
				isCompileTurnWaiting = false;
			else if (options->isCompileTurnWaiting == "true" || options->isCompileTurnWaiting == "TRUE")
				isCompileTurnWaiting = true;
			else
			{
				printWarning("Invalid --isCompileTurnWaiting argument: %S, use value from INI: %s",
					options->isCompileTurnWaiting, isCompileTurnWaiting ? "true" : "false");
			}
		}

		return true;
	}

	void CompileSetting::printSettings() const
	{
		printInfo("SourceDir                     : %s", sourceDir.c_str());
		printInfo("OutputDir                     : %s", outputDir.c_str());
		printInfo("omrpDir                       : %s", omrpDir.c_str());
		printInfo("dataVersion                   : %s", dataVersion.c_str());
		printInfo("meshIdsForCompiling           : %s", meshIdsForCompiling.c_str());
		printInfo("ThreadNumber                  : %d", threadNumber);
		printInfo("boundNumber                   : %d", boundNumber);
		printInfo("isOutputRoutingData           : %d", isOutputRoutingData);
		printInfo("isOutputHdRoutingData         : %d", isOutputHdRoutingData);
		printInfo("isNotCompileUrbanData         : %d", isNotCompileUrbanData);
		printInfo("isGenerateHdData              : %d", isGenerateHdData);
		printInfo("isModifyHeightFromAbsToRel    : %d", isModifyHeightFromAbsToRel);
		printInfo("isDaimlerShangHai             : %d", isDaimlerShangHai);
		printInfo("isCompileTurnWaiting          : %d", isCompileTurnWaiting);
		printInfo("isWriteSpatialite             : %d", isWriteSpatialite);
		printInfo("isCompileNdsData              : %d", isCompileNdsData);
	}

	OMDB::CompileSetting* CompileSetting::instance()
	{
		return &g_sInstance;
	}

}
