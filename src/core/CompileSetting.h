#pragma once
#include <string>
namespace OMDB
{
	struct CompilerOptions
	{
		std::string sourceDir;
		std::string outputDir;
		std::string dataVersion;
		int threadNumber;
		int capacityNumber;
		std::string isOutputRoutingData;
		std::string chinaRefMapFile;
		std::string isNotCompileUrbanData;
		std::string isGenerateHdData;
		std::string isModifyHeightFromAbsToRel;
		int boundNumber;
		std::string isDaimlerShangHai;
		std::string isCompileTurnWaiting;
		std::string isWriteSpatialite;
	};

	class CompileSetting
	{
	private:
		CompileSetting();
		static CompileSetting g_sInstance;

	public:
		std::string sourceDir;
		std::string outputDir;
		std::string dataVersion;
		int threadNumber;
		int capacityNumber;
		std::string chinaRefMapFile;
		int boundNumber;
		int writeSpatialiteFileMeshId{ 0 };

		bool isOutputRoutingData{ false };				// 默认不输出mapbox格式数据文件
		bool isNotCompileUrbanData{ false };			// 默认编译普通路
		bool isGenerateHdData{ false };				    // 默认不根据LINK生成高精数据
		bool isModifyHeightFromAbsToRel{ false };		// 默认不调整高度
		bool isDaimlerShangHai{ false };		        // 默认不是戴姆勒项目
		bool isCompileTurnWaiting{ false };             // 默认不编译待转区
		bool isCompileTransparency{ false };            // 默认不编译高架半透明
		bool isWriteSpatialite{false};				    // 默认不写出Spatialite数据

		bool load(const IniFile* ini);
		bool applyCompilerOptionsAndCheck(const CompilerOptions* options);
		void printSettings() const;

		static CompileSetting* instance();
		
	};

	forceinline void ToString(const wchar_t* wchar, std::string& szDst)
	{
		if (wchar == nullptr)
			return;

		DWORD dwNum = WideCharToMultiByte(CP_OEMCP, NULL, wchar, -1, NULL, 0, NULL, FALSE);
		char* psText;
		psText = new char[dwNum];
		WideCharToMultiByte(CP_OEMCP, NULL, wchar, -1, psText, dwNum, NULL, FALSE);
		szDst = psText;
		delete[]psText;
	}

}

