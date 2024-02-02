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

		bool isOutputRoutingData{ false };				// Ĭ�ϲ����mapbox��ʽ�����ļ�
		bool isNotCompileUrbanData{ false };			// Ĭ�ϱ�����ͨ·
		bool isGenerateHdData{ false };				    // Ĭ�ϲ�����LINK���ɸ߾�����
		bool isModifyHeightFromAbsToRel{ false };		// Ĭ�ϲ������߶�
		bool isDaimlerShangHai{ false };		        // Ĭ�ϲ��Ǵ�ķ����Ŀ
		bool isCompileTurnWaiting{ false };             // Ĭ�ϲ������ת��
		bool isCompileTransparency{ false };            // Ĭ�ϲ�����߼ܰ�͸��
		bool isWriteSpatialite{false};				    // Ĭ�ϲ�д��Spatialite����

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

