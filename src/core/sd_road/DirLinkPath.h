#pragma once

#include "DirLink.h"

#include <vector>
#include <memory>


namespace OMDB
{
namespace sd
{

	/**
	 * @brief 表示由一个或多个部分(Portion)依次序连接而成的一条路径。
	*/
	class DirLinkPath
	{
	public:
		/**
		 * @brief 一个Portion的核心是DirLink，也可以携带其它属性。
		*/
		struct Portion
		{
			DirLink dirLink;
			// 从Path起点到达该Portion的DirLink的末端点的累计长度，单位米。
			double accLength;

			/**
			 * @brief 反转当前portion
			*/
			virtual void reverse(double pathLength);

			/**
			 * @brief 统计该Portion的顶点数量。因为一个Portion可能是DirLink的其中一部分。
			 * @return 
			*/
			virtual std::size_t countVertexNum();

			virtual ~Portion() = default;
		};

		/**
		 * @brief 访问这条路径上每一部分的道路形状线的顶点
		*/
		class VertexAccessor
		{
		public:
			struct AccessInfo
			{
				// 访问的顶点在路径几何顶点中的索引。
				std::size_t index;
				// 路径中所有几何顶点数量
				std::size_t totalVertexNum;
				// 访问的顶点到路径起点的长度，单位米。
				double accLength;
			};
			/**
			 * @brief 该函数接收顶点并对其修改（可选）
			 * @param vertex 顶点坐标
			 * @param index 
			 * @param totalVertexNum  通过SingleDirRoad对象访问顶点则该值为所有道路中所有顶点中的数量。
			 * @param accLength 通过SingleDirRoad对象访问顶点则该值为道路累计长度。
			*/
			virtual void access(MapPoint3D64& vertex, const AccessInfo& accessInfo) = 0;

			virtual ~VertexAccessor() {};
		};

		/**
		 * @brief 负责访问Link对象Line图形顶点
		*/
		class DbLinkVertexAccessor
		{
			DbLink& m_link;

		public:

			DbLinkVertexAccessor(DbLink& link) : m_link(link) {}

			/**
			 * @brief 从indexFrom的顶点访问到indexTo的顶点（依次序的，ordered）。
			 * @param indexFrom
			 * @param indexTo
			 * @param vertexIndex indexFrom的顶点在DirLinkPath中的顶点。
			 * @param accLength indexFrom的顶点在DirLinkPath起点的距离。
			 * @param totalVertexNum DirLinkPath中所有的Link的顶点数
			 * @param accessor
			*/
			void accessVerticesOrdered(int indexFrom, int indexTo, std::size_t& vertexIndex, double& accLength, const std::size_t totalVertexNum, DirLinkPath::VertexAccessor& accessor);

			void accessAllVerticesForward(std::size_t& vertexIndex, double& accLength, const std::size_t totalVertexNum, DirLinkPath::VertexAccessor& accessor)
			{
				accessVerticesOrdered(0, int(m_link.geometry.vertexes.size()) - 1, vertexIndex, accLength, totalVertexNum, accessor);
			}

			void accessAllVerticesBackward(std::size_t& vertexIndex, double& accLength, const std::size_t totalVertexNum, DirLinkPath::VertexAccessor& accessor)
			{
				accessVerticesOrdered(int(m_link.geometry.vertexes.size()) - 1, 0, vertexIndex, accLength, totalVertexNum, accessor);
			}

			void accessVerticesToEndForward(int indexFrom, std::size_t& vertexIndex, double& accLength, const std::size_t totalVertexNum, DirLinkPath::VertexAccessor& accessor)
			{
				accessVerticesOrdered(indexFrom, int(m_link.geometry.vertexes.size()) - 1, vertexIndex, accLength, totalVertexNum, accessor);
			}

			void accessVerticesToEndBackward(int indexFrom, std::size_t& vertexIndex, double& accLength, const std::size_t totalVertexNum, DirLinkPath::VertexAccessor& accessor)
			{
				accessVerticesOrdered(indexFrom, 0, vertexIndex, accLength, totalVertexNum, accessor);
			}

			void accessVerticesForward(std::size_t& vertexIndex, double& accLength, const std::size_t totalVertexNum, DirLinkPath::VertexAccessor& accessor)
			{
				accessVerticesOrdered(0, int(m_link.geometry.vertexes.size()) - 1, vertexIndex, accLength, totalVertexNum, accessor);
			}

			void accessVerticesBackward(std::size_t& vertexIndex, double& accLength, const std::size_t totalVertexNum, DirLinkPath::VertexAccessor& accessor)
			{
				accessVerticesOrdered(int(m_link.geometry.vertexes.size()) - 1, 0, vertexIndex, accLength, totalVertexNum, accessor);
			}

			void accessVerticesToMidForward(int midIndex, std::size_t& vertexIndex, double& accLength, const std::size_t totalVertexNum, DirLinkPath::VertexAccessor& accessor)
			{
				accessVerticesOrdered(0, midIndex, vertexIndex, accLength, totalVertexNum, accessor);
			}

			void accessVerticesToMidBackward(int midIndex, std::size_t& vertexIndex, double& accLength, const std::size_t totalVertexNum, DirLinkPath::VertexAccessor& accessor)
			{
				accessVerticesOrdered(int(m_link.geometry.vertexes.size()) - 1, midIndex, vertexIndex, accLength, totalVertexNum, accessor);
			}
		};

		virtual ~DirLinkPath() = default;
		
		/**
		 * @brief Iterator
		 * @return 
		*/
		std::vector<Portion>::iterator begin() { return m_portions.begin(); }
		std::vector<Portion>::iterator end() { return m_portions.end(); }

		std::vector<Portion>& portions() { return m_portions; }

		/**
		 * @brief 判断当前路径是否有效。有效的对象才允许访问其它方法。
		 * @return 
		*/
		bool valid() { return !m_portions.empty(); }

		/**
		 * @brief 给该路径追加一部分
		 * @param portion 
		*/
		void addPortion(Portion portion) { m_portions.push_back(portion); }
		/**
		 * @brief 新增完全由dirLink构成的Portion
		 * @param dirLink 
		*/
		void addPortion(DirLink& dirLink);

		/**
		 * @brief 将当前有向路径反转。
		 *        Note: 该函数是否满足里氏替换原则需要检查
		*/
		void reverse();

		/**
		 * @brief 在最后追加path。调用者负责保证提供的path与当前path方向一致。
		 *        Note: 该函数是否满足里氏替换原则需要检查
		 * @param path 
		*/
		void appendPath(DirLinkPath& path);

		/**
		 * @brief 依次序从起点到终点访问该路径的所有顶点
		 * @param accessor 
		*/
		virtual void accessVertices(VertexAccessor& accessor);


	protected:
		/**
		 * @brief 统计所有Portion的link的顶点数量总和
		 * @return 
		*/
		std::size_t calTotalVertexNum();

	private:
		std::vector<Portion> m_portions;

	};


}
}
