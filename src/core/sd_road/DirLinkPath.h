#pragma once

#include "DirLink.h"

#include <vector>
#include <memory>


namespace OMDB
{
namespace sd
{

	/**
	 * @brief ��ʾ��һ����������(Portion)���������Ӷ��ɵ�һ��·����
	*/
	class DirLinkPath
	{
	public:
		/**
		 * @brief һ��Portion�ĺ�����DirLink��Ҳ����Я���������ԡ�
		*/
		struct Portion
		{
			DirLink dirLink;
			// ��Path��㵽���Portion��DirLink��ĩ�˵���ۼƳ��ȣ���λ�ס�
			double accLength;

			/**
			 * @brief ��ת��ǰportion
			*/
			virtual void reverse(double pathLength);

			/**
			 * @brief ͳ�Ƹ�Portion�Ķ�����������Ϊһ��Portion������DirLink������һ���֡�
			 * @return 
			*/
			virtual std::size_t countVertexNum();

			virtual ~Portion() = default;
		};

		/**
		 * @brief ��������·����ÿһ���ֵĵ�·��״�ߵĶ���
		*/
		class VertexAccessor
		{
		public:
			struct AccessInfo
			{
				// ���ʵĶ�����·�����ζ����е�������
				std::size_t index;
				// ·�������м��ζ�������
				std::size_t totalVertexNum;
				// ���ʵĶ��㵽·�����ĳ��ȣ���λ�ס�
				double accLength;
			};
			/**
			 * @brief �ú������ն��㲢�����޸ģ���ѡ��
			 * @param vertex ��������
			 * @param index 
			 * @param totalVertexNum  ͨ��SingleDirRoad������ʶ������ֵΪ���е�·�����ж����е�������
			 * @param accLength ͨ��SingleDirRoad������ʶ������ֵΪ��·�ۼƳ��ȡ�
			*/
			virtual void access(MapPoint3D64& vertex, const AccessInfo& accessInfo) = 0;

			virtual ~VertexAccessor() {};
		};

		/**
		 * @brief �������Link����Lineͼ�ζ���
		*/
		class DbLinkVertexAccessor
		{
			DbLink& m_link;

		public:

			DbLinkVertexAccessor(DbLink& link) : m_link(link) {}

			/**
			 * @brief ��indexFrom�Ķ�����ʵ�indexTo�Ķ��㣨������ģ�ordered����
			 * @param indexFrom
			 * @param indexTo
			 * @param vertexIndex indexFrom�Ķ�����DirLinkPath�еĶ��㡣
			 * @param accLength indexFrom�Ķ�����DirLinkPath���ľ��롣
			 * @param totalVertexNum DirLinkPath�����е�Link�Ķ�����
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
		 * @brief �жϵ�ǰ·���Ƿ���Ч����Ч�Ķ���������������������
		 * @return 
		*/
		bool valid() { return !m_portions.empty(); }

		/**
		 * @brief ����·��׷��һ����
		 * @param portion 
		*/
		void addPortion(Portion portion) { m_portions.push_back(portion); }
		/**
		 * @brief ������ȫ��dirLink���ɵ�Portion
		 * @param dirLink 
		*/
		void addPortion(DirLink& dirLink);

		/**
		 * @brief ����ǰ����·����ת��
		 *        Note: �ú����Ƿ����������滻ԭ����Ҫ���
		*/
		void reverse();

		/**
		 * @brief �����׷��path�������߸���֤�ṩ��path�뵱ǰpath����һ�¡�
		 *        Note: �ú����Ƿ����������滻ԭ����Ҫ���
		 * @param path 
		*/
		void appendPath(DirLinkPath& path);

		/**
		 * @brief ���������㵽�յ���ʸ�·�������ж���
		 * @param accessor 
		*/
		virtual void accessVertices(VertexAccessor& accessor);


	protected:
		/**
		 * @brief ͳ������Portion��link�Ķ��������ܺ�
		 * @return 
		*/
		std::size_t calTotalVertexNum();

	private:
		std::vector<Portion> m_portions;

	};


}
}
