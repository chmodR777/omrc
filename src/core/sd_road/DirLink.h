#pragma once

#include <cstdint>
#include <vector>
#include <cmath>

#include "core/record/DbRecord.h"
#include "core/Segment.h"

namespace OMDB
{
namespace sd
{

	/**
	 * @brief ���з����Link�����з����У�����(forward)->��Link���߷�����ͬ������(backward)->��Link���߷����෴��
	 *        ��SD��·һ�¡�
	*/
	class DirLink
	{
		int64 m_linkuuid;
		bool m_forward; // �����򼴷���
		DSegmentId m_dsegID; // �˴���DSegmentIdӦ��ֻ�ڵ�ǰ����ʹ��!!!��Ҫ��¶�������ⲿOMRC��DSegmentID���岻һ����
		DbLink* m_link; // ֻ�ǽ��øö��󣬲�����

	public:
		/**
		 * @brief ����һ���յ�DirLink���󣬲�Ҫ���øö�����κη���(����valid)������
		*/
		DirLink();

		/**
		 * @brief
		 * @param link Not NULL!!!
		 * @param forward
		*/
		DirLink(DbLink* link, bool forward);

		/**
		 * @brief ����һ�������DirLink����
		 * @return 
		*/
		DirLink getReverseDirLink() { return DirLink{ m_link, !m_forward }; }

		DbLink* link() { return m_link; }

		bool forward() const { return m_forward; }

		/**
		 * @brief ��鵱ǰDirLink�����Ƿ���Ч����Ч�Ķ���������ó��������κη�������
		 * @return
		*/
		bool valid() const { return m_link != nullptr; }

		/**
		 * @brief ��ǰ������other���е��޷���Link�����Ƿ�һ����
		 * @param other
		 * @return
		*/
		bool rawLinkEqual(const DirLink& other) const { return this->m_link == other.m_link; }

		/**
		 * @brief ���ؿ���Ψһ��ʾ��ǰDirLink�����ID���ο���·ģ������: ʹ�����λ��¼����1 ����0 ����
		*/
		int64 linkUUIDWithDir() { return (m_link->uuid << 1) | static_cast<int64>(m_forward); }
		static int64 linkUUIDWithDirStatic(int64 uuid, bool forward) { return (uuid << 1) | static_cast<int64>(forward); }

		/**
		 * @brief ���ص�ǰDirLink����ʱ�Ķ���ID��
		*/
		int64 linkUUIDWithReversedDir() { return (m_link->uuid << 1) | (1 - static_cast<int64>(m_forward)); }

		////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/////////                                                                                    ///////////////
		/////////                           ������غ���                                              ///////////////
		/////////                                                                                   ///////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////////////

		/**
		 * @brief DirLink���εڶ����㵽��һ������������Եڶ�����Ϊԭ���ƽ��ֱ������ϵ�µ���+X��ʱ�뷽��ļнǡ��нǵ�λ���ȡ�
		 * @return ���ȣ���Χ[0, 2PI)
		*/
		double startHeadingAngle();
		/**
		 * @brief DirLink���ε����ڶ����㵽������һ������������Ե����ڶ�����Ϊԭ���ƽ��ֱ������ϵ�µ���+X��ʱ�뷽��ļнǡ��нǵ�λ���ȡ�
		 * @return ���ȣ���Χ[0, 2PI)
		*/
		double endHeadingAngle();

		/**
		 * @brief ��ȡDirLink ����ĩ�˵�ĸ̡߳��̵߳�λ���ס�
		 * @return
		*/
		double getStartHeight();
		double getEndHeight();

		/**
		 * @brief ��ȡDirLink��һ��������á�ע���ⲿ���ܻ��޸�����㡣
		*/
		MapPoint3D64& getStartPointRef() { return m_forward ? m_link->geometry.vertexes.front() : m_link->geometry.vertexes.back(); }

		/**
		 * @brief ��ȡDirLink���һ��������á�ע���ⲿ���ܻ��޸�����㡣
		*/
		MapPoint3D64& getEndPointRef() { return m_forward ? m_link->geometry.vertexes.back() : m_link->geometry.vertexes.front(); }

		/**
		 * @brief ��ȡDireLink˳�����ָ���������ĵ����á�ע���ⲿ���ܻ��޸�����㡣
		 * @param index 
		 * @return 
		*/
		MapPoint3D64& getPointRefInDir(int index) { return m_forward ? m_link->geometry.vertexes[index] : m_link->geometry.vertexes[int(m_link->geometry.vertexes.size()) - 1 - index]; };

		/**
		 * @brief ����Link���ȣ���λ�ס�
		 * @return
		*/
		double calLength() const;

		/**
		* @brief ��������(indexFrom & indexTo)���ۼƳ��ȣ���λ�ס�����indexFrom & indexTo��С��ϵ����ν��
		*/
		double calPortionLength(int indexFrom, int indexTo) const;

		/**
		 * @brief ����DirLink����㵽ָ����ĳ��ȡ���λ�ס�
		 *        Note: �ü����뵱ǰDirLink��������ء�
		*/
		double calStartToMidLength(int index) const { return m_forward ? calToMidForwardLength(index) : calToMidBackwardLength(index); }

		/**
		 * @brief ����DirLink��ָ���㵽ĩ�˵�ĳ��ȡ���λ�ס�
		 *        Note: �ü����뵱ǰDirLink��������ء�
		*/
		double calMidToEndLength(int index) const { return m_forward ? calToEndForwardLength(index) : calToEndBackwardLength(index); }

		/**
		 * @brief ����Link����㿪ʼ��ָ����(λ���м�ĵ�)���ȡ���λ�ס�
		 *        Note: �ü����޹ص�ǰDirLink������
		*/
		double calToMidForwardLength(int index) const { return calPortionLength(0, index); };

		/**
		 * @brief ����Link��ĩ�˵���ָ����ĳ��ȡ���λ�ס�
		 *        Note: �ü����޹ص�ǰDirLink������
		*/
		double calToMidBackwardLength(int index) const { return calPortionLength(int(m_link->geometry.vertexes.size()) - 1, index); };

		/**
		 * @brief ����Link��ָ���㿪ʼ�����һ������ۼƳ��ȡ���λ�ס�
		 *        Note: �ü����޹ص�ǰDirLink������
		*/
		double calToEndForwardLength(int index) const { return calPortionLength(index, int(m_link->geometry.vertexes.size()) - 1); }

		/**
		 * @brief ����Link��ָ���㵽��ʼ����ۼƳ��ȡ���λ�ס�
		 *        Note: �ü����޹ص�ǰDirLink������
		*/
		double calToEndBackwardLength(int index) const { return calPortionLength(index, 0); }


		////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/////////                                                                                    ///////////////
		/////////                           ·��������غ���                                           ///////////////
		/////////                                                                                   ///////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////////////

		/**
		 * @brief ��ȡ�������Linkĩ�˽ڵ�ֱ����������Link���������������Link����
		 * @return
		*/
		std::vector<DirLink> getEndDirectOutlinks();
		std::vector<int64> getEndDirectOutlinksUUID();
		/**
		 * @brief ��ȡ�������Linkĩ�˽ڵ�ֱ����������Link�� �� �������������Link����
		 * @return
		*/
		std::vector<DirLink> getEndDirectOutlinksExceptSelf();
		std::vector<int64> getEndDirectOutlinksExceptSelfUUID();
	};

}
}
