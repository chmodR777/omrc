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
	 * @brief 具有方向的Link，其中方向有：正向(forward)->与Link画线方向相同，反向(backward)->与Link画线方向相反。
	 *        与SD道路一致。
	*/
	class DirLink
	{
		int64 m_linkuuid;
		bool m_forward; // 非正向即反向
		DSegmentId m_dsegID; // 此处的DSegmentId应当只在当前类内使用!!!不要暴露，其与外部OMRC的DSegmentID定义不一样。
		DbLink* m_link; // 只是借用该对象，不具有

	public:
		/**
		 * @brief 构造一个空的DirLink对象，不要调用该对象的任何方法(除了valid)！！！
		*/
		DirLink();

		/**
		 * @brief
		 * @param link Not NULL!!!
		 * @param forward
		*/
		DirLink(DbLink* link, bool forward);

		/**
		 * @brief 创建一个反向的DirLink对象。
		 * @return 
		*/
		DirLink getReverseDirLink() { return DirLink{ m_link, !m_forward }; }

		DbLink* link() { return m_link; }

		bool forward() const { return m_forward; }

		/**
		 * @brief 检查当前DirLink对象是否无效。无效的对象不允许调用除了其它任何方法！！
		 * @return
		*/
		bool valid() const { return m_link != nullptr; }

		/**
		 * @brief 当前对象与other持有的无方向Link对象是否一样。
		 * @param other
		 * @return
		*/
		bool rawLinkEqual(const DirLink& other) const { return this->m_link == other.m_link; }

		/**
		 * @brief 返回可以唯一表示当前DirLink对象的ID。参考算路模块做法: 使用最低位记录方向：1 正向，0 反向。
		*/
		int64 linkUUIDWithDir() { return (m_link->uuid << 1) | static_cast<int64>(m_forward); }
		static int64 linkUUIDWithDirStatic(int64 uuid, bool forward) { return (uuid << 1) | static_cast<int64>(forward); }

		/**
		 * @brief 返回当前DirLink反向时的对象ID。
		*/
		int64 linkUUIDWithReversedDir() { return (m_link->uuid << 1) | (1 - static_cast<int64>(m_forward)); }

		////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/////////                                                                                    ///////////////
		/////////                           几何相关函数                                              ///////////////
		/////////                                                                                   ///////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////////////

		/**
		 * @brief DirLink几何第二个点到第一个点的向量在以第二个点为原点的平面直角坐标系下的与+X逆时针方向的夹角。夹角单位弧度。
		 * @return 弧度，范围[0, 2PI)
		*/
		double startHeadingAngle();
		/**
		 * @brief DirLink几何倒数第二个点到倒数第一个点的向量在以倒数第二个点为原点的平面直角坐标系下的与+X逆时针方向的夹角。夹角单位弧度。
		 * @return 弧度，范围[0, 2PI)
		*/
		double endHeadingAngle();

		/**
		 * @brief 获取DirLink 起点和末端点的高程。高程单位厘米。
		 * @return
		*/
		double getStartHeight();
		double getEndHeight();

		/**
		 * @brief 获取DirLink第一个点的引用。注意外部可能会修改这个点。
		*/
		MapPoint3D64& getStartPointRef() { return m_forward ? m_link->geometry.vertexes.front() : m_link->geometry.vertexes.back(); }

		/**
		 * @brief 获取DirLink最后一个点的引用。注意外部可能会修改这个点。
		*/
		MapPoint3D64& getEndPointRef() { return m_forward ? m_link->geometry.vertexes.back() : m_link->geometry.vertexes.front(); }

		/**
		 * @brief 获取DireLink顺方向的指定点索引的点引用。注意外部可能会修改这个点。
		 * @param index 
		 * @return 
		*/
		MapPoint3D64& getPointRefInDir(int index) { return m_forward ? m_link->geometry.vertexes[index] : m_link->geometry.vertexes[int(m_link->geometry.vertexes.size()) - 1 - index]; };

		/**
		 * @brief 计算Link长度，单位米。
		 * @return
		*/
		double calLength() const;

		/**
		* @brief 计算两点(indexFrom & indexTo)的累计长度，单位米。其中indexFrom & indexTo大小关系无所谓。
		*/
		double calPortionLength(int indexFrom, int indexTo) const;

		/**
		 * @brief 计算DirLink从起点到指定点的长度。单位米。
		 *        Note: 该计算与当前DirLink对象方向相关。
		*/
		double calStartToMidLength(int index) const { return m_forward ? calToMidForwardLength(index) : calToMidBackwardLength(index); }

		/**
		 * @brief 计算DirLink从指定点到末端点的长度。单位米。
		 *        Note: 该计算与当前DirLink对象方向相关。
		*/
		double calMidToEndLength(int index) const { return m_forward ? calToEndForwardLength(index) : calToEndBackwardLength(index); }

		/**
		 * @brief 计算Link从起点开始到指定点(位于中间的点)长度。单位米。
		 *        Note: 该计算无关当前DirLink对象方向。
		*/
		double calToMidForwardLength(int index) const { return calPortionLength(0, index); };

		/**
		 * @brief 计算Link从末端点起到指定点的长度。单位米。
		 *        Note: 该计算无关当前DirLink对象方向。
		*/
		double calToMidBackwardLength(int index) const { return calPortionLength(int(m_link->geometry.vertexes.size()) - 1, index); };

		/**
		 * @brief 计算Link从指定点开始到最后一个点的累计长度。单位米。
		 *        Note: 该计算无关当前DirLink对象方向。
		*/
		double calToEndForwardLength(int index) const { return calPortionLength(index, int(m_link->geometry.vertexes.size()) - 1); }

		/**
		 * @brief 计算Link从指定点到起始点的累计长度。单位米。
		 *        Note: 该计算无关当前DirLink对象方向。
		*/
		double calToEndBackwardLength(int index) const { return calPortionLength(index, 0); }


		////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/////////                                                                                    ///////////////
		/////////                           路网拓扑相关函数                                           ///////////////
		/////////                                                                                   ///////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////////////

		/**
		 * @brief 获取与该有向Link末端节点直接相连有向Link。包含自身反方向的Link对象。
		 * @return
		*/
		std::vector<DirLink> getEndDirectOutlinks();
		std::vector<int64> getEndDirectOutlinksUUID();
		/**
		 * @brief 获取与该有向Link末端节点直接相连有向Link。 不 包含自身反方向的Link对象。
		 * @return
		*/
		std::vector<DirLink> getEndDirectOutlinksExceptSelf();
		std::vector<int64> getEndDirectOutlinksExceptSelfUUID();
	};

}
}
