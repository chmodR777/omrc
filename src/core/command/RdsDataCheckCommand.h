#pragma once
#include <vector>
#include <stdafx.h>
#include "../CompileSetting.h"
#include "navi_rds/rds.h"
#include "geometry/map_point3d64.h"
#include "tool_kit/boost_geometry_utils.h"

namespace OMDB
{
    namespace RdsCheck
    {
        template <typename Container>
        class VectorIndexable {
            using size_type = typename Container::size_type;
            using cref = const typename Container::value_type&;
            Container const& container;

        public:
            using result_type = cref;

            explicit VectorIndexable(Container const& c)
                : container(c)
            {
            }

            result_type operator()(size_type i) const
            {
                return container[i];
            }
        };

        struct Box2DRTree {
            using guardParam = bgi::quadratic<16>;
            using indexGetterBox = RdsCheck::VectorIndexable<std::vector<box_2t>>;
            using RTree = bgi::rtree<std::vector<box_2t>::size_type, guardParam, indexGetterBox>;
            using Ptr = std::shared_ptr<RTree>;
        };

        struct RoadGeometries {
            std::vector<int64> ids;

            std::vector<ring_t> rings;
            std::vector<ring_2t> ring2ds;
            std::vector<box_2t> ringBox2ds;

            std::vector<segment_t> stopLines;
            std::vector<segment_2t> stopLine2ds;
            std::vector<box_2t> stopLineBox2ds;
        };

        inline RdsCheck::Box2DRTree::Ptr makeRTree2ForRings(const RoadGeometries& roads)
        {
            RdsCheck::Box2DRTree::Ptr Box2dRtree;
            RdsCheck::Box2DRTree::guardParam param;
            RdsCheck::Box2DRTree::indexGetterBox originIndBox(roads.ringBox2ds);
            Box2dRtree = std::make_shared<RdsCheck::Box2DRTree::RTree>(boost::irange<std::size_t>(0lu, roads.ringBox2ds.size()), param, originIndBox);
            return Box2dRtree;
        }

        inline RdsCheck::Box2DRTree::Ptr makeRTree2ForSegments(const RoadGeometries& roads)
        {
            RdsCheck::Box2DRTree::Ptr Box2dRtree;
            RdsCheck::Box2DRTree::guardParam param;
            RdsCheck::Box2DRTree::indexGetterBox originIndBox(roads.stopLineBox2ds);
            Box2dRtree = std::make_shared<RdsCheck::Box2DRTree::RTree>(boost::irange<std::size_t>(0lu, roads.stopLineBox2ds.size()), param, originIndBox);
            return Box2dRtree;
        }
    }

    class RdsDataCheckCommand : public CliCommand
    {
    private:
        RdsDataCheckCommand();
        RdsDataCheckCommand(const RdsDataCheckCommand& command);
        ~RdsDataCheckCommand();
        virtual const cqWCHAR* name() override;
		virtual const cqWCHAR* shortDescription() override;
		virtual void printHelp() override;
        virtual bool parseArgs(ArgParser* parser) override;

    public:
        void checkRdsTile(RDS::RdsTile* pTile);
        void checkGuardrail(RDS::RdsTile* pTile);
        void checkRoadCavity(RDS::RdsTile* pTile);

    public:
        static RdsDataCheckCommand* instance();
    public:
		virtual int exec() override;

    private:
        CompilerOptions m_options;
		static RdsDataCheckCommand g_sCommand;

    };
}

