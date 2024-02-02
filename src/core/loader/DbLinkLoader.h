#pragma once
#include "DataLoader.h"
#include "../record/DbRecord.h"
#include "DbRecordIterator.h"
namespace OMDB
{
    class DbLinkLoader : public DataLoader
    {
    protected:
        virtual void    load() override;
        void            loadLink();

        void            loadNodeForm();
        void            loadNodeMesh();
        void            loadNode();

        void            loadLinkLanePa();
        void            loadLgLink();
        void            loadLgAssociation();

        void            loadLinkPA();
        void            loadLinkPAValue();

		void            loadLinkDataLevel();
		void            loadLinkSeparation();
		void            loadLinkMedian();
		void            loadLinkOverheadObstruction();
		void            loadLinkTollArea();
		void            loadLinkForm();
		void            loadLinkName();
		void            loadRoadName();

        void            loadZLevel();
        void            loadLaneInfo();
        void            loadLaneInfoLink();

        void            loadRdLinkLanePa();
        void            loadRdLaneLinkCLM();
        void            loadRdLaneLinkCLMAcess();
        void            loadRdLaneLinkCLMCondition();
        void            loadRdLaneLinkCLMSpeedLimit();

        void            loadRdLaneLanePa();
        void            loadRdLaneTopoDetail();
        void            loadRdLaneTopoCond();
        void            loadRdLaneTopoVia();

        //
        void loadLinkSpeedLimit();
    private:
        DbRecordIterator recordIterator;
    };


}

