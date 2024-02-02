#include "stdafx.h"
#include "navi_core_env.h"

NaviCoreEnv::NaviCoreEnv()
{
	cq_logSetLevel(MapbarLogLevel_fatal);
	Stdlib_init();
	App_init();
	NdsDb_globalInit();
}

NaviCoreEnv::~NaviCoreEnv()
{
	NdsDb_globalCleanup();
	App_cleanup();
	Stdlib_cleanup();
}
