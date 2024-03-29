/*
** gluos.h - operating system dependencies for GLU
**
** $Header: //depot/main/gfx/lib/glu/include/gluos.h#4 $
*/

#include "render-system/cq_opengl.h"

#ifdef _WIN32

#define WIN32_LEAN_AND_MEAN
#define NOGDI
#define NOIME
#include <windows.h>

#ifndef GLAPIENTRY

#define GLAPIENTRY APIENTRY

#endif

#ifndef WINGDIAPI
	#define WINGDIAPI
#endif



/* Disable warnings */
#pragma warning(disable : 4101)
#pragma warning(disable : 4244)
#pragma warning(disable : 4761)

#else

/* Disable Microsoft-specific keywords */

#define GLAPIENTRY
#define WINGDIAPI
#ifndef APIENTRY
#	define APIENTRY
#endif
#define CALLBACK

#endif

