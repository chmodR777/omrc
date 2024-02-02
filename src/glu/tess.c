/*
** License Applicability. Except to the extent portions of this file are
** made subject to an alternative license as permitted in the SGI Free
** Software License B, Version 1.1 (the "License"), the contents of this
** file are subject only to the provisions of the License. You may not use
** this file except in compliance with the License. You may obtain a copy
** of the License at Silicon Graphics, Inc., attn: Legal Services, 1600
** Amphitheatre Parkway, Mountain View, CA 94043-1351, or at:
** 
** http://oss.sgi.com/projects/FreeB
** 
** Note that, as provided in the License, the Software is distributed on an
** "AS IS" basis, with ALL EXPRESS AND IMPLIED WARRANTIES AND CONDITIONS
** DISCLAIMED, INCLUDING, WITHOUT LIMITATION, ANY IMPLIED WARRANTIES AND
** CONDITIONS OF MERCHANTABILITY, SATISFACTORY QUALITY, FITNESS FOR A
** PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
** 
** Original Code. The Original Code is: OpenGL Sample Implementation,
** Version 1.2.1, released January 26, 2000, developed by Silicon Graphics,
** Inc. The Original Code is Copyright (c) 1991-2000 Silicon Graphics, Inc.
** Copyright in any portions created by third parties is as indicated
** elsewhere herein. All Rights Reserved.
** 
** Additional Notice Provisions: The application programming interfaces
** established by SGI in conjunction with the Original Code are The
** OpenGL(R) Graphics System: A Specification (Version 1.2.1), released
** April 1, 1999; The OpenGL(R) Graphics System Utility Library (Version
** 1.3), released November 4, 1998; and OpenGL(R) Graphics with the X
** Window System(R) (Version 1.3), released October 19, 1998. This software
** was created using the OpenGL(R) version 1.2.1 Sample Implementation
** published by SGI, but has not been independently verified as being
** compliant with the OpenGL(R) version 1.2.1 Specification.
**
*/
/*
** Author: Eric Veach, July 1994.
**
** $Date$ $Revision$
** $Header: //depot/main/gfx/lib/glu/libtess/tess.c#7 $
*/

#include "gluos.h"
#include <stddef.h>
#include <assert.h>
#include <setjmp.h>
#include "memalloc.h"
#include "tess.h"
#include "mesh.h"
#include "normal.h"
#include "sweep.h"
#include "tessmono.h"
#include "render.h"


#define CGLU_TESS_DEFAULT_TOLERANCE 0.0
#define CGLU_TESS_MESH		100112	/* void (*)(GLUmesh *mesh)	    */

/*ARGSUSED*/ static void GLAPIENTRY noBegin( GLenum type ) {UNUSED_VAR(type);}
/*ARGSUSED*/ static void GLAPIENTRY noEdgeFlag( GLboolean boundaryEdge ) {UNUSED_VAR(boundaryEdge);}
/*ARGSUSED*/ static void GLAPIENTRY noVertex( void *data ) {UNUSED_VAR(data);}
/*ARGSUSED*/ static void GLAPIENTRY noEnd( void ) {}
/*ARGSUSED*/ static void GLAPIENTRY noError( GLenum errnum ) {UNUSED_VAR(errnum);}
/*ARGSUSED*/ static void GLAPIENTRY noCombine( GLdouble coords[3], void *data[4],
                                    GLfloat weight[4], void **dataOut ) {UNUSED_VAR(coords);UNUSED_VAR(data);UNUSED_VAR(weight);UNUSED_VAR(dataOut);}
/*ARGSUSED*/ static void GLAPIENTRY noMesh( GLUmesh *mesh ) {UNUSED_VAR(mesh);}


/*ARGSUSED*/ void GLAPIENTRY __cgl_noBeginData( GLenum type,
					     void *polygonData ) {UNUSED_VAR(type);UNUSED_VAR(polygonData);}
/*ARGSUSED*/ void GLAPIENTRY __cgl_noEdgeFlagData( GLboolean boundaryEdge, 
				       void *polygonData ) {UNUSED_VAR(boundaryEdge);UNUSED_VAR(polygonData);}
/*ARGSUSED*/ void GLAPIENTRY __cgl_noVertexData( void *data,
					      void *polygonData ) {UNUSED_VAR(data);UNUSED_VAR(polygonData);}
/*ARGSUSED*/ void GLAPIENTRY __cgl_noEndData( void *polygonData ) {UNUSED_VAR(polygonData);}
/*ARGSUSED*/ void GLAPIENTRY __cgl_noErrorData( GLenum errnum,
					     void *polygonData ) {UNUSED_VAR(errnum);UNUSED_VAR(polygonData);}
/*ARGSUSED*/ void GLAPIENTRY __cgl_noCombineData( GLdouble coords[3],
					       void *data[4],
					       GLfloat weight[4],
					       void **outData,
					       void *polygonData ) {UNUSED_VAR(coords);UNUSED_VAR(data);UNUSED_VAR(weight);UNUSED_VAR(outData);UNUSED_VAR(polygonData);}

/* Half-edges are allocated in pairs (see mesh.c) */
typedef struct { CGLUhalfEdge e, eSym; } EdgePair;

#define MAX(a,b)	((a) > (b) ? (a) : (b))
#define MAX_FAST_ALLOC	(MAX(sizeof(EdgePair), \
			 MAX(sizeof(GLUvertex),sizeof(GLUface))))


CGLUtesselator * GLAPIENTRY
cgluNewTess( void )
{
  CGLUtesselator *tess;

  /* Only initialize fields which can be changed by the api.  Other fields
   * are initialized where they are used.
   */

  //if (memInit( MAX_FAST_ALLOC ) == 0) {
  //   return 0;			/* out of memory */
  //}
  tess = (CGLUtesselator *)memAlloc( sizeof( CGLUtesselator ));
  if (tess == NULL) {
     return 0;			/* out of memory */
  }

  tess->state = T_DORMANT;

  tess->normal[0] = 0;
  tess->normal[1] = 0;
  tess->normal[2] = 0;

  tess->relTolerance = CGLU_TESS_DEFAULT_TOLERANCE;
  tess->windingRule = CGLU_TESS_WINDING_ODD;
  tess->flagBoundary = FALSE;
  tess->boundaryOnly = FALSE;

  tess->callBegin = &noBegin;
  tess->callEdgeFlag = &noEdgeFlag;
  tess->callVertex = &noVertex;
  tess->callEnd = &noEnd;

  tess->callError = &noError;
  tess->callCombine = &noCombine;
  tess->callMesh = &noMesh;

  tess->callBeginData= &__cgl_noBeginData;
  tess->callEdgeFlagData= &__cgl_noEdgeFlagData;
  tess->callVertexData= &__cgl_noVertexData;
  tess->callEndData= &__cgl_noEndData;
  tess->callErrorData= &__cgl_noErrorData;
  tess->callCombineData= &__cgl_noCombineData;

  tess->polygonData= NULL;

  return tess;
}

static void MakeDormant( CGLUtesselator *tess )
{
  /* Return the tessellator to its original dormant state. */

  if( tess->mesh != NULL ) {
    __cgl_meshDeleteMesh( tess->mesh );
  }
  tess->state = T_DORMANT;
  tess->lastEdge = NULL;
  tess->mesh = NULL;
}

#define RequireState( tess, s )   if( tess->state != s ) GotoState(tess,s)

static void GotoState( CGLUtesselator *tess, enum CTessState newState )
{
  while( tess->state != newState ) {
    /* We change the current state one level at a time, to get to
     * the desired state.
     */
    if( tess->state < newState ) {
      switch( tess->state ) {
      case T_DORMANT:
	CALL_ERROR_OR_ERROR_DATA( CGLU_TESS_MISSING_BEGIN_POLYGON );
	cgluTessBeginPolygon( tess, NULL );
	break;
      case T_IN_POLYGON:
	CALL_ERROR_OR_ERROR_DATA( CGLU_TESS_MISSING_BEGIN_CONTOUR );
	cgluTessBeginContour( tess );
	break;
          default:
              break;
      }
    } else {
      switch( tess->state ) {
      case T_IN_CONTOUR:
	CALL_ERROR_OR_ERROR_DATA( CGLU_TESS_MISSING_END_CONTOUR );
	cgluTessEndContour( tess );
	break;
      case T_IN_POLYGON:
	CALL_ERROR_OR_ERROR_DATA( CGLU_TESS_MISSING_END_POLYGON );
	/* gluTessEndPolygon( tess ) is too much work! */
	MakeDormant( tess );
	break;
              default:
              break;
      }
    }
  }
}


void GLAPIENTRY
cgluDeleteTess( CGLUtesselator *tess )
{
  RequireState( tess, T_DORMANT );
  memFree( tess );
}


void GLAPIENTRY
cgluTessProperty( CGLUtesselator *tess, GLenum which, GLdouble value )
{
  GLenum windingRule;

  switch( which ) {
  case CGLU_TESS_TOLERANCE:
    if( value < 0.0 || value > 1.0 ) break;
    tess->relTolerance = value;
    return;

  case CGLU_TESS_WINDING_RULE:
    windingRule = (GLenum) value;
    if( windingRule != value ) break;	/* not an integer */

    switch( windingRule ) {
    case CGLU_TESS_WINDING_ODD:
    case CGLU_TESS_WINDING_NONZERO:
    case CGLU_TESS_WINDING_POSITIVE:
    case CGLU_TESS_WINDING_NEGATIVE:
    case CGLU_TESS_WINDING_ABS_GEQ_TWO:
      tess->windingRule = windingRule;
      return;
    default:
      break;
    }

  case CGLU_TESS_BOUNDARY_ONLY:
    tess->boundaryOnly = (value != 0);
    return;

  default:
    CALL_ERROR_OR_ERROR_DATA( CGLU_INVALID_ENUM );
    return;
  }
  CALL_ERROR_OR_ERROR_DATA( CGLU_INVALID_VALUE );
}

/* Returns tessellator property */
void GLAPIENTRY
cgluGetTessProperty( CGLUtesselator *tess, GLenum which, GLdouble *value )
{
   switch (which) {
   case CGLU_TESS_TOLERANCE:
      /* tolerance should be in range [0..1] */
      assert(0.0 <= tess->relTolerance && tess->relTolerance <= 1.0);
      *value= tess->relTolerance;
      break;    
   case CGLU_TESS_WINDING_RULE:
      assert(tess->windingRule == CGLU_TESS_WINDING_ODD ||
	     tess->windingRule == CGLU_TESS_WINDING_NONZERO ||
	     tess->windingRule == CGLU_TESS_WINDING_POSITIVE ||
	     tess->windingRule == CGLU_TESS_WINDING_NEGATIVE ||
	     tess->windingRule == CGLU_TESS_WINDING_ABS_GEQ_TWO);
      *value= tess->windingRule;
      break;
   case CGLU_TESS_BOUNDARY_ONLY:
      assert(tess->boundaryOnly || !tess->boundaryOnly);
      *value= tess->boundaryOnly;
      break;
   default:
      *value= 0.0;
      CALL_ERROR_OR_ERROR_DATA( CGLU_INVALID_ENUM );
      break;
   }
} /* gluGetTessProperty() */

void GLAPIENTRY
cgluTessNormal( CGLUtesselator *tess, GLdouble x, GLdouble y, GLdouble z )
{
  tess->normal[0] = x;
  tess->normal[1] = y;
  tess->normal[2] = z;
}

void GLAPIENTRY
cgluTessCallback( CGLUtesselator *tess, GLenum which, void (GLAPIENTRY *fn)())
{
  switch( which ) {
  case CGLU_TESS_BEGIN:
    tess->callBegin = (fn == NULL) ? &noBegin : (void (GLAPIENTRY *)(GLenum)) fn;
    return;
  case CGLU_TESS_BEGIN_DATA:
    tess->callBeginData = (fn == NULL) ?
	&__cgl_noBeginData : (void (GLAPIENTRY *)(GLenum, void *)) fn;
    return;
  case CGLU_TESS_EDGE_FLAG:
    tess->callEdgeFlag = (fn == NULL) ? &noEdgeFlag :
					(void (GLAPIENTRY *)(GLboolean)) fn;
    /* If the client wants boundary edges to be flagged,
     * we render everything as separate triangles (no strips or fans).
     */
    tess->flagBoundary = (fn != NULL);
    return;
  case CGLU_TESS_EDGE_FLAG_DATA:
    tess->callEdgeFlagData= (fn == NULL) ?
	&__cgl_noEdgeFlagData : (void (GLAPIENTRY *)(GLboolean, void *)) fn; 
    /* If the client wants boundary edges to be flagged,
     * we render everything as separate triangles (no strips or fans).
     */
    tess->flagBoundary = (fn != NULL);
    return;
  case CGLU_TESS_VERTEX:
    tess->callVertex = (fn == NULL) ? &noVertex :
				      (void (GLAPIENTRY *)(void *)) fn;
    return;
  case CGLU_TESS_VERTEX_DATA:
    tess->callVertexData = (fn == NULL) ?
	&__cgl_noVertexData : (void (GLAPIENTRY *)(void *, void *)) fn;
    return;
  case CGLU_TESS_END:
    tess->callEnd = (fn == NULL) ? &noEnd : (void (GLAPIENTRY *)(void)) fn;
    return;
  case CGLU_TESS_END_DATA:
    tess->callEndData = (fn == NULL) ? &__cgl_noEndData : 
                                       (void (GLAPIENTRY *)(void *)) fn;
    return;
  case CGLU_TESS_ERROR:
    tess->callError = (fn == NULL) ? &noError : (void (GLAPIENTRY *)(GLenum)) fn;
    return;
  case CGLU_TESS_ERROR_DATA:
    tess->callErrorData = (fn == NULL) ?
	&__cgl_noErrorData : (void (GLAPIENTRY *)(GLenum, void *)) fn;
    return;
  case CGLU_TESS_COMBINE:
    tess->callCombine = (fn == NULL) ? &noCombine :
	(void (GLAPIENTRY *)(GLdouble [3],void *[4], GLfloat [4], void ** )) fn;
    return;
  case CGLU_TESS_COMBINE_DATA:
    tess->callCombineData = (fn == NULL) ? &__cgl_noCombineData :
                                           (void (GLAPIENTRY *)(GLdouble [3],
						     void *[4], 
						     GLfloat [4], 
						     void **,
						     void *)) fn;
    return;
  case CGLU_TESS_MESH:
    tess->callMesh = (fn == NULL) ? &noMesh : (void (GLAPIENTRY *)(GLUmesh *)) fn;
    return;
  default:
    CALL_ERROR_OR_ERROR_DATA( CGLU_INVALID_ENUM );
    return;
  }
}

static int AddVertex( CGLUtesselator *tess, GLdouble coords[3], void *data )
{
  CGLUhalfEdge *e;

  e = tess->lastEdge;
  if( e == NULL ) {
    /* Make a self-loop (one vertex, one edge). */

    e = __cgl_meshMakeEdge( tess->mesh );
    if (e == NULL) return 0;
    if ( !__cgl_meshSplice( e, e->Sym ) ) return 0;
  } else {
    /* Create a new vertex and edge which immediately follow e
     * in the ordering around the left face.
     */
    if (__cgl_meshSplitEdge( e ) == NULL) return 0;
    e = e->Lnext;
  }

  /* The new vertex is now e->Org. */
  e->Org->data = data;
  e->Org->coords[0] = coords[0];
  e->Org->coords[1] = coords[1];
  e->Org->coords[2] = coords[2];
  
  /* The winding of an edge says how the winding number changes as we
   * cross from the edge''s right face to its left face.  We add the
   * vertices in such an order that a CCW contour will add +1 to
   * the winding number of the region inside the contour.
   */
  e->winding = 1;
  e->Sym->winding = -1;

  tess->lastEdge = e;

  return 1;
}


static void CacheVertex( CGLUtesselator *tess, GLdouble coords[3], void *data )
{
  CachedVertex *v = &tess->cache[tess->cacheCount];

  v->data = data;
  v->coords[0] = coords[0];
  v->coords[1] = coords[1];
  v->coords[2] = coords[2];
  ++tess->cacheCount;
}


static int EmptyCache( CGLUtesselator *tess )
{
  CachedVertex *v = tess->cache;
  CachedVertex *vLast;

  tess->mesh = __cgl_meshNewMesh();
  if (tess->mesh == NULL) return 0;

  for( vLast = v + tess->cacheCount; v < vLast; ++v ) {
    if ( !AddVertex( tess, v->coords, v->data ) ) return 0;
  }
  tess->cacheCount = 0;
  tess->emptyCache = FALSE;

  return 1;
}


void GLAPIENTRY
cgluTessVertex( CGLUtesselator *tess, GLdouble coords[3], void *data )
{
  int i, tooLarge = FALSE;
  GLdouble x, clamped[3];

  RequireState( tess, T_IN_CONTOUR );

  if( tess->emptyCache ) {
    if ( !EmptyCache( tess ) ) {
       CALL_ERROR_OR_ERROR_DATA( CGLU_OUT_OF_MEMORY );
       return;
    }
    tess->lastEdge = NULL;
  }
  for( i = 0; i < 3; ++i ) {
    x = coords[i];
    if( x < - CGLU_TESS_MAX_COORD ) {
      x = - CGLU_TESS_MAX_COORD;
      tooLarge = TRUE;
    }
    if( x > CGLU_TESS_MAX_COORD ) {
      x = CGLU_TESS_MAX_COORD;
      tooLarge = TRUE;
    }
    clamped[i] = x;
  }
  if( tooLarge ) {
    CALL_ERROR_OR_ERROR_DATA( CGLU_TESS_COORD_TOO_LARGE );
  }

  if( tess->mesh == NULL ) {
    if( tess->cacheCount < CTESS_MAX_CACHE ) {
      CacheVertex( tess, clamped, data );
      return;
    }
    if ( !EmptyCache( tess ) ) {
       CALL_ERROR_OR_ERROR_DATA( CGLU_OUT_OF_MEMORY );
       return;
    }
  }
  if ( !AddVertex( tess, clamped, data ) ) {
       CALL_ERROR_OR_ERROR_DATA( CGLU_OUT_OF_MEMORY );
  }
}


void GLAPIENTRY
cgluTessBeginPolygon( CGLUtesselator *tess, void *data )
{
  RequireState( tess, T_DORMANT );

  tess->state = T_IN_POLYGON;
  tess->cacheCount = 0;
  tess->emptyCache = FALSE;
  tess->mesh = NULL;

  tess->polygonData= data;
}


void GLAPIENTRY
cgluTessBeginContour( CGLUtesselator *tess )
{
  RequireState( tess, T_IN_POLYGON );

  tess->state = T_IN_CONTOUR;
  tess->lastEdge = NULL;
  if( tess->cacheCount > 0 ) {
    /* Just set a flag so we don't get confused by empty contours
     * -- these can be generated accidentally with the obsolete
     * NextContour() interface.
     */
    tess->emptyCache = TRUE;
  }
}


void GLAPIENTRY
cgluTessEndContour( CGLUtesselator *tess )
{
  RequireState( tess, T_IN_CONTOUR );
  tess->state = T_IN_POLYGON;
}

void GLAPIENTRY
cgluTessEndPolygon( CGLUtesselator *tess )
{
  GLUmesh *mesh;

  if (setjmp(tess->env) != 0) {	
     /* come back here if out of memory */
     CALL_ERROR_OR_ERROR_DATA( CGLU_OUT_OF_MEMORY );
     return;
  }

  RequireState( tess, T_IN_POLYGON );
  tess->state = T_DORMANT;
  
  if( tess->mesh == NULL ) {
    if( ! tess->flagBoundary && tess->callMesh == &noMesh ) {

      /* Try some special code to make the easy cases go quickly
       * (eg. convex polygons).  This code does NOT handle multiple contours,
       * intersections, edge flags, and of course it does not generate
       * an explicit mesh either.
       */
      if( __cgl_renderCache( tess )) {
	tess->polygonData= NULL; 
	return;
      }
    }
    if ( !EmptyCache( tess ) ) longjmp(tess->env,1); /* could've used a label*/
  }
      
  /* Determine the polygon normal and project vertices onto the plane
   * of the polygon.
   */
  __cgl_projectPolygon( tess );
  
  /* __cgl_computeInterior( tess ) computes the planar arrangement specified
   * by the given contours, and further subdivides this arrangement
   * into regions.  Each region is marked "inside" if it belongs
   * to the polygon, according to the rule given by tess->windingRule.
   * Each interior region is guaranteed be monotone.
   */
  if ( !__cgl_computeInterior( tess ) ) {
     longjmp(tess->env,1);	/* could've used a label */
  }
 
  mesh = tess->mesh;
  if( ! tess->fatalError ) {
    int rc = 1;

    /* If the user wants only the boundary contours, we throw away all edges
     * except those which separate the interior from the exterior.
     * Otherwise we tessellate all the regions marked "inside".
     */
    if( tess->boundaryOnly ) {
      rc = __cgl_meshSetWindingNumber( mesh, 1, TRUE );
    } else {
      rc = __cgl_meshTessellateInterior( mesh ); 
    }
    if (rc == 0) longjmp(tess->env,1);	/* could've used a label */

    __cgl_meshCheckMesh( mesh );

    if( tess->callBegin != &noBegin || tess->callEnd != &noEnd
       || tess->callVertex != &noVertex || tess->callEdgeFlag != &noEdgeFlag 
       || tess->callBeginData != &__cgl_noBeginData 
       || tess->callEndData != &__cgl_noEndData
       || tess->callVertexData != &__cgl_noVertexData
       || tess->callEdgeFlagData != &__cgl_noEdgeFlagData )
    {
      if( tess->boundaryOnly ) {
	__cgl_renderBoundary( tess, mesh );  /* output boundary contours */
      } else {
	__cgl_renderMesh( tess, mesh );	   /* output strips and fans */
      }
    }
    if( tess->callMesh != &noMesh ) {

      /* Throw away the exterior faces, so that all faces are interior.
       * This way the user doesn't have to check the "inside" flag,
       * and we don't need to even reveal its existence.  It also leaves
       * the freedom for an implementation to not generate the exterior
       * faces in the first place.
       */
      __cgl_meshDiscardExterior( mesh );
      (*tess->callMesh)( mesh );		/* user wants the mesh itself */
      tess->mesh = NULL;
      tess->polygonData= NULL;
      return;
    }
  }
  __cgl_meshDeleteMesh( mesh );
  tess->polygonData= NULL;
  tess->mesh = NULL;
}

