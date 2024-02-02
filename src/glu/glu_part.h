/* 
** Modification we applied:
**		1) Delete all things except TESS related and mip-map related contents;
**      2) Define GLdouble as GLfloat.
**      3) Add CGLU_INVALID_OPERATION.
** Mapbar Inc.
*/

/*
** Copyright 1991-1993, Silicon Graphics, Inc.
** All Rights Reserved.
** 
** This is UNPUBLISHED PROPRIETARY SOURCE CODE of Silicon Graphics, Inc.;
** the contents of this file may not be disclosed to third parties, copied or
** duplicated in any form, in whole or in part, without the prior written
** permission of Silicon Graphics, Inc.
** 
** RESTRICTED RIGHTS LEGEND:
** Use, duplication or disclosure by the Government is subject to restrictions
** as set forth in subdivision (c)(1)(ii) of the Rights in Technical Data
** and Computer Software clause at DFARS 252.227-7013, and/or in similar or
** successor clauses in the FAR, DOD or NASA FAR Supplement. Unpublished -
** rights reserved under the Copyright Laws of the United States.
*/

/*
** Return the error string associated with a particular error code.
** This will return 0 for an invalid error code.
**
** The generic function prototype that can be compiled for ANSI or Unicode
** is defined as follows:
**
** LPCTSTR APIENTRY gluErrorStringWIN (GLenum errCode);
*/

#ifndef _CGLU_PART_H
#define _CGLU_PART_H

#include "gluos.h"

#ifdef MAPBAR_USE_OPENGL
#	include "render-system/cq_opengl.h"
#else
#	include "cq_gl.h"  /* For the glu-TESS functions. */
#endif

#ifdef __cplusplus
extern "C" {
#endif

//GLdouble should be defined as a macro after gl.h, since in which there is such a statement:
//  typedef double GLdouble;
#define GLdouble GLfloat


//-- #ifdef UNICODE
//-- #define gluErrorStringWIN(errCode) ((LPCSTR)  gluErrorUnicodeStringEXT(errCode))
//-- #else
//-- #define gluErrorStringWIN(errCode) ((LPCWSTR) gluErrorString(errCode))
//-- #endif
//-- 
//-- const GLubyte* APIENTRY gluErrorString (
//--     GLenum   errCode);
//-- 
//-- const wchar_t* APIENTRY gluErrorUnicodeStringEXT (
//--     GLenum   errCode);
//-- 
//-- const GLubyte* APIENTRY gluGetString (
//--     GLenum   name);
//-- 
//-- void APIENTRY gluOrtho2D (
//--     GLdouble left, 
//--     GLdouble right, 
//--     GLdouble bottom, 
//--     GLdouble top);
//-- 
//-- void APIENTRY gluPerspective (
//--     GLdouble fovy, 
//--     GLdouble aspect, 
//--     GLdouble zNear, 
//--     GLdouble zFar);
//-- 
//-- void APIENTRY gluPickMatrix (
//--     GLdouble x, 
//--     GLdouble y, 
//--     GLdouble width, 
//--     GLdouble height, 
//--     GLint    viewport[4]);
//-- 
//-- void APIENTRY gluLookAt (
//--     GLdouble eyex, 
//--     GLdouble eyey, 
//--     GLdouble eyez, 
//--     GLdouble centerx, 
//--     GLdouble centery, 
//--     GLdouble centerz, 
//--     GLdouble upx, 
//--     GLdouble upy, 
//--     GLdouble upz);
//-- 
//-- int APIENTRY gluProject (
//--     GLdouble        objx, 
//--     GLdouble        objy, 
//--     GLdouble        objz,  
//--     const GLdouble  modelMatrix[16], 
//--     const GLdouble  projMatrix[16], 
//--     const GLint     viewport[4], 
//--     GLdouble        *winx, 
//--     GLdouble        *winy, 
//--     GLdouble        *winz);
//-- 
//-- int APIENTRY gluUnProject (
//--     GLdouble       winx, 
//--     GLdouble       winy, 
//--     GLdouble       winz, 
//--     const GLdouble modelMatrix[16], 
//--     const GLdouble projMatrix[16], 
//--     const GLint    viewport[4], 
//--     GLdouble       *objx, 
//--     GLdouble       *objy, 
//--     GLdouble       *objz);
//-- 
//-- 
//-- int APIENTRY gluScaleImage (
//--     GLenum      format, 
//--     GLint       widthin, 
//--     GLint       heightin, 
//--     GLenum      typein, 
//--     const void  *datain, 
//--     GLint       widthout, 
//--     GLint       heightout, 
//--     GLenum      typeout, 
//--     void        *dataout);
//-- 
//-- 
//-- int APIENTRY gluBuild1DMipmaps (
//--     GLenum      target, 
//--     GLint       components, 
//--     GLint       width, 
//--     GLenum      format, 
//--     GLenum      type, 
//--     const void  *data);

#if !defined(MAPBAR_ANDROID) && !defined(MAPBAR_IPHONE)
int APIENTRY cgluBuild2DMipmaps (
    GLenum      target, 
    GLint       components, 
    GLint       width, 
    GLint       height, 
    GLenum      format, 
    GLenum      type, 
    const void  *data);
#endif

#ifdef __cplusplus

//-- class GLUnurbs;
//-- class GLUquadric;
class CGLUtesselator;

//-- /* backwards compatibility: */
//-- typedef class GLUnurbs GLUnurbsObj;
//-- typedef class GLUquadric GLUquadricObj;
//-- typedef class CGLUtesselator CGLUtesselatorObj;
//-- typedef class CGLUtesselator GLUtriangulatorObj;

#else

//-- typedef struct GLUnurbs GLUnurbs;
//-- typedef struct GLUquadric GLUquadric;
typedef struct CGLUtesselator CGLUtesselator;

//-- /* backwards compatibility: */
//-- typedef struct GLUnurbs GLUnurbsObj;
//-- typedef struct GLUquadric GLUquadricObj;
//-- typedef struct CGLUtesselator CGLUtesselatorObj;
//-- typedef struct CGLUtesselator GLUtriangulatorObj;

#endif


//-- GLUquadric* APIENTRY gluNewQuadric (void);
//-- void APIENTRY gluDeleteQuadric (
//--     GLUquadric          *state);
//-- 
//-- void APIENTRY gluQuadricNormals (
//--     GLUquadric          *quadObject, 
//--     GLenum              normals);
//-- 
//-- void APIENTRY gluQuadricTexture (
//--     GLUquadric          *quadObject, 
//--     GLboolean           textureCoords);
//-- 
//-- void APIENTRY gluQuadricOrientation (
//--     GLUquadric          *quadObject, 
//--     GLenum              orientation);
//-- 
//-- void APIENTRY gluQuadricDrawStyle (
//--     GLUquadric          *quadObject, 
//--     GLenum              drawStyle);
//-- 
//-- void APIENTRY gluCylinder (
//--     GLUquadric          *qobj, 
//--     GLdouble            baseRadius, 
//--     GLdouble            topRadius, 
//--     GLdouble            height, 
//--     GLint               slices, 
//--     GLint               stacks);
//-- 
//-- void APIENTRY gluDisk (
//--     GLUquadric          *qobj, 
//--     GLdouble            innerRadius, 
//--     GLdouble            outerRadius, 
//--     GLint               slices, 
//--     GLint               loops);
//-- 
//-- void APIENTRY gluPartialDisk (
//--     GLUquadric          *qobj, 
//--     GLdouble            innerRadius, 
//--     GLdouble            outerRadius, 
//--     GLint               slices, 
//--     GLint               loops, 
//--     GLdouble            startAngle, 
//--     GLdouble            sweepAngle);
//-- 
//-- void APIENTRY gluSphere (
//--     GLUquadric          *qobj, 
//--     GLdouble            radius, 
//--     GLint               slices, 
//--     GLint               stacks);
//-- 
//-- void APIENTRY gluQuadricCallback (
//--     GLUquadric          *qobj, 
//--     GLenum              which, 
//--     void                (CALLBACK* fn)());

CGLUtesselator* APIENTRY  cgluNewTess(          
    void );

void APIENTRY  cgluDeleteTess(       
    CGLUtesselator       *tess );

void APIENTRY  cgluTessBeginPolygon( 
    CGLUtesselator       *tess,
    void                *polygon_data );

void APIENTRY  cgluTessBeginContour( 
    CGLUtesselator       *tess );

void APIENTRY  cgluTessVertex(       
    CGLUtesselator       *tess,
    GLdouble            coords[3], 
    void                *data );

void APIENTRY  cgluTessEndContour(   
    CGLUtesselator       *tess );

void APIENTRY  cgluTessEndPolygon(   
    CGLUtesselator       *tess );

void APIENTRY  cgluTessProperty(     
    CGLUtesselator       *tess,
    GLenum              which, 
    GLdouble            value );
 
void APIENTRY  cgluTessNormal(       
    CGLUtesselator       *tess, 
    GLdouble            x,
    GLdouble            y, 
    GLdouble            z );

void APIENTRY  cgluTessCallback(     
    CGLUtesselator       *tess,
    GLenum              which, 
    void                (CALLBACK *fn)());

void APIENTRY  cgluGetTessProperty(  
    CGLUtesselator       *tess,
    GLenum              which, 
    GLdouble            *value );
 
//-- GLUnurbs* APIENTRY gluNewNurbsRenderer (void);
//-- 
//-- void APIENTRY gluDeleteNurbsRenderer (
//--     GLUnurbs            *nobj);
//-- 
//-- void APIENTRY gluBeginSurface (
//--     GLUnurbs            *nobj);
//-- 
//-- void APIENTRY gluBeginCurve (
//--     GLUnurbs            *nobj);
//-- 
//-- void APIENTRY gluEndCurve (
//--     GLUnurbs            *nobj);
//-- 
//-- void APIENTRY gluEndSurface (
//--     GLUnurbs            *nobj);
//-- 
//-- void APIENTRY gluBeginTrim (
//--     GLUnurbs            *nobj);
//-- 
//-- void APIENTRY gluEndTrim (
//--     GLUnurbs            *nobj);
//-- 
//-- void APIENTRY gluPwlCurve (
//--     GLUnurbs            *nobj, 
//--     GLint               count, 
//--     GLfloat             *array, 
//--     GLint               stride, 
//--     GLenum              type);
//-- 
//-- void APIENTRY gluNurbsCurve (
//--     GLUnurbs            *nobj, 
//--     GLint               nknots, 
//--     GLfloat             *knot, 
//--     GLint               stride, 
//--     GLfloat             *ctlarray, 
//--     GLint               order, 
//--     GLenum              type);
//-- 
//-- void APIENTRY 
//-- gluNurbsSurface(     
//--     GLUnurbs            *nobj, 
//--     GLint               sknot_count, 
//--     float               *sknot, 
//--     GLint               tknot_count, 
//--     GLfloat             *tknot, 
//--     GLint               s_stride, 
//--     GLint               t_stride, 
//--     GLfloat             *ctlarray, 
//--     GLint               sorder, 
//--     GLint               torder, 
//--     GLenum              type);
//-- 
//-- void APIENTRY 
//-- gluLoadSamplingMatrices (
//--     GLUnurbs            *nobj, 
//--     const GLfloat       modelMatrix[16], 
//--     const GLfloat       projMatrix[16], 
//--     const GLint         viewport[4] );
//-- 
//-- void APIENTRY 
//-- gluNurbsProperty (
//--     GLUnurbs            *nobj, 
//--     GLenum              property, 
//--     GLfloat             value );
//-- 
//-- void APIENTRY 
//-- gluGetNurbsProperty (
//--     GLUnurbs            *nobj, 
//--     GLenum              property, 
//--     GLfloat             *value );
//-- 
//-- void APIENTRY 
//-- gluNurbsCallback (
//--     GLUnurbs            *nobj, 
//--     GLenum              which, 
//--     void                (CALLBACK* fn)() );


/****           Callback function prototypes    ****/

//-- /* gluQuadricCallback */
//-- typedef void (CALLBACK* GLUquadricErrorProc) (GLenum);

/* gluTessCallback */
typedef void (CALLBACK* CGLUtessBeginProc)        (GLenum);
typedef void (CALLBACK* CGLUtessEdgeFlagProc)     (GLboolean);
typedef void (CALLBACK* CGLUtessVertexProc)       (void *);
typedef void (CALLBACK* CGLUtessEndProc)          (void);
typedef void (CALLBACK* CGLUtessErrorProc)        (GLenum);
typedef void (CALLBACK* CGLUtessCombineProc)      (GLdouble[3],
                                                  void*[4], 
                                                  GLfloat[4],
                                                  void** );
typedef void (CALLBACK* CGLUtessBeginDataProc)    (GLenum, void *);
typedef void (CALLBACK* CGLUtessEdgeFlagDataProc) (GLboolean, void *);
typedef void (CALLBACK* CGLUtessVertexDataProc)   (void *, void *);
typedef void (CALLBACK* CGLUtessEndDataProc)      (void *);
typedef void (CALLBACK* CGLUtessErrorDataProc)    (GLenum, void *);
typedef void (CALLBACK* CGLUtessCombineDataProc)  (GLdouble[3],
                                                  void*[4], 
                                                  GLfloat[4],
                                                  void**,
                                                  void* );

//-- /* gluNurbsCallback */
//-- typedef void (CALLBACK* GLUnurbsErrorProc)   (GLenum);
//-- 

/****           Generic constants               ****/

/* Version */
#define CGLU_VERSION_1_1                 1
#define CGLU_VERSION_1_2                 1

/* Errors: (return value 0 = no error) */
#define CGLU_INVALID_ENUM        100900
#define CGLU_INVALID_VALUE       100901
#define CGLU_OUT_OF_MEMORY       100902
#define CGLU_INCOMPATIBLE_GL_VERSION     100903
#define CGLU_INVALID_OPERATION   100904

/* StringName */
#define CGLU_VERSION             100800
#define CGLU_EXTENSIONS          100801

/* Boolean */
#define CGLU_TRUE                GL_TRUE
#define CGLU_FALSE               GL_FALSE


/****           Quadric constants               ****/

//-- /* QuadricNormal */
//-- #define CGLU_SMOOTH              100000
//-- #define CGLU_FLAT                100001
//-- #define CGLU_NONE                100002
//-- 
//-- /* QuadricDrawStyle */
//-- #define CGLU_POINT               100010
//-- #define CGLU_LINE                100011
//-- #define CGLU_FILL                100012
//-- #define CGLU_SILHOUETTE          100013

/* QuadricOrientation */
#define CGLU_OUTSIDE             100020
#define CGLU_INSIDE              100021

/* Callback types: */
/*      CGLU_ERROR               100103 */


/****           Tesselation constants           ****/

#define CGLU_TESS_MAX_COORD              1.0e37f  /* 1.0e150 */

/* TessProperty */
#define CGLU_TESS_WINDING_RULE           100140
#define CGLU_TESS_BOUNDARY_ONLY          100141
#define CGLU_TESS_TOLERANCE              100142

/* TessWinding */
#define CGLU_TESS_WINDING_ODD            100130
#define CGLU_TESS_WINDING_NONZERO        100131
#define CGLU_TESS_WINDING_POSITIVE       100132
#define CGLU_TESS_WINDING_NEGATIVE       100133
#define CGLU_TESS_WINDING_ABS_GEQ_TWO    100134

/* TessCallback */
#define CGLU_TESS_BEGIN          100100  /* void (CALLBACK*)(GLenum    type)  */
#define CGLU_TESS_VERTEX         100101  /* void (CALLBACK*)(void      *data) */
#define CGLU_TESS_END            100102  /* void (CALLBACK*)(void)            */
#define CGLU_TESS_ERROR          100103  /* void (CALLBACK*)(GLenum    errno) */
#define CGLU_TESS_EDGE_FLAG      100104  /* void (CALLBACK*)(GLboolean boundaryEdge)  */
#define CGLU_TESS_COMBINE        100105  /* void (CALLBACK*)(GLdouble  coords[3],
                                                            void      *data[4],
                                                            GLfloat   weight[4],
                                                            void      **dataOut)     */
#define CGLU_TESS_BEGIN_DATA     100106  /* void (CALLBACK*)(GLenum    type,  
                                                            void      *polygon_data) */
#define CGLU_TESS_VERTEX_DATA    100107  /* void (CALLBACK*)(void      *data, 
                                                            void      *polygon_data) */
#define CGLU_TESS_END_DATA       100108  /* void (CALLBACK*)(void      *polygon_data) */
#define CGLU_TESS_ERROR_DATA     100109  /* void (CALLBACK*)(GLenum    errno, 
                                                            void      *polygon_data) */
#define CGLU_TESS_EDGE_FLAG_DATA 100110  /* void (CALLBACK*)(GLboolean boundaryEdge,
                                                            void      *polygon_data) */
#define CGLU_TESS_COMBINE_DATA   100111  /* void (CALLBACK*)(GLdouble  coords[3],
                                                            void      *data[4],
                                                            GLfloat   weight[4],
                                                            void      **dataOut,
                                                            void      *polygon_data) */

/* TessError */
#define CGLU_TESS_ERROR1     100151
#define CGLU_TESS_ERROR2     100152
#define CGLU_TESS_ERROR3     100153
#define CGLU_TESS_ERROR4     100154
#define CGLU_TESS_ERROR5     100155
#define CGLU_TESS_ERROR6     100156
#define CGLU_TESS_ERROR7     100157
#define CGLU_TESS_ERROR8     100158

#define CGLU_TESS_MISSING_BEGIN_POLYGON  CGLU_TESS_ERROR1
#define CGLU_TESS_MISSING_BEGIN_CONTOUR  CGLU_TESS_ERROR2
#define CGLU_TESS_MISSING_END_POLYGON    CGLU_TESS_ERROR3
#define CGLU_TESS_MISSING_END_CONTOUR    CGLU_TESS_ERROR4
#define CGLU_TESS_COORD_TOO_LARGE        CGLU_TESS_ERROR5
#define CGLU_TESS_NEED_COMBINE_CALLBACK  CGLU_TESS_ERROR6

//-- /****           NURBS constants                 ****/
//-- 
//-- /* NurbsProperty */
//-- #define CGLU_AUTO_LOAD_MATRIX    100200
//-- #define CGLU_CULLING             100201
//-- #define CGLU_SAMPLING_TOLERANCE  100203
//-- #define CGLU_DISPLAY_MODE        100204
//-- #define CGLU_PARAMETRIC_TOLERANCE        100202
//-- #define CGLU_SAMPLING_METHOD             100205
//-- #define CGLU_U_STEP                      100206
//-- #define CGLU_V_STEP                      100207
//-- 
//-- /* NurbsSampling */
//-- #define CGLU_PATH_LENGTH                 100215
//-- #define CGLU_PARAMETRIC_ERROR            100216
//-- #define CGLU_DOMAIN_DISTANCE             100217
//-- 
//-- 
//-- /* NurbsTrim */
//-- #define CGLU_MAP1_TRIM_2         100210
//-- #define CGLU_MAP1_TRIM_3         100211
//-- 
//-- /* NurbsDisplay */
//-- /*      CGLU_FILL                100012 */
//-- #define CGLU_OUTLINE_POLYGON     100240
//-- #define CGLU_OUTLINE_PATCH       100241
//-- 
//-- /* NurbsCallback */
//-- /*      CGLU_ERROR               100103 */
//-- 
//-- /* NurbsErrors */
//-- #define CGLU_NURBS_ERROR1        100251
//-- #define CGLU_NURBS_ERROR2        100252
//-- #define CGLU_NURBS_ERROR3        100253
//-- #define CGLU_NURBS_ERROR4        100254
//-- #define CGLU_NURBS_ERROR5        100255
//-- #define CGLU_NURBS_ERROR6        100256
//-- #define CGLU_NURBS_ERROR7        100257
//-- #define CGLU_NURBS_ERROR8        100258
//-- #define CGLU_NURBS_ERROR9        100259
//-- #define CGLU_NURBS_ERROR10       100260
//-- #define CGLU_NURBS_ERROR11       100261
//-- #define CGLU_NURBS_ERROR12       100262
//-- #define CGLU_NURBS_ERROR13       100263
//-- #define CGLU_NURBS_ERROR14       100264
//-- #define CGLU_NURBS_ERROR15       100265
//-- #define CGLU_NURBS_ERROR16       100266
//-- #define CGLU_NURBS_ERROR17       100267
//-- #define CGLU_NURBS_ERROR18       100268
//-- #define CGLU_NURBS_ERROR19       100269
//-- #define CGLU_NURBS_ERROR20       100270
//-- #define CGLU_NURBS_ERROR21       100271
//-- #define CGLU_NURBS_ERROR22       100272
//-- #define CGLU_NURBS_ERROR23       100273
//-- #define CGLU_NURBS_ERROR24       100274
//-- #define CGLU_NURBS_ERROR25       100275
//-- #define CGLU_NURBS_ERROR26       100276
//-- #define CGLU_NURBS_ERROR27       100277
//-- #define CGLU_NURBS_ERROR28       100278
//-- #define CGLU_NURBS_ERROR29       100279
//-- #define CGLU_NURBS_ERROR30       100280
//-- #define CGLU_NURBS_ERROR31       100281
//-- #define CGLU_NURBS_ERROR32       100282
//-- #define CGLU_NURBS_ERROR33       100283
//-- #define CGLU_NURBS_ERROR34       100284
//-- #define CGLU_NURBS_ERROR35       100285
//-- #define CGLU_NURBS_ERROR36       100286
//-- #define CGLU_NURBS_ERROR37       100287
//-- 
//-- /****           Backwards compatibility for old tesselator           ****/
//-- 
//-- void APIENTRY   gluBeginPolygon( CGLUtesselator *tess );
//-- 
//-- void APIENTRY   gluNextContour(  CGLUtesselator *tess, 
//--                                  GLenum        type );
//-- 
//-- void APIENTRY   gluEndPolygon(   CGLUtesselator *tess );
//-- 
//-- /* Contours types -- obsolete! */
//-- #define CGLU_CW          100120
//-- #define CGLU_CCW         100121
//-- #define CGLU_INTERIOR    100122
//-- #define CGLU_EXTERIOR    100123
//-- #define CGLU_UNKNOWN     100124
//-- 
//-- /* Names without "TESS_" prefix */
//-- #define CGLU_BEGIN       CGLU_TESS_BEGIN
//-- #define CGLU_VERTEX      CGLU_TESS_VERTEX
//-- #define CGLU_END         CGLU_TESS_END
//-- #define CGLU_ERROR       CGLU_TESS_ERROR
//-- #define CGLU_EDGE_FLAG   CGLU_TESS_EDGE_FLAG

#ifdef __cplusplus
}
#endif

#endif  //_CGLU_PART_H
