#include "stdafx.h"
#include "polygon_tessellator.h"
#include "glu/glu_part.h"

void PolygonTessellator::combineCallback(GLdouble coords[3], void* data[4], GLfloat weight[4], void** dataout, void* o)
{
	UNUSED_VAR(data);
	UNUSED_VAR(weight);
	PolygonTessellator* tessellator = (PolygonTessellator*)o;
	Vector3 *newPoint = (Vector3*)tessellator->m_allocator.allocMemory(sizeof(Vector3));
	newPoint->x = (GLfloat)coords[0];
	newPoint->y = (GLfloat)coords[1];
	newPoint->z = (GLfloat)coords[2];
	*dataout = (GLfloat*)newPoint;
}

void PolygonTessellator::vertexCallback(void* vertexData, void *o)
{
	PolygonTessellator* tessellator = (PolygonTessellator*)o;
	TessResult& tessResult = tessellator->m_tessResult;
	tessResult.vertices.push_back(*(Vector3*)vertexData);
}

void PolygonTessellator::beginCallback(GLenum type, void *o)
{
	PolygonTessellator* tessellator = (PolygonTessellator*)o;
	TessResult& tessResult = tessellator->m_tessResult;
	tessResult.renderType = type;
}

void PolygonTessellator::endCallback(void *o)
{
	PolygonTessellator* tessellator = (PolygonTessellator*)o;
	TessResult& tessResult = tessellator->m_tessResult;
	if(!tessResult.erroredInGroup)
	{
		uint16* pIndices;
		bool isEven;
		size_t iV = tessResult.groupBeginVertex;
		size_t iIdx = tessResult.groupBeginIndex;
		size_t iVNew = tessResult.vertices.size();
		size_t tessPtNum = iVNew - iV;
		CQ_ASSERT(tessPtNum >= 3);
		size_t iIdxNew = 0;

		switch(tessResult.renderType)
		{
		case GL_TRIANGLES:
			iIdxNew = iIdx + tessPtNum;
			tessResult.indices.resize(iIdxNew);
			pIndices = &tessResult.indices[iIdx];
			for(; iV<iVNew; iV++)
			{
				*pIndices++ = (uint16)iV;
			}
			break;

		case GL_TRIANGLE_STRIP:
			//CQ_ASSERT(tessPtNum%2 == 0);
			iIdxNew = iIdx + (tessPtNum-2)*3;
			tessResult.indices.resize(iIdxNew);
			pIndices = &tessResult.indices[iIdx];
			isEven = true;
			for(iV+=2; iV!=iVNew; iV++)
			{
				if(isEven)
				{
					*pIndices++ = (uint16)(iV-2);
					*pIndices++ = (uint16)(iV-1);
				}
				else
				{
					*pIndices++ = (uint16)(iV-1);
					*pIndices++ = (uint16)(iV-2);
				}
				*pIndices++ = (uint16)(iV  );
				isEven = !isEven;
			}
			break;

		case GL_TRIANGLE_FAN:
			iIdxNew = iIdx + (tessPtNum-2)*3;
			tessResult.indices.resize(iIdxNew);
			pIndices = &tessResult.indices[iIdx];
			for(iV+=2; iV!=iVNew;iV++)
			{
				*pIndices++ = (uint16)(tessResult.groupBeginVertex);
				*pIndices++ = (uint16)(iV-1);
				*pIndices++ = (uint16)(iV  );
			}
			break;

		default:
			CQ_TRACE("[ERROR]GLU TESS shouldn't output this type of shape: ", tessResult.renderType);
			CQ_ASSERT(FALSE);
		}
		tessResult.groupBeginVertex = iVNew;
		tessResult.groupBeginIndex = iIdxNew;
	}
	else
	{
		tessResult.vertices.resize(tessResult.groupBeginVertex);
		tessResult.indices.resize(tessResult.groupBeginIndex);
		tessResult.erroredInGroup = false;
	}
}

void PolygonTessellator::errorCallback(GLenum errorCode, void *o)
{
	PolygonTessellator* tessellator = (PolygonTessellator*)o;
	TessResult& tessResult = tessellator->m_tessResult;
	UNUSED_VAR(errorCode);
	CQ_TRACE("[ERROR] Tess-errorCallback() received an error: ", errorCode);
	tessResult.erroredInGroup = true;
}

void PolygonTessellator::_setCallbackForTess()
{
	CGLUtesselator* tessObj = m_gluTess;
	cgluTessCallback(tessObj, CGLU_TESS_VERTEX_DATA, (GLvoid(CALLBACK*) ())vertexCallback);
	cgluTessCallback(tessObj, CGLU_TESS_BEGIN_DATA, (GLvoid(CALLBACK*) ())beginCallback);
	cgluTessCallback(tessObj, CGLU_TESS_END_DATA, (GLvoid(CALLBACK*) ())endCallback);
	cgluTessCallback(tessObj, CGLU_TESS_ERROR_DATA, (GLvoid(CALLBACK*) ()) errorCallback);

	CGLUtessCombineProc combineCb = (CGLUtessCombineProc)combineCallback;  //To avoid a strange compile error
	cgluTessCallback(tessObj, CGLU_TESS_COMBINE_DATA, (GLvoid(CALLBACK*) ())combineCb);
}

PolygonTessellator::PolygonTessellator() : m_allocator(20 * sizeof(Vector3))
{
	m_gluTess = cgluNewTess();
	cgluTessNormal(m_gluTess, 0, 0, 1); // 手动指定glu的法向量，这样第一 glu内部不用根据你传入的点重新计算法向量， 第二，能保证输出的三角形都是逆时针的
	cgluTessProperty(m_gluTess, CGLU_TESS_WINDING_RULE, CGLU_TESS_WINDING_NONZERO);
	_setCallbackForTess();
	m_tessResult.vertices.reserve(100);
	m_tessResult.indices.reserve(200);
}

PolygonTessellator::~PolygonTessellator()
{
	cgluDeleteTess(m_gluTess);
}

TessResult* PolygonTessellator::tessellate(const Vector2* vertices, int n, int stride)
{
	if (NULL == vertices || n < 3)
	{
		return NULL;
	}
		
	beginPolygon();
	beginContour();
	for (int i = 0; i < n; i++)
	{
		_addOnePointToTess(vec3(*vertices, 0));
		vertices = (const Vector2*)((const uint8*)vertices + stride);
	}
	endContour();
	endPolygon();

	return &m_tessResult;
}

void PolygonTessellator::_resetResult()
{
	m_allocator.freeAll();
	m_tessResult.vertices.clear();
	m_tessResult.indices.clear();
	m_tessResult.renderType = 0;
	m_tessResult.groupBeginVertex = 0;
	m_tessResult.groupBeginIndex = 0;
	m_tessResult.erroredInGroup = false;
}

void PolygonTessellator::_addOnePointToTess(Vector3 point)
{
	Vector3 *newPoint = (Vector3*)m_allocator.allocMemory(sizeof(Vector3));
	*newPoint = point;
	cgluTessVertex(m_gluTess, (GLdouble*)(newPoint), newPoint);
}

void PolygonTessellator::beginPolygon()
{
	_resetResult();
	cgluTessBeginPolygon(m_gluTess, this);
}

void PolygonTessellator::beginContour()
{
	cgluTessBeginContour(m_gluTess);
}

void PolygonTessellator::addPoints(const Vector3* points, int n)
{
	for (int i = 0; i < n; i++, points++)
	{
		_addOnePointToTess(*points);
	}
}

void PolygonTessellator::endContour()
{
	cgluTessEndContour(m_gluTess);
}

TessResult* PolygonTessellator::endPolygon()
{
	cgluTessEndPolygon(m_gluTess);
	return &m_tessResult;
}

