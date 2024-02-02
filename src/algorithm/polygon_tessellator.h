#pragma once

#include <vector>
#include "math3d/vector_math.h"
#include "glu/glu_part.h"

class CGLUtesselator;
struct TessResult
{
	std::vector<Vector3>  vertices;
	std::vector<uint16>  indices;
	unsigned int renderType;  //GLenum
	size_t groupBeginVertex;
	size_t groupBeginIndex;
	bool erroredInGroup;
};

/**
	@brief
		Designed for complex polygon triangulation.
	*/
class PolygonTessellator
{
public:
	PolygonTessellator();
	~PolygonTessellator();

	/** A shortcut to process single contour polygon(including complex polygon).
		@return
			NULL if failed.
		@remarks
			Support both CW and CCW polygon.
	*/
	TessResult* tessellate(const Vector2* vertices, int n, int stride);

	/**
		* Handle complex polygon with multiple contours.
		*/
	void beginPolygon();
	void beginContour();
	void addPoints(const Vector3* points, int n);
	void endContour();
	TessResult* endPolygon();

private:
	CGLUtesselator* m_gluTess;
	TessResult m_tessResult;
	BatchedAllocator m_allocator;

	void _setCallbackForTess();
	void _resetResult();
	void _addOnePointToTess(Vector3 point);

	static void CALLBACK combineCallback(GLdouble coords[3], void* data[4], GLfloat weight[4], void** dataout, void* o);
	static void CALLBACK vertexCallback(void* vertexData, void *o);
	static void CALLBACK beginCallback(GLenum type, void *o);
	static void CALLBACK errorCallback(GLenum errorCode, void *o);
	static void CALLBACK endCallback(void *o);
};
