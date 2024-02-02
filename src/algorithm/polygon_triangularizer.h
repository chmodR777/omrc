#pragma once
#include <vector>
#include "math3d/vector_math.h"
enum PolygonDirection
{
	PolygonDirection_unknown = 0,
	PolygonDirection_CCW = 1,  //counter clock-wise
	PolygonDirection_CW = 2    //clock-wise
};

#define EPSILON 0.0000001f			 /* 10^-8 would make FLOAT_EQUAL(var, 0.0f) too strict */
class PolygonTessellator;
struct TessResult;

class PolygonTriangularizer
{
public:
	PolygonTriangularizer();
	~PolygonTriangularizer();
	/**
		Triangularize a simple polygon (using ear clipping algorithm, O(n^2)) or a complex polygon with self-intersections.
		@note
			1. The @a contour can be a loop (the last point equals the first point) or not,
				and they produce the same result.
			2. Triangulation is performed on x-y plane for 3D version; real 3D surface mesh generation is not supported.
	*/
	bool triangularize(const Vector2* contour, size_t n, PolygonDirection dir, size_t inputStride);

	/**
		@brief
			Triangulate with minimum weight triangulation (MWT) algorithm.
			Note the time complexity is O(n^3); it could be extremely slow with large vertex count.
			For more information see http://www.ist.tugraz.at/_attach/Publish/Eaa19/Chapter_04_MWT_handout.pdf
		@param contour Not closed polygon (either CW or CCW, and the result is in the same direction)
		@note
			1、主要适用路面，而对于某些特殊的凹多边形可能会产生区域外的三角形结果。
			2、支持自交多边形，例如自交的环形匝道。
	*/
	bool triangularizeMWT(const Vector2* contour, int n, int inputStride);

	/// Designed specifically for stroke triangulation, suitable for regular roads/lanes.
	/// It's very efficient and time complexity is O(n).
	///
	/// The contour is of length (@p rightN + @p leftN), organized in this form:
	///
	/// contour[rightN+leftN-1]                    contour[rightN]
	/// --------------------<-----------------------
	///                                            |
	///                                            ^
	///                                            |
	/// -------------------->-----------------------
	/// contour[0]                                 contour[rightN-1]
	///
	/// It Produces CCW triangles.
	bool triangularizeStroke(const Vector2* contour, int rightN, int leftN, int inputStride);

	/// The contour is of length (@p halfCount * 2), organized in this form:
	///
	/// contour[halfCount * 2 - 1]                 contour[halfCount]
	/// --------------------<-----------------------
	///                                            |
	///                                            ^
	///                                            |
	/// -------------------->-----------------------
	/// contour[0]                                 contour[halfCount-1]
	///
	/// It Produces CCW triangles..
	bool triangularizeStrokeSymmetrical(const Vector2* contour, int halfCount, int inputStride);

	/**
		@brief
			Designed for complex polygon triangulation.
	*/
	void beginComplexPolygon();
	void beginContour();
	/**
		@brief
			After the points have been added, the space occupied by points can be freed/reused
	*/
	void addPoints(const Vector3* points, int n);
	void endContour();
	bool endComplexPolygon();

	forceinline size_t pointNumber() {
		return m_resultPoints.size();
	}
	forceinline Vector2* pointArray() {
		return &m_resultPoints[0];
	}

	/**
		An array of length pointNumber(), each of which is the index of that result point to the original contour.

		@note
			In some cases (like non-simple polygons), the mapping could not be built,
			and NULL will be returned.
	*/
	forceinline int* pointMapping() { return m_resultMapping.size() > 0 ? &m_resultMapping[0] : NULL; }

	forceinline size_t indexNumber() {
		return m_resultIndex.size();
	}
	forceinline uint16* indexArray() {
		return &m_resultIndex[0];
	}

	void copyPoints(Vector2* target, size_t targetStride);
	void copyIndex(uint16* target, uint16 startIndex);
	void copyConstFloat(float* target, float constValue, size_t targetStride);
	void copyConstUint32(uint32* target, uint32 constValue, size_t targetStride);

private:
	// mwt
	cqstd::vector<float> m_weights;
	cqstd::vector<int> m_S;

	void _makeMWTIndexes(int i, int j, int n);

	cqstd::vector<Vector2> m_resultPoints;
	cqstd::vector<uint16> m_resultIndex;
	cqstd::vector<int> m_resultMapping;
	size_t m_floatStride;

	std::shared_ptr<PolygonTessellator> m_tessellator;

	static bool _insideTriangle(float Ax, float Ay,
		float Bx, float By,
		float Cx, float Cy,
		float Px, float Py);

	bool _snip(const float* contour, size_t u, size_t v, size_t w, size_t n, std::vector<int>&);

	bool _triangulateWithTessellator(const Vector2* vertices, int n, int stride);
	void _dumpResult(TessResult* result);
	void _resetResult();
};

float Triangularizer_calculatePolygonArea(const Vector2* contour, size_t n);

/// @note It's not real 3D, but based on 2D projection.
float Triangularizer_calculatePolygonArea3D(const Vector3* contour, size_t n);

forceinline PolygonDirection Triangularizer_getPolygonDirection(const Vector2* points, size_t num)
{
	float a = Triangularizer_calculatePolygonArea(points, num);
	return a > EPSILON ? PolygonDirection_CCW : (a < EPSILON ? PolygonDirection_CW : PolygonDirection_unknown);
}

/// @note It's not real 3D, but based on 2D projection.
forceinline PolygonDirection Triangularizer_getPolygonDirection3D(const Vector3* points, size_t num)
{
	float a = Triangularizer_calculatePolygonArea3D(points, num);
	return a > EPSILON ? PolygonDirection_CCW : (a < EPSILON ? PolygonDirection_CW : PolygonDirection_unknown);
}
