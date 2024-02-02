#include "stdafx.h"
#include "polygon_tessellator.h"
#include "polygon_triangularizer.h"
#include <float.h>

forceinline static const Vector2* _vec2BufElem(const Vector2* points, int stride, int i)
{
	return (const Vector2*)((uint8*)points + i * stride);
}

template <class PointT>
static float _Triangularizer_calculatePolygonArea(const PointT* contour, size_t n)
{
	float A = 0.0f;

	for (size_t p = n - 1, q = 0; q < n; p = q++)
	{
		A += contour[p].x * contour[q].y - contour[q].x * contour[p].y;
	}
	return A * 0.5f;
}

float Triangularizer_calculatePolygonArea(const Vector2* contour, size_t n)
{
	return _Triangularizer_calculatePolygonArea(contour, n);
}

float Triangularizer_calculatePolygonArea3D(const Vector3* contour, size_t n)
{
	return _Triangularizer_calculatePolygonArea(contour, n);
}

forceinline float _calcCircumference(Vector2 a, Vector2 b, Vector2 c)
{
	return (a - b).length() + (b - c).length() + (c - a).length();
}

static float _calculatePolygonArea(const float* contour, size_t n, size_t floatStride)
{
	float A = 0.0f;

	for (size_t p = n - 1, q = 0; q < n; p = q++)
	{
		A += contour[p * floatStride] * contour[q * floatStride + 1] - contour[q * floatStride] * contour[p * floatStride + 1];
	}
	return A * 0.5f;
}

PolygonTriangularizer::PolygonTriangularizer()
{
	m_tessellator = std::make_shared<PolygonTessellator>();
}

// decide if point Px/Py is inside triangle defined by
// (Ax,Ay) (Bx,By) (Cx,Cy)
bool PolygonTriangularizer::_insideTriangle(float Ax, float Ay,
	float Bx, float By,
	float Cx, float Cy,
	float Px, float Py)
{
	float ax, ay, bx, by, cx, cy, apx, apy, bpx, bpy, cpx, cpy;
	float cCROSSap, bCROSScp, aCROSSbp;

	ax = Cx - Bx;  ay = Cy - By;
	bx = Ax - Cx;  by = Ay - Cy;
	cx = Bx - Ax;  cy = By - Ay;
	apx = Px - Ax;  apy = Py - Ay;
	bpx = Px - Bx;  bpy = Py - By;
	cpx = Px - Cx;  cpy = Py - Cy;

	aCROSSbp = ax*bpy - ay*bpx;
	cCROSSap = cx*apy - cy*apx;
	bCROSScp = bx*cpy - by*cpx;

	return ((aCROSSbp >= 0.0f) && (bCROSScp >= 0.0f) && (cCROSSap >= 0.0f));
}


bool PolygonTriangularizer::_snip(const float* contour, size_t u, size_t v, size_t w, size_t n, std::vector<int>& V)
{
	size_t p;
	float Ax, Ay, Bx, By, Cx, Cy, Px, Py;

	Ax = contour[V[u] * m_floatStride];
	Ay = contour[V[u] * m_floatStride + 1];

	Bx = contour[V[v] * m_floatStride];
	By = contour[V[v] * m_floatStride + 1];

	Cx = contour[V[w] * m_floatStride];
	Cy = contour[V[w] * m_floatStride + 1];

	if (EPSILON > (((Bx - Ax)*(Cy - Ay)) - ((By - Ay)*(Cx - Ax)))) return false;

	for (p = 0; p < n; p++)
	{
		if ((p == u) || (p == v) || (p == w)) continue;
		Px = contour[V[p] * m_floatStride];
		Py = contour[V[p] * m_floatStride + 1];
		if (_insideTriangle(Ax, Ay, Bx, By, Cx, Cy, Px, Py)) return false;
	}

	return true;
}

bool PolygonTriangularizer::_triangulateWithTessellator(const Vector2* vertices, int n, int stride)
{
	_resetResult();
	TessResult* result = m_tessellator->tessellate(vertices, n, stride);
	bool success = false;
	if (result != NULL && !result->indices.empty())
	{
		_dumpResult(result);
		success = true;
	}
	return success;
}

bool PolygonTriangularizer::triangularize(const Vector2* contour, size_t n, PolygonDirection dir, size_t stride)
{
	if (n < 3)
		return false;

	CQ_ASSERT(stride % 4 == 0 && "Triangularize input data stride must be aligned to 4 bytes and pointNum >= 1");

	/* allocate and initialize list of Vertices in polygon */
	const float* inputBuffer = (float*)contour;
	m_floatStride = stride / 4;

	// remove the last point if it's the same with the first one.
	if (inputBuffer[0] == inputBuffer[(n - 1) * m_floatStride]
		&& inputBuffer[1] == inputBuffer[(n - 1) * m_floatStride + 1])
	{
		n--;
	}

	if (n < 3)
		return false;

	m_resultPoints.clear();
	m_resultIndex.clear();

	std::vector<int> V; 
	V.reserve(n);
	V.resize(n);

	/* we want a counter-clockwise polygon in V */
	// fixit: if the data is all counter-clock wise, we won't need to reverse them.
	if (dir == PolygonDirection_unknown)
	{
		if (0.0f < _calculatePolygonArea(inputBuffer, n, m_floatStride))
			dir = PolygonDirection_CCW;
		else
			dir = PolygonDirection_CW;
	}

	if (dir == PolygonDirection_CCW)
		for (size_t v = 0; v<n; v++) V[v] = (int)v;
	else
		for (size_t v = 0; v<n; v++) V[v] = (int)((n - 1) - v);

	size_t nv = n;

	// copy points
	m_resultPoints.resize(n);
	m_resultMapping.resize(n);
	Vector2* buffer = &m_resultPoints[0];
	for (size_t v = 0; v < n; v++)
	{
		buffer[v].x = inputBuffer[v * m_floatStride];
		buffer[v].y = inputBuffer[v * m_floatStride + 1];

		m_resultMapping[v] = ((int)v);
	}

	/*  remove nv-2 Vertices, creating 1 triangle every time */
	int count = (int)(2 * nv);   /* error detection */

	for (size_t m = 0, v = nv - 1; nv>2;)
	{
		/* if we loop, it is probably a non-simple polygon */
		if (0 >= (count--))
		{
			//** Failed to triangulate, usually it's because crossing edges in the polygon.
			//cq_trace("triangulate error", 1);
			
			//If all points of the polygon is in same line, it'll return false
			return _triangulateWithTessellator((const Vector2*)inputBuffer, (int)n, (int)m_floatStride * sizeof(float));
		}

		/* three consecutive vertices in current polygon, <u,v,w> */
		size_t u = v; if (nv <= u) u = 0;     /* previous */
		v = u + 1; if (nv <= v) v = 0;     /* new v    */
		size_t w = v + 1; if (nv <= w) w = 0;     /* next     */

		if (_snip(inputBuffer, u, v, w, nv, V))
		{
			size_t a, b, c, s, t;

			/* true names of the vertices */
			a = V[u]; b = V[v]; c = V[w];

			/* output Triangle */
			m_resultIndex.push_back((uint16)a);
			m_resultIndex.push_back((uint16)b);
			m_resultIndex.push_back((uint16)c);
			m++;

			/* remove v from remaining polygon */
			for (s = v, t = v + 1; t<nv; s++, t++) V[s] = V[t]; nv--;

			/* resest error detection counter */
			count = (int)(2 * nv);
		}
	}

	return true;
}

void PolygonTriangularizer::copyPoints(Vector2* target, size_t targetStride)
{
	size_t num = m_resultPoints.size();
	Vector2* src = &m_resultPoints[0];
	Vector2* srcEnd = src + num;
	float* dest = (float*)target;
	
	CQ_ASSERT(targetStride % 4 == 0 && "Triangularize input data stride must be aligned to 4 bytes");
	size_t floatStride = targetStride / 4;

	for (; src != srcEnd; src++)
	{
		dest[0] = src->x;
		dest[1] = src->y;
		dest += floatStride;
	}
}

void PolygonTriangularizer::copyIndex(uint16* target, uint16 startIndex)
{
	size_t num = m_resultIndex.size();
	uint16* src = &m_resultIndex[0];
	uint16* srcEnd = src + num;
	while(src != srcEnd)
	{
		*target++ = (*src++) + startIndex;
	}
}

void PolygonTriangularizer::copyConstFloat(float* target, float constValue, size_t targetStride)
{
	CQ_ASSERT(targetStride % 4 == 0 && "Triangularize input data stride must be aligned to 4 bytes");
	size_t floatStride = targetStride / 4;

	float* targetEnd = target + m_resultPoints.size() * floatStride;
	while ( target != targetEnd )
	{
		*target = constValue;
		target += floatStride;
	}
}

void PolygonTriangularizer::copyConstUint32(uint32* target, uint32 constValue, size_t targetStride)
{
	CQ_ASSERT(targetStride % 4 == 0 && "Triangularize input data stride must be aligned to 4 bytes");
	size_t floatStride = targetStride / 4;

	uint32* targetEnd = target + m_resultPoints.size() * floatStride;
	while (target != targetEnd)
	{
		*target = constValue;
		target += floatStride;
	}
}

void PolygonTriangularizer::_dumpResult(TessResult* result)
{
	//Copy vertex data, with format converting.
	size_t tessPtNum = result->vertices.size();
	Vector3* srcVertex = &result->vertices[0];
	Vector3* srcVertexEnd = srcVertex + tessPtNum;

	m_resultPoints.resize(tessPtNum);
	Vector2* destVertex = &m_resultPoints[0];

	for (; srcVertex != srcVertexEnd; ++srcVertex, ++destVertex)
	{
		destVertex->x = srcVertex->x;
		destVertex->y = srcVertex->y;
	}

	//Copy index data.
	size_t tessIdxNum = result->indices.size();
	uint16* srcIndex = &result->indices[0];
	m_resultIndex.assign(srcIndex, srcIndex + tessIdxNum);
}

void PolygonTriangularizer::beginComplexPolygon()
{
	_resetResult();
	m_tessellator->beginPolygon();
}

bool PolygonTriangularizer::endComplexPolygon()
{
	TessResult* result = m_tessellator->endPolygon();
	if (result->indices.empty())
		return false;

	_dumpResult(result);
	return true;
}

void PolygonTriangularizer::_resetResult()
{
	m_resultPoints.clear();
	m_resultIndex.clear();
	m_resultMapping.clear();
}

void PolygonTriangularizer::beginContour()
{
	m_tessellator->beginContour();
}

bool PolygonTriangularizer::triangularizeMWT(const Vector2* contour, int n, int inputStride)
{
	_resetResult();

	if (n < 3)
		return false;

	// copy points
	m_resultPoints.resize(n);
	for (int i = 0; i < n; i++)
		m_resultPoints[i] = *_vec2BufElem(contour, inputStride, i);

	// Request memory
	int nSquared = n * n;
	if (nSquared > (int)m_weights.size())
	{
		m_weights.resize(nSquared);
		m_S.resize(nSquared);
	}

	memset(m_S.begin(), 0, sizeof(int) * nSquared);

	for (int i = 0; i < n - 1; i++)
		m_weights[i * n + i + 1] = 0.f;

	// mwt process
	for (int l = 2; l < n; l++)
	{
		for (int i = 0; i < n - l; i++)
		{
			int j = i + l;
			m_weights[i * n + j] = FLT_MAX;

			for (int k = i + 1; k < j; k++)
			{
				float circumference = _calcCircumference(m_resultPoints[i], m_resultPoints[j], m_resultPoints[k]);
				float weight = m_weights[i * n + k] + m_weights[k * n + j] + circumference;
				if (weight < m_weights[i * n +j])
				{
					m_weights[i * n + j] = weight;
					m_S[i * n + j] = k;
				}
			}
		}
	}

	// make edge
	m_resultIndex.clear();
	_makeMWTIndexes(0, n - 1, n);
	m_resultMapping.resize(n);
	for (int i = 0; i < n; i++)
		m_resultMapping[i] = i;
	return (int)m_resultIndex.size() == (n - 2) * 3;
}

bool PolygonTriangularizer::triangularizeStroke(const Vector2* contour, int rightN, int leftN, int inputStride)
{
	_resetResult();

	if (leftN < 1 || rightN < 1 || leftN + rightN < 3 || leftN + rightN > 0xffff)
		return false;

	for (int i = 0; i < leftN + rightN; i++)
		m_resultPoints.push_back(*_vec2BufElem(contour, inputStride, i));

	for (int i = 0; i < leftN + rightN; i++)
		m_resultMapping.push_back(i);

	for (uint16 leftIdx = leftN + rightN - 1, rightIdx = 0; leftIdx > rightN || rightIdx + 1 < rightN;)
	{
		m_resultIndex.push_back(leftIdx);
		m_resultIndex.push_back(rightIdx);

		// Heuristically find the minimum weight

		if (leftIdx == rightN)// left out of range
			m_resultIndex.push_back(++rightIdx);
		else if (rightIdx + 1 == rightN)// right out of range
			m_resultIndex.push_back(--leftIdx);
		else //All in range
		{
			float leftWeight = (*_vec2BufElem(contour, inputStride, leftIdx - 1) - *_vec2BufElem(contour, inputStride, rightIdx)).lengthSquared();
			float rightWeight = (*_vec2BufElem(contour, inputStride, rightIdx + 1) - *_vec2BufElem(contour, inputStride, leftIdx)).lengthSquared();
			if (leftWeight < rightWeight)
				m_resultIndex.push_back(--leftIdx);
			else
				m_resultIndex.push_back(++rightIdx);
		}
	}

	return true;
}

bool PolygonTriangularizer::triangularizeStrokeSymmetrical(const Vector2* contour, int halfCount, int inputStride)
{
	_resetResult();

	int count = halfCount * 2;
	if (count < 3 || count > 0xffff)
		return false;

	for (int i = 0; i < count; i++)
		m_resultPoints.push_back(*_vec2BufElem(contour, inputStride, i));

	for (int i = 0; i < count; i++)
		m_resultMapping.push_back(i);


	uint16 leftIdx = count - 1;
	uint16 rightIdx = 0;
	for (int i = 0; i < halfCount; i++)
	{
		// triangle 1
		m_resultIndex.push_back(leftIdx);
		m_resultIndex.push_back(rightIdx);
		m_resultIndex.push_back(rightIdx + 1);

		// triangle 2
		m_resultIndex.push_back(leftIdx);
		m_resultIndex.push_back(rightIdx + 1);
		m_resultIndex.push_back(leftIdx - 1);

		leftIdx--;
		rightIdx++;
	}

	return true;
}

void PolygonTriangularizer::addPoints(const Vector3* points, int n)
{
	m_tessellator->addPoints(points, n);
}

void PolygonTriangularizer::endContour()
{
	m_tessellator->endContour();
}

PolygonTriangularizer::~PolygonTriangularizer()
{
}

void PolygonTriangularizer::_makeMWTIndexes(int i, int j, int n)
{
	if (i >= j - 1 || (int)m_resultIndex.size() > (n - 2) * 3)
		return;

	int k = m_S[i * n + j];
	m_resultIndex.push_back((uint16)i);
	m_resultIndex.push_back((uint16)k);
	m_resultIndex.push_back((uint16)j);

	CQ_ASSERT(i <= k && k <= j);
	
	_makeMWTIndexes(i, k, n);
	_makeMWTIndexes(k, j, n);
}