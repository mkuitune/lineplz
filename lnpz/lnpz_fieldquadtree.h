// This file lnpz_fieldquadtree.h is part of lineplz - Public domain simple drawing library
//
// The original author of this software is Mikko Kuitunen, and its permanent
// home is <http://github.com/mkuitune/lineplz/>. If you find this software
// useful, an acknowledgement in your source text and/or product documentation
// is appreciated, but not required.
//
// This is free and unencumbered software released into the public domain.
//
// Anyone is free to copy, modify, publish, use, compile, sell, or
// distribute this software, either in source code form or as a compiled
// binary, for any purpose, commercial or non-commercial, and by any
// means.
//
// In jurisdictions that recognize copyright laws, the author or authors
// of this software dedicate any and all copyright interest in the
// software to the public domain. We make this dedication for the benefit
// of the public at large and to the detriment of our heirs and
// successors. We intend this dedication to be an overt act of
// relinquishment in perpetuity of all present and future rights to this
// software under copyright law.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
// OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
//
// For more information, please refer to <http://unlicense.org/>

#pragma once

#include <stdint.h>
#include <vector>
#include <functional>
#include <algorithm>
#include <stack>

#include <lnpz/lnpz_linalgd.h>

#include "lnpz_util.h"

namespace lnpz {


	//  i:th level cell
	//  x--x
	//  |  |
	//  x--x
	//	
	//  vertex indices
	//  3--2
	//  |  |
	//  0--1
	//   
	//  i+1:th level cells
	//
	// x--x--x
	// |nw|ne|
	// x--x--x
	// |sw|se|
	// x--x--x
	//	
	// x--x--x
	// |3 |2 |
	// x--x--x
	// |0 |1 |
	// x--x--x
	//
	// Sample points in cell
	// x--3--x
	// |     |
	// 4  0  2  
	// |     |
	// x--1--x

	struct fq_coordinate_t {
		float x, y;
	};

	struct fq_corners_t {
		fq_coordinate_t corners[4];
	};

	struct FieldQuadtreeNode;

	struct fq_interpoloation_samples_t {
		fq_coordinate_t coords[5];
		float values[5];

		float sampleDifference(float measured[5]) const {
			float d = 0.f;
			for (int i = 0; i < 5; i++) {
				d += fabsf(values[i] - measured[i]);
			}
			return d;
		}

		float maxSampleDifference(float measured[5]) const {
			float d = 0.f;
			for (int i = 0; i < 5; i++) {
				d += fabsf(values[i] - measured[i]);
			}

			return d;
		}

		float maxSampleDifferenceRelative(float measured[5]) const {
			int i = 0;
			float d = fabsf((values[i] - measured[i]) / measured[i]);
			i++;
			for (; i < 5; i++) {
				d = std::max(d, fabsf((values[i] - measured[i]) / measured[i]));
			}

			return d;
		}

		void sampleField(float realValues[5], std::function<float(float, float)> field) const {
			for (int i = 0; i < 5; i++) {
				auto pnt = coords[i];
				realValues[i] = field(pnt.x, pnt.y);
			}
		}
	};

	struct cornerdata_t {
		float field;
	};

	struct FieldQuadtreeNode {
		cornerdata_t cornerdata[4]; // store adf value here
		size_t childs; // every subdividion creates four children
		float x0, y0, d;
		uint8_t depth = 0;

		float maxVal, minVal;

		// compute these after tree is built
		void computeFieldDomain(std::vector<FieldQuadtreeNode>& buf) {
			using namespace util;

			maxVal = MaxOf(
				cornerdata[0].field,
				cornerdata[1].field,
				cornerdata[2].field,
				cornerdata[3].field);

			minVal = MinOf(
				cornerdata[0].field,
				cornerdata[1].field,
				cornerdata[2].field,
				cornerdata[3].field);

			if (childs != 0) {
				buf[childs].computeFieldDomain(buf);
				buf[childs + 1].computeFieldDomain(buf);
				buf[childs + 2].computeFieldDomain(buf);
				buf[childs + 3].computeFieldDomain(buf);

				float childmax = MaxOf(
					buf[childs].maxVal,
					buf[childs + 1].maxVal,
					buf[childs + 2].maxVal,
					buf[childs + 3].maxVal);

				float childmin = MinOf(
					buf[childs].minVal,
					buf[childs + 1].minVal,
					buf[childs + 2].minVal,
					buf[childs + 3].minVal);

				maxVal = std::max(maxVal, childmax);
				minVal = std::min(minVal, childmin);
			}
		}

		fq_corners_t corners() const {
			return {
				{{x0,y0},{x0 + d, y0},{x0 + d, y0 + d},{x0, y0 + d}}
			};
		}

		float divergence(float measured[5]) const;

		float rot() const {
			float s = 0.f;
			for (int i = 1; i < 4; i++)
				s += (cornerdata[i % 4].field - cornerdata[(i - 1) % 4].field);
			return s;
		}

		bool isInside(float x, float y) const {
			return (x >= x0 && x <= (x0 + d)) &&
				(y >= y0 && y <= (y0 + d));
		}

		void applyField(std::function<float(float, float)> field) {
			auto corpos = corners();
			for (int i = 0; i < 4; i++) {
				auto pos = corpos.corners[i];
				cornerdata[i].field = field(pos.x, pos.y);
			}
		}

		void applyFieldToExisting(std::function<float(float, float)> field) {
			auto corpos = corners();
			for (int i = 0; i < 4; i++) {
				auto pos = corpos.corners[i];
				float smp = field(pos.x, pos.y);
				if (fabsf(cornerdata[i].field) > fabsf(smp))
					cornerdata[i].field = smp;
			}
		}

		void initFromPrevious(const FieldQuadtreeNode& prev) {
			auto corpos = corners();
			for (int i = 0; i < 4; i++) {
				auto pos = corpos.corners[i];
				cornerdata[i].field = prev.sampleCorners(pos.x, pos.y);
			}
		}

		fq_interpoloation_samples_t samplepoints() const {
			float h = d / 2;
			fq_interpoloation_samples_t s = {
			   {{x0 + h,y0 + h},{x0 + h, y0},{x0 + d, y0 + h},{x0 + h, y0 + d}, {x0, y0 + h},
				},
			   {0.f, 0.f, 0.f, 0.f, 0.f}
			};
			for (int i = 0; i < 5; i++) {
				auto pnt = s.coords[i];
				s.values[i] = sampleCorners(pnt.x, pnt.y);
			}
			return s;
		}

		float interpolateX(float x) const {
			return (x - x0) / d;
		}
		float interpolateY(float y) const {
			return (y - y0) / d;
		}

		// bilinear interpolation of corner values
		float sampleCorners(float x, float y) const {
			using namespace util;

			float u = interpolateX(x);
			float v = interpolateY(y);

#if 0 // uncomment if need to debug this for some reason
			if (u < 0.f || u > 1.f)
				throw std::exception("u wrong");
			if (v < 0.f || v > 1.f)
				throw std::exception("v wrong");
#endif

			float r1 = Lerp(cornerdata[3].field, cornerdata[2].field, u);
			float r0 = Lerp(cornerdata[0].field, cornerdata[1].field, u);
			return Lerp(r0, r1, v);
		}

		//constexpr static float FieldInitial() {
	//		return -1.0e9;
	//	}
		constexpr static float FieldInitial() {
			return 1.0e9;
		}

		size_t getChildIdx(float x, float y) const {
			size_t childIdx = 0;
			float h = d / 2;
			if ((y - y0) > h) {
				childIdx = (x - x0) < h ? 3 : 2;
			}
			else {
				childIdx = (x - x0) < h ? 0 : 1;
			}
			return childs + childIdx;
		}

		static FieldQuadtreeNode Init(float x, float y, float d) {
			FieldQuadtreeNode n;
			n.childs = 0;
			n.x0 = x;
			n.y0 = y;
			n.d = d;
			n.cornerdata[0].field = FieldInitial();
			n.cornerdata[1].field = FieldInitial();
			n.cornerdata[2].field = FieldInitial();
			n.cornerdata[3].field = FieldInitial();
			return n;
		}
	};

	struct FieldQuadtree {
		std::vector<FieldQuadtreeNode> nodes;
		static const size_t NPOS = -1;

		size_t maxDepth() const {
			const float d0 = nodes[0].d;
			float cd = 1.0f;
			for (const auto& n : nodes) {
				float cdn = (d0) / (n.d);
				cd = std::max(cd, cdn);
			}
			return (size_t)(cd * 0.5f);
		}

		float getDeepSampleInt(size_t idx, float x, float y) const {
			while (nodes[idx].childs != 0) {
				idx = nodes[idx].getChildIdx(x, y);
			}
			return nodes[idx].sampleCorners(x, y);
		}

		float getDeepSampleIntClamped(size_t idx, float x, float y, float domainMin, float domainMax) const {
			using namespace util;
			float iMax = nodes[idx].maxVal;
			float iMin = nodes[idx].minVal;
			if (iMax < domainMin)
				return iMax;
			if (iMin > domainMax)
				return iMin;

			while (nodes[idx].childs != 0) {
				idx = nodes[idx].getChildIdx(x, y);

				iMax = nodes[idx].maxVal;
				iMin = nodes[idx].minVal;
				if (iMax < domainMin)
					return iMax;
				if (iMin > domainMax)
					return iMin;
			}

			float sample = nodes[idx].sampleCorners(x, y);
			return Clampf(sample, domainMin, domainMax);
		}

		float getDeepSample(float x, float y) const {
			if (!nodes[0].isInside(x, y))
				return FieldQuadtreeNode::FieldInitial();

			return getDeepSampleInt(0, x, y);
		}

		// If the tree value is not within domain return just upper or lower value of the domain
		// this can be used to limit the depth of tree and simplify the computation,
		// but the input domain should be carefully selected
		// In some cases the clamped sample is more or less equal in time as the 
		// unclampled one - test performance
		float getDeepSampleClamped(float x, float y, float domainMin, float domainMax) const {
			if (!nodes[0].isInside(x, y))
				return FieldQuadtreeNode::FieldInitial();

			return getDeepSampleIntClamped(0, x, y, domainMin, domainMax);
		}

		size_t getIdxAtFromInt(size_t idx, float x, float y) const {
			while (nodes[idx].childs != 0) {
				idx = nodes[idx].getChildIdx(x, y);
			}
			return idx;
		}

		size_t getIdxAt(float x, float y) const {
			if (!nodes[0].isInside(x, y))
				return NPOS;

			return getIdxAtFromInt(0, x, y);
		}
	};

	struct FieldQuadtreeBuilder {

		FieldQuadtree  tree;
		float x; float y; float d;
		//float threshold = 1.f; // jaggies in lines
		//float threshold = 0.5f;
		//float threshold = 0.2f;
		float threshold = 0.1f;
		float thresholdrelative = 0.01f;

		uint8_t maxnodedepth = 10;

		FieldQuadtreeBuilder(float xin, float yin, float din) :x(xin), y(yin), d(din) {
		}

		FieldQuadtree  build() {
			tree.nodes[0].computeFieldDomain(tree.nodes);
			return tree;
		}

		void add(std::function<float(float, float)> field) {
			if (tree.nodes.empty())
				addNew(field);
			else
				addExisting(field);
		}

		void addExisting(std::function<float(float, float)> field);
		void addNew(std::function<float(float, float)> field);
	};

	FieldQuadtree Transform(const FieldQuadtree& src, const util::LinearMap2D& map);
}
