// This file lnpz_fieldquadtree.cpp is part of lineplz - Public domain simple drawing library
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

#include "lnpz_fieldquadtree.h"

namespace lnpz {

	void FieldQuadtreeBuilder::addExisting(std::function<float(float, float)> field) {

		std::stack<size_t> notProcessed;
		for (size_t n = 0; n < tree.nodes.size(); n++)
			notProcessed.push(n);

		// Need to refer to the old node
		FieldQuadtreeNode oldNode;
		size_t parentCount = tree.nodes.size();

		while (!notProcessed.empty()) {
			size_t ni = notProcessed.top();
			notProcessed.pop();
			FieldQuadtreeNode& node = tree.nodes[ni];

			if (ni < parentCount) {
				// This is parent node, override the current parent - DON'T USE NEW NODES AS REFERENCE FIELD as that
				// loses information
				oldNode = node;
				node.applyFieldToExisting(field);
			}

			// Store old node so we can sample the field
			// subdivide only current leafs. Othewise just apply field 
			if (node.childs != 0) {
				continue;
			}

			// Find of precision suffices or do we need to subdivide
			fq_interpoloation_samples_t samplepoints = node.samplepoints();
			float realValues[5]; // Real values at sample points
			for (int i = 0; i < 5; i++) {
				auto pnt = samplepoints.coords[i];
				float interpolatedFieldValue = oldNode.sampleCorners(pnt.x, pnt.y);
				float newFieldValue = field(pnt.x, pnt.y);
				realValues[i] = std::min(interpolatedFieldValue, newFieldValue);
			}

			float diff = samplepoints.maxSampleDifference(realValues);
			bool subdivide = (diff > threshold) && node.depth < maxnodedepth;

			if (subdivide) {
				// subdivided
				float d2 = node.d / 2;
				size_t newIdx = tree.nodes.size();

				FieldQuadtreeNode n0 = FieldQuadtreeNode::Init(node.x0, node.y0, d2);
				FieldQuadtreeNode n1 = FieldQuadtreeNode::Init(node.x0 + d2, node.y0, d2);
				FieldQuadtreeNode n2 = FieldQuadtreeNode::Init(node.x0 + d2, node.y0 + d2, d2);
				FieldQuadtreeNode n3 = FieldQuadtreeNode::Init(node.x0, node.y0 + d2, d2);

				// Apply sampling of current field and old field. 
				n0.initFromPrevious(oldNode);
				n0.applyFieldToExisting(field);
				n1.initFromPrevious(oldNode);
				n1.applyFieldToExisting(field);
				n2.initFromPrevious(oldNode);
				n2.applyFieldToExisting(field);
				n3.initFromPrevious(oldNode);
				n3.applyFieldToExisting(field);
				n0.depth = node.depth + 1;
				n1.depth = node.depth + 1;
				n2.depth = node.depth + 1;
				n3.depth = node.depth + 1;

				node.childs = newIdx;
				tree.nodes.push_back(n0);
				tree.nodes.push_back(n1);
				tree.nodes.push_back(n2);
				tree.nodes.push_back(n3);
				notProcessed.push(newIdx);
				notProcessed.push(newIdx + 1);
				notProcessed.push(newIdx + 2);
				notProcessed.push(newIdx + 3);
			}
		}
	}

	void FieldQuadtreeBuilder::addNew(std::function<float(float, float)> field) {
		{
			FieldQuadtreeNode node = FieldQuadtreeNode::Init(x, y, d);
			node.applyField(field);
			tree.nodes.push_back(node);
		}

		std::stack<size_t> notProcessed;
		notProcessed.push(0);

		while (!notProcessed.empty()) {
			size_t ni = notProcessed.top();
			notProcessed.pop();

			FieldQuadtreeNode& node = tree.nodes[ni];
			float real[5];
			auto samples = node.samplepoints();
			samples.sampleField(real, field);

			bool subdivide;
			float diff = samples.maxSampleDifference(real);
			subdivide = (diff > threshold) && node.depth < maxnodedepth;

			if (subdivide) {
				// subdivided
				float d2 = node.d / 2;
				FieldQuadtreeNode n0 = FieldQuadtreeNode::Init(node.x0, node.y0, d2);
				FieldQuadtreeNode n1 = FieldQuadtreeNode::Init(node.x0 + d2, node.y0, d2);
				FieldQuadtreeNode n2 = FieldQuadtreeNode::Init(node.x0 + d2, node.y0 + d2, d2);
				FieldQuadtreeNode n3 = FieldQuadtreeNode::Init(node.x0, node.y0 + d2, d2);

				n0.applyField(field);
				n1.applyField(field);
				n2.applyField(field);
				n3.applyField(field);
				n0.depth = node.depth + 1;
				n1.depth = node.depth + 1;
				n2.depth = node.depth + 1;
				n3.depth = node.depth + 1;
				size_t newIdx = tree.nodes.size();
				node.childs = newIdx;
				tree.nodes.push_back(n0);
				tree.nodes.push_back(n1);
				tree.nodes.push_back(n2);
				tree.nodes.push_back(n3);
				notProcessed.push(newIdx);
				notProcessed.push(newIdx + 1);
				notProcessed.push(newIdx + 2);
				notProcessed.push(newIdx + 3);
			}
		}
	}

	float FieldQuadtreeNode::divergence(float measured[5]) const
	{
		/*

		The node points {c_i} and and the field samples {s_i} are used to estimate the divergence in the cell

			c3--s3--c2
			|        |
			s4  s0  s2
			|        |
			c0--s1--c1

			compute relative divergence from points x0+h, y+h; x+3/2h, y+h; x0+3/2h, y0+3/2h; x0+h,y0+3/2h
			The flux is computed in a diagonal coordinate system where the basis vectors go from corner to corner.

			*----*----*
			| p3 | p2 |
			*----*----*
			| p0 | p1 |
			*----*----*

			so the coordinate axes x',y' are (looking at point p0)

			s4   s0
			  \ /
			   X
			  / \
			c0   s1

			x': along from c0 to s0; y' along from s1 to s4

			the line normals through which the flux is calculated are in this coord system (-1,0),(0,-1),(1,0)(0,1)

			The difference is (F(x + h) - F(x-h))/2h
			Here H = 2h

			And the differences are the diagonal points as

				   s0        s0
				 /           |
			   /       H     h
			 /               |
			c0               s1

			H = sqrt(2) h
			h  = d/2 => H = d/(sqrt(2))

			Call the diagonal axes x',y'

			The x',y' difference for p0 Fp0 is
				Fp0x = (s0 - c0) / H
				Fp0y = (s4 - s1) / H

			The normal n14 in x',y' of edge s1->s4 is(-1,0)
			The contribution to divergence over edge s1-s4 is
			n14 <dot> Fp0 dH = (c0 - s0)/H * H = c0-s0

			Similarly for points p1,p2,p3 we find the total contribution to the divergence integral
			(c0-s0)+ (c1 - s0) + (c2 - s0) + (c3 - s0) =
			c0 + c1 + c2 + c3 - 4*s0

		*/

		float res = cornerdata[0].field + cornerdata[1].field + cornerdata[2].field + cornerdata[3].field - 4.f * measured[0];
		return res;
	}

	FieldQuadtree Transform(const FieldQuadtree& src, const util::LinearMap2D& map)
	{
		FieldQuadtree res = src;

		for (auto& node : res.nodes) {
			auto newxy = map.map({ node.x0,node.y0 });
			node.x0 = newxy.x; node.y0 = newxy.y;
			auto newd = map.s * node.d;
			node.d = newd;
			node.maxVal *= map.s;
			node.minVal *= map.s;
			for (auto& c : node.cornerdata) {
				c.field *= map.s;
			}
		}

		return res;
	}
}
