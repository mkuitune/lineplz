// This file lnpz.cpp is part of lineplz - Public domain simple drawing library

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

#include <lnpz/lnpz2d.h>

#include "lnpz_util.h"
#include "lnpz_fieldquadtree.h"
#include "clipper.hpp"

#include "nlohmann/json.hpp"

#include <vector>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <map>
#include <unordered_set>
#include <algorithm>

#define STB_IMAGE_RESIZE_IMPLEMENTATION
#include "stb/stb_image_resize.h"
#undef STB_IMAGE_RESIZE_IMPLEMENTATION

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"
#undef STB_IMAGE_WRITE_IMPLEMENTATION

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"
#undef STB_IMAGE_IMPLEMENTATION

using namespace std;

//
// Utilities
//
namespace lnpz {

	//
	// Image and rasterization
	//
	namespace internal_detail {

		struct TripletBool {
			bool fst, snd, thrd;

			TripletBool invert() const {
				return { !fst, !snd, !thrd };
			}

			bool all() const {
				return fst && snd && thrd;
			}

			bool allNot() const {
				return (!fst) && (!snd) && (!thrd);
			}
		};

		inline float clampf(float val, float minval, float maxval) {
			return val < minval ? minval : (val > maxval ? maxval : val);
		}

		inline double clampd(double val, double minval, double maxval) {
			return val < minval ? minval : (val > maxval ? maxval : val);
		}

		struct LineStringDistance {

			const vector<point2_t>& lines;

			inline float unsignedDistance(const point2_t& p) const
			{
				size_t N = lines.size();
				auto dist0 = p - lines[0];
				double d = dot(dist0, dist0);

				for (size_t i = 0, j = 1; j < N; j++, i++)
				{
					auto pi = lines[i];
					auto pj = lines[j];
					auto e = pj - pi;
					auto w = p - pi;
					auto b = w - e * clampd(dot(w, e) / dot(e, e), 0.0, 1.0);
					d = min(d, dot(b, b));
				}
				return (float)sqrt(d);
			}

			inline float signedDistance(const point2_t& p) const
			{
				size_t N = lines.size();
				auto dist0 = p - lines[0];
				double d = dot(dist0, dist0);

				double s = 1.0;

				for (size_t i = 0, j = 1; i < N; j++, i++)
				{
					auto pi = lines[i];
					auto pj = lines[j];
					auto e = pj - pi;
					auto w = p - pi;
					auto b = w - e * clampd(dot(w, e) / dot(e, e), 0.0, 1.0);
					d = min(d, dot(b, b));
					TripletBool c = { p.y >= pi.y, p.y < pj.y, e.x* w.y > e.y * w.x };
					if (c.all() || c.allNot())
						s *= -1.0;
				}
				return s * (float)sqrt(d);
			}

			float signedDistance(double x, double y) const {
				return signedDistance({ x,y });
			};
			float unsignedDistance(double x, double y) const {
				return unsignedDistance({ x,y });
			}

			function<float(float, float)> bindSigned() const {
				return[this](float a, float b) {
					return this->signedDistance(a, b);
				};
			}

			function<float(float, float)> bindUnsigned() const {
				return[this](float a, float b) {
					return this->unsignedDistance(a, b);
				};
			}
		};

		//------
		// TODO make sures this works
		//------

		struct PolyFaceDistance {

#if 0  // TODO instead of outer and inner wires at this stage have just wires
			const vector<point2_t>& outerWire;
			const vector<vector<point2_t>>& innerWires;
#endif

			// Remember to add the last-to-start edge here
			//const vector<vector<point2_t>>& wires;
			const vector<linestring_t>& wires;

			// for computing signed distance over several wires
			struct calculatesigned_t {
				bool inside = false;

				float distance(const point2_t& p, const vector<linestring_t>& wires) {

					auto dist = distance(p, wires[0].points);
					for (size_t i = 1; i < wires.size(); i++) {
						auto idist = distance(p, wires[i].points);
						if (fabsf(idist) < fabsf(dist))
							dist = idist;
					}
					return dist;
				}

				inline float distance(const point2_t& p, const vector<point2_t>& lines)
				{
					size_t N = lines.size();
					double d = numeric_limits<double>::max();
					int rcross = 0;
					for (size_t i = N - 1, j = 0; j < N; i = j, j++) {
						auto pi = lines[i];
						auto pj = lines[j];
						auto e = pj - pi;
						auto w = p - pi;
						auto b = w - e * clampd(dot(w, e) / dot(e, e), 0.0, 1.0);
						auto bb = dot(b, b);
						bool isleft = e.x * w.y > e.y * w.x; // is w cross e down (point outside) or up (point inside)
						d = min(d, dot(b, b));

						if (p.y < pi.y) {
							if (pj.y <= p.y) {
								if ((p.y - pj.y) * (pi.x - pj.x) > (p.x - pj.x) * (pi.y - pj.y))
									inside = !inside;
							}
						}
						else if (p.y < pj.y) {
							if ((p.y - pj.y) * (pi.x - pj.x) < (p.x - pj.x) * (pi.y - pj.y))
								inside = !inside;
						}

						return d;
					}
				}
			};

			inline float unsignedDistance(const point2_t& p) const {
				auto dist = unsignedDistance(p, wires[0].points);
				for (size_t i = 1; i < wires.size(); i++) {
					auto idist = unsignedDistance(p, wires[i].points);
					dist = min(dist, idist);
				}
				return dist;
			}

			// Polygon face source data does NOT contain the last edge - that is implicit handled
			// in rendering - and preprocessing needs to clean it so it's correct
			inline float unsignedDistance(const point2_t& p, const vector<point2_t>& lines) const
			{
				size_t N = lines.size();
				auto dist0 = p - lines[0];
				double d = dot(dist0, dist0);

				for (size_t i = N - 1, j = 0; j < N; i = j, j++) {
					auto pi = lines[i];
					auto pj = lines[j];
					auto e = pj - pi;
					auto w = p - pi;
					auto b = w - e * clampd(dot(w, e) / dot(e, e), 0.0, 1.0);
					d = min(d, dot(b, b));
				}
				return sqrtf(d);
			}

			inline float signedDistance(const point2_t& p)const {
				double s = 1.0;
				auto dist = signedDistanceForceWinding(p, wires[0].points, s);
				for (size_t i = 1; i < wires.size(); i++) {
					auto idist = signedDistanceForceWinding(p, wires[i].points, s);
					if (idist < dist)
						dist = idist;
				}
				return s * dist;
			}
#if 0
			inline float signedDistance(const point2_t& p) const {
				auto dist = signedDistance(p, wires[0].points);
				for (size_t i = 1; i < wires.size(); i++) {
					auto idist = signedDistance(p, wires[i].points);
					if (fabsf(idist) < fabsf(dist))
						dist = idist;
				}
				return dist;

			}

			// Polygon face source data does NOT contain the last edge - that is implicit handled
			// in rendering - and preprocessing needs to clean it so it's correct
			inline float signedDistance(const point2_t& p, const vector<point2_t>& lines) const
			{
				size_t N = lines.size();
				//auto dist0 = p - lines[0];
				//double d = dot(dist0,dist0);
				double d = numeric_limits<double>::max();
				int rcross = 0;
				// Compute the winding number while iterating over edges to find the shortest distance
				bool inside = false;
				for (size_t i = N - 1, j = 0; j < N; i = j, j++) {
					auto pi = lines[i];
					auto pj = lines[j];
					auto e = pj - pi;
					auto w = p - pi;
					auto b = w - e * clampd(dot(w, e) / dot(e, e), 0.0, 1.0);
					auto bb = dot(b, b);
					bool isleft = e.x * w.y > e.y * w.x; // is w cross e down (point outside) or up (point inside)
					d = min(d, dot(b, b));

					// compute edge crossing with infinite ray along x from p
						//U0 = pj
						//U1 = pi
					if (p.y < pi.y) {
						// pi above ray
						if (pj.y <= p.y) {
							// pj on or below ray
							if ((p.y - pj.y) * (pi.x - pj.x) > (p.x - pj.x) * (pi.y - pj.y))
								inside = !inside;
						}
					}
					else if (p.y < pj.y) {
						// pi on or below ray, pj above ray
						if ((p.y - pj.y) * (pi.x - pj.x) < (p.x - pj.x) * (pi.y - pj.y))
							inside = !inside;
					}


					//TripletBool c = { p.y >= pi.y, p.y < pj.y, e.x* w.y > e.y * w.x };
					//if (c.all() || c.allNot())
					//	s *= -1.0;

				}

				double s = inside ? -1.0 : 1.0;
				return s * sqrtf(d);
			}
#endif
			// apply sign only after returning
			inline float signedDistanceForceWinding(const point2_t& p, const vector<point2_t>& lines, double& s) const
			{
				size_t N = lines.size();
				auto dist0 = p - lines[0];
				double d = dot(dist0, dist0);

				//double s = 1.0;

				for (size_t i = N - 1, j = 0; j < N; i = j, j++) {
					auto pi = lines[i];
					auto pj = lines[j];
					auto e = pj - pi;
					auto w = p - pi;
					auto b = w - e * clampd(dot(w, e) / dot(e, e), 0.0, 1.0);
					d = min(d, dot(b, b));
					TripletBool c = { p.y >= pi.y, p.y < pj.y, e.x* w.y > e.y * w.x };

					if (c.all() || c.allNot())
						s *= -1.0;

				}
				return sqrtf(d);
			}

#if 0
			inline float signedDistanceForceWinding(const point2_t& p, const vector<point2_t>& lines, double& s) const
			{
				size_t N = lines.size();
				auto dist0 = p - lines[0];
				double d = dot(dist0, dist0);

				//double s = 1.0;

				for (size_t i = N - 1, j = 0; j < N; i = j, j++) {
					auto pi = lines[i];
					auto pj = lines[j];
					auto e = pj - pi;
					auto w = p - pi;
					auto b = w - e * clampd(dot(w, e) / dot(e, e), 0.0, 1.0);
					d = min(d, dot(b, b));
					TripletBool c = { p.y >= pi.y, p.y < pj.y, e.x* w.y > e.y * w.x };

					if (c.all() || c.allNot())
						s *= -1.0;

				}
				return s * sqrtf(d);
			}
#endif

			float signedDistance(double x, double y) const {
				return signedDistance({ x,y });
			};
			float unsignedDistance(double x, double y) const {
				return unsignedDistance({ x,y });
			}

			function<float(float, float)> bindSigned() const {
				return[this](float a, float b) {
					return this->signedDistance(a, b);
				};
			}

			function<float(float, float)> bindUnsigned() const {
				return[this](float a, float b) {
					return this->unsignedDistance(a, b);
				};
			}
		};

		// Callback for stb_image_write
		void WriteBytes(void* context, void* data, int size) {
			vector<uint8_t>* bytes = (vector<uint8_t>*)(context);
			uint8_t* byteData = (uint8_t*)data;
			*bytes = vector<uint8_t>(byteData, byteData + size);
		}

		template<class T>
		rectangle_t GetImageArrayBounds(const Array2D<T>& arr) {
			return rectangle_t::InitializeFromPair({ 0.0,0.0 }, { (double)arr.dim1(), (double)arr.dim2() });
		}

		struct PixelCover {
			size_t x0, y0, xmax, ymax;
			struct enumerator_t {
				size_t x0, y0, xmax, ymax;
				size_t x, y;
				float spatialx, spatialy;

				bool next() {
					x++;
					if (x == xmax) {
						y++;
						x = 0;
						if (y == ymax)
							return false;
					}
					spatialx = ((float)x) + 0.5f;
					spatialy = ((float)y) + 0.5f;
					return true;
				}

				static enumerator_t Create(size_t x0, size_t y0, size_t xmax, size_t ymax) {
					size_t xinit = numeric_limits<size_t>::max();
					return { x0, y0, xmax, ymax, xinit,0 };
				}
			};

			enumerator_t enumerate() const { return enumerator_t::Create(x0, y0, xmax, ymax); }

			static bool Get(const rectangle_t& rect, PixelCover& res) {
				double xmaxd = rect.max().x;
				double ymaxd = rect.max().y;
				double xmind = max(0.0, rect.min().x);
				double ymind = max(0.0, rect.min().y);
				size_t x0 = xmind;
				size_t y0 = ymind;
				size_t xmax = xmaxd;
				size_t ymax = ymaxd;
				if (xmax == x0 || ymax == y0)
					return false;
				res = { x0, y0, xmax, ymax };
				return true;
			}
		};

		struct Material2D {
			float linewidth = 1.0; // sceneunits
			RGBAFloat32 colorFill;
			RGBAFloat32 colorLine;

			static Material2D CreateDefault() { return { 1.0f, RGBAFloat32::White(), RGBAFloat32::Black() }; }
		};

		inline float smoothstepClampRangeUnityf(float x, float rangeStart, float rangeEnd) {
			float delta = rangeEnd - rangeStart;
			float u = (x - rangeStart) / delta;
			if (u < 0.f) return 0.f;
			if (u >= 1.f) return 1.f;
			return u * u * (3.f - 2.f * u);
		}

		// smoothstep between 0 and 1 - map values outside interval to either 0 or 1
		inline float smoothstepClampf(float x) {
			if (x > 0.f) {
				if (x < 1.f)
					return (x * x * (3.f - 2.f * x));
				else
					return 1.f;
			}
			else return 0.f;
		}

		struct ColorizeMaterial2D {
			Material2D src;
			RGBAFloat32 lineColorPremul;
			RGBAFloat32 fillColorPremul;
			float f;
			float w;
			float wf0;
			float wf1;
			float f2;

			// return false if totally transparent - has no effect
			bool lineColorFromDistance(float d, RGBAFloat32& out) {
				d = fabsf(d);
				// todo remove this and can colorize areas?
				if (d < 0.f)
					return false;

				if (d > wf1)
					return false;

				float x = wf1 - d;
				float u = smoothstepClampRangeUnityf(x, 0.f, f2);
				out = lineColorPremul.premultiplyAlpha(u);
				return true;
			}

			// lerp anti-alias - fill areas - antialias f beyond and f inside the border
			bool fillColorFromDistance(float d, RGBAFloat32& out) {
				if (d > f)
					return false;

				float v = f - d;
				if (v <= 0.f) {
					return true;
					out = fillColorPremul;
				}
				float x = v / f2;
				float u = smoothstepClampf(x);
				out = fillColorPremul.premultiplyAlpha(u);
				return true;
			}

			static ColorizeMaterial2D Create(
				const material_t& materialIn) {

				auto lineColor = materialIn.lineColor;
				auto faceColor = materialIn.faceColor;

				ColorizeMaterial2D c;
				c.src.colorFill = faceColor;
				c.src.colorLine = lineColor;
				c.src.linewidth = materialIn.lineWidth;
				c.lineColorPremul = lineColor.toPremultiplied();
				c.fillColorPremul = faceColor.toPremultiplied();
				float lineWidthInRaster = c.src.linewidth;
				c.w = lineWidthInRaster * 0.5f;
				static const float f = 0.5f; // antialias distance/2
				//static const float f = 1.0f; // antialias distance/2
				c.f = f;
				c.wf0 = c.w - c.f;
				c.wf1 = c.w + c.f;
				c.f2 = 2.0f * c.f;

				return c;
			}
		};

		RGBAFloat32 BlendPreMultipliedAlpha(const RGBAFloat32 source, const RGBAFloat32 target)
		{
			const float fb = 1.f - source.a;
			return{ source.r + target.r * fb,
					source.g + target.g * fb,
					source.b + target.b * fb,
					source.a + target.a * fb };
		}

		inline void BlendPixelPremuli(size_t x, size_t y, const RGBAFloat32& color, ImageRGBA32Linear& img)
		{
			if (x < 0.0f || y < 0.0f)
				return;

			size_t height = img.dim2();
			if (x >= img.dim1() || y >= img.dim2())
				return;

			auto dst = img.at(x, (height - y - 1));
			auto out = BlendPreMultipliedAlpha(color, dst);
			img.set(x, (height - y - 1), out);
		}

		void RasterizeLineString(const matrix33_t sceneToPixel, const Scene::instance_t& inst, const linestring_t& lines, ImageRGBA32Linear& framebuffer) {
			using namespace lnpz_linalg;
			// get scenespace geometry
			// convert to distance field in pixelspace
			// colorize distance field
			auto localToPixel = mul(sceneToPixel, inst.localToWorld.matrix());
			linestring_t linesInPixels = lines;
			for (auto& p : linesInPixels.points) {
				p = apply_to_point2(localToPixel, p);
			}

			auto frameBufferBounds = GetImageArrayBounds(framebuffer);

			// Pixel bounds
			auto linePixelBounds = linesInPixels.boundingRectangle();
			linePixelBounds = linePixelBounds.expandSymmetric(inst.material.lineWidth * 1.05);

			rectangle_t rasterizationBounds;
			if (!linePixelBounds.intersect(frameBufferBounds, rasterizationBounds)) {
				return;
			}

			auto rasterDiagonal = rasterizationBounds.diagonal();
			auto areaInPixels = rasterDiagonal.x * rasterDiagonal.y;
			if (areaInPixels < 3.0 /* Some small enough cutoff */) {
				return;
			}

			// Build adaptive signed distance tree
			double maxSize = largestElement(rasterDiagonal);
			float originx = (float)rasterizationBounds.min().x;
			float originy = (float)rasterizationBounds.min().y;
			FieldQuadtreeBuilder fieldBuilder(originx, originy, maxSize);
			// Build distance field
			LineStringDistance dist{ linesInPixels.points };
			auto fun = dist.bindUnsigned();
			fieldBuilder.add(fun);
			auto field = fieldBuilder.build();
			PixelCover pixelCoords;
			if (!PixelCover::Get(rasterizationBounds, pixelCoords)) {
				return;
			}
			auto pixelEnum = pixelCoords.enumerate();

			ColorizeMaterial2D material = ColorizeMaterial2D::Create(inst.material);
			RGBAFloat32 colorTmp;
			while (pixelEnum.next()) {
				float x = pixelEnum.spatialx;
				float y = pixelEnum.spatialy;
				float dist = field.getDeepSample(x, y);
				//if (dist < 20) {
				bool valid = material.lineColorFromDistance(dist, colorTmp);
				if (valid) {
					if (colorTmp.a > 0.001f) {
						BlendPixelPremuli((unsigned)pixelEnum.x, (unsigned)pixelEnum.y, colorTmp, framebuffer);
					}
				}
				//}
			}
		}

		void RasterizePolygonFace(const matrix33_t sceneToPixel,
			const Scene::instance_t& inst, const renderablepolygonface_t& face, ImageRGBA32Linear& framebuffer) {

			// convert to distance field in scenespace
			// colorize distance field
			auto localToPixel = mul(sceneToPixel, inst.localToWorld.matrix());

			renderablepolygonface_t faceInPixelSpace = face;

			for (auto& wire : faceInPixelSpace.wires) {
				for (auto& p : wire.points) {
					p = apply_to_point2(localToPixel, p);
				}
			}
			auto frameBufferBounds = GetImageArrayBounds(framebuffer);

			// Pixel bounds
			auto linePixelBounds = faceInPixelSpace.boundingRectangle();
			linePixelBounds = linePixelBounds.expandSymmetric(inst.material.lineWidth * 1.05);

			rectangle_t rasterizationBounds;
			if (!linePixelBounds.intersect(frameBufferBounds, rasterizationBounds)) {
				return;
			}

			auto rasterDiagonal = rasterizationBounds.diagonal();
			auto areaInPixels = rasterDiagonal.x * rasterDiagonal.y;
			if (areaInPixels < 3.0 /* Some small enough cutoff */) {
				return;
			}

			// Build adaptive signed distance tree
			double maxSize = largestElement(rasterDiagonal);
			float originx = (float)rasterizationBounds.min().x;
			float originy = (float)rasterizationBounds.min().y;
			FieldQuadtreeBuilder fieldBuilder(originx, originy, maxSize);
			// Build distance field

			//LineStringDistance dist{ linesInPixels.points };
			PolyFaceDistance dist{ faceInPixelSpace.wires };

			auto fun = dist.bindSigned();
			fieldBuilder.add(fun);
			auto field = fieldBuilder.build();
			PixelCover pixelCoords;
			if (!PixelCover::Get(rasterizationBounds, pixelCoords)) {
				return;
			}
			auto pixelEnum = pixelCoords.enumerate();

			ColorizeMaterial2D material = ColorizeMaterial2D::Create(inst.material);
			RGBAFloat32 colorTmp;
			RGBAFloat32 colorTmpArea;
			while (pixelEnum.next()) {
				float x = pixelEnum.spatialx;
				float y = pixelEnum.spatialy;
				float dist = field.getDeepSample(x, y);
				//if (dist < 20) {

				if (inst.material.lineColor.a == 0.0) { // No outline
					if (material.fillColorFromDistance(dist, colorTmpArea)) {
						//m_painter.BlendPixelPremuli((unsigned)x, (unsigned)y, colorTmpArea); // todo is premultiplied what we want
						BlendPixelPremuli((unsigned)pixelEnum.x, (unsigned)pixelEnum.y, colorTmpArea, framebuffer);
					}
				}
				else { // Blend outline and polygon against eachother
					if (material.fillColorFromDistance(dist, colorTmpArea)) {

						if (material.lineColorFromDistance(dist, colorTmp)) {
							colorTmpArea = BlendPreMultipliedAlpha(colorTmp, colorTmpArea);
						}

						//m_painter.BlendPixelPremuli((unsigned)x, (unsigned)y, colorTmpArea); // todo is premultiplied what we want
						BlendPixelPremuli((unsigned)pixelEnum.x, (unsigned)pixelEnum.y, colorTmpArea, framebuffer);
					}
					else {
						if (material.lineColorFromDistance(dist, colorTmp)) {
							BlendPixelPremuli((unsigned)pixelEnum.x, (unsigned)pixelEnum.y, colorTmp, framebuffer);
						}
					}
				}


				//Line
				//if (colorTmp.a > 0.001f) {
				//	BlendPixelPremuli((unsigned)pixelEnum.x, (unsigned)pixelEnum.y, colorTmp, framebuffer);
				//}

			//}
			}

		}

		void RasterizeInstance(const Scene& scene, const matrix33_t sceneToPixel, const Scene::instance_t& inst, ImageRGBA32Linear& framebuffer) {
			if (inst.type == InstanceType::LineString) {
				RasterizeLineString(sceneToPixel, inst, scene.m_lines[inst.idx], framebuffer);
			}
			else if (inst.type == InstanceType::PolygonFace) {
				RasterizePolygonFace(sceneToPixel, inst, scene.m_renderablePolygons[inst.idx], framebuffer);
			}
		}


		//-------------------------------------------
		// Geometry preprocessing
		//-------------------------------------------

		struct edge_t {
			uint32_t fst;
			uint32_t snd;

			uint64_t as64() const {
				uint64_t u = *((uint64_t*)this);
				return u;
			}

			bool isUndirectedSame(const edge_t& e) const {
				return (fst == e.fst && snd == e.snd) || (fst == e.snd && snd == e.fst);
			}

			static edge_t From64(uint64_t u) {
				edge_t w = *((edge_t*)(&u));
				return w;
			}
		};

		struct indexedwire_t {
			vector<uint32_t> indices;

			vector<edge_t> toEdges() const {
				vector<edge_t> edges;
				uint32_t sz = indices.size();
				for (uint32_t i = 0; i < sz; i++) {
					uint32_t j = (i + 1) % sz;
					edges.push_back({ indices[i], indices[j] });
				}
				return edges;
			}
		};


		struct indexedpolyface_t {
			vector<indexedwire_t>  outerWires;
			vector<indexedwire_t>  innerWires;
			vector<point2_t> vertices;
		};

		void PolygonFaceToIndexed(
			const polygonface_t& polyface,
			indexedpolyface_t& result
		) {

			{
				indexedwire_t indexedOuter;
				for (const auto& owv : polyface.outerWire.points) {
					result.vertices.push_back(owv);
					indexedOuter.indices.push_back(result.vertices.size() - 1);
				}
				result.outerWires.push_back(indexedOuter);
			}

			for (const auto& innerWire : polyface.innerWires) {
				indexedwire_t indexedInner;
				for (const auto& iwv : innerWire.points) {
					result.vertices.push_back(iwv);
					indexedInner.indices.push_back(result.vertices.size() - 1);
				}
				result.innerWires.push_back(indexedInner);
			}
		}

		typedef lnpz_linalg::clusteringradius<double> clusteringradius_t;

		struct clusteredvertices_t {

			struct clustervertex_t {
				point2_t pnt;
				uint32_t sourceIndex; // index in source data - as we sort vertices this will change

				bool operator<(const clustervertex_t& r) const noexcept {
					if (pnt.x < r.pnt.x) return true;
					if (pnt.x > r.pnt.x) return false;
					// x == r.x
					if (pnt.y < r.pnt.y) return true;
					// for cases pnt.y >= r.pnt.y
					return false;
				}
			};

			vector<clustervertex_t> clusteredInput;
			vector<point2_t> resultVertices;
			unordered_map<uint32_t, uint32_t> sourceToResult; // map from source index to eposition

			void insert(point2_t pnt, uint32_t sourceIndex) {
				clusteredInput.push_back({ pnt, sourceIndex });
			}

		private:

			void sortVertices() {
				sort(clusteredInput.begin(), clusteredInput.end());
			}

			void sortedToResult(clusteringradius_t radius) {

				resultVertices.push_back(clusteredInput[0].pnt);
				vector<int> clusterIdx(clusteredInput.size(), -1);
				clusterIdx[0] = 0;

				sourceToResult[clusteredInput[0].sourceIndex] = resultVertices.size() - 1;

				for (uint32_t i = 1; i < clusteredInput.size(); i++) {
					if (clusterIdx[i] >= 0) { // if already clustered in some group the index is already set
						continue;
					}

					// This is not clusterd yet. Write this to buffer, and find clusterable relations ahead
					auto pi = clusteredInput[i].pnt;
					resultVertices.push_back(pi);
					int curidx = resultVertices.size() - 1;
					sourceToResult[clusteredInput[i].sourceIndex] = curidx;
					clusterIdx[i] = curidx;


					// try find clusterable vertices ahead
					for(uint32_t j = i + 1; j < clusteredInput.size(); j++){
						auto pj = clusteredInput[j].pnt;
						if (!radius.withinRangeSingle(pi.x, pj.x)) { // sorted along x
							break;
						}
						if (radius.withinRange(pi, pj)) {
							clusterIdx[j] = curidx;
							sourceToResult[clusteredInput[j].sourceIndex] = curidx;
						}
					}
				}
			}

		public:
			void doClustering(clusteringradius_t radius) {
				sortVertices();
				sortedToResult(radius);
			}

			indexedwire_t remapWire(const indexedwire_t& wireIn) {
				indexedwire_t res;
				for (auto v : wireIn.indices)
					res.indices.push_back(sourceToResult[v]);

				return res;
			}
		};

		struct clusteredvertexbuilder_t {
			clusteredvertices_t clustered;
			std::string err;

			indexedpolyface_t build(const indexedpolyface_t& indexed, clusteringradius_t clusteringRadius) {
				for (size_t i = 0; i < indexed.vertices.size(); i++) {
					clustered.insert(indexed.vertices[i], i);
				}

				clustered.doClustering(clusteringRadius);

				// map source vertices to target vertices
				indexedpolyface_t res;
				res.vertices = clustered.resultVertices;

				for (const auto& ow : indexed.outerWires) {
					auto remappedOuter = clustered.remapWire(ow);
					res.outerWires.push_back(remappedOuter);
				}
				for (const auto& iw : indexed.innerWires) {
					auto remappedInner = clustered.remapWire(iw);
					res.innerWires.push_back(remappedInner);
				}

				return res;
			}
		};

		size_t NumberOfUniqueVertices(const vector<edge_t>& input) {
			unordered_set<uint32_t> usedVertices;
			for (auto& e : input) {
				usedVertices.insert(e.fst);
				usedVertices.insert(e.snd);
			}
			return usedVertices.size();
		}

		struct edgefaces_t {
			vector<vector<edge_t>> outerWires;
			vector<vector<edge_t>> innerWires;

			vector<point2_t> vertices;

			rectangle_t getBounds() const noexcept {
				rectangle_t res = rectangle_t::InitializeEmpty();
				
				for (const auto& ow : outerWires) {
					for (const auto& p : ow) {
						res.coverInPlace(vertices[p.fst]);
					}
				}
				
				for (const auto& iw : innerWires) {
					for (const auto& p : iw) {
						res.coverInPlace(vertices[p.fst]);
					}
				}

				return res;
			}

			static edgefaces_t Create(indexedpolyface_t& pf) {
				edgefaces_t  ef;
				ef.vertices = pf.vertices;
				for (const auto& outerwire : pf.outerWires)
					ef.outerWires.push_back(outerwire.toEdges());

				for (const auto& innerwire : pf.innerWires)
					ef.innerWires.push_back(innerwire.toEdges());

				return ef;
			}
		};

		/* Remove 1 or 2 vertex shapes*/
		edgefaces_t RemoveSimpleDegeneracies(const edgefaces_t& input, bool& resultNotDegenrate) {
			edgefaces_t res;

			vector<bool> vertexUsed(input.vertices.size(), false);

			map<uint32_t, uint32_t> oldToNewVertex;
			auto registerVertex = [&](uint32_t oldVertex) {
				if (oldToNewVertex.count(oldVertex) == 0) {
					res.vertices.push_back(input.vertices[oldVertex]);
					oldToNewVertex[oldVertex] = res.vertices.size() - 1;
				}
				return oldToNewVertex[oldVertex];
			};

			resultNotDegenrate = false;

			auto processWireset = [&](const vector<vector<edge_t>>& input, vector<vector<edge_t>>& output) {
				for (const auto& iw : input) {
					if (iw.size() < 3) {
						continue;
					}
					size_t nUniqueVerts = NumberOfUniqueVertices(iw);
					if (nUniqueVerts < 3) {
						continue;
					}
					vector<edge_t> res;
					for (const auto& inputEdge : iw) {
						auto fst = registerVertex(inputEdge.fst);
						auto snd = registerVertex(inputEdge.snd);
						res.push_back({ fst, snd });
					}

					output.push_back(res);
				}
			};

			processWireset(input.outerWires, res.outerWires);
			processWireset(input.innerWires, res.innerWires);

			resultNotDegenrate = res.outerWires.size() > 0;

			return res;
		}

		/* Clean up polygon wires using clipper */
		struct polygon2d_cleaner_t {

			ClipperLib::Paths outerWires;
			ClipperLib::Paths innerWires;

			double scale;
			point2_t offset;

			static constexpr double INTEGERDIM = 1e6;

			ClipperLib::IntPoint toClipper(const point2_t& pnt) const {
				auto p = (pnt - offset) * scale;
				int x = p.x; int y = p.y;
				return { x,y };
			}
			
			point2_t fromClipper(const ClipperLib::IntPoint& ipnt) const {
				point2_t pnt;
				pnt.x = ipnt.X;
				pnt.y= ipnt.Y;
				pnt = (pnt / scale) + offset;
				return pnt;
			}

			linestring_t linestringFromWire(const ClipperLib::Path& wire) const {
				linestring_t ls;
				for (const auto& p : wire) {
					ls.points.push_back(fromClipper(p));
				}
				return ls;
			}

			// Clean set of inner or outerwires 
			static ClipperLib::Paths CleanWires(const ClipperLib::Paths& wiresIn) {
				using namespace ClipperLib;
				Paths output;
				SimplifyPolygons(wiresIn, output, pftNonZero);
				return output;
			}

			void initFrom(const edgefaces_t& edgeFaces) {
				using namespace lnpz_linalg;
				rectangle_t rec = edgeFaces.getBounds();
				auto diag = rec.diagonal();
				double geomsize = largestElement(diag);
				scale = INTEGERDIM / geomsize;
				offset = rec.min();

				for (const auto& ow : edgeFaces.outerWires) {
					ClipperLib::Path cow;
					for (const auto& p : ow) {
						cow.push_back(toClipper(edgeFaces.vertices[p.fst]));
					}
					outerWires.push_back(cow);
				}
				
				for (const auto& iw : edgeFaces.innerWires) {
					ClipperLib::Path ciw;
					for (const auto& p : iw) {
						ciw.push_back(toClipper(edgeFaces.vertices[p.fst]));
					}
					innerWires.push_back(ciw);
				}

				auto cleanedOuter = CleanWires(outerWires);
				outerWires = cleanedOuter;
				auto cleanedInner = CleanWires(innerWires);
				innerWires = cleanedInner;
			}

			void cleanAndWrite(renderablepolygonface_t& res) {
				using namespace ClipperLib;
				Clipper c;
				c.AddPaths(outerWires, ptSubject, true);
				c.AddPaths(innerWires, ptClip, true);
				//c.Execute(ctIntersection, solution, pftNonZero, pftNonZero);
				Paths solution;
				c.Execute(ctDifference, solution, pftNonZero, pftNonZero);

				// get inner and outerwires from solution
				for (auto& solpath : solution) {
					linestring_t wireOut = linestringFromWire(solpath);
					res.wires.push_back(wireOut);
				}
			}
		};

		string CleanInClipper(edgefaces_t edgeFaces, renderablepolygonface_t& res) {
			// 0. scale to clipper
			// 1. merge outer wires
			// 2. merge inner wires
			// 3. clip outer wires with inner wires
			// 4. categorize result wires to outer and inner wires

			polygon2d_cleaner_t cleaner;
			cleaner.initFrom(edgeFaces);
			cleaner.cleanAndWrite(res);

			return "";
		}

		string CleanIndexedPolyfaceToRenderable(const indexedpolyface_t& source, double mergeRadius, edgefaces_t& res) {

			// cluster vertices explicitly here before assigning to clipper lib

			clusteringradius_t clusteringRadius = clusteringradius_t::Create(mergeRadius);
			clusteredvertexbuilder_t builder;
			indexedpolyface_t clustered = builder.build(source, clusteringRadius);
			if (!builder.err.empty())
				return builder.err;

			edgefaces_t  initialEdges = edgefaces_t::Create(clustered);

			bool cleanedNonDegenerate;
			res = RemoveSimpleDegeneracies(initialEdges, cleanedNonDegenerate);
			if (!cleanedNonDegenerate)
				return "Input was degenerate";

			// cleaned to clipper lib
			return "";
		}

		bool ProcessPolyface(const polygonface_t& polyface, renderablepolygonface_t& res) {

			auto bounds = polyface.boundingRectangle();
			auto maxsize = lnpz_linalg::largestElement(bounds.diagonal());
			double mergeRadius = 0.0001 * maxsize;

			indexedpolyface_t indexed;
			PolygonFaceToIndexed(polyface, indexed);

			edgefaces_t edgefaces;
			auto cleanres = CleanIndexedPolyfaceToRenderable(indexed, mergeRadius, edgefaces);
			if (!cleanres.empty())
				return false;
			
			auto processResult = CleanInClipper(edgefaces, res);
			if (!processResult.empty())
				return false;

			//-------------------------------------------
			// TODO: convert cleaned to renderable
			//-------------------------------------------

			//------------------------------------------------
			// Trivial conversion for debugging purposes
			//------------------------------------------------
			// COMMENT OUT

			//res.wires.push_back(polyface.outerWire);
			//for (const auto& inw : polyface.innerWires) {
			//	//res.wires.push_back(inw.reversed());
			//	res.wires.push_back(inw);
			//}


			return true;
		}
#if 0
		bool ProcessPolyface(const polygonface_t& polyface, renderablepolygonface_t& res) {

			//
			// Trivial conversion for debugging purposes
			//

			res.wires.push_back(polyface.outerWire);
			for (const auto& inw : polyface.innerWires) {
				//res.wires.push_back(inw.reversed());
				res.wires.push_back(inw);
			}

			return true;
		}
#endif

		} // end internal_detail

		//
		// SimpleBuilder
		//


		void SimpleBuilder::setMaterial(const material_t& mat) {
			m_currentMaterial = mat;
		}

		void SimpleBuilder::addLineString(const linestring_t& ls) {
			m_scene.m_lines.push_back(ls);
			auto idx = m_scene.m_lines.size() - 1;
			localToWorldTransform_t trf;
			m_scene.m_instances.push_back({ trf, m_currentMaterial, InstanceType::LineString, idx });
		}

		void SimpleBuilder::addPolygonFace(const polygonface_t& pf) {
			renderablepolygonface_t renderable;

			bool isValid = internal_detail::ProcessPolyface(pf, renderable);

			if (!isValid)
				return;

			m_scene.m_polygonFaces.push_back(pf);

			renderable.sourcePolygonIndex = m_scene.m_polygonFaces.size() - 1;
			m_scene.m_renderablePolygons.push_back(renderable);
			auto idx = m_scene.m_renderablePolygons.size() - 1;

			localToWorldTransform_t trf;
			m_scene.m_instances.push_back({ trf, m_currentMaterial, InstanceType::PolygonFace, idx });
		}

		Scene SimpleBuilder::build() const {
			return m_scene;
		}

		//Scene SimpleBuilder::FromJson(const std::string& str)
		//{
		//}

		//
		// Renderer
		//
		typedef Array2D<RGBAFloat32> ImageRGBA32Linear;

		void Renderer2S::draw(const Scene& scene) {

			// Figure out framebuffer size and scene to framebuffer transform
			const auto wb = scene.getWorldBounds();
			const auto diag = wb.diagonal();
			const auto sceneWidth = diag.x;
			const auto sceneHeight = diag.y;

			const int32_t	outputPixelHeight = m_sceneConfig.outputHeightPixels;
			const int32_t	outputScenePixelHeight = outputPixelHeight - 2 * m_sceneConfig.paddingInPixels;
			const auto		sceneToPixelScale = outputScenePixelHeight / sceneHeight;
			const int32_t	outputPixelWidth = (int32_t)(sceneToPixelScale * sceneWidth) + (int32_t)(2 * m_sceneConfig.paddingInPixels);
			const point2_t	sceneOrigInPixel(m_sceneConfig.paddingInPixels);
			const auto		wbOriginPixelUnits = wb.min() * sceneToPixelScale;
			const auto		offset = sceneOrigInPixel - wbOriginPixelUnits;

			//matrix33_t sceneToPixel = { {sceneToPixelScale, 0, offset.x},
			//							{0,sceneToPixelScale,  offset.y},
			//							{0, 0, 1} };
			matrix33_t sceneToPixel = { {sceneToPixelScale, 0, offset.x},
										{0,sceneToPixelScale,  offset.y},
										{0, 0, 1} };

			sceneToPixel = lnpz_linalg::transpose(sceneToPixel); // row to column major

			// Rasterize in 4-byte floating point precision

			ImageRGBA32Linear framebuffer = ImageRGBA32Linear(outputPixelWidth, outputPixelHeight);
			framebuffer.fill(m_sceneConfig.background);

			// Rasterize scene instances in order
			for (const auto& inst : scene.m_instances) {
				internal_detail::RasterizeInstance(scene, sceneToPixel, inst, framebuffer);
			}

			// Convert to 1 byte per channel SRGBA
			m_framebuffer = ConvertRBGA32LinearToSrgba(framebuffer);
		}

		string Renderer2S::write(const string& pathOut) const {
			vector<uint8_t> byteArray;
			int width = (int)m_framebuffer.dim1();
			int height = (int)m_framebuffer.dim2();
			int rowlength = 4 * width;
			stbi_write_png_to_func(internal_detail::WriteBytes, (void*)&byteArray, width, height, 4, m_framebuffer.data(), rowlength);
			return util::WriteBytesToPath(byteArray, pathOut);
		}

		string Renderer2S::getSRGBABytes() const {
			string res;
			size_t size = m_framebuffer.sizeInBytes();
			res.resize(size);
			const char* src = (const char*)m_framebuffer.data();
			for (size_t i = 0; i < size; i++)
				res[i] = src[i];

			return res;
		}

		//
		// Color
		//
		namespace internal_detail {
			uint8_t LinearFloatToSRGBUint8(const float f) {
				return stbir__linear_to_srgb_uchar(f);
			}


			SRGBA ToSRGBA(const RGBAFloat32& fcolor) {
				uint8_t a = (uint8_t)(255.f * clampf(fcolor.a, 0.f, 1.f));// TODO FIXME
				return{ LinearFloatToSRGBUint8(fcolor.r), LinearFloatToSRGBUint8(fcolor.g),
					LinearFloatToSRGBUint8(fcolor.b), a };
			}

			template<class T>
			class Sequence1D {
				T m_begin;
				T m_end;
			public:
				struct const_iterator {
					T val;
					void operator++() { val += 1; }
					bool operator!=(const const_iterator& rhs) { return val != rhs.val; }
					const T& operator*() { return val; }
				};

				Sequence1D(T fst, T snd) : m_begin(fst), m_end(snd) {}
				Sequence1D(T snd) : m_begin(0), m_end(snd) {}
				const_iterator begin() const { return { m_begin }; }
				const_iterator end() const { return { m_end }; }
			};

			template<class T>
			inline Sequence1D<T> make_seq(T endSize) { return Sequence1D<T>(endSize); }

			typedef Sequence1D<size_t> Sequence1Ds;
			typedef Sequence1D<int> Sequence1Di;
		}

		ImageRGBA8SRGB ConvertRBGA32LinearToSrgba(const ImageRGBA32Linear& linear)
		{
			using namespace internal_detail;
			ImageRGBA8SRGB res(linear.size());
			auto seq = make_seq(linear.elementCount());
			for (auto i : seq) {
				res.at(i) = ToSRGBA(linear.at(i));
			}
			return res;
		}
}

