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

#include <lnpz/lnpz.h>

#include "lnpz_util.h"
#include "lnpz_fieldquadtree.h"

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

			bool containedIn(vector<edge_t*>& edges, size_t& containedIdx) const {
				for (size_t i = 0; i < edges.size(); i++) {
					auto e = edges[i];
					if (isUndirectedSame(*e)) {
						containedIdx = i;
						return true;
					}
				}
				return false;
			}

			static edge_t From64(uint64_t u) {
				edge_t w = *((edge_t*)(&u));
				return w;
			}
		};
		
		struct typed_edge_t {
			uint32_t fst;
			uint32_t snd;

			bool outer;

			bool visited;
			point2_t direction(const vector<point2_t>& vertices) const {
				auto f = vertices[fst];
				auto s = vertices[snd];
				auto dir = s - f;
				dir = dir / lnpz_linalg::length(dir);
				return dir;
			}

			bool tryGetOpposite(uint32_t vi, uint32_t& res) const noexcept {
				if (vi == fst) {
					res = snd;
					return true;
				}
				if (vi == snd) {
					res = fst;
					return true;
				}
				return false;
			}

			bool isIncomingTo(uint32_t vert) const noexcept{
				return (snd == vert) && (fst != vert);
			}

			bool isOutgoingFrom(uint32_t vert) const noexcept{
				return (snd != vert) && (fst == vert);
			}

			bool isSame(const typed_edge_t& e) const noexcept {
				return fst == e.fst && snd == e.snd;
			}

			bool isUndirectedSame(const typed_edge_t& e) const {
				return (fst == e.fst && snd == e.snd) || (fst == e.snd && snd == e.fst);
			}

			bool containedIn(vector<typed_edge_t*>& edges, size_t& containedIdx) const {
				for (size_t i = 0; i < edges.size(); i++) {
					auto e = edges[i];
					if (isUndirectedSame(*e)) {
						containedIdx = i;
						return true;
					}
				}
				return false;
			}
		};

		struct indexedwire_t {
			vector<uint32_t> indices;

			vector<edge_t> toEdges() const {
				vector<edge_t> edges;
				uint32_t sz = indices.size();
				for (uint32_t i = 0; i < sz; i++) {
					uint32_t j = (i + 1) % sz;
					edges.push_back({ i, j });
				}
				return edges;
			}
		};


		struct indexedpolyface_t {
			vector<indexedwire_t>  outerWires;
			vector<indexedwire_t>  innerWires;
			vector<point2_t> vertices;
		};

		struct edgefaces_t {

			//vector<vector<edge_t>> edgesAtVertex;
			//unordered_set<uint64_t> edges;

			vector<vector<edge_t>> outerWires;
			vector<vector<edge_t>> innerWires;

			vector<point2_t> vertices;

			static edgefaces_t Create(indexedpolyface_t& pf) {
				edgefaces_t  ef;
				ef.vertices = pf.vertices;
				for (const auto& outerwire : pf.outerWires)
					ef.outerWires.push_back(outerwire.toEdges());

				for (const auto& innerwire : pf.innerWires)
					ef.innerWires.push_back(innerwire.toEdges());

				//auto edgeEnum = ef.getEdgeEnumerator();
				//while (edgeEnum.next()) {
				//	ef.edges.insert(edgeEnum.edge.as64());
				//}

				return ef;
			}

			struct edgeenumerator_t {
				const edgefaces_t& edges;
				uint32_t outerWireIdx = 0;
				uint32_t edgeIdx = 0;
				uint32_t innerWireIdx = 0;
				edge_t edge;

				bool next() {
					if (outerWireIdx < edges.outerWires.size()) {
						if (edgeIdx < edges.outerWires[outerWireIdx].size()) {
							edge = edges.outerWires[outerWireIdx][edgeIdx];
						}
						else {
							edgeIdx = 0;
							outerWireIdx++;
							if (outerWireIdx < edges.outerWires.size()) {
								edge = edges.outerWires[outerWireIdx][edgeIdx];
							}
						}
					}

					if (outerWireIdx >= edges.outerWires.size() && innerWireIdx < edges.innerWires.size()) {
						if (edgeIdx < edges.innerWires[innerWireIdx].size()) {
							edge = edges.innerWires[innerWireIdx][edgeIdx];
						}
						else {
							edgeIdx = 0;
							innerWireIdx++;
							if (innerWireIdx < edges.innerWires.size()) {
								edge = edges.innerWires[innerWireIdx][edgeIdx];
							}
							else {
								return false;
							}
						}
					}
					else {
						return false;
					}

					edgeIdx++;
				}
			};

			edgeenumerator_t getEdgeEnumerator() const {
				return { *this,0,0,0,{0,0} };
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

		/* Remove 1 or 2 vertex shapes*/
		/* Self intersections and so are handled separately */
		// TODO Figure out if later processing implicitly deals with these issues
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

			resultNotDegenrate = res.outerWires.size() > 1;

			return res;
		}

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
				sourceToResult[clusteredInput[0].sourceIndex] = resultVertices.size() - 1;

				for (uint32_t i = 1; i < clusteredInput.size(); i++) {
					if (!radius.withinRange(clusteredInput[i].pnt, resultVertices.back())) {
						resultVertices.push_back(clusteredInput[i].pnt);
					}

					sourceToResult[clusteredInput[i].sourceIndex] = resultVertices.size() - 1;
				}
			}

		public:
			void doClustering(clusteringradius_t radius) {
				sortVertices();
				sortedToResult(radius);
			}

			//indexedpolyface_t getClustered() {
			//}

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

				for (const auto& ow : indexed.outerWires)
					res.outerWires.push_back(clustered.remapWire(ow));
				for (const auto& iw : indexed.innerWires)
					res.innerWires.push_back(clustered.remapWire(iw));

				return res;
			}
		};

		//indexedpolyface_t RemoveDegeneracies(const indexedpolyface_t& input) {
		//}

		// Contain information :
		// edge A->B
		// new vertex C in the middle, split edge to edges A->C, C->B
		// now as information refers to edge A->B by index (a cut)
		// we can refer it to either child. If 
		// Use same vertex clustering parametrization for new cuts!
		// so that if we have several cuts targeting edge A->B, there is the possibility they will create only one new vertex, 
		// or none at all if the new vertex matches either of the existing vertices
		// the edges are collected back from picking only those edges that don't have children

		struct split_tree_t {

			struct split_t {
				static const uint32_t npos = 0;
				edge_t parent;
				uint32_t childFst, childSnd;
				
				lnpz_linalg::segment2<double> toSegment(vector<point2_t>& vertices) const noexcept{
					return { vertices[parent.fst], vertices[parent.snd] };
				}

				void appendChilds(vector<split_t>& splits, edge_t fst, edge_t snd) {
					splits.push_back({ fst,split_t::npos, split_t::npos });
					childFst = splits.size() - 1;
					splits.push_back({ snd,split_t::npos, split_t::npos });
					childSnd = splits.size() - 1;
				}
			};

			clusteringradius_t clustering;

			vector<split_t> edges;
			vector<point2_t>& vertices;
			size_t originalVertexSize; // initial vertex buffer size. Any vertex at index larger than this is a cut vertex

			vector<pair<point2_t, uint32_t>> cutVertices; // position, address in vertex array. Pos = 0 implies not set yet

			uint32_t pushCutVertex(const point2_t& pnt) {
				cutVertices.push_back({pnt, 0});
				return cutVertices.size() - 1;
			}

			const point2_t& cutPoint(uint32_t cutVertIdx) const {
				return cutVertices[cutVertIdx].first;
			}

			uint32_t verifyCutStored(uint32_t cutVertIdx) { 

				if (cutVertices[cutVertIdx].second != 0)
					return cutVertices[cutVertIdx].second;
				
				vertices.push_back(cutVertices[cutVertIdx].first);
				auto vertexBufferIdx = vertices.size() - 1;
				cutVertices[cutVertIdx].second = vertexBufferIdx;

				return vertexBufferIdx;
			}

			//const lnpz_linalg::segment2<double>::intersect_res_t& isect

			// the srcEdgeIdx must be a leaf edge
			void cutLeafEdge(size_t srcEdgeIdx, uint32_t cutVertexIdx) {
				if (edges[srcEdgeIdx].childFst != split_t::npos) {
					throw runtime_error("cutLeafEdge: Algorithm error, edge is not leaf");
				}

				split_t& srcNode = edges[srcEdgeIdx];
				auto segmentAtIdx = srcNode.toSegment(vertices);
				auto cutVertex = cutPoint(cutVertexIdx);
				bool cutAtExistingVertex = clustering.withinRange(segmentAtIdx.fst, cutVertex) || clustering.withinRange(segmentAtIdx.fst, cutVertex);
				if (cutAtExistingVertex)
					return;
				
				uint32_t cutVertexBufferIdx = verifyCutStored(cutVertexIdx);
				
				edge_t childFst = {srcNode.parent.fst, cutVertexBufferIdx};
				edge_t childSnd = {cutVertexBufferIdx, srcNode.parent.snd};

				srcNode.appendChilds(edges, childFst, childSnd);
			}

			void collectLeafs(uint32_t src, vector<uint32_t>& leaf) {
				stack<uint32_t> edgePath;
				edgePath.push(src);
				while (!edgePath.empty()) {
					uint32_t candIdx = edgePath.top();
					if (candIdx == split_t::npos) {
						throw std::runtime_error("collectLeafs: algorithm data is malformed");
					}
					edgePath.pop();
					const split_t& cand = edges[candIdx];

					if (cand.childFst == split_t::npos) {
						leaf.push_back(candIdx);
					}
					else {
						edgePath.push(cand.childFst);
						edgePath.push(cand.childSnd);
					}
				}
			}

			void collectAllLeafs(vector<edge_t>& edgesOut) {
				for (const auto& s : edges) {
					if (s.childFst == split_t::npos)
						edgesOut.push_back(s.parent);
				}
			}

			void candidatePairs(uint32_t srcI, uint32_t srcJ, vector<pair<uint32_t, uint32_t>>& res) { 
				vector<uint32_t> iLeafs;
				vector<uint32_t> jLeafs;
				collectLeafs(srcI, iLeafs);
				collectLeafs(srcJ, jLeafs);
				for (uint32_t i = 0; i < iLeafs.size(); i++) {
					for (uint32_t j = 0; j < jLeafs.size(); j++)
						res.push_back({i,j});
				}

			}

			void initialIntersectingPairs() { // collect all pairs from input
				// test each edge with each other edge
				//for edges a,b,c,d 
				//a ->b,c,d, b->c,d, c->d
				size_t initialSize = edges.size();

				vector<pair<uint32_t, uint32_t>> candidateLeafs; // tmp

				// first collect all intersections
				for (size_t iSrc = 0; iSrc < initialSize - 1; iSrc++) {
					for (size_t jSrc = iSrc + 1; jSrc < initialSize; jSrc++) {

						candidateLeafs.clear();
						candidatePairs(iSrc, jSrc, candidateLeafs);
						for (const auto& ij : candidateLeafs) {
							auto i = ij.first;
							auto j = ij.second;
							auto ei = edges[i].toSegment(vertices);
							auto ej = edges[j].toSegment(vertices);
							lnpz_linalg::segment2<double>::intersect_res_t isect = ei.intersect(ej);

							for (uint32_t indexOfCut = 0; indexOfCut < isect.count; indexOfCut++) {
								uint32_t cutVertexIdx = pushCutVertex(isect.pnt[indexOfCut]);
								cutLeafEdge(i, cutVertexIdx);
								cutLeafEdge(j, cutVertexIdx);
							}
						}
					}
				}
			}

			void doSelfIntersect() {
				initialIntersectingPairs();
			}

			static split_tree_t Init(const vector<edge_t>& input, vector<point2_t>& vertices, clusteringradius_t clustering) {
				size_t originalVertexSize = vertices.size();
				vector<split_t> edges;
				for (const auto& e : input) {
					edges.push_back({ e,split_t::npos, split_t::npos });
				}
				vector<pair<point2_t, uint32_t>> cutVertices;
				split_tree_t res = { clustering, edges, vertices, originalVertexSize,  cutVertices};
				return res;
			}
		};

		struct orientable_edge_t {
			size_t srcIndex;
			point2_t outerPoint;

			// need just ccw sorting as cw sorting is ccw sorting reversed

			// is a at lower index than b
			static bool OrderCCW(const orientable_edge_t& a, const orientable_edge_t& b) {

				// divide points to the four quadrants x>0y>0,x<0y>0,x<0y<0 and x>0y<0 and start sorting from x>0y>0

				bool aTop = a.outerPoint.y > 0;
				bool bTop = b.outerPoint.y > 0;

				if (aTop != bTop) // a and b other side of y=0 line, if a is top it goes first
					return aTop;

				if(aTop){ // top row , a & b both > 0
					if (a.outerPoint.x > b.outerPoint.x)
						return true;
					else if (a.outerPoint.x < b.outerPoint.x)
						return false;

					if (a.outerPoint.x > 0) // top right
						return a.outerPoint.y < b.outerPoint.y;
					else // top left
						return a.outerPoint.y > b.outerPoint.y;
				}
				else { // bottom row , a & b both < 0
					if (a.outerPoint.x < b.outerPoint.x)
						return true;
					else if (a.outerPoint.x > b.outerPoint.x)
						return false;

					if (a.outerPoint.x > 0) // bottom right
						return a.outerPoint.y < b.outerPoint.y;
					else // bottom left
						return a.outerPoint.y > b.outerPoint.y;
				}

			}

			static void SortCounterClockwise(vector<orientable_edge_t>& orientable) {
				sort(orientable.begin(), orientable.end(), orientable_edge_t::OrderCCW);
			}

			static void SortClockwise(vector<orientable_edge_t>& orientable) {
				sort(orientable.begin(), orientable.end(), orientable_edge_t::OrderCCW);
				reverse(orientable.begin(), orientable.end());
			}

			static vector<orientable_edge_t> GetOrientableEdges(uint32_t srcVertex, const vector<edge_t*>& edgesIn, const vector<point2_t>& vertices) noexcept {
				const point2_t vertexCoord = vertices[srcVertex];
				vector<orientable_edge_t> res;
				const point2_t* vertBuf = vertices.data();
				for (size_t i = 0; i < edgesIn.size(); i++) {
					const auto& edge = *edgesIn[i];
					point2_t vertOut;
					// set src vertex as coordinate origin
					if (srcVertex == edge.fst)
						vertOut = vertBuf[edge.snd] - vertexCoord;
					else // src == snd
						vertOut = vertBuf[edge.fst] - vertexCoord;

					res.push_back({ i, vertOut });
				}
			}

			static vector<size_t> GetCounterClockwiseOrder(uint32_t srcVertex, const vector<edge_t*>& edgesIn, const vector<point2_t>& vertices) {
				auto oriented = GetOrientableEdges(srcVertex, edgesIn, vertices);
				SortCounterClockwise(oriented);
				vector<size_t> res;
				for (auto ord : oriented)
					res.push_back(ord.srcIndex);
				return res;
			}
			
			static vector<size_t> GetClockwiseOrder(uint32_t srcVertex, const vector<edge_t*>& edgesIn, const vector<point2_t>& vertices) {
				auto oriented = GetOrientableEdges(srcVertex, edgesIn, vertices);
				SortClockwise(oriented);
				vector<size_t> res;
				for (auto ord : oriented)
					res.push_back(ord.srcIndex);
				return res;
			}
		};

		// use this to clean self intersections in single outer and inner wires
		// after vertex clustering and edge clipping
		struct clean_self_intersections_t {
			vector<bool> edgeVisited;
			vector<bool> vertexVisited;
			vector<vector<edge_t>> cleaned;
			map<uint32_t, vector<edge_t*>> edgesAroundVertex;

			void buildEdgeMap(const vector<edge_t>& edges) {
				// don't allow same edge several times
				for (const auto& e : edges) {
					edgesAroundVertex[]
				}
			}

			// collect each loop from vertex to vertex
			// if in the set of collected loops any is inside one another,remove
			// the loop that is inside the other
			void doClean(const vector<edge_t>& edges, const vector<point2_t>& vertices, bool outer /*is the winding rule cw or ccw*/) {
				edgeVisited = vector<bool>(edges.size(), false);
				vertexVisited = vector<bool>(vertices.size(), false);

			}

		};

		struct vertex_edge_map_t {

			vector<typed_edge_t> edges;
			const vector<point2_t>* vertices = nullptr;
			/*
				edgesFromVertex
				Each element contains list of edges coming and going from vertex
				-- ei --> [vert] -- ej -->

			*/

			vector<vector<typed_edge_t*>> edgesFromVertex;

			vector<bool> vertexVisited;

			vector<vector<edge_t>> outers;

			vertex_edge_map_t() {
			}

			void addEdges(const vector<edge_t>& edgesIn, bool outer) {
				for (const auto& e : edgesIn) {
					typed_edge_t te;
					te.fst = e.fst;
					te.snd = e.snd;
					te.outer = outer;
					edges.push_back(te);
				}
			}

			void setVertices(const vector<point2_t>* verticesIn) {
				vertices = verticesIn;
				vertexVisited = vector<bool>(vertices->size(), false);
			}

			vector<orientable_edge_t> getOrientableEdges(uint32_t srcVertex, const point2_t vertexCoord, const vector<typed_edge_t*>& typedEdges) noexcept {
				vector<orientable_edge_t> res;
				const point2_t* vertBuf = vertices->data();
				for (size_t i = 0; i < typedEdges.size(); i++) {
					const auto& edge = *typedEdges[i];
					point2_t vertOut;
					// set src vertex as coordinate origin
					if (srcVertex == edge.fst)
						vertOut = vertBuf[edge.snd] - vertexCoord;
					else // src == snd
						vertOut = vertBuf[edge.fst] - vertexCoord;

					res.push_back({ i, vertOut });
				}
			}



			void buildRelations(bool priorityOuter) {
				uint32_t maxVert = 0;

				for (const auto& e : edges) {
					maxVert = max(maxVert, e.fst);
					maxVert = max(maxVert, e.snd);
				}
				edgesFromVertex.resize(maxVert);

				// fill relations. Maintain priority so that there is only one edge
				// between vertices, and if there are both inner and outer wires, the wire with priority
				// is written
				for (auto& e : edges) {
					auto& fstContainer = edgesFromVertex[e.fst];
					auto& sndContainer = edgesFromVertex[e.snd];
					size_t idxAt;
					if (!e.containedIn(fstContainer, idxAt))
						fstContainer.push_back(&e);
					else if (e.outer == priorityOuter)
						fstContainer[idxAt] = &e;

					if (!e.containedIn(sndContainer, idxAt))
						sndContainer.push_back(&e);
					else if (e.outer == priorityOuter)
						sndContainer[idxAt] = &e;
				}

				// sort edges around vertices in counter clockwise or clockwise order
				const bool orderCCW = priorityOuter;
				for (size_t i = 0; i < edgesFromVertex.size(); i++) {
					vector<typed_edge_t*>& edgesAroundVertex = edgesFromVertex[i];
					if (edgesAroundVertex.empty())
						continue;

					vector<orientable_edge_t> orientable = getOrientableEdges(i, vertices->at(i), edgesAroundVertex);
					if (orderCCW)
						SortCounterClockwise(orientable);
					else
						SortClockwise(orientable);
					
					// reorder based on indices
					vector<typed_edge_t*> edgesTmp(edgesAroundVertex.size());
					for (size_t j = 0; j < edgesAroundVertex.size(); j++){
						const orientable_edge_t& oe = orientable[j];
						edgesTmp[j] = edgesAroundVertex[oe.srcIndex];
					}
					
					edgesFromVertex[i] = edgesTmp;
				}

				// now each vertex has it's edges sorted in counterclockwise or clockwise order
			}

			// find leftmost lowest vertex that has incoming and outgoing edges
			// never start from visited vertex - each valid loop must have at least two unvisited vertices
			string findCollectionStartVertexAndEdge(uint32_t& vert, typed_edge_t& edgeOutResult) const  {
				// Find lowest, leftmost vertex
				// First need to find all valid candidates as first vertex

				// Get all vertices that have outgoing and incoming edges that are unvisited

				vector<uint32_t> candidates;
				for (uint32_t vi = 0; vi < edgesFromVertex.size(); vi++) {
					const auto& edges = edgesFromVertex[vi];
					if (edges.empty())
						continue;
				
					/* Include only unused edges and those that can be returned to */
					int unvisitedCount = 0;
					for (const auto& te : edges) {
						if (!te->visited)
							unvisitedCount++;
					}

					if (unvisitedCount < 2)
						continue;

					bool hasOutgoing = false;
					bool hasIncoming = false;
					for (const auto& e : edges) {
						hasIncoming = hasIncoming || ((!e.visited) && e.isIncomingTo(vi));
						hasOutgoing = hasOutgoing || ((!e.visited) && e.isOutgoingFrom(vi));
					}
					if (hasOutgoing && hasIncoming)
						candidates.push_back(vi);
				}

				if(candidates.empty())
					return "Could not find any edges"; // something is wrong
				
				const point2_t* vb = &(*vertices)[0];
				uint32_t startVertIdx = candidates[0];
				point2_t lowest = vb[startVertIdx];
				for (auto vi : candidates) {
					auto p = vb[vi];
					if (p.y > lowest.y)
						continue;
					if (p.y < lowest.y || (p.x < lowest.x) /* when y is equal, find leftmost*/) {
						lowest = p;
						startVertIdx = vi;
					}
				}

				// now pick the outgoing edge with a direction with the lowest y value. If several outgoing edges
				// have SAME y value the earlier clustering is wrong 
				// the input polygon may have crossing edges, but the clustering and clipping stage should have cleaned them up into
				// non-intersecting segments
				typed_edge_t outEdge;
				float outY;
				bool foundEdge = false;
				const auto& edgesOfVert = edgesFromVertex[startVertIdx];
				for (const auto& e : edgesOfVert) {
					if (e.isOutgoingFrom(startVertIdx)) {
						if (!foundEdge) {
							foundEdge = true;
							auto dir = e.direction(*vertices);
							outY = dir.y;
							outEdge = e;
						}
						else {
							auto dir = e.direction(*vertices);
							if (dir.y < outY) {
								outY = dir.y;
								outEdge = e;
							}
						}
					}
				}

				if (!foundEdge)
					return "Could not find candidate edge";
				
				edgeOutResult = outEdge;

				return "";
			}

			uint32_t findPosition(const vector<typed_edge_t>& edges, const typed_edge_t& edgeToFind){
				const uint32_t npos = numeric_limits<uint32_t>::max();
				for (size_t i = 0; i < edges.size(); i++) {
					if (edgeToFind.isSame(edges[i])) {
						return i;
					}
				}
				return npos;
			}


			void extractLoop(const vector<uint32_t>& vertIdx) {

			}

			// call this sequentially to collect all loops into
			bool getNextLoop() {


			}

			// collect outer edges. return non-empty string with error message in case of error
			string collectOuterEdges() {
				// start from left bottom vertex, follow edges until a visited vertex is found. remove edges from visit set traveling backwards from the visited vertex to it's first occurrence
				for (auto& e : edges)
					e.visited = false;
				for (auto& vv : vertexVisited)
					vv = false;


				// First find lowest vertex
				uint32_t curVertIdx;
				typed_edge_t curEdge;
				string startRes = findCollectionStartVertexAndEdge(curVertIdx, curEdge);

				const uint32_t npos = numeric_limits<uint32_t>::max();
				map<uint32_t, typed_edge_t*> visited; // visited, coming from
				bool iterate = true;

				visited[curEdge.fst] = nullptr;
				vector<typed_edge_t> path;
				while (iterate) {
					auto nextVert = curEdge.snd;

					if (visited.count(nextVert) != 0) { // we have visited this vertex already - loop is complete
						// TODO
						// coming to an existing vertex - collect loop backwards
						vector<uint32_t> inloop;
						uint32_t startVert = nextVert;
						uint32_t loopVert = visited[startVert];
						inloop.push_back(startVert);
						while (loopVert != startVert) {
							inloop.push_back(loopVert);
							loopVert = visited[loopVert];
						}

						// Check if anything left to iterate over
						
					}

					// edges oriented in ccw | cw order based on prior configuration
					const auto& nextEdges = edgesFromVertex[nextVert];
					auto idx = findPosition(nextEdges, curEdge);
					auto nextEdge = nextEdges[(idx + 1) % nextEdges.size()];

				}


			}
		};

		// Use only for single wires
		// while the algorithm adds vertices, this stage does not yet remove them (as cuts before boolean only add vertices, never remove them)
		vector<vector<edge_t>> ClipSelfIntersectionsAndFixOrientation(const vector<edge_t>& input, bool outerWire, vector<point2_t>& vertices, clusteringradius_t clustering) {
			// Algorithm:
			// 1. clip edges.
			// 2. remove edges that are inside polygon by wa
			// 3. verify / fix winding to match that of outer or innerwire

			// Clean self intersections
			split_tree_t splitTree = split_tree_t::Init(input, vertices, clustering); // use split tree edges from now on
			splitTree.doSelfIntersect();

			vector<edge_t> selfEdges;
			splitTree.collectAllLeafs(selfEdges);

			vertex_edge_map_t vem;
			vem.addEdges(selfEdges, outerWire);
			vem.setVertices(&vertices);
			vem.buildRelations(outerWire);
			vem.collectOuterEdges();
			auto fixedWires = vem.outers;

			// lastly
			// go through results, see their winding, and reverse them if the winding number is wrong

			vector<vector<edge_t>> res;
			return res;
		}

		struct facemerge_t {
			// TODO
			// data such that
			// per vertex can have only one outgoing edge, e.g 
			// procedge{edge, bool outer?inner, bool used}
			// set<uint64_t procedge>;

			//TODO cleanup situations where outer wires are combined using single edge -> merge them
			// normal edge pickup up builder will omit those
			// 1. start pickup only from edge vertices (does not have symmetric pair)
			// 2. omit symmetric pairs from output (they will be left over, don't consider this as a bug

		};

		string CleanIndexedPolyfaceToRenderable(const indexedpolyface_t& source, double mergeRadius, indexedpolyface_t& result) {


			// TODO ADD HERE CLIPPER LIB

			clusteringradius_t clusteringRadius = clusteringradius_t::Create(mergeRadius);
			clusteredvertexbuilder_t builder;
			indexedpolyface_t clustered = builder.build(source, clusteringRadius);
			if (!builder.err.empty())
				return builder.err;

			edgefaces_t  initialEdges = edgefaces_t::Create(clustered);

			bool cleanedNonDegenerate;
			edgefaces_t cleaned = RemoveSimpleDegeneracies(initialEdges, cleanedNonDegenerate);
			if (!cleanedNonDegenerate)
				return "Input was degenerate";

			// a. Clean clustered data
			// b. Remove degenerate loops
			// c. compute intersection
			// d. 


			return "";
		}

		bool ProcessPolyface(const polygonface_t& polyface, renderablepolygonface_t& res) {

			auto bounds = polyface.boundingRectangle();
			auto maxsize = lnpz_linalg::largestElement(bounds.diagonal());
			double mergeRadius = 0.0001 * maxsize;

			indexedpolyface_t indexed;
			PolygonFaceToIndexed(polyface, indexed);

			indexedpolyface_t cleaned;
			auto cleanres = CleanIndexedPolyfaceToRenderable(indexed, mergeRadius, cleaned);
			if (!cleanres.empty())
				return false;

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

		//
		// Renderer
		//
		typedef Array2D<RGBAFloat32> ImageRGBA32Linear;

		void Renderer::draw(const Scene& scene) {

			// Figure out framebuffer size and scene to framebuffer transform
			const auto wb = scene.getWorldBounds();
			const auto diag = wb.diagonal();
			const auto sceneWidth = diag.x;
			const auto sceneHeight = diag.y;

			const int32_t	outputPixelHeight = m_sceneConfig.outputHeightPixels;
			const int32_t	outputScenePixelHeight = outputPixelHeight - 2 * m_sceneConfig.paddingInPixels;
			const auto		sceneToPixelScale = outputScenePixelHeight / sceneHeight;
			const int32_t	outputPixelWidth = (int32_t)(sceneToPixelScale * sceneWidth) + (int32_t)(2 * m_sceneConfig.paddingInPixels);
			const point2_t	sceneOrig(m_sceneConfig.paddingInPixels);
			const auto		offset = sceneOrig - wb.min();

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

		string Renderer::write(const string& pathOut) const {
			vector<uint8_t> byteArray;
			int width = (int)m_framebuffer.dim1();
			int height = (int)m_framebuffer.dim2();
			int rowlength = 4 * width;
			stbi_write_png_to_func(internal_detail::WriteBytes, (void*)&byteArray, width, height, 4, m_framebuffer.data(), rowlength);
			return util::WriteBytesToPath(byteArray, pathOut);
		}

		string Renderer::getSRGBABytes() const {
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

