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

#define STB_IMAGE_RESIZE_IMPLEMENTATION
#include "stb/stb_image_resize.h"
#undef STB_IMAGE_RESIZE_IMPLEMENTATION

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"
#undef STB_IMAGE_WRITE_IMPLEMENTATION

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"
#undef STB_IMAGE_IMPLEMENTATION

//
// Utilities
//
namespace lnpz{

	//
	// Image and rasterization
	//
	namespace internal_detail{

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

		struct LineStringDistance{

			const std::vector<point2_t>& lines;

			inline float unsignedDistance(const point2_t& p) const
			{
				size_t N = lines.size();
				auto dist0 = p - lines[0];
				double d = dot(dist0,dist0);

				for (size_t i = 0, j = N - 1; i < N; j = i, i++)
				{
					auto pi = lines[i];
					auto pj = lines[j];
					auto e = pj - pi;
					auto w = p - pi;
					auto b = w - e * clampd(dot(w,e) / dot(e,e), 0.0, 1.0);
					d = std::min(d, dot(b,b));
				}
				return (float)sqrt(d);
			}

			inline float signedDistance(const point2_t& p) const
			{
				size_t N = lines.size();
				auto dist0 = p - lines[0];
				double d = dot(dist0,dist0);

				double s = 1.0;

				for (size_t i = 0, j = N - 1; i < N; j = i, i++)
				{
					auto pi = lines[i];
					auto pj = lines[j];
					auto e = pj - pi;
					auto w = p - pi;
					auto b = w - e * clampd(dot(w,e) / dot(e,e), 0.0, 1.0);
					d = std::min(d, dot(b,b));
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

			std::function<float(float, float)> bindSigned() const {
				return[this](float a, float b) {
					return this->signedDistance(a, b);
				};
			}

			std::function<float(float, float)> bindUnsigned() const {
				return[this](float a, float b) {
					return this->unsignedDistance(a, b);
				};
			}
		};

		struct PolyFaceDistance{

			const std::vector<point2_t>& outerWire;
			const std::vector<std::vector<point2_t>>& innerWires;

			inline float unsignedDistance(const point2_t& p) const {
				auto dist = unsignedDistance(p, outerWire);
				for (const auto& inner : innerWires) {
					auto idist = unsignedDistance(p, inner);
					dist = std::min(dist, idist);
				}
				return dist;
			}

			inline float signedDistance(const point2_t& p) const {
				auto dist = signedDistance(p, outerWire);
				for (const auto& inner : innerWires) {
					auto idist = signedDistance(p, inner);
					if (fabsf(idist) < fabsf(dist))
						dist = idist;
				}
				return dist;

			}

			inline float unsignedDistance(const point2_t& p, const std::vector<point2_t>& lines) const
			{
				size_t N = lines.size();
				auto dist0 = p - lines[0];
				double d = dot(dist0,dist0);

				for (size_t i = 0, j = N - 1; i < N; j = i, i++)
				{
					auto pi = lines[i];
					auto pj = lines[j];
					auto e = pj - pi;
					auto w = p - pi;
					auto b = w - e * clampd(dot(w,e) / dot(e,e), 0.0, 1.0);
					d = std::min(d, dot(b,b));
				}
				return sqrtf(d);
			}

			inline float signedDistance(const point2_t& p, const std::vector<point2_t>& lines) const
			{
				size_t N = lines.size();
				auto dist0 = p - lines[0];
				double d = dot(dist0,dist0);

				double s = 1.0;

				for (size_t i = 0, j = N - 1; i < N; j = i, i++)
				{
					auto pi = lines[i];
					auto pj = lines[j];
					auto e = pj - pi;
					auto w = p - pi;
					auto b = w - e * clampd(dot(w,e) / dot(e,e), 0.0, 1.0);
					d = std::min(d, dot(b,b));
					TripletBool c = { p.y >= pi.y, p.y < pj.y, e.x* w.y > e.y * w.x };
					if (c.all() || c.allNot())
						s *= -1.0;
				}
				return s * sqrtf(d);
			}

			float signedDistance(double x, double y) const {
				return signedDistance({ x,y });
			};
			float unsignedDistance(double x, double y) const {
				return unsignedDistance({ x,y });
			}

			std::function<float(float, float)> bindSigned() const {
				return[this](float a, float b) {
					return this->signedDistance(a, b);
				};
			}

			std::function<float(float, float)> bindUnsigned() const {
				return[this](float a, float b) {
					return this->unsignedDistance(a, b);
				};
			}
		};

		// Callback for stb_image_write
		void WriteBytes(void* context, void* data, int size) {
			std::vector<uint8_t>* bytes = (std::vector<uint8_t>*)(context);
			uint8_t* byteData = (uint8_t*)data;
			*bytes = std::vector<uint8_t>(byteData, byteData + size);
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
					size_t xinit = std::numeric_limits<size_t>::max();
					return {x0, y0, xmax, ymax, xinit,0}; 
				}
			};

			enumerator_t enumerate() const { return enumerator_t::Create(x0, y0, xmax, ymax); }

			static bool Get(const rectangle_t& rect, PixelCover& res) {
				double xmaxd = rect.max().x;
				double ymaxd = rect.max().y;
				double xmind = std::max(0.0, rect.min().x);
				double ymind = std::max(0.0, rect.min().y);
				size_t x0 = xmind;
				size_t y0 = ymind;
				size_t xmax = xmaxd;
				size_t ymax = ymaxd;
				if (xmax == x0 || ymax == y0)
					return false;
				res = {x0, y0, xmax, ymax};
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
			//bool unsignedDistanceToColor(float d, RGBAFloat32& out) {
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
			//bool signedDistanceToColor(float d, RGBAFloat32& out) {
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

			static ColorizeMaterial2D Create(const material_t& linesMaterial, const material_t& polygonMaterial) {
				ColorizeMaterial2D c;
				c.src.colorFill = polygonMaterial.color;
				c.src.colorLine = linesMaterial.color;
				c.src.linewidth = linesMaterial.lineWidth;
				c.lineColorPremul = linesMaterial.color.toPremultiplied();
				c.fillColorPremul = polygonMaterial.color.toPremultiplied();
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
				apply_to_point2(localToPixel, p);
			}

			auto frameBufferBounds = GetImageArrayBounds(framebuffer);

			// Pixel bounds
			auto linePixelBounds = linesInPixels.boundingRectangle();

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
			LineStringDistance dist{lines.points};
			auto fun = dist.bindUnsigned();
			fieldBuilder.add(fun);
			auto field =  fieldBuilder.build();
			PixelCover pixelCoords;
			if (!PixelCover::Get(rasterizationBounds, pixelCoords)) {
				return;
			}
			auto pixelEnum = pixelCoords.enumerate();

			ColorizeMaterial2D material = ColorizeMaterial2D::Create(inst.material, inst.material);
			RGBAFloat32 colorTmp;
			while (pixelEnum.next()) {
				float x = pixelEnum.spatialx;
				float y = pixelEnum.spatialy;
				float dist = field.getDeepSample(x, y);
				material.lineColorFromDistance(dist, colorTmp);
				if (colorTmp.a > 0.001f) {
					BlendPixelPremuli((unsigned)pixelEnum.x, (unsigned)pixelEnum.y, colorTmp, framebuffer);
				}
			}

		}

		void RasterizePolygonFace(const matrix33_t sceneToPixel, const Scene::instance_t& inst, const polygonface_t& face, ImageRGBA32Linear& framebuffer) {
			// get scenespace geometry
			// convert to distance field in scenespace
			// colorize distance field

		}

		void RasterizeInstance(const Scene& scene, const matrix33_t sceneToPixel, const Scene::instance_t& inst, ImageRGBA32Linear& framebuffer) {
			if (inst.type == InstanceType::LineString) {
				RasterizeLineString(sceneToPixel, inst, scene.m_lines[inst.idx], framebuffer);
			}
			else if (inst.type == InstanceType::PolygonFace) {
				RasterizePolygonFace(sceneToPixel, inst, scene.m_polygonFaces[inst.idx], framebuffer);
			}
		}
	}

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

		const matrix33_t sceneToPixel = { {sceneToPixelScale, 0, offset.x},
									{0,sceneToPixelScale,  offset.y},
									{0, 0, 1} };

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

	std::string Renderer::write(const std::string& pathOut) const {
		std::vector<uint8_t> byteArray;
		int width = (int)m_framebuffer.dim1();
		int height = (int)m_framebuffer.dim2();
		int rowlength = 4 * width;
		stbi_write_png_to_func(internal_detail::WriteBytes, (void*)&byteArray, width, height, 4, m_framebuffer.data(), rowlength);
		return util::WriteBytesToPath(byteArray, pathOut);
	}

	std::string Renderer::getSRGBABytes() const {
		std::string res;
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

