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
		// Callback for stb_image_write
		void WriteBytes(void* context, void* data, int size) {
			std::vector<uint8_t>* bytes = (std::vector<uint8_t>*)(context);
			uint8_t* byteData = (uint8_t*)data;
			*bytes = std::vector<uint8_t>(byteData, byteData + size);
		}

		void RasterizeLineString(const matrix33_t sceneToPixel, const Scene::instance_t& inst, const linestring_t& lines, ImageRGBA32Linear& framebuffer){
			// get scenespace geometry
			// convert to distance field in scenespace
			// colorize distance field

		}
		
		void RasterizePolygonFace(const matrix33_t sceneToPixel, const Scene::instance_t& inst, const polygonface_t& face, ImageRGBA32Linear& framebuffer){
			// get scenespace geometry
			// convert to distance field in scenespace
			// colorize distance field

		}

		void RasterizeInstance(const Scene& scene, const matrix33_t sceneToPixel, const Scene::instance_t& inst, ImageRGBA32Linear& framebuffer){
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

		const matrix33_t sceneToPixel = {	{sceneToPixelScale, 0, offset.x},
									{0,sceneToPixelScale,  offset.y},
									{0, 0, 1}};

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
	namespace internal_detail{
		uint8_t LinearFloatToSRGBUint8(const float f) {
			return stbir__linear_to_srgb_uchar(f);
		}

		inline float clampf(float val, float minval, float maxval) {
			return val < minval ? minval : (val > maxval ? maxval : val);
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

