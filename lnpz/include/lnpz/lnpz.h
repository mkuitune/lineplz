// lineplz - Public domain simple drawing library
//
// The intent of this library is to offer the capability to implment quick 
// visualizations - debug or otherwise - for simple data such as lines without
// including a larger library. This is not intended as a replacement for a 
// an actual drawing library such as Cairo or Skia.
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

#include <vector>
#include <limits>
#include <algorithm>
#include <stdint.h>

#include <lnpz/lnpz_linalgd.h>
#include <lnpz/lnpz_array2d.h>
#include <lnpz/lnpz_color.h>

namespace lnpz{

	typedef lnpz_linalg::double2            point2_t;
	typedef lnpz_linalg::double3x3          matrix33_t;
	typedef lnpz_linalg::nslab<double, 2>   rectangle_t;

	using namespace std;

	// Single line is a linestring with two points
	struct linestring_t {
		vector<point2_t> points;

		rectangle_t boundingRectangle() const noexcept {
			using namespace std;
			rectangle_t r = rectangle_t::InitializeEmpty();
			for (const auto& p : points) {
				r.coverInPlace(p);
			}
			return r;
		}

		linestring_t reversed() const noexcept {
			return { std::vector<point2_t>(points.rbegin(), points.rend()) };
		}
	};

	struct polygonface_t {

		/*  polygonface_t wires do not contain the last edge, it is implicit. 
			This way polygonface is always closed by definition (and it's not up to the
			input data to close it or not).
		*/

		linestring_t outerWire;
		std::vector<linestring_t> innerWires;

		rectangle_t boundingRectangle() const {
			return outerWire.boundingRectangle();
		}
	};

	struct renderablepolygonface_t {
		size_t sourcePolygonIndex;
		std::vector<linestring_t> wires;
		
		rectangle_t boundingRectangle() const {
			rectangle_t r = wires[0].boundingRectangle();
			for (const auto& w : wires) {
				r.coverInPlace(w.boundingRectangle());
			}
			return r;
		}
	};

	struct localToWorldTransform_t {
		lnpz_linalg::double2 position = { 0,0 };
		double rotation = 0.f; // radians
		double scale = 1.f;

		matrix33_t matrix() const {
			return lnpz_linalg::localToWorld2_matrix<double>(position, scale, rotation);
		}
	};

	struct material_t {
		RGBAFloat32 lineColor = RGBAFloat32::Black();
		RGBAFloat32 faceColor = RGBAFloat32::Red();
		double lineWidth = 1.0; // pixels
	};

	enum class InstanceType { LineString, PolygonFace };

	class Scene {
	public:
		struct instance_t {
			localToWorldTransform_t localToWorld;
			material_t material;
			InstanceType type;
			size_t idx; // Index to definition
		};

		std::vector<linestring_t> m_lines;
		std::vector<polygonface_t> m_polygonFaces; // original data for polygons, store for reference
		std::vector<renderablepolygonface_t> m_renderablePolygons; // Geometry preprocessed for rendering
		std::vector<instance_t> m_instances;

		rectangle_t getInstanceLocalBounds(const instance_t& inst) const {
			rectangle_t r = rectangle_t::InitializeEmpty();
			if (inst.type == InstanceType::LineString) {
				auto rInst = m_lines[inst.idx].boundingRectangle();
				r.coverInPlace(rInst);
			}
			else if (inst.type == InstanceType::PolygonFace) {
				auto rInst = m_polygonFaces[inst.idx].boundingRectangle();
				r.coverInPlace(rInst);
			}
			return r;
		}

		rectangle_t getInstanceWorldBounds(const instance_t& inst) const {
			using namespace lnpz_linalg;
			rectangle_t r = getInstanceLocalBounds(inst);
			auto pmin = r.min();
			auto pmax = r.max();
			auto mat = inst.localToWorld.matrix();
			pmin = apply_to_point2(mat, pmin);
			pmax = apply_to_point2(mat, pmax);

			rectangle_t wb = rectangle_t::InitializeFromPair(pmin, pmax);
			return wb;
		}

		rectangle_t getWorldBounds() const {
			rectangle_t res = rectangle_t::InitializeEmpty();
			for (const auto& inst : m_instances) {
				auto wb = getInstanceWorldBounds(inst);
				res.coverInPlace(wb);
			}
			return res;
		}
	};

	typedef Array2D<SRGBA> ImageRGBA8SRGB;
	typedef Array2D<RGBAFloat32> ImageRGBA32Linear;

	ImageRGBA8SRGB ConvertRBGA32LinearToSrgba(const ImageRGBA32Linear& linear);

	class SimpleBuilder {
		Scene m_scene;
		material_t m_currentMaterial;

	public:
		
		void setMaterial(const material_t& mat);
		void addLineString(const linestring_t& ls);
		void addPolygonFace(const polygonface_t& pf);

		Scene build() const;
	};

	struct SceneConfig {
		RGBAFloat32 background = RGBAFloat32::White();
		uint32_t outputHeightPixels = 256;
		uint32_t paddingInPixels = 10; // padding around scene - is not added to output size but scene is contracted
	};

	class Renderer {
		SceneConfig m_sceneConfig;

		ImageRGBA8SRGB m_framebuffer = ImageRGBA8SRGB(10, 10);
	public:
		Renderer() {}
		Renderer(SceneConfig sceneConfig) :m_sceneConfig(sceneConfig) {}
		void setConfig() {}
		void draw(const Scene& scene);
		/** Write out the current framebuffer (i.e. the pixels drawn) to a PNG file.*/
		std::string write(const std::string& pathOut) const;
		/** Return byte array of the current framebuffer (i.e. the pixels drawn).*/
		std::string getSRGBABytes() const;
	};

}
