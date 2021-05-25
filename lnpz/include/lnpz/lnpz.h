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

namespace lnpz{

    template<class T>
    class Array2D {
        std::vector<T> m_data;
        size_t m_dim1;
        size_t m_dim2;
    public:

        Array2D(size_t dim1, size_t dim2)
            :m_dim1(dim1), m_dim2(dim2), m_data(dim1* dim2) {
        }

        Array2D(size_t dim1, size_t dim2, T defaultValue)
            :m_dim1(dim1), m_dim2(dim2), m_data(dim1* dim2, defaultValue) {
        }

        Array2D(const std::pair<size_t, size_t>& dims) :Array2D(dims.first, dims.second) {}

        Array2D(const Array2D& rhs)
            : m_data(rhs.m_data), m_dim1(rhs.m_dim1), m_dim2(rhs.m_dim2) {
        }

        Array2D(Array2D&& rhs) noexcept
            : m_data(std::move(rhs.m_data)), m_dim1(rhs.m_dim1), m_dim2(rhs.m_dim2) {
        }

        Array2D& operator=(const Array2D& rhs)
        {
            m_data = rhs.m_data;
            m_dim1 = rhs.m_dim1;
            m_dim2 = rhs.m_dim2;
            return *this;
        }

        Array2D& operator=(Array2D&& rhs) noexcept
        {
            m_data = std::move(rhs.m_data);
            m_dim1 = rhs.m_dim1;
            m_dim2 = rhs.m_dim2;
            return *this;
        }

        std::vector<T>& asVector() { return m_data; }

        typename std::vector<T>::iterator begin() { return m_data.begin(); }
        typename std::vector<T>::iterator end() { return m_data.end(); }
        typename std::vector<T>::const_iterator begin() const { return m_data.begin(); }
        typename std::vector<T>::const_iterator end() const { return m_data.end(); }

        inline size_t index2d(size_t x, size_t y) const { return (y * m_dim1) + x; }

        size_t dim1() const { return m_dim1; }
        size_t dim2() const { return m_dim2; }

        std::pair<size_t, size_t> size() const { return { m_dim1, m_dim2 }; }

        size_t sizeInBytes() const { return m_dim1 * m_dim2 * sizeof(T); }

        size_t elementCount() const { return m_dim1 * m_dim2; }

        T* data() { return m_data.data(); }

        const T* data() const { return m_data.data(); }

        T& at(size_t x, size_t y) noexcept { return m_data[index2d(x, y)]; }
        const T& at(size_t x, size_t y) const noexcept { return m_data[index2d(x, y)]; }

        T& at(size_t i) noexcept { return m_data[i]; }
        const T& at(size_t i) const noexcept { return m_data[i]; }

        T& at(const std::pair<size_t, size_t>& idx) noexcept { return at(idx.first, idx.second); }
        const T& at(const std::pair<size_t, size_t>& idx) const noexcept { return at(idx.first, idx.second); }

        void set(size_t x, size_t y, const T& value) noexcept { m_data[index2d(x, y)] = value; }
        void setIfLarger(size_t x, size_t y, const T& value) noexcept { if (m_data[index2d(x, y)] < value) m_data[index2d(x, y)] = value; }
        void setIfSmaller(size_t x, size_t y, const T& value) noexcept { if (m_data[index2d(x, y)] > value) m_data[index2d(x, y)] = value; }
        void set(const std::pair<size_t, size_t>& idx, const T& value) { set(idx.first, idx.second, value); }

        void setAll(const T& value) {
            for (auto& v : m_data) v = value;
        }

        typedef typename std::vector<T>::const_iterator const_iterator_t;
        typedef typename std::vector<T>::iterator iterator_t;

        const_iterator_t constIteratorAt(size_t x, size_t y) const {
            return m_data.begin() + index2d(x, y);
        }

        iterator_t iteratorAt(size_t x, size_t y) {
            return m_data.begin() + index2d(x, y);
        }

        bool validIndex(const std::pair<size_t, size_t>& pos) const {
            return (pos.first < m_dim1) && (pos.second < m_dim2);
        }

        void copyRowFrom(const Array2D<T>& src, std::pair<size_t, size_t> pos, std::pair<size_t, size_t> srcPos, size_t count) {
            if (!validIndex(pos))
                return;
            if (!src.validIndex(srcPos))
                return;

            size_t thisCountLimit = std::min(count, m_dim1 - pos.first);
            count = std::min(thisCountLimit, src.m_dim1 - srcPos.first);
            const_iterator_t srcStart = src.constIteratorAt(srcPos.first, srcPos.second);
            const_iterator_t srcEnd = srcStart + count;
            iterator_t targetStart = iteratorAt(pos.first, pos.second);
            std::copy(srcStart, srcEnd, targetStart);
        }

        void fill(const T& element) {
            std::fill(m_data.begin(), m_data.end(), element);
        }

    };

	typedef lnpz_linalg::double2            point2_t;
	typedef lnpz_linalg::double3x3          matrix33_t;
	typedef lnpz_linalg::nslab<double, 2>   rectangle_t;

	inline point2_t ApplyToPoint(const matrix33_t& m, const point2_t& p) {
		using namespace lnpz_linalg;
		auto r0 = m.row(0);
		auto r1 = m.row(1);
		double x = dot(r0.xy(), p) + r0.z;
		double y = dot(r0.xy(), p) + r1.z;
		return { x,y };
	}

	struct SRGBA {
		uint8_t r;
		uint8_t g;
		uint8_t b;
		uint8_t a;
	};

	/** Linear colorspace */
	struct RGBAFloat32 {
		float r;
		float g;
		float b;
		float a;

		RGBAFloat32 premultiplyAlpha(float aa) const {
			return { aa * r, aa * g, aa * b, aa * a };
		}

		RGBAFloat32 toPremultiplied() const {
			return { a * r, a * g, a * b, a };
		}

		bool operator==(const RGBAFloat32& c) { return r == c.r && g == c.g && b == c.b && a == c.a; }
		uint64_t toHash() const {
			uint16_t mv = std::numeric_limits<uint16_t>::max();
			uint64_t rs = (uint64_t)(r * (uint64_t)mv);
			uint64_t gs = (uint64_t)(g * (uint64_t)mv);
			uint64_t bs = (uint64_t)(b * (uint64_t)mv);
			uint64_t as = (uint64_t)(a * (uint64_t)mv);
			uint64_t res = (rs << 48) | (gs << 32) | (bs << 16) | as;
			return res;
		}

		static RGBAFloat32 CreatePremultiplied(float r, float g, float b, float a) {
			return { r * a, g * a, b * a, a };
		}

		static RGBAFloat32 Create(float v) { return { v, v, v, 1.f }; }
		static RGBAFloat32 Create(float r, float g, float b, float a) { return { r, g, b, a }; }

		static RGBAFloat32 Transparent() { return{ 0.f, 0.f, 0.f, 0.f }; }

		static RGBAFloat32 Red() { return{ 1.f, 0.f, 0.f, 1.f }; }
		static RGBAFloat32 Green() { return{ 0.f, 1.f, 0.f, 1.f }; }
		static RGBAFloat32 Blue() { return{ 0.f, 0.f, 1.f, 1.f }; }
		static RGBAFloat32 Cyan() { return{ 0.f, 1.f, 1.f, 1.f }; }

		static RGBAFloat32 Violet() { return{ 1.f, 0.f, 1.f, 1.f }; }
		static RGBAFloat32 Yellow() { return{ 1.f, 1.f, 0.f, 1.f }; }
		static RGBAFloat32 Black() { return{ 0.f, 0.f, 0.f, 1.f }; }
		static RGBAFloat32 White() { return{ 1.f, 1.f, 1.f, 1.f }; }
		static RGBAFloat32 Orange() { return{ 1.f, 0.65f, 0.f, 1.f }; }
		static RGBAFloat32 Navy() { return{ 0.f, 0.0f, 0.502f, 1.f }; }

		static RGBAFloat32 Pink() { return{ 1.f, 105.f / 255.f, 180.f / 255.f, 1.f }; } // Actually HotPink...
	};


	// Single line is a linestring with two points
	struct linestring_t {
		std::vector<point2_t> points;

		rectangle_t boundingRectangle() const noexcept {
			using namespace std;
			rectangle_t r = rectangle_t::InitializeEmpty();
			for (const auto& p : points) {
				r.coverInPlace(p);
			}
			return r;
		}
	};

	struct polygonface_t {
		linestring_t outerWire;
		std::vector<linestring_t> innerWires;

		rectangle_t boundingRectangle() const {
			return outerWire.boundingRectangle();
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
		RGBAFloat32 color = RGBAFloat32::Black();
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
		std::vector<polygonface_t> m_polygonFaces;
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
			pmin = ApplyToPoint(mat, pmin);
			pmax = ApplyToPoint(mat, pmax);

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
		void setMaterial(const material_t& mat) {
			m_currentMaterial = mat;
		}

		void addLineString(const linestring_t& ls) {
			m_scene.m_lines.push_back(ls);
			auto idx = m_scene.m_lines.size() - 1;
			localToWorldTransform_t trf;
			m_scene.m_instances.push_back({ trf, m_currentMaterial, InstanceType::LineString, idx });
		}

		void addPolygonFace(const polygonface_t& pf) {
			m_scene.m_polygonFaces.push_back(pf);
			auto idx = m_scene.m_polygonFaces.size() - 1;
			localToWorldTransform_t trf;
			m_scene.m_instances.push_back({ trf, m_currentMaterial, InstanceType::PolygonFace, idx });
		}

		Scene build() const {
			return m_scene;
		}
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
		void write(const std::string& pathOut) const;
		/** Return byte array of the current framebuffer (i.e. the pixels drawn).*/
		std::string getSRGBABytes() const;
	};

}
