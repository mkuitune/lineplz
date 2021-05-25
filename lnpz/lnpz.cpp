#include <lnpz/lnpz.h>

#define STB_IMAGE_RESIZE_IMPLEMENTATION
#include "stb/stb_image_resize.h"
#undef STB_IMAGE_RESIZE_IMPLEMENTATION

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"
#undef STB_IMAGE_WRITE_IMPLEMENTATION

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"
#undef STB_IMAGE_IMPLEMENTATION

namespace lnpz {

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

		// Rasterize in 4 byte floating point precision
		ImageRGBA32Linear framebuffer = ImageRGBA32Linear(outputPixelWidth, outputPixelHeight);

		// Rasterize scene instances in order
		for (const auto& inst : scene.m_instances) {
			
		}

		m_framebuffer = ConvertRBGA32LinearToSrgb(framebuffer);

	}

	void Renderer::write(const std::string& pathOut) const{

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
	namespace {
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

	ImageRGBA8SRGB ConvertRBGA32LinearToSrgb(const ImageRGBA32Linear& linear)
	{
		ImageRGBA8SRGB res(linear.size());
		auto seq = make_seq(linear.elementCount());
		for (auto i : seq) {
			res.at(i) = ToSRGBA(linear.at(i));
		}
		return res;
	}
}

