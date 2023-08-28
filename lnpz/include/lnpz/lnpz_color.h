// This file lnpz_color.h is part of lineplz - Public domain simple drawing library
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

#include <limits>
#include <stdint.h>

namespace lnpz {

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
		static RGBAFloat32 VenetianRed() { return{ 0.577581, 0.00242821, 0.00749903, 1.f }; }
	

		static RGBAFloat32 Green() { return{ 0.f, 1.f, 0.f, 1.f }; }
		static RGBAFloat32 Blue() { return{ 0.f, 0.f, 1.f, 1.f }; }
		static RGBAFloat32 Cyan() { return{ 0.f, 1.f, 1.f, 1.f }; }
		
		static RGBAFloat32 Turquoise() { return{ 0.051f, 0.745f, 0.630f, 1.f }; }


		static RGBAFloat32 Violet() { return{ 1.f, 0.f, 1.f, 1.f }; }
		static RGBAFloat32 Yellow() { return{ 1.f, 1.f, 0.f, 1.f }; }
		static RGBAFloat32 Black() { return{ 0.f, 0.f, 0.f, 1.f }; }
		static RGBAFloat32 Grey() { return{ 0.21f, 0.21f, 0.21f, 1.f }; }
		static RGBAFloat32 White() { return{ 1.f, 1.f, 1.f, 1.f }; }
		static RGBAFloat32 Orange() { return{ 1.f, 0.65f, 0.f, 1.f }; }
		static RGBAFloat32 Navy() { return{ 0., 0., 0.215861, 1.f }; }

		static RGBAFloat32 Pink() { return{ 1.f, 0.527f, 0.597f, 1.f }; } 
	};
}
