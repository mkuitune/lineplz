// This file lnpz_util.h is part of lineplz - Public domain simple drawing library
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

#include <vector>
#include <string>
#include <algorithm>

#include <lnpz/lnpz_linalgd.h>

namespace lnpz {
	namespace util {
		template<class T>
		void DoTimes(size_t count, const T& fun) { for (size_t i = 0; i < count; i++) fun(); }

		template<class T>
		T MaxOf(const T& a, const T& b, const T& c, const T& d) {
			return std::max(a, std::max(b, std::max(c, d)));
		}

		template<class T>
		T MinOf(const T& a, const T& b, const T& c, const T& d) {
			return std::min(a, std::min(b, std::min(c, d)));
		}

		inline float Lerp(float src, float dst, float u) {
			return (1.0f - u) * src + u * dst;
		}

		inline float Clampf(float val, float minval, float maxval) {
			return val < minval ? minval : (val > maxval ? maxval : val);
		}

		typedef lnpz_linalg::vec<float, 2> Pairf;

		struct LinearMap2D {
			float s;
			Pairf offset;
			Pairf map(const Pairf& p) const {
				return p * s + offset;
			}
		};

		std::string PreparePathForWriting(const std::string& pathStr);
		std::string WriteBytesToPath(const std::vector<uint8_t>& bytes, const std::string& pathStr);
	}
}
