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
