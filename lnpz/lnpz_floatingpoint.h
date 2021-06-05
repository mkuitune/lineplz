// This file lnpz_floatingpoint.h is part of lineplz - Public domain simple drawing library
//
// The original author of this software is Mikko Kuitunen, and its permanent
// home is <http://github.com/mkuitune/lineplz/>. If you find this software
// useful, an acknowledgement in your source text and/or product documentation
// is appreciated, but not required.
// 
// The floating point comparison routines are from the work of
// Matt Kline (https://bitbashing.io/comparing-floats.html)
// Bruce Dawson (https://randomascii.wordpress.com/)
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

#include <string>
#include <limits>
#include <cmath>
#include <stdint.h>

namespace lnpz {

	class Float64 {
	public:
		typedef int64_t ulps_t;

		static constexpr double Max = std::numeric_limits<double>::max();
		//static constexpr double GeometryEpsilon = 1.0e-10;
		//static constexpr double NegativeGeometryEpsilon = -1.0e-10;
		static constexpr double GeometryEpsilon = 1.0e-8;
		static constexpr double NegativeGeometryEpsilon = -1.0e-8;
		static constexpr double VertexPositionTolerance = 1.0e-5;
		static constexpr ulps_t ulpsEpsilon = 10;

		static constexpr double PI = 3.14159265358979323846;
		/** Full circle in radians*/
		static constexpr double FullCircle = 2 * PI;

		static bool GeometryIsNonZero(double a) {
			return !GeometryIsCloseToZero(a);
		}

		static bool GeometryIsCloseToZero(double a) {
			//return fabs(a) < GeometryEpsilon; // alternative !(a > ge) && !( a < -ge)
			return (a < GeometryEpsilon) && (a > NegativeGeometryEpsilon);
		}

		static bool GeometryIsGreaterThanZero(double a) {
			return a > GeometryEpsilon;
		}

		static bool GeometryIsLessThanZero(double a) {
			return a < NegativeGeometryEpsilon;
		}

		static bool GeometryAreEqual(double a, double b) {
			return GeometryIsCloseToZero(b - a);
		}
		// a > b
		static bool GeometryIsGreaterThan(double a, double b) {
			return b < (a - GeometryEpsilon);
		}

		static ulps_t IntegerOfBytes(double d) {
			return *((ulps_t*)&d);
		}

		static bool NearlyEqual(double a, double b,
			double fixedEpsilon, int64_t ulpsEpsilon)
		{
			// Handle the near-zero case.
			const double difference = fabs(a - b);
			if (difference <= fixedEpsilon) return true;

			return UlpsDistance(a, b) <= ulpsEpsilon;
		}

		static bool UlpsAreNearlyEqual(double a, double b, ulps_t eps = ulpsEpsilon) {
			ulps_t au = IntegerOfBytes(a);
			ulps_t bu = IntegerOfBytes(b);
			return abs(au - bu) < eps;
		}

		static const int64_t int64SignBit = (int64_t)1 << 63;

		static int64_t DoubleToNativeSignedUlps(double f)
		{
			int64_t i;
			memcpy(&i, &f, sizeof(double));

			// Positive values are the same in both
			// two's complement and signed magnitude.
			// For negative values, remove the sign bit
			// and negate the result (subtract from 0).
			return i >= 0 ? i : -(i & ~int64SignBit);
		}

		double NativeSignedUlpsToDouble(int64_t ulps)
		{
			if (ulps < 0) {
				ulps = -ulps;
				ulps |= int64SignBit;
			}
			double f;
			memcpy(&f, &ulps, sizeof(double));
			return f;
		}

		double UlpsIncrement(double f, int64_t ulps)
		{
			if (isnan(f) || isinf(f)) return f;
			int64_t i = DoubleToNativeSignedUlps(f);
			i += ulps;
			return NativeSignedUlpsToDouble(i);
		}

		/**/
		static int64_t UlpsDistance(const double a, const double b)
		{
			// We can skip all the following work if they're equal.
			if (a == b) return 0;

			constexpr auto max = std::numeric_limits<int64_t>::max();

			// We first check if the values are NaN.
			// If this is the case, they're inherently unequal;
			// return the maximum distance between the two.
			if (isnan(a) || isnan(b)) return max;

			// If one's infinite, and they're not equal,
			// return the max distance between the two.
			if (isinf(a) || isinf(b)) return max;

			// At this point we know that the floating-point values aren't equal and
			// aren't special values (infinity/NaN).
			// Because of how IEEE754 floats are laid out
			// (sign bit, then exponent, then mantissa), we can examine the bits
			// as if they were integers to get the distance between them in units
			// of least precision (ULPs).
			static_assert(sizeof(double) == sizeof(int64_t), "What size is float?");

			// memcpy to get around the strict aliasing rule.
			// The compiler knows what we're doing and will just transfer the float
			// values into integer registers.
			int64_t ia, ib;
			memcpy(&ia, &a, sizeof(double));
			memcpy(&ib, &b, sizeof(double));

			// If the signs of the two values aren't the same,
			// return the maximum distance between the two.
			// This is done to avoid integer overflow, and because the bit layout of
			// floats is closer to sign-magnitude than it is to two's complement.
			// This *also* means that if you're checking if a value is close to zero,
			// you should probably just use a fixed epsilon instead of this function.
			if ((ia < 0) != (ib < 0)) return max;

			// If we've satisfied all our caveats above, just subtract the values.
			// The result is the distance between the values in ULPs.
			int64_t distance = ia - ib;
			if (distance < 0) distance = -distance;
			return distance;
		}

	};

	class Float32 {
	public:
		static constexpr float Max = std::numeric_limits<float>::max();
		//static constexpr float GeometryEpsilon = 1.0e-10f;
		static constexpr float GeometryEpsilon = std::numeric_limits<float>::epsilon() * 10.f;
		static constexpr double NegativeGeometryEpsilon = -GeometryEpsilon;
		static constexpr float VertexPositionTolerance = 1.0e-5f;
		static constexpr int32_t ulpsEpsilon = 10;

		static constexpr float PI = 3.14159265f;
		static constexpr float DEG2RAD = PI / 180.f;
		static constexpr float RAD2DEG = 180.f / PI;

		static float degreesToRadians(float deg) {
			return DEG2RAD * deg;
		}

		static float radiansToDegrees(float rad) {
			return RAD2DEG * rad;
		}

		static bool GeometryIsNonZero(float a) {
			return !GeometryIsCloseToZero(a);
		}

		static bool GeometryIsCloseToZero(float a) {
			//return fabs(a) < GeometryEpsilon; // alternative !(a > ge) && !( a < -ge)
			return (a < GeometryEpsilon) && (a > NegativeGeometryEpsilon);
		}

		static bool GeometryIsGreaterThanZero(float a) {
			return a > GeometryEpsilon;
		}

		static bool GeometryIsLessThanZero(float a) {
			return a < NegativeGeometryEpsilon;
		}

		static bool GeometryAreEqual(float a, float b) {
			return GeometryIsCloseToZero(b - a);
		}
		// a > b
		static bool GeometryIsGreaterThan(float a, float b) {
			return b < (a - GeometryEpsilon);
		}

		bool NearlyEqual(float a, float b,
			float fixedEpsilon, int ulpsEpsilon)
		{
			// Handle the near-zero case.
			const float difference = fabs(a - b);
			if (difference <= fixedEpsilon) return true;

			return UlpsDistance(a, b) <= ulpsEpsilon;
		}

		static const int32_t int32SignBit = (int32_t)1 << 31;

		static int32_t FloatToNativeSignedUlps(float f)
		{
			int32_t i;
			memcpy(&i, &f, sizeof(float));

			// Positive values are the same in both
			// two's complement and signed magnitude.
			// For negative values, remove the sign bit
			// and negate the result (subtract from 0).
			return i >= 0 ? i : -(i & ~int32SignBit);
		}

		float NativeSignedUlpsToFloat(int32_t ulps)
		{
			if (ulps < 0) {
				ulps = -ulps;
				ulps |= int32SignBit;
			}
			float f;
			memcpy(&f, &ulps, sizeof(float));
			return f;
		}

		float UlpsIncrement(float f, int32_t ulps)
		{
			if (isnan(f) || isinf(f)) return f;
			int32_t i = FloatToNativeSignedUlps(f);
			i += ulps;
			return NativeSignedUlpsToFloat(i);
		}

		// http://bitbashing.io/comparing-floats.html
		int32_t UlpsDistance(const float a, const float b)
		{
			// We can skip all the following work if they're equal.
			if (a == b) return 0;

			constexpr auto max = std::numeric_limits<int32_t>::max();

			// We first check if the values are NaN.
			// If this is the case, they're inherently unequal;
			// return the maximum distance between the two.
			if (isnan(a) || isnan(b)) return max;

			// If one's infinite, and they're not equal,
			// return the max distance between the two.
			if (isinf(a) || isinf(b)) return max;

			// At this point we know that the floating-point values aren't equal and
			// aren't special values (infinity/NaN).
			// Because of how IEEE754 floats are laid out
			// (sign bit, then exponent, then mantissa), we can examine the bits
			// as if they were integers to get the distance between them in units
			// of least precision (ULPs).
			static_assert(sizeof(float) == sizeof(int32_t), "What size is float?");

			// memcpy to get around the strict aliasing rule.
			// The compiler knows what we're doing and will just transfer the float
			// values into integer registers.
			int32_t ia, ib;
			memcpy(&ia, &a, sizeof(float));
			memcpy(&ib, &b, sizeof(float));

			// If the signs of the two values aren't the same,
			// return the maximum distance between the two.
			// This is done to avoid integer overflow, and because the bit layout of
			// floats is closer to sign-magnitude than it is to two's complement.
			// This *also* means that if you're checking if a value is close to zero,
			// you should probably just use a fixed epsilon instead of this function.
			if ((ia < 0) != (ib < 0)) return max;

			// If we've satisfied all our caveats above, just subtract the values.
			// The result is the distance between the values in ULPs.
			int32_t distance = ia - ib;
			if (distance < 0) distance = -distance;
			return distance;
		}
	};
}

