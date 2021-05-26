// This file lnpz_array2d.h is part of lineplz - Public domain simple drawing library
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

namespace lnpz {

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
}
