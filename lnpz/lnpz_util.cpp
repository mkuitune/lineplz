// This file lnpz_util.cpp is part of lineplz - Public domain simple drawing library
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

#include "lnpz_util.h"

#include <vector>
#include <filesystem>
#include <iostream>
#include <fstream>

namespace lnpz {

	namespace util {

		std::string PreparePathForWriting(const std::string& pathStr) {
			std::filesystem::path path(pathStr);
			if (path.has_parent_path()) {
				if (!std::filesystem::exists(path.parent_path())) {
					std::error_code err;
					if (!std::filesystem::create_directories(path.parent_path(), err)) {
						std::ostringstream ostr;
						ostr << "preparePathForWriting:Could not create path '"
							<< path.parent_path().generic_string();
						ostr << "' with error code " << err;
						return ostr.str();
					}
				}
			}
			return "";
		}

		std::string WriteBytesToPath(const std::vector<uint8_t>& bytes, const std::string& pathStr)
		{
			using namespace std;
			auto pathres = PreparePathForWriting(pathStr);
			if (!pathres.empty())
				return pathres;

			ofstream fout;
			fout.open(pathStr, ios::binary | ios::out);
			fout.write((char*)bytes.data(), bytes.size());
			fout.close();
			return "";
		}

	}
}

