
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

