#include <lnpz/lnpz.h>

namespace lnpz {

	typedef Array2D<RGBAFloat32> ImageRGBA32Linear;

	void Renderer::draw(const Scene& scene) {

		// Figure out framebuffer size
	}

	void Renderer::write(const std::string& pathOut) const{

	}

	std::string Renderer::getSRGBABytes() const {
		return {};
	}
}

