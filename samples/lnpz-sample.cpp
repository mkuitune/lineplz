#include <lnpz/lnpz.h>
#include<iostream>

void drawSquare() {
	using namespace lnpz;
	SimpleBuilder builder;
	material_t mat;
	mat.color = RGBAFloat32::Black();
	mat.lineWidth = 3.f;
	builder.setMaterial(mat);
	builder.addLineString({ {{0,0},{1,0}, {1,1}, {0,1}, {0,0}} });
	builder.addPolygonFace({ {{{0,0},{1,0}, {1,1}, {0,1}}},{} });
	Scene scene = builder.build();

	SceneConfig sceneConfig;
	sceneConfig.outputHeightPixels = 256;
	sceneConfig.paddingInPixels = 5;
	Renderer renderer(sceneConfig);
	renderer.draw(scene);
	renderer.write("square.png");

}


int main(int arc, char* argv[]) {
	drawSquare();
	return 0;
}