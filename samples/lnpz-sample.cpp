#include <lnpz/lnpz.h>
#include<iostream>

void drawSquare() {
	using namespace lnpz;
	SimpleBuilder builder;
	material_t mat;
	mat.lineColor = RGBAFloat32::Black();
	mat.faceColor = RGBAFloat32::Red();
	//mat.lineWidth = 15.f;
	mat.lineWidth = 10.f;
	builder.setMaterial(mat);

	//builder.addLineString({ {{0,0},{1,0}, {1,1}, {0,1}, {0,0}} });
	//std::vector<linestring_t> inner = { {{{0.5,0.4},{0.4,0.4},{0.45,0.6}}} };

	std::vector<linestring_t> inner = { {{{0.8,0.2},{0.2,0.2},{0.5,0.8}}} };
	//std::vector<linestring_t> inner;

	linestring_t outer = { {{0,0},{1,0}, {1,1}, {0,1}} };
	outer = outer.reversed();
	//linestring_t outer = { {   {1,0}, {0,0},{0,1}, {1,1}} };
	//std::vector<linestring_t> inner = { {{{0.2,0.2},{0.8,0.2},{0.5,0.8}}} };
	builder.addPolygonFace({ outer,inner});
	Scene scene = builder.build();

	SceneConfig sceneConfig;
	sceneConfig.background = RGBAFloat32::White();
	sceneConfig.outputHeightPixels = 256;
	sceneConfig.paddingInPixels = 50;
	Renderer renderer(sceneConfig);
	renderer.draw(scene);
	renderer.write("square.png");

}


int main(int arc, char* argv[]) {
	drawSquare();
	return 0;
}