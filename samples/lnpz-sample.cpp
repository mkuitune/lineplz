#include <lnpz/lnpz.h>
#include<iostream>

void drawSquare() {
	using namespace lnpz;
	SimpleBuilder builder;
	builder.addLineString({ {{0,0},{0,1}, {1,1}, {0,1}, {0,0}} });
	builder.addPolygonFace({ {{{0,0},{0,1}, {1,1}, {0,1}}},{} });
	Scene scene = builder.build();
	Renderer renderer;
	renderer.draw(scene);
	renderer.write("square.png");

}


int main(int arc, char* argv[]) {
	drawSquare();
	return 0;
}