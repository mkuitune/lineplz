#include <lnpz/lnpz2d.h>
#include<iostream>

namespace lnpz {
	void simpleOut(const Scene& scene, const std::string& name, RGBAFloat32 back, int height) {
		SceneConfig sceneConfig;
		sceneConfig.background = back;
		sceneConfig.outputHeightPixels = height;
		sceneConfig.paddingInPixels = 50;

		Renderer2S renderer(sceneConfig);
		renderer.draw(scene);
		renderer.write(name);
	}
}

void drawSquare() {
	using namespace lnpz;
	SimpleBuilder builder;
	material_t mat;
	mat.lineColor = RGBAFloat32::Black();
	mat.faceColor = RGBAFloat32::Red();
	mat.lineWidth = 1.5f;
	builder.setMaterial(mat);

	std::vector<linestring_t> inner = { {{{0.8,0.2},{0.2,0.2},{0.5,0.8}}} };

	linestring_t outer = { {{0,0},{1,0}, {1,1}, {0,1}} };
	outer = outer.reversed();
	builder.addPolygonFace({ outer,inner});
	Scene scene = builder.build();

	simpleOut(scene, "square.png", RGBAFloat32::White(), 256);
}

namespace mk {
	struct Circle {
		lnpz::point2_t origin;
		double radius;
	};

	lnpz::linestring_t tessellate(Circle c, int tessellation = 48) {
		using namespace lnpz;
		linestring_t out;
		double delta = 2.0 * 3.14159 / tessellation;
		for (int i = 0; i < tessellation; i++) {
			double angle = i * delta;
			point2_t pnt = {c.radius * cos(angle), c.radius * sin(angle)};
			out.points.push_back(pnt + c.origin);
		}
		return out;
	}

	lnpz::linestring_t circ(double x, double y, double radius, int tessellation = 48) {
		return tessellate(Circle{ {x,y},radius }, tessellation);
	}
	
	lnpz::linestring_t rect(lnpz::point2_t p, double w, double h) {
		using namespace lnpz;
		point2_t pw = { w,0 };
		point2_t ph = { 0,h };
		return { {p,p + pw, p + pw + ph,p + ph}};
	}
}

void dots1() {
	using namespace lnpz;
	SimpleBuilder builder;
	material_t mat;
	mat.lineColor = RGBAFloat32::Black();
	mat.faceColor = {0.6f, 0.05f, 0.05f, 1.0f};
	mat.lineWidth = 1.5f;
	
	material_t matShad;
	matShad.lineColor.a = 0;
	matShad.faceColor = RGBAFloat32::Create(0.1);
	matShad.faceColor.a = 0.8;
	matShad.lineWidth = 1.5f;

	double r0 = 6.5;

	auto mkCirc = [&](double x, double y) {
		int tess = 5;
		builder.setMaterial(matShad);
		builder.addPolygonFace({ mk::circ(x-0.5,y-0.5,r0, tess) ,{} });
		builder.setMaterial(mat);
		builder.addPolygonFace({ mk::circ(x,y,r0, tess) ,{} });
	};

	mkCirc(0,0);
	mkCirc(10,0);
	mkCirc(10,10);
	mkCirc(0,10);

	Scene scene = builder.build();

	simpleOut(scene, "dots1.png", RGBAFloat32::White(), 756);
}

void rects1() {
	using namespace lnpz;

	SimpleBuilder builder;
	material_t mat;
	mat.lineColor = RGBAFloat32::Black();
	mat.faceColor = { 0.8f, 0.05f, 0.05f, 1.0f };
	mat.lineWidth = 1.5f;

	material_t matShad;
	matShad.lineColor.a = 0;
	matShad.faceColor = RGBAFloat32::Create(0.1);
	matShad.faceColor.a = 0.8;
	matShad.lineWidth = 1.5f;

	int w = 5;
	int h = 5;
	point2_t d = { -10, -10 };
	//point2_t d = { 0, 0 };
	auto mkRect = [&](point2_t p) {
		builder.setMaterial(mat);
		builder.addPolygonFace({ mk::rect(p + d,w,h) ,{}});
	};

	mkRect({ 0, 0 });
	mkRect({ 10, 0 });
	mkRect({ 10, 10 });
	mkRect({ 0, 10 });

	Scene scene = builder.build();

	SceneConfig sceneConfig;
	sceneConfig.background = RGBAFloat32::White();
	sceneConfig.outputHeightPixels = 782;
	//sceneConfig.paddingInPixels = 32;
	sceneConfig.paddingInPixels = 10;
	Renderer2S renderer(sceneConfig);
	renderer.draw(scene);
	renderer.write("rects1.png");

}

int main(int arc, char* argv[]) {
	drawSquare();
	dots1();
	rects1();
	return 0;
}

