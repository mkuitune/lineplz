#include <lnpz/lnpz2d.h>
#include "../lnpz/lnpz_util.h"
#include<iostream>
#include<map>
#include<vector>
#include<fstream>

#define STB_TRUETYPE_IMPLEMENTATION
#include "../stb/stb_truetype.h"

namespace lnpz {


	void AdaptiveOut(const Scene& scene, const std::string& name, RGBAFloat32 back, int height) {
		SceneConfigAdaptive sceneConfig;
		sceneConfig.background = back;
		sceneConfig.outputHeightPixels = height;
		sceneConfig.paddingInPixels = 50;

		Renderer2S renderer(sceneConfig);
		renderer.draw(scene);
		renderer.write(name);
	}

	void FixedOut(const Scene& scene, const std::string& name, SceneConfigFixed sceneConfig) {
		Renderer2S renderer(sceneConfig);
		renderer.draw(scene);
		renderer.write(name);
	}

	struct fontdata {
		const unsigned char* buffer;
		stbtt_fontinfo fontInfo;
	};

	struct GlyphPoly {
		std::vector<polygonface_t> polyfaces;
		bool empty() const noexcept { return polyfaces.empty(); }
	};


	point2_t bezier(const point2_t& P0, const point2_t& P1, const point2_t& P2, double t) {
		double x = (1 - t) * (1 - t) * P0.x + 2 * (1 - t) * t * P1.x + t * t * P2.x;
		double y = (1 - t) * (1 - t) * P0.y + 2 * (1 - t) * t * P1.y + t * t * P2.y;
		return { x, y };
	}

	std::vector<point2_t> tessellateBezier(const point2_t& P0, const point2_t& P1, const point2_t& P2, int n) {
		std::vector<point2_t> points;
		for (int i = 1; i <= n; ++i) {
			double t = static_cast<double>(i) / n;
			points.push_back(bezier(P0, P1, P2, t));
		}
		return points;
	}
	double computeSignedArea(const std::vector<point2_t>& points) {
		double area = 0.0;
		int n = points.size();

		for (int i = 0; i < n; ++i) {
			double x1 = points[i].x;
			double y1 = points[i].y;
			double x2 = points[(i + 1) % n].x;  // Wrap around to the start for the last point
			double y2 = points[(i + 1) % n].y;

			area += (x1 * y2 - x2 * y1);
		}

		return area * 0.5;
	}

	bool isOuterWire(const std::vector<point2_t>& points) {
		return computeSignedArea(points) < 0;
	}

	GlyphPoly TryGetGlyphface(const fontdata& fdata, int codepoint) {
		//std::map<>
			// Get just glyph for a
		int glyphIndex = stbtt_FindGlyphIndex(&fdata.fontInfo, codepoint);
		stbtt_vertex* vertices;
		int numVertices = stbtt_GetGlyphShape(&fdata.fontInfo, glyphIndex, &vertices);

		enum class VertType { Move, Line, Quadratic };

		GlyphPoly polyOut;
		polygonface_t curPoly;
		linestring_t current;

		point2_t pos;
		for (int i = 0; i < numVertices; ++i) {
			switch (vertices[i].type) {
			case STBTT_vmove:
				if (!current.points.empty()) {
					if (isOuterWire(current.points)) {
						if (curPoly.outerWire.points.empty()) {
							curPoly.outerWire = current;
						}
						else {
							polyOut.polyfaces.push_back(curPoly);
							curPoly = { current, {} };
						}
					}
					else {
						curPoly.innerWires.push_back(current);
					}

					current.points.clear();
				}
				current.points.push_back({ static_cast<double>(vertices[i].x), static_cast<double>(vertices[i].y) });
				break;

			case STBTT_vline:
				current.points.push_back({ static_cast<double>(vertices[i].x), static_cast<double>(vertices[i].y) });
				break;

			//case STBTT_vcurve:
			//	point2_t P0 = current.points.back();
			//	point2_t P1 = { static_cast<double>(vertices[i].cx), static_cast<double>(vertices[i].cy) };
			//	point2_t P2 = { static_cast<double>(vertices[i + 1].x), static_cast<double>(vertices[i + 1].y) };

			//	int n = 4;
			//	std::vector<point2_t> curvePoints = tessellateBezier(P0, P1, P2, n);
			//	current.points.insert(current.points.end(), curvePoints.begin(), curvePoints.end());

			//	++i;
			//	break;
			case STBTT_vcurve:
				point2_t P0 = current.points.back();
				point2_t P1 = { static_cast<double>(vertices[i].cx), static_cast<double>(vertices[i].cy) };
				point2_t P2 = { static_cast<double>(vertices[i].x), static_cast<double>(vertices[i].y) };  // Use the current vertex as the end point

				int n = 10;
				std::vector<point2_t> curvePoints = tessellateBezier(P0, P1, P2, n);
				current.points.insert(current.points.end(), curvePoints.begin(), curvePoints.end());

				// Do not increment i here
				break;
				// ... (handle other vertex types if needed)
			}
		}
		
		if (!current.points.empty()) {
			if (isOuterWire(current.points)) {
				if (curPoly.outerWire.points.empty()) {
					curPoly.outerWire = current;
				}
				else {
					polyOut.polyfaces.push_back(curPoly);
					curPoly = { current, {} };
				}
			}
			else {
				curPoly.innerWires.push_back(current);
			}
		}

		if (!curPoly.outerWire.points.empty()) {
			polyOut.polyfaces.push_back(curPoly);
		}

		// Clean up
		stbtt_FreeShape(&fdata.fontInfo, vertices);

		return polyOut;
	}

	GlyphPoly LayoytString(const fontdata& fdata, const std::string& text) {
		std::vector<polygonface_t> output;
		float cursorX = 0.0f;
		int prevGlyphIndex = -1;

		for (char c : text) {
			int codepoint = static_cast<int>(c);
			GlyphPoly glyphPoly = TryGetGlyphface(fdata, codepoint);
			int glyphIndex = stbtt_FindGlyphIndex(&fdata.fontInfo, codepoint);

			// Adjust for kerning if the previous glyph exists
			if (prevGlyphIndex >= 0) {
				cursorX += stbtt_GetGlyphKernAdvance(&fdata.fontInfo, prevGlyphIndex, glyphIndex);
			}

			// Adjust each point in the glyph by the current cursor position
			for (polygonface_t& poly : glyphPoly.polyfaces) {
				for (point2_t& pt : poly.outerWire.points) {
					pt.x += cursorX;
				}
				for (linestring_t& inner : poly.innerWires) {
					for (point2_t& pt : inner.points) {
						pt.x += cursorX;
					}
				}
				output.push_back(poly);
			}

			// Move the cursor by the advance width of the glyph
			int advanceWidth, leftBearing;
			stbtt_GetGlyphHMetrics(&fdata.fontInfo, glyphIndex, &advanceWidth, &leftBearing);
			cursorX += advanceWidth;

			prevGlyphIndex = glyphIndex;
		}

		return { output };
	}

	GlyphPoly GetFontData(char glyph) {
		auto font = getDbgFont();
		fontdata fdata;
		fdata.buffer = font.first;
		stbtt_InitFont(&fdata.fontInfo, &fdata.buffer[0], 0);
		return TryGetGlyphface(fdata, glyph);
	}

	GlyphPoly GetFontDataString(const std::string& str) {
		auto font = getDbgFont();
		fontdata fdata;
		fdata.buffer = font.first;
		stbtt_InitFont(&fdata.fontInfo, &fdata.buffer[0], 0);
		return LayoytString(fdata, str);
	}

	double computeMaxY(const std::vector<polygonface_t>& polygons) {
		double maxY = -std::numeric_limits<double>::infinity();
		for (const auto& poly : polygons) {
			for (const auto& pt : poly.outerWire.points) {
				if (pt.y > maxY) {
					maxY = pt.y;
				}
			}
			for (const auto& hole : poly.innerWires) {
				for (const auto& pt : hole.points) {
					if (pt.y > maxY) {
						maxY = pt.y;
					}
				}
			}
		}
		return maxY;
	}

	void WriteSvg(const std::vector<polygonface_t>& polygons, const std::string& filename) {
		std::ofstream ofs(filename);
		if (!ofs.is_open()) {
			std::cerr << "Failed to open file: " << filename << std::endl;
			return;
		}

		double maxY = computeMaxY(polygons);

		// SVG header
		ofs << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n";
		ofs << "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\">\n";

		for (const auto& poly : polygons) {
			// Write outer wire
			ofs << "<path d=\"M ";
			for (const auto& pt : poly.outerWire.points) {
				ofs << pt.x << "," << (maxY - pt.y) << " ";  // Flip the y-coordinate
			}
			ofs << "Z\" fill=\"none\" stroke=\"black\"/>\n";  // Close path and set style

			// Write inner wires (holes)
			for (const auto& hole : poly.innerWires) {
				ofs << "<path d=\"M ";
				for (const auto& pt : hole.points) {
					ofs << pt.x << "," << (maxY - pt.y) << " ";  // Flip the y-coordinate
				}
				ofs << "Z\" fill=\"none\" stroke=\"black\"/>\n";  // Close path and set style
			}
		}

		// SVG footer
		ofs << "</svg>\n";
		ofs.close();
	}

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
			point2_t pnt = { c.radius * cos(angle), c.radius * sin(angle) };
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
		return { {p,p + pw, p + pw + ph,p + ph} };
	}
}



void testFont() {
	using namespace lnpz;

	material_t mat;
	mat.lineColor = RGBAFloat32::Black();
	mat.faceColor = RGBAFloat32::VenetianRed();
	mat.lineWidth = 1.5f;

	//GlyphPoly gp = GetFontDataString("The spry fox runs.");
	//GlyphPoly gp = GetFontDataString("o");
	{
		GlyphPoly gp = GetFontData('o');
		SimpleBuilder builder;
		builder.setMaterial(mat);
		builder.addPolygonFaces(gp.polyfaces);
		Scene scene = builder.build();

		AdaptiveOut(scene, "testFont.png", RGBAFloat32::White(), 256);
	}
	{
		GlyphPoly gp = GetFontData('g');
		SimpleBuilder builder;
		builder.setMaterial(mat);

		builder.addPolygonFaces(gp.polyfaces);
		Scene scene = builder.build();

		AdaptiveOut(scene, "testFont2.png", RGBAFloat32::White(), 256);
	}
	{
		GlyphPoly gp = GetFontData('B');
		SimpleBuilder builder;
		builder.setMaterial(mat);

		builder.addPolygonFaces(gp.polyfaces);
		Scene scene = builder.build();

		AdaptiveOut(scene, "testFont3.png", RGBAFloat32::White(), 256);
	}
	{
		GlyphPoly gp = GetFontData('e');
		SimpleBuilder builder;
		builder.setMaterial(mat);

		builder.addPolygonFaces(gp.polyfaces);
		Scene scene = builder.build();
		WriteSvg(gp.polyfaces, "e.svg");;
		AdaptiveOut(scene, "testFont4.png", RGBAFloat32::White(), 256);
	}
	{
		GlyphPoly gp = GetFontDataString("The spry fox.");
		SimpleBuilder builder;
		builder.setMaterial(mat);

		builder.addPolygonFaces(gp.polyfaces);
		Scene scene = builder.build();

		AdaptiveOut(scene, "testFont5.png", RGBAFloat32::White(), 256);
	}
	auto c1 = mk::circ(100,100, 100);
	auto c2 = mk::circ(100, 100, 90);
	c2 = c2.reversed();
	polygonface_t pf = { c1, {c2} };
	{
		SimpleBuilder builder;
		material_t mat;
		builder.setMaterial(mat);
		builder.addPolygonFace(pf);
		Scene scene = builder.build();

		AdaptiveOut(scene, "testFontRef.png", RGBAFloat32::White(), 256);
	}

}

void positionScene() {
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
	builder.addPolygonFace({ outer,inner });
	Scene scene = builder.build();

	SceneConfigFixed fixed = SceneConfigFixed::FitSceneToCenter(scene.getWorldBounds(), 10, 512, 256);
	fixed.background = RGBAFloat32::White();
	
	FixedOut(scene, "positionSceneFixed.png", fixed);

	AdaptiveOut(scene, "positionScene.png", RGBAFloat32::White(), 256);
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

	AdaptiveOut(scene, "square.png", RGBAFloat32::White(), 256);
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

	AdaptiveOut(scene, "dots1.png", RGBAFloat32::White(), 756);
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

	SceneConfigAdaptive sceneConfig;
	sceneConfig.background = RGBAFloat32::White();
	sceneConfig.outputHeightPixels = 782;
	//sceneConfig.paddingInPixels = 32;
	sceneConfig.paddingInPixels = 10;
	Renderer2S renderer(sceneConfig);
	renderer.draw(scene);
	renderer.write("rects1.png");

}

int main(int arc, char* argv[]) {
	positionScene();
	testFont();
	drawSquare();
	dots1();
	rects1();
	return 0;
}

