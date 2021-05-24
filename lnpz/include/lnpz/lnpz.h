// lineplz - Public domain simple drawing library
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
#include <limits>
#include <algorithm>
#include <stdint.h>

#include <lnpz/lnpz_linalgd.h>

namespace lnpz{
    typedef lnpz_linalg::double2 point_t;
    typedef lnpz_linalg::double3x3 matrix33_t;

    inline point_t ApplyToPoint(const matrix33_t& m, const point_t& p) {
        using namespace lnpz_linalg;
        auto r0 = m.row(0);
        auto r1 = m.row(1);
        double x = dot(r0.xy(), p)  + r0.z;
        double y = dot(r0.xy(), p) + r1.z;
        return { x,y };
    }

    inline point_t SmallestElements(const point_t& lhs, const point_t& rhs){
        double xelem = std::min(lhs.x, rhs.x);
        double yelem = std::min(lhs.y, rhs.y);
        return point_t(xelem, yelem);
    }
    
    inline point_t LargestElements(const point_t& lhs, const point_t& rhs){
        double xelem = std::max(lhs.x, rhs.x);
        double yelem = std::max(lhs.y, rhs.y);
        return point_t(xelem, yelem);
    }

    inline bool ElementsLargerThanOrEqual(const point_t& lhs, const point_t& rhs){
        return (lhs.x >= rhs.x) && (lhs.y >= rhs.y);
    }

struct rectangle_t {
    point_t minpoint;
    point_t maxpoint;

    bool valid() const noexcept{
        return ElementsLargerThanOrEqual(maxpoint, minpoint);
    }

};

struct color_t {
    double r, g, b,a;
    static constexpr color_t Black() { return {0., 0., 0., 1.}; }
    static constexpr color_t Blue() { return {0., 0., 1., 1.}; }
    static constexpr color_t Red() { return {1., 0., 0., 1.}; }
    static constexpr color_t Yellow() { return {1., 1., 0., 1.}; }
    static constexpr color_t Green() { return {0., 1., 0., 1.}; }
    static constexpr color_t White() { return {1., 1., 1., 1.}; }
};

// Single line is a linestring with two points
struct linestring_t{
    std::vector<point_t> points;
    
    rectangle_t boundingRectangle() const noexcept {
        using namespace std;
        rectangle_t r;
        constexpr double nMin = numeric_limits<double>::min();
        constexpr double nMax = numeric_limits<double>::max();
        r.minpoint = { nMax, nMax };
        r.maxpoint = { nMin, nMin };
        for (const auto& p : points) {
            r.minpoint = SmallestElements(r.minpoint, p);
            r.maxpoint = LargestElements(r.maxpoint, p);
        }

        return r;
    }
};

struct polygonface_t {
    linestring_t outerWire;
    std::vector<linestring_t> innerWires;
};

struct localToWorldTransform_t {
    lnpz_linalg::double2 position = {0,0};
    double rotation = 0.f; // radians
    double scale = 1.f;

    matrix33_t matrix() const {
        return lnpz_linalg::localToWorld2_matrix(position, scale, rotation);
    }
};

struct material_t{
    color_t color = color_t::Black();
    double lineWidth = 1.0; // pixels
};

enum class InstanceType {LineString, PolygonFace};

class Scene {
public:
    struct instance_t {
        localToWorldTransform_t localToWorld;
        material_t material;
        InstanceType type;
        size_t idx; // Index to definition
    };

    std::vector<linestring_t> m_lines;
    std::vector<polygonface_t> m_polygonFaces;
    std::vector<instance_t> m_instances;
};

#if 0
class SceneBuilder{

public:

    template<class X0_TYPE, class Y0_TYPE, class X1_TYPE, class Y1_TYPE>
    void addLine(const X0_TYPE& x0, const Y0_TYPE& y0, const X1_TYPE& x1, const Y1_TYPE& y1){
        point_t fst;
        point_t snd;
        fst.x = static_cast<double>(x0);
        fst.y = static_cast<double>(y0);
        snd.x = static_cast<double>(x1);
        snd.y = static_cast<double>(y1);
        m_lines.push_back({ {fst, snd} });
    }
};
#endif

class SimpleBuilder {
    Scene m_scene;
    material_t m_currentMaterial;

public:
    void setMaterial(const material_t& mat ){
        m_currentMaterial = mat;
    }

    void addLineString(const linestring_t& ls) {
        m_scene.m_lines.push_back(ls);
        auto idx = m_scene.m_lines.size() - 1;
        localToWorldTransform_t trf;
        m_scene.m_instances.push_back({trf, m_currentMaterial, InstanceType::LineString, idx});
    }
    
    void addPolygonFace(const polygonface_t& pf) {
        m_scene.m_polygonFaces.push_back(pf);
        auto idx = m_scene.m_polygonFaces.size() - 1;
        localToWorldTransform_t trf;
        m_scene.m_instances.push_back({trf, m_currentMaterial, InstanceType::PolygonFace, idx});
    }

    Scene build() const {
        return m_scene;
    }
};

struct SceneConfig {
    color_t background = color_t::White();
    uint32_t outputHeightPixels = 256;
    uint32_t paddingInPixels = 10; // padding around scene
};

class Renderer {
    SceneConfig m_sceneConfig;

public:
    Renderer() {}
    Renderer(SceneConfig sceneConfig):m_sceneConfig(sceneConfig) {}
    void setConfig() {}
    void draw(const Scene& scene);
    void write(const std::string& pathOut) const;
};



}