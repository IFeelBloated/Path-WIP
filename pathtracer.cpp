#include "pathtracer.h"

#include <iostream>

#include <Eigen/Dense>

#include <util/CS123Common.h>

#include "glm_fix.hxx"
#include "Ray.hxx"

using namespace Eigen;

PathTracer::PathTracer(int width, int height)
    : m_width(width), m_height(height)
{
}

void PathTracer::traceScene(QRgb *imageData, const Scene& scene)
{
    std::vector<Vector3f> intensityValues(m_width * m_height);
    Matrix4f invViewMat = (scene.getCamera().getScaleMatrix() * scene.getCamera().getViewMatrix()).inverse();
    for(int y = 0; y < m_height; ++y) {
        //#pragma omp parallel for
        for(int x = 0; x < m_width; ++x) {
            int offset = x + (y * m_width);
            intensityValues[offset] = tracePixel(x, y, scene, invViewMat);
        }
    }

    toneMap(imageData, intensityValues);
}

Vector3f PathTracer::tracePixel(int x, int y, const Scene& scene, const Matrix4f &invViewMatrix)
{
    auto n_samples = 25000;
    static auto sigma = 0.5;

    static auto rd = std::random_device{};
    static auto gen = std::mt19937{ rd() };
    static auto randn = std::normal_distribution{ 0., sigma };

    Vector3f p(0, 0, 0);
    auto res = Vector3f{ 0, 0, 0 };
    auto w_sum = 0.;

    std::cout << "x = " << x << " y = " << y << std::endl;

    for (auto i = 0; i < n_samples; ++i) {
        auto xx = randn(gen);
        auto yy = randn(gen);
        auto w = std::exp(-(xx * xx + yy * yy) / (2 * sigma * sigma));

        w_sum += w;

        auto d = Vector3f{ (2.f * (x + xx) / m_width) - 1, 1 - (2.f * (y + yy) / m_height), -1 };
        d.normalize();

        auto r = _Ray(p, d);
        r = r.transform(invViewMatrix);

        auto intensity = traceRay(r, scene);
        res += intensity * w;
    }

    return res / w_sum;
}

Vector3f PathTracer::traceRay(const _Ray& r, const Scene& scene)
{
    auto eye = glm::vec3{ r.o[0], r.o[1], r.o[2] };
    auto dir = glm::vec3{ r.d[0], r.d[1], r.d[2] };
    auto color = Ray::Trace(eye, dir, scene);
    return Vector3f{ color[0], color[1], color[2] };
}

void PathTracer::toneMap(QRgb* imageData, std::vector<Vector3f>& intensityValues) {
    for (int y = 0; y < m_height; ++y)
        for (int x = 0; x < m_width; ++x) {
            int offset = x + (y * m_width);
            auto to_int = [](auto x) {
                return std::min(std::max(std::pow(x, 1. / 2.2) * 255., 0.), 255.);
            };
            auto r = to_int(intensityValues[offset][0] / (1 + intensityValues[offset][0]));
            auto g = to_int(intensityValues[offset][1] / (1 + intensityValues[offset][1]));
            auto b = to_int(intensityValues[offset][2] / (1 + intensityValues[offset][2]));
            imageData[offset] = qRgb(r, g, b);
        }
}