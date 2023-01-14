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

    auto to_glm = [](auto&& x) { return glm::vec3{ x[0], x[1], x[2] }; };

    using CameraType = struct {
        glm::vec3 Position;
        glm::vec3 Look;
        glm::vec3 Up;
        double HeightAngle;
        double FocalLength;
        double Aperture;
    };
    auto Camera = CameraType{
        .Position = to_glm(scene.m_camera.m_position),
        .Look = glm::normalize(to_glm(scene.m_camera.m_direction)),
        .Up = glm::normalize(to_glm(scene.m_camera.m_up)),
        .HeightAngle = glm::radians(scene.m_camera.m_heightAngle),
        .FocalLength = 3.2,
        .Aperture = 0.3
    };
    auto PixelAggregator = ViewPlane::ConfigurePixelAggregator(
        [&](auto&& eye, auto&& dir) { return Ray::Trace(eye, dir, scene); },
        ViewPlane::ConfigureRayCaster(Camera, IMAGE_WIDTH, IMAGE_HEIGHT),
        25000,
        0.5
    );

    std::vector<Vector3f> intensityValues(m_width * m_height);
    
    for(int y = 0; y < m_height; ++y) {
        //#pragma omp parallel for
        for(int x = 0; x < m_width; ++x) {
            std::cout << "x = " << x << " y = " << y << std::endl;
            int offset = x + (y * m_width);
            auto color = PixelAggregator(x, y);
            intensityValues[offset] = Vector3f{ color[0], color[1], color[2] };
        }
    }

    toneMap(imageData, intensityValues);
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