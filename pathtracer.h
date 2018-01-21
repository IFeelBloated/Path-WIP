#ifndef PATHTRACER_H
#define PATHTRACER_H

#include <QImage>

#include "scene/scene.h"

class PathTracer
{
public:
    PathTracer(int width, int height);

    void traceScene(QRgb *imageData, const Scene &scene);

private:
    int m_width, m_height;

    QRgb tracePixel(int x, int y, const Scene &scene, const Eigen::Matrix4f &invViewMatrix);

    QRgb traceRay(const Ray& r, const Scene &scene, const Eigen::Matrix4f &invViewMatrix);
};

#endif // PATHTRACER_H
