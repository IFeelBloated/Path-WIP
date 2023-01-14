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

    void toneMap(QRgb *imageData, std::vector<Eigen::Vector3f> &intensityValues);
};

#endif // PATHTRACER_H
