#ifndef Ray_h
#define Ray_h

#include "Eigen/Dense"

#include "util/CS123Common.h"

struct _Ray {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    _Ray(const Eigen::Vector3f o, const Eigen::Vector3f d)
        :o(o), d(d.normalized()), inv_d(Eigen::Vector3f(1, 1, 1).cwiseQuotient(d).normalized()){}

    _Ray transform(Eigen::Matrix4f mat) const {
        Eigen::Vector4f oo = mat * vec3Tovec4(o, 1);
        Eigen::Vector4f od = mat * vec3Tovec4(d, 0);
        return _Ray(oo.head<3>(), od.head<3>());
    }

    _Ray transform(Eigen::Affine3f transform) const {
        Eigen::Vector3f oo = transform * o;
        Eigen::Vector3f od = transform.linear().inverse().transpose() * d;
        return _Ray(oo, od);
    }

  Eigen::Vector3f o; // Ray Origin
  Eigen::Vector3f d; // Ray Direction
  Eigen::Vector3f inv_d; // Inverse of each Ray Direction component
};

#endif
