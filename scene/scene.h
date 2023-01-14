#ifndef SCENE_H
#define SCENE_H

#include <QString>

#include "BVH/BVH.h"

#include "basiccamera.h"

#include "util/CS123SceneData.h"

#include "shape/mesh.h"

#include <memory>
#include "../glm_fix.hxx"

class Scene
{
public:
    Scene();
    virtual ~Scene();

    static bool load(QString filename, Scene **scenePointer);

    void setBVH(const BVH &bvh);
    const BVH& getBVH() const;

    const BasicCamera& getCamera() const;

    void setCamera(const BasicCamera& camera);
    void setGlobalData(const CS123SceneGlobalData& data);
    void addLight(const CS123SceneLightData& data);

    const std::vector<CS123SceneLightData>& getLights();

    bool getIntersection(const _Ray& ray, IntersectionInfo* I) const;

public:

    BVH *m_bvh;
    std::vector<Object *> *_objects;

    BasicCamera m_camera;

    CS123SceneGlobalData m_globalData;

    std::vector<CS123SceneLightData> m_lights;

    static bool parseTree(CS123SceneNode *root, Scene *scene, const std::string& baseDir);
    static void parseNode(CS123SceneNode *node, const Eigen::Affine3f &parentTransform, std::vector<Object *> *objects, const std::string& baseDir);
    static void addPrimitive(CS123ScenePrimitive *prim, const Eigen::Affine3f &transform, std::vector<Object *> *objects, const std::string& baseDir);
    static Mesh *loadMesh(std::string filePath, const Eigen::Affine3f &transform, const std::string& baseDir);
};

namespace Ray {
    constexpr auto NoIntersection = std::numeric_limits<double>::infinity();
    inline auto SelfIntersectionDisplacement = 1e-3f;

    auto Reflect(auto&& IncomingDirection, auto&& SurfaceNormal) {
        return glm::normalize(IncomingDirection + 2 * glm::dot(SurfaceNormal, -IncomingDirection) * SurfaceNormal);
    }
    auto Refract(auto&& IncomingDirection, auto&& SurfaceNormal, auto η) {
        auto cosθ1 = glm::dot(-SurfaceNormal, IncomingDirection);
        if (auto Discriminant = 1 - η * η * (1 - cosθ1 * cosθ1); Discriminant < 0)
            return glm::vec3{};
        else
            return glm::normalize(η * IncomingDirection + (η * cosθ1 - std::sqrt(Discriminant)) * SurfaceNormal);
    }
    auto Intersect(SubtypeOf<glm::vec3> auto&& EyePoint, SubtypeOf<glm::vec3> auto&& RayDirection, SubtypeOf<Scene> auto&& ObjectRecords) {
        static auto dummy_mat = MaterialType{};

        auto i = IntersectionInfo{};
        auto p = Eigen::Vector3f{ EyePoint.x, EyePoint.y, EyePoint.z };
        auto d = Eigen::Vector3f{ RayDirection.x, RayDirection.y, RayDirection.z };

        if (auto ray = _Ray{ p, d }; ObjectRecords.getIntersection(ray, &i)) {
            auto t = static_cast<double>(i.t);
            auto tri = static_cast<const Triangle*>(i.data);
            auto& mat = tri->Material;
            auto n = tri->getNormal(i);
            return std::tuple{ t, glm::vec3{ n[0], n[1], n[2] } } + std::tuple<const MaterialType&>{ mat };
        }
        else
            return std::tuple{ NoIntersection, glm::vec3{} } + std::tuple<const MaterialType&>{ dummy_mat };
    }
}

#endif // SCENE_H
