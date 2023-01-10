#pragma once
#include "Infrastructure.hxx"
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/transform.hpp"
#include "glm/gtx/quaternion.hpp"

auto operator*(std::floating_point auto x, AnyOf<glm::vec3, glm::vec4> auto&& y) requires requires {
	{ y[0] }->AnyBut<decltype(x)>;
} {
	return static_cast<float>(x) * y;
}

auto operator/(AnyOf<glm::vec3, glm::vec4> auto&& x, std::floating_point auto y) requires requires {
	{ x[0] }->AnyBut<decltype(y)>;
} {
	return x / static_cast<float>(y);
}