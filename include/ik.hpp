#pragma once
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

struct Joint
{
    glm::vec3 offset {0.f, 1.f, 0.f};
    glm::vec3 axis {1.f, 0.f, 0.f};
    float angle {0.f};
};

std::vector<Joint> solve_ik(std::vector<Joint>& joints, const glm::vec3& target, float alpha = 0.5f, size_t n_iterations = 1);
