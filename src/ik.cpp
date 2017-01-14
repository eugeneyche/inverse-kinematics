#include "ik.hpp"
#include <cstdio>

std::vector<Joint> solve_ik(std::vector<Joint>& joints, const glm::vec3& target, float alpha, size_t n_iterations)
{
    std::vector<Joint> sofar_joints = joints;
    std::vector<glm::vec3> jacobian (joints.size());

    while (n_iterations--) {
        glm::vec3 sofar_position;
        glm::mat3 sofar_basis {1.f};
        for (size_t i = 0; i < sofar_joints.size(); i++) {
            const Joint& joint = sofar_joints[i];
            glm::vec3 axis = glm::normalize(sofar_basis * joint.axis);
            jacobian[i] = glm::cross(axis, target - sofar_position);
            sofar_basis = sofar_basis * glm::mat3_cast(glm::rotate(glm::quat{}, joint.angle, joint.axis));
            sofar_position += sofar_basis * joint.offset;
        }
        glm::vec3 error = target - sofar_position;
        float damp_alpha = alpha * glm::clamp(glm::length(error), 0.2f, 2.f);
        for (size_t i = 0; i < sofar_joints.size(); i++) {
            Joint& joint = sofar_joints[i];
            joint.angle += damp_alpha * glm::dot(jacobian[i], error);
        }
    }

    return sofar_joints;
}
