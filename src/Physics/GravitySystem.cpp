#include "GravitySystem.hpp"

#include <glm/geometric.hpp>

namespace Orbit {

    GravitySystem::GravitySystem(double mu, glm::dvec3 bodyPos)
        : _mu(mu), _bodyPos(bodyPos) {}

    AccelFunc GravitySystem::makeAccelFunc() const {
        return [mu = _mu, bodyPos = _bodyPos](const glm::dvec3& pos, const glm::dvec3&) -> glm::dvec3 {
            const glm::dvec3 r = pos - bodyPos;
            const double dist = glm::length(r);
            if (dist < 1.0)
                return glm::dvec3(0.0);
            return -(mu / (dist * dist)) * glm::normalize(r);
        };
    }

    double GravitySystem::getMu() const {
        return _mu;
    }

    const glm::dvec3& GravitySystem::getBodyPosition() const {
        return _bodyPos;
    }

}
