#include "VelocityVerlet.hpp"

namespace Orbit {
    VelocityVerlet::VelocityVerlet() {}

    void VelocityVerlet::step(State& s, const AccelFunc& accel, double dt) const {
        glm::dvec3 x = s.getPosition();
        glm::dvec3 v = s.getVelocity();

        const glm::dvec3 a0 = accel(x, v);
        x += v * dt + a0 * (0.5 * dt * dt);

        const glm::dvec3 vPredicted = v + a0 * dt;
        const glm::dvec3 a1 = accel(x, vPredicted);
        v += (a0 + a1) * (0.5 * dt);

        s.setPosition(x);
        s.setVelocity(v);
    }
}
