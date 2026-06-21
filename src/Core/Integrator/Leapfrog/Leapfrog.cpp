#include "Leapfrog.hpp"

namespace Orbit {
    Leapfrog::Leapfrog() {}

    void Leapfrog::step(State& s, const AccelFunc& accel, double dt) const {
        glm::dvec3 x = s.getPosition();
        glm::dvec3 v = s.getVelocity();

        v += accel(x, v) * (0.5 * dt);
        x += v * dt;
        v += accel(x, v) * (0.5 * dt);

        s.setPosition(x);
        s.setVelocity(v);
    }
}