#include "SemiImplicitEuler.hpp"

namespace Orbit {
    SemiImplicitEuler::SemiImplicitEuler() {}

    void SemiImplicitEuler::step(State& s, const AccelFunc& accel, double dt) const {
        glm::dvec3 x = s.getPosition();
        glm::dvec3 v = s.getVelocity();

        v += accel(x, v) * dt;
        x += v * dt;

        s.setVelocity(v);
        s.setPosition(x);
    }
}
