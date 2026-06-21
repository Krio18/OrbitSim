#include "RK4.hpp"

namespace Orbit {
    RK4::RK4() {}

    void RK4::step(State& s, const AccelFunc& accel, double dt) const {
        const glm::dvec3 x0 = s.getPosition();
        const glm::dvec3 v0 = s.getVelocity();

        const State::Derivative k1{
            v0,
            accel(x0, v0)
        };
        const State::Derivative k2{
            v0 + k1.dVelocity * (0.5 * dt),
            accel(x0 + k1.dPosition * (0.5 * dt), v0 + k1.dVelocity * (0.5 * dt))
        };
        const State::Derivative k3{
            v0 + k2.dVelocity * (0.5 * dt),
            accel(x0 + k2.dPosition * (0.5 * dt), v0 + k2.dVelocity * (0.5 * dt))
        };
        const State::Derivative k4{
            v0 + k3.dVelocity * dt,
            accel(x0 + k3.dPosition * dt, v0 + k3.dVelocity * dt)
        };

        s.setPosition(x0 + (k1.dPosition + 2.0*(k2.dPosition + k3.dPosition) + k4.dPosition) * (dt / 6.0));
        s.setVelocity(v0 + (k1.dVelocity + 2.0*(k2.dVelocity + k3.dVelocity) + k4.dVelocity) * (dt / 6.0));
    }
}
