#pragma once

#include "../IIntegrator.hpp"

namespace Orbit {
    class VelocityVerlet : public IIntegrator {
        public:
            VelocityVerlet();
            ~VelocityVerlet() = default;

            void step(State& s, const AccelFunc& accel, double dt) const override;
        private:
    };
}
