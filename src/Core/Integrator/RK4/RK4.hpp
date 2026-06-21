#pragma once

#include "../IIntegrator.hpp"

namespace Orbit {
    class RK4 : public IIntegrator {
        public:
            RK4();
            ~RK4() = default;

            void step(State& s, const AccelFunc& accel, double dt) const override;
        private:
    };
}
