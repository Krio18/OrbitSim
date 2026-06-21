#pragma once

#include "../IIntegrator.hpp"

namespace Orbit {
    class Leapfrog : public IIntegrator {
        public:
            Leapfrog();
            ~Leapfrog() = default;

            void step(State& s, const AccelFunc& accel, double dt) const override;
        private:
    };
}
