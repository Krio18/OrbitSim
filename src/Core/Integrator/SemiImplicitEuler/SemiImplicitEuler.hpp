#pragma once

#include "../IIntegrator.hpp"

namespace Orbit {
    class SemiImplicitEuler : public IIntegrator {
        public:
            SemiImplicitEuler();
            ~SemiImplicitEuler() = default;

            void step(State& s, const AccelFunc& accel, double dt) const override;
        private:
    };
}
