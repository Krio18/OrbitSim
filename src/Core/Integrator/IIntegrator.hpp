#pragma once

#include <functional>
#include <glm/glm.hpp>

#include "../State/State.hpp"

namespace Orbit {
    using AccelFunc = std::function<glm::dvec3(const glm::dvec3& position,
                                               const glm::dvec3& velocity)>;
    class IIntegrator {
        public:
            virtual ~IIntegrator() = default;

            virtual void step(State& state, const AccelFunc& accel, double dt) const = 0;
    };
}
