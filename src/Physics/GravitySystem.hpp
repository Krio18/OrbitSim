#pragma once

#include <glm/glm.hpp>
#include "Core/Integrator/IIntegrator.hpp"

namespace Orbit {

    class GravitySystem {
        public:
            explicit GravitySystem(double mu, glm::dvec3 bodyPos = {0.0, 0.0, 0.0});

            [[nodiscard]] AccelFunc makeAccelFunc() const;
            [[nodiscard]] double getMu() const;
            [[nodiscard]] const glm::dvec3& getBodyPosition() const;

        private:
            double _mu;
            glm::dvec3 _bodyPos;
    };

}
