#pragma once

#include <glm/glm.hpp>

namespace Orbit {
    class State {
        public:
            State();
            ~State() = default;

            struct Derivative
            {
                glm::dvec3 dPosition;
                glm::dvec3 dVelocity;
            };

            void setPosition(const glm::dvec3& position);
            [[nodiscard]] const glm::dvec3& getPosition() const;

            void setVelocity(const glm::dvec3& velocity);
            [[nodiscard]] const glm::dvec3& getVelocity() const;

            void setMass(double mass);
            [[nodiscard]] const double& getMass() const;

        private:
            glm::dvec3 _position;
            glm::dvec3 _velocity;
            double _mass;
    };
}
