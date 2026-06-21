#include "State.hpp"

namespace Orbit {
    State::State() : _position(0.0), _velocity(0.0), _mass(1.0) {}

    void State::setPosition(const glm::dvec3& position) {
        this->_position = position;
    }

    const glm::dvec3& State::getPosition() const {
        return this->_position;
    }

    void State::setVelocity(const glm::dvec3& velocity) {
        this->_velocity = velocity;
    }

    const glm::dvec3& State::getVelocity() const {
        return this->_velocity;
    }

    void State::setMass(double mass) {
        this->_mass = mass;
    }

    const double& State::getMass() const {
        return this->_mass;
    }
}