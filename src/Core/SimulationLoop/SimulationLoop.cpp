#include "SimulationLoop.hpp"

namespace Orbit {
    SimulationLoop::SimulationLoop(double dt) : _dt(dt), _accumulator(0.0) {}

    double SimulationLoop::update(double elapsedTime, std::function<void(double)> physicsStep) {
        this->_accumulator += elapsedTime;
        if (this->_accumulator > this->_dt * 8.0)
            this->_accumulator = this->_dt * 8.0;

        for (int i = 0; (this->_accumulator >= this->_dt && i < 8); i++) {
            if (physicsStep) physicsStep(this->_dt);
            this->_accumulator -= this->_dt;
        }

        double alpha = this->_accumulator / this->_dt;
        return alpha;
    }

    void SimulationLoop::setPrevousState(const State& previousState) {
        this->_previousState = previousState;
    }

    const State& SimulationLoop::getPreviousState() const {
        return this->_previousState;
    }

    const State& SimulationLoop::getCurrentState() const {
        return this->_currentState;
    }
}
