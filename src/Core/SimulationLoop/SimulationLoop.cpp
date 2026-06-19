#include "SimulationLoop.hpp"

namespace Orbit {
    SimulationLoop::SimulationLoop(double dt) : _dt(dt), _accumulator(0.0) {}

    double SimulationLoop::update(double elapsedTime) {
        this->_accumulator += elapsedTime;

        for (int i = 0; (this->_accumulator >= this->_dt && i < 8); i++) {
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
