#pragma once

#include "../State/State.hpp"

namespace Orbit {
    class SimulationLoop {
        public:
            SimulationLoop(double dt = 1.0/60.0);
            ~SimulationLoop() = default;

            double update(double elapsedTime);

            void setPrevousState(const State& previousState);

            [[nodiscard]] const State& getPreviousState() const;
            [[nodiscard]] const State& getCurrentState() const;

        private:
            double _dt;
            double _accumulator;
            State _previousState;
            State _currentState;
    };
}
