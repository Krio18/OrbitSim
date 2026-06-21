#pragma once

#include <chrono>

namespace Orbit {
    class TimeManager {
        public:
            enum class State {
                Stopped,
                Running,
                Paused
            };

            TimeManager();
            ~TimeManager() = default;

            void update() noexcept;

            void start() noexcept;
            void pause() noexcept;
            void resume() noexcept;
            void stop() noexcept;

            void setFixedDt(double dt) noexcept;
            void setTimeScale(double scale) noexcept;
            void setWarpTime(int level) noexcept;

            [[nodiscard]] State getState() const noexcept;
            [[nodiscard]] bool isRunning() const noexcept;
            [[nodiscard]] bool isPaused() const noexcept;

            [[nodiscard]] int getWarpLevel() const noexcept;
            [[nodiscard]] double getSimTime() const noexcept;
            [[nodiscard]] double getFPS() const noexcept;
            [[nodiscard]] double getDeltaTime() const noexcept;
            [[nodiscard]] double getUnscaledDeltaTime() const noexcept;
            [[nodiscard]] double getElapsedTime() const noexcept;
            [[nodiscard]] double getTimeScale() const noexcept;
            [[nodiscard]] double getFixedDt() const noexcept;

            struct Calendar {
                int days;
                int month;
                int years;
            };
            [[nodiscard]] Calendar getCalendar() const noexcept;

        private:
            void resetClock() noexcept;

            static constexpr double MAX_DELTA_TIME = 0.1;

            State _state;
            int _warpLevel;
            double _fps;
            double _simTime;
            double _deltaTime;
            double _unscaledDeltaTime;
            double _elapsedTime;
            double _timeScale;
            double _fixedDeltaTime;
            std::chrono::steady_clock::time_point _lastFrameTime;
    };
}