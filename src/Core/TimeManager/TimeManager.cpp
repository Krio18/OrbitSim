#include "TimeManager.hpp"

#include <algorithm>

namespace {
    constexpr long long days_from_civil(long long y, unsigned m, unsigned d) noexcept {
        y -= (m <= 2);
        const long long era = (y >= 0 ? y : y - 399) / 400;
        const unsigned yoe = static_cast<unsigned>(y - era * 400);
        const unsigned doy = (153u * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1;
        const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
        return era * 146097 + static_cast<long long>(doe) - 719468;
    }

    struct CivilDate { long long year; unsigned month; unsigned day; };

    constexpr CivilDate civil_from_days(long long z) noexcept {
        z += 719468;
        const long long era = (z >= 0 ? z : z - 146096) / 146097;
        const unsigned doe = static_cast<unsigned>(z - era * 146097);
        const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
        const long long y = static_cast<long long>(yoe) + era * 400;
        const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
        const unsigned mp = (5 * doy + 2) / 153;
        const unsigned d = doy - (153 * mp + 2) / 5 + 1;
        const unsigned m = mp < 10 ? mp + 3 : mp - 9;
        return CivilDate{ y + (m <= 2), m, d };
    }
}

namespace Orbit {
    TimeManager::TimeManager() :
        _state(State::Stopped), _warpLevel(1), _fps(0.0),
        _simTime(0.0), _deltaTime(0.0), _unscaledDeltaTime(0.0),
        _elapsedTime(0.0), _timeScale(1.0), _fixedDeltaTime(1.0 / 60.0),
        _lastFrameTime(std::chrono::steady_clock::now()) {}

    void TimeManager::update() noexcept {
        const auto now = std::chrono::steady_clock::now();
        double rawDeltaTime = std::chrono::duration<double>(now - this->_lastFrameTime).count();
        this->_lastFrameTime = now;

        rawDeltaTime = std::min(rawDeltaTime, MAX_DELTA_TIME);

        this->_unscaledDeltaTime = rawDeltaTime;
        this->_fps = (rawDeltaTime > 0.0) ? (1.0 / rawDeltaTime) : 0.0;

        if (this->_state == State::Stopped) {
            this->_deltaTime = 0.0;
            return;
        }

        this->_elapsedTime += rawDeltaTime;

        if (this->_state == State::Paused) {
            this->_deltaTime = 0.0;
            return;
        }

        this->_deltaTime = rawDeltaTime * this->_timeScale;
        this->_simTime  += this->_deltaTime * this->_warpLevel;
    }

    void TimeManager::start() noexcept {
        if (this->_state == State::Stopped) {
            resetClock();
            this->_state = State::Running;
        }
    }

    void TimeManager::pause() noexcept {
        if (this->_state == State::Running)
            this->_state = State::Paused;
    }

    void TimeManager::resume() noexcept {
        if (this->_state == State::Paused) {
            resetClock();
            this->_state = State::Running;
        }
    }

    void TimeManager::stop() noexcept {
        this->_state = State::Stopped;
        this->_deltaTime = 0.0;
    }

    void TimeManager::resetClock() noexcept {
        this->_lastFrameTime = std::chrono::steady_clock::now();
    }

    void TimeManager::setFixedDt(double dt) noexcept {
        if (this->_state == State::Stopped && dt > 0.0)
            this->_fixedDeltaTime = dt;
    }

    void TimeManager::setTimeScale(double scale) noexcept {
        if (scale >= 0.0)
            this->_timeScale = scale;
    }

    void TimeManager::setWarpTime(int level) noexcept {
        if (level > 0)
            this->_warpLevel = level;
    }

    TimeManager::State TimeManager::getState()  const noexcept { return _state; }
    bool TimeManager::isRunning() const noexcept { return _state == State::Running; }
    bool TimeManager::isPaused()  const noexcept { return _state == State::Paused; }

    int TimeManager::getWarpLevel() const noexcept { return this->_warpLevel; }
    double TimeManager::getSimTime() const noexcept { return this->_simTime; }
    double TimeManager::getFPS() const noexcept { return this->_fps; }
    double TimeManager::getDeltaTime() const noexcept { return this->_deltaTime; }
    double TimeManager::getUnscaledDeltaTime() const noexcept { return this->_unscaledDeltaTime; }
    double TimeManager::getElapsedTime() const noexcept { return this->_elapsedTime; }
    double TimeManager::getTimeScale() const noexcept { return this->_timeScale; }
    double TimeManager::getFixedDt() const noexcept { return this->_fixedDeltaTime; }

    TimeManager::Calendar TimeManager::getCalendar() const noexcept {
        constexpr long long EPOCH_DAYS = days_from_civil(2026, 1, 1);
        const long long totalDays = EPOCH_DAYS + static_cast<long long>(this->_simTime / 86400.0);
        const CivilDate date = civil_from_days(totalDays);

        return Calendar{
            static_cast<int>(date.day),
            static_cast<int>(date.month),
            static_cast<int>(date.year)
        };
    }
}