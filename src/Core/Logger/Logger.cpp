#include "Logger.hpp"

#include <iostream>

namespace Orbit {

    Logger::Level Logger::_minLevel = Logger::Level::Info;

    void Logger::setLevel(Level minLevel) noexcept {
        _minLevel = minLevel;
    }

    void Logger::info(std::string_view msg) noexcept {
        log(Level::Info, msg);
    }

    void Logger::warning(std::string_view msg) noexcept {
        log(Level::Warning, msg);
    }

    void Logger::error(std::string_view msg) noexcept {
        log(Level::Error, msg);
    }

    void Logger::log(Level level, std::string_view msg) noexcept {
        if (level < _minLevel)
            return;

        switch (level) {
            case Level::Info:    std::cout << "[INFO]    " << msg << '\n'; break;
            case Level::Warning: std::cout << "[WARNING] " << msg << '\n'; break;
            case Level::Error:   std::cerr << "[ERROR]   " << msg << '\n'; break;
        }
    }

}
