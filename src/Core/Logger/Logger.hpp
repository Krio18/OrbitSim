#pragma once

#include <string_view>

namespace Orbit {

    class Logger {
    public:
        enum class Level { Info, Warning, Error };

        static void setLevel(Level minLevel) noexcept;

        static void info   (std::string_view msg) noexcept;
        static void warning(std::string_view msg) noexcept;
        static void error  (std::string_view msg) noexcept;

    private:
        static void log(Level level, std::string_view msg) noexcept;

        static Level _minLevel;
    };

}

#include <cstdlib>
#include <string>

#ifndef NDEBUG
    #define SIM_ASSERT(expr, msg)                                               \
        do {                                                                    \
            if (!(expr)) {                                                      \
                ::Orbit::Logger::error(                                         \
                    std::string("Assert [") + __FILE__ + ":"                   \
                    + std::to_string(__LINE__) + "] " #expr " — " + (msg));    \
                std::abort();                                                   \
            }                                                                   \
        } while(0)
#else
    #define SIM_ASSERT(expr, msg) ((void)0)
#endif
