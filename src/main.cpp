#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include <glm/glm.hpp>

#include "Core/Constants.hpp"
#include "Core/Logger/Logger.hpp"
#include "Core/TimeManager/TimeManager.hpp"
#include "Core/SimulationLoop/SimulationLoop.hpp"
#include "Core/State/State.hpp"
#include "Core/Integrator/IIntegrator.hpp"
#include "Core/Integrator/SemiImplicitEuler/SemiImplicitEuler.hpp"
#include "Core/Integrator/VelocityVerlet/VelocityVerlet.hpp"
#include "Core/Integrator/RK4/RK4.hpp"
#include "Core/Integrator/Leapfrog/Leapfrog.hpp"

using Clock = std::chrono::steady_clock;

namespace {

    std::string fmtDouble(double v, int prec = 3) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(prec) << v;
        return ss.str();
    }

    std::string fmtVec(const glm::dvec3& v) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2)
           << "(" << v.x << ", " << v.y << ", " << v.z << ")";
        return ss.str();
    }

    Orbit::AccelFunc earthGravity() {
        return [](const glm::dvec3& pos, const glm::dvec3&) -> glm::dvec3 {
            const double r = glm::length(pos);
            if (r < 1.0) return glm::dvec3(0.0);
            return -(Orbit::Constants::MU_EARTH / (r * r)) * glm::normalize(pos);
        };
    }

    void testIntegrator(const std::string& name, Orbit::IIntegrator& integrator) {
        Orbit::State s;
        s.setPosition({7'000'000.0, 0.0, 0.0});
        s.setVelocity({0.0, 7'500.0, 0.0});

        const glm::dvec3 posBefore = s.getPosition();
        integrator.step(s, earthGravity(), 1.0 / 60.0);

        Orbit::Logger::info(name
            + " | avant=" + fmtVec(posBefore)
            + " → après=" + fmtVec(s.getPosition()));
    }

}

int main() {
    Orbit::Logger::info("OrbitSim v0.1.0 — Phase 0.x Integration Test");
    Orbit::Logger::info("==============================================");

    Orbit::Logger::info("");
    Orbit::Logger::info("=== State ===");
    {
        Orbit::State s;
        s.setPosition({1.0, 2.0, 3.0});
        s.setVelocity({4.0, 5.0, 6.0});
        s.setMass(1000.0);

        SIM_ASSERT(s.getPosition() == glm::dvec3(1.0, 2.0, 3.0), "position incorrecte");
        SIM_ASSERT(s.getVelocity() == glm::dvec3(4.0, 5.0, 6.0), "vitesse incorrecte");
        SIM_ASSERT(s.getMass()     == 1000.0,                     "masse incorrecte");
        Orbit::Logger::info("Getters/setters OK — pos=" + fmtVec(s.getPosition())
                                              + " vel=" + fmtVec(s.getVelocity())
                                              + " mass=" + fmtDouble(s.getMass(), 0));
    }

    Orbit::Logger::info("");
    Orbit::Logger::info("=== Intégrateurs (1 pas, dt=1/60s, LEO) ===");
    {
        Orbit::SemiImplicitEuler euler;
        Orbit::VelocityVerlet    verlet;
        Orbit::RK4               rk4;
        Orbit::Leapfrog          leapfrog;

        testIntegrator("SemiImplicitEuler", euler);
        testIntegrator("VelocityVerlet",    verlet);
        testIntegrator("RK4",               rk4);
        testIntegrator("Leapfrog",          leapfrog);

        std::unique_ptr<Orbit::IIntegrator> active = std::make_unique<Orbit::VelocityVerlet>();
        Orbit::State s;
        s.setPosition({7'000'000.0, 0.0, 0.0});
        s.setVelocity({0.0, 7'500.0, 0.0});
        active->step(s, earthGravity(), 1.0 / 60.0);
        Orbit::Logger::info("Switch unique_ptr → VelocityVerlet OK | pos=" + fmtVec(s.getPosition()));
    }

    Orbit::Logger::info("");
    Orbit::Logger::info("=== Boucle de simulation (5s, dt=1/60s) ===");
    {
        Orbit::TimeManager    timeManager;
        Orbit::SimulationLoop simLoop(timeManager.getFixedDt());

        timeManager.start();

        constexpr int    RUN_SECONDS = 5;
        constexpr auto   FRAME       = std::chrono::microseconds(1'000'000 / 60);

        const auto simStart   = Clock::now();
        auto       nextFrame  = simStart;
        auto       printTimer = simStart;

        while (true) {
            nextFrame += FRAME;

            timeManager.update();
            const double alpha = simLoop.update(timeManager.getUnscaledDeltaTime());

            SIM_ASSERT(alpha >= 0.0 && alpha <= 1.0, "alpha hors [0,1]");

            const auto  now         = Clock::now();
            const double wallElapsed = std::chrono::duration<double>(now - simStart).count();

            if (std::chrono::duration<double>(now - printTimer).count() >= 1.0) {
                const auto cal = timeManager.getCalendar();
                Orbit::Logger::info(
                    "t=" + fmtDouble(wallElapsed, 1) + "s"
                    + " | FPS="     + std::to_string(static_cast<int>(timeManager.getFPS()))
                    + " | simTime=" + fmtDouble(timeManager.getSimTime(), 3) + "s"
                    + " | "         + std::to_string(cal.years) + "-"
                                    + std::to_string(cal.month) + "-"
                                    + std::to_string(cal.days)
                    + " | alpha="   + fmtDouble(alpha, 4)
                );
                printTimer = now;
            }

            if (wallElapsed >= static_cast<double>(RUN_SECONDS))
                break;

            std::this_thread::sleep_until(nextFrame);
        }

        timeManager.stop();
    }

    Orbit::Logger::info("");
    Orbit::Logger::info("Phase 0.x — succès ✓");
    return 0;
}
