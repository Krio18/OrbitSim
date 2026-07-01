#pragma once

#include <array>
#include <deque>
#include <memory>
#include <string>

#include <glm/glm.hpp>

#include "Core/TimeManager/TimeManager.hpp"
#include "Core/SimulationLoop/SimulationLoop.hpp"
#include "Core/State/State.hpp"
#include "Core/Integrator/IIntegrator.hpp"
#include "Physics/GravitySystem.hpp"

#ifdef ORBIT_HAS_SDL2
struct SDL_Window;
struct SDL_Renderer;
#endif

namespace Orbit {
    class Application {
        public:
            Application();
            ~Application();
            int run();

    private:
        void initParticles();
        void stepPhysics(double dt);
        void updateConservation();

        void render();
        void renderBackground();
        void renderEarth();
        void renderTrail(const std::deque<glm::dvec2>& trail, uint8_t r, uint8_t g, uint8_t b) const;
        void renderParticle(const State& state, uint8_t r, uint8_t g, uint8_t b) const;
        void renderHUD();
        void renderPanel(int x, int y, int w, int h, uint8_t alpha = 180) const;

        glm::dvec2 worldToScreen(const glm::dvec3& pos) const;
        void drawFilledCircle(int cx, int cy, int radius, uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255);

        static constexpr double PHYSICS_DT = 60.0;
        static constexpr double ORBIT_RADIUS = 7'000'000.0;

        TimeManager _timeManager;
        SimulationLoop _simLoop;
        GravitySystem _gravity;

        State _eulerState;
        State _verletState;
        std::unique_ptr<IIntegrator> _eulerIntegrator;
        std::unique_ptr<IIntegrator> _verletIntegrator;

        static constexpr size_t TRAIL_CAP = 3000;
        std::deque<glm::dvec2> _eulerTrail;
        std::deque<glm::dvec2> _verletTrail;

        struct ConservationData {
            double energy = 0.0;
            double angMom = 0.0;
            double radius = 0.0;
            double period = 0.0;
        };
        ConservationData _eulerCons;
        ConservationData _verletCons;

        struct Star { int16_t x, y; uint8_t bright; };
        static constexpr int NUM_STARS = 220;
        std::array<Star, NUM_STARS> _stars;

        double _simTime = 0.0;
        double _warpFactor = 4000.0;
        bool _paused = false;
        bool _running = true;

        double _viewScale;
        glm::dvec2 _viewCenter;

        #ifdef ORBIT_HAS_SDL2
            SDL_Window* _window = nullptr;
            SDL_Renderer* _renderer = nullptr;
            static constexpr int WIN_W = 940;
            static constexpr int WIN_H = 720;
        #endif
    };

}
