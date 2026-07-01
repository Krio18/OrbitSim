#include "Application.hpp"
#include "Font8x8.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

#include <glm/geometric.hpp>

#include "Core/Constants.hpp"
#include "Core/Logger/Logger.hpp"
#include "Core/Integrator/SemiImplicitEuler/SemiImplicitEuler.hpp"
#include "Core/Integrator/VelocityVerlet/VelocityVerlet.hpp"

#ifdef ORBIT_HAS_SDL2
#include <SDL2/SDL.h>
#endif

namespace Orbit {

    Application::Application()
        : _simLoop(PHYSICS_DT), _gravity(Constants::MU_EARTH)
        , _eulerIntegrator(std::make_unique<SemiImplicitEuler>())
        , _verletIntegrator(std::make_unique<VelocityVerlet>())
        , _viewScale(300.0 / ORBIT_RADIUS)
    {
        #ifdef ORBIT_HAS_SDL2
            _viewCenter = {WIN_W * 0.5, WIN_H * 0.5};

            if (SDL_Init(SDL_INIT_VIDEO) != 0) {
                Logger::error(std::string("SDL_Init: ") + SDL_GetError());
                _running = false;
                return;
            }

            _window = SDL_CreateWindow(
                "OrbitSim - Phase 1",
                SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                WIN_W, WIN_H,
                SDL_WINDOW_SHOWN);
            if (!_window) {
                Logger::error(std::string("SDL_CreateWindow: ") + SDL_GetError());
                _running = false;
                return;
            }

            _renderer = SDL_CreateRenderer(_window, -1,
                SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
            if (!_renderer) {
                Logger::error(std::string("SDL_CreateRenderer: ") + SDL_GetError());
                _running = false;
                return;
            }
            SDL_SetRenderDrawBlendMode(_renderer, SDL_BLENDMODE_BLEND);
        #else
            _viewCenter = {470.0, 360.0};
            Logger::warning("SDL2 absent — mode headless.");
        #endif

            uint32_t rng = 0xDEADBEEFu;
            auto next = [&]() -> uint32_t { rng = rng * 1664525u + 1013904223u; return rng; };
            for (auto& s : _stars) {
                #ifdef ORBIT_HAS_SDL2
                        s.x = static_cast<int16_t>(next() % WIN_W);
                        s.y = static_cast<int16_t>(next() % WIN_H);
                #else
                        s.x = static_cast<int16_t>(next() % 940);
                        s.y = static_cast<int16_t>(next() % 720);
                #endif
                        s.bright = 80 + static_cast<uint8_t>(next() % 175);
            }

            initParticles();
            Logger::info("Application initialisee — orbite LEO r=" +
                        std::to_string(static_cast<int>(ORBIT_RADIUS / 1000.0)) + " km");
    }

    Application::~Application() {
        #ifdef ORBIT_HAS_SDL2
            if (_renderer) SDL_DestroyRenderer(_renderer);
            if (_window) SDL_DestroyWindow(_window);
            SDL_Quit();
        #endif
    }

    void Application::initParticles() {
        const double vCirc = std::sqrt(Constants::MU_EARTH / ORBIT_RADIUS);

        _eulerState = State{};
        _eulerState.setPosition({ORBIT_RADIUS, 0.0, 0.0});
        _eulerState.setVelocity({0.0, vCirc, 0.0});
        _eulerState.setMass(1000.0);
        _verletState = _eulerState;

        _eulerTrail.clear();
        _verletTrail.clear();
        _simTime = 0.0;
        _eulerCons = {};
        _verletCons = {};

        Logger::info("Reset — v_circ=" + std::to_string(static_cast<int>(vCirc)) + " m/s");
    }

    void Application::stepPhysics(double dt) {
        const AccelFunc accel = _gravity.makeAccelFunc();

        _eulerIntegrator->step(_eulerState, accel, dt);
        _verletIntegrator->step(_verletState, accel, dt);
        _simTime += dt;

        _eulerTrail.push_back({_eulerState.getPosition().x, _eulerState.getPosition().y});
        _verletTrail.push_back({_verletState.getPosition().x, _verletState.getPosition().y});
        if (_eulerTrail.size() > TRAIL_CAP) _eulerTrail.pop_front();
        if (_verletTrail.size() > TRAIL_CAP) _verletTrail.pop_front();

        updateConservation();
    }

    void Application::updateConservation() {
        const double mu = _gravity.getMu();

        auto compute = [&](const State& s) -> ConservationData {
            const glm::dvec3& r = s.getPosition();
            const glm::dvec3& v = s.getVelocity();
            const double rMag = glm::length(r);
            const double vSq = glm::dot(v, v);
            const double eps = 0.5 * vSq - mu / rMag;
            const double h = glm::length(glm::cross(r, v));
            const double a    = -mu / (2.0 * eps);
            const double T    = Constants::TWO_PI * std::sqrt(a * a * a / mu);
            return {eps, h, rMag, T};
        };

        _eulerCons = compute(_eulerState);
        _verletCons = compute(_verletState);
    }

    glm::dvec2 Application::worldToScreen(const glm::dvec3& pos) const {
        return {
            _viewCenter.x + pos.x * _viewScale,
            _viewCenter.y - pos.y * _viewScale
        };
    }

    void Application::drawFilledCircle(int cx, int cy, int radius, uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
        #ifdef ORBIT_HAS_SDL2
            SDL_SetRenderDrawColor(_renderer, r, g, b, a);
            for (int dy = -radius; dy <= radius; dy++) {
                const int dx = static_cast<int>(
                    std::sqrt(static_cast<double>(radius * radius - dy * dy)));
                SDL_RenderDrawLine(_renderer, cx - dx, cy + dy, cx + dx, cy + dy);
            }
        #endif
            (void)cx; (void)cy; (void)radius; (void)r; (void)g; (void)b; (void)a;
    }

    void Application::renderPanel(int x, int y, int w, int h, uint8_t alpha) const {
        #ifdef ORBIT_HAS_SDL2
            SDL_SetRenderDrawColor(_renderer, 4, 8, 20, alpha);
            const SDL_Rect rect{x, y, w, h};
            SDL_RenderFillRect(_renderer, &rect);
            SDL_SetRenderDrawColor(_renderer, 60, 80, 130, 200);
            SDL_RenderDrawRect(_renderer, &rect);
        #endif
            (void)x; (void)y; (void)w; (void)h; (void)alpha;
    }

    void Application::renderBackground() {
        #ifdef ORBIT_HAS_SDL2
            SDL_SetRenderDrawColor(_renderer, 4, 4, 14, 255);
            SDL_RenderClear(_renderer);

            for (const auto& s : _stars) {
                const uint8_t b = s.bright;
                const uint8_t blue = std::min(255, b + 25);
                SDL_SetRenderDrawColor(_renderer, b, b, blue, 255);
                SDL_RenderDrawPoint(_renderer, s.x, s.y);
                if (b > 200) {
                    SDL_SetRenderDrawColor(_renderer, b, b, blue, 60);
                    SDL_RenderDrawPoint(_renderer, s.x + 1, s.y);
                    SDL_RenderDrawPoint(_renderer, s.x - 1, s.y);
                    SDL_RenderDrawPoint(_renderer, s.x, s.y + 1);
                    SDL_RenderDrawPoint(_renderer, s.x, s.y - 1);
                }
            }
        #endif
    }

    void Application::renderEarth() {
        #ifdef ORBIT_HAS_SDL2
            const auto sc = worldToScreen(_gravity.getBodyPosition());
            const int cx = static_cast<int>(sc.x);
            const int cy = static_cast<int>(sc.y);

            for (int r = 84; r >= 44; r -= 2) {
                const float t = static_cast<float>(r - 44) / 40.0f;
                const uint8_t a = static_cast<uint8_t>(60.0f * std::exp(-t * 3.5f));
                drawFilledCircle(cx, cy, r, 50, 110, 255, a);
            }

            drawFilledCircle(cx, cy, 42, 20, 65, 185, 255);
            drawFilledCircle(cx - 10, cy - 12, 26, 60, 130, 230, 110);
            drawFilledCircle(cx - 16, cy - 18, 13, 90, 155, 240, 70);
        #endif
    }

    void Application::renderTrail(const std::deque<glm::dvec2>& trail, uint8_t r, uint8_t g, uint8_t b) const {
        #ifdef ORBIT_HAS_SDL2
            if (trail.empty()) return;

            const size_t n = trail.size();
            const size_t seg = n / 4;
            const float fade[4] = {0.10f, 0.28f, 0.58f, 1.00f};

            std::vector<SDL_Point> pts;
            pts.reserve(seg + 1);

            for (int band = 0; band < 4; band++) {
                const size_t from = static_cast<size_t>(band) * seg;
                const size_t to = (band == 3) ? n : (static_cast<size_t>(band + 1) * seg);
                const float f = fade[band];

                pts.clear();
                for (size_t i = from; i < to; i++) {
                    const auto s = worldToScreen({trail[i].x, trail[i].y, 0.0});
                    pts.push_back({static_cast<int>(s.x), static_cast<int>(s.y)});
                }
                SDL_SetRenderDrawColor(_renderer,
                    static_cast<uint8_t>(r * f),
                    static_cast<uint8_t>(g * f),
                    static_cast<uint8_t>(b * f),
                    255);
                if (!pts.empty())
                    SDL_RenderDrawPoints(_renderer, pts.data(), static_cast<int>(pts.size()));
            }
        #endif
            (void)r; (void)g; (void)b;
    }

    void Application::renderParticle(const State& state, uint8_t r, uint8_t g, uint8_t b) const {
        #ifdef ORBIT_HAS_SDL2
            const auto sc = worldToScreen(state.getPosition());
            const int sx = static_cast<int>(sc.x);
            const int sy = static_cast<int>(sc.y);

            SDL_SetRenderDrawColor(_renderer, r, g, b, 35);
            const SDL_Rect h1{sx - 9, sy - 9, 19, 19};
            SDL_RenderFillRect(_renderer, &h1);

            SDL_SetRenderDrawColor(_renderer, r, g, b, 90);
            const SDL_Rect h2{sx - 5, sy - 5, 11, 11};
            SDL_RenderFillRect(_renderer, &h2);

            SDL_SetRenderDrawColor(_renderer, r, g, b, 255);
            const SDL_Rect core{sx - 3, sy - 3, 7, 7};
            SDL_RenderFillRect(_renderer, &core);

            SDL_SetRenderDrawColor(_renderer, 255, 255, 255, 255);
            const SDL_Rect center{sx - 1, sy - 1, 3, 3};
            SDL_RenderFillRect(_renderer, &center);
        #endif
            (void)r; (void)g; (void)b;
    }

    void Application::renderHUD() {
        #ifdef ORBIT_HAS_SDL2
            using C = SDL_Color;
            constexpr C WHITE = {230, 235, 245, 255};
            constexpr C DIM = {130, 145, 170, 255};
            constexpr C ACCENT = {100, 180, 255, 255};
            constexpr C EULER = {230, 80, 80, 255};
            constexpr C VERLET = {70, 220, 110, 255};
            constexpr C TITLE = {180, 210, 255, 255};

            const double T_orb = Constants::TWO_PI *
                std::sqrt(ORBIT_RADIUS * ORBIT_RADIUS * ORBIT_RADIUS / Constants::MU_EARTH);
            const double orbCount = _simTime / T_orb;

            char buf[80];
            constexpr int LH = 11;

            auto drawSep = [&](int x, int y, int w) {
                SDL_SetRenderDrawColor(_renderer, 60, 80, 130, 130);
                SDL_RenderDrawLine(_renderer, x, y, x + w, y);
            };

            constexpr int PL_X = 10, PL_Y = 10, PL_W = 205, PL_H = 112;
            renderPanel(PL_X, PL_Y, PL_W, PL_H);

            int lx = PL_X + 8;
            int ly = PL_Y + 8;

            font8x8::drawShadow(_renderer, "ORBITSIM  PHASE 1", lx, ly, TITLE); ly += LH + 2;
            drawSep(lx, ly, PL_W - 16); ly += 6;

            std::snprintf(buf, sizeof(buf), "temps  : %8.0f s", _simTime);
            font8x8::drawShadow(_renderer, buf, lx, ly, WHITE); ly += LH;

            std::snprintf(buf, sizeof(buf), "orbites: %8.2f", orbCount);
            font8x8::drawShadow(_renderer, buf, lx, ly, WHITE); ly += LH + 4;

            drawSep(lx, ly, PL_W - 16); ly += 6;

            const int warpLevel = static_cast<int>(std::round(std::log2(_warpFactor / 125.0)));
            constexpr int N_PIPS = 11, PIP_W = 10, PIP_H = 9, PIP_GAP = 2;
            for (int i = 0; i < N_PIPS; i++) {
                const SDL_Rect pip{lx + i * (PIP_W + PIP_GAP), ly, PIP_W, PIP_H};
                if (i <= warpLevel) {
                    SDL_SetRenderDrawColor(_renderer, 45, 120, 230, 255);
                    SDL_RenderFillRect(_renderer, &pip);
                    SDL_SetRenderDrawColor(_renderer, 90, 170, 255, 255);
                } else {
                    SDL_SetRenderDrawColor(_renderer, 20, 30, 55, 255);
                    SDL_RenderFillRect(_renderer, &pip);
                    SDL_SetRenderDrawColor(_renderer, 45, 60, 95, 255);
                }
                SDL_RenderDrawRect(_renderer, &pip);
            }

            const int pipsEnd = lx + N_PIPS * (PIP_W + PIP_GAP) + 4;
            if (_warpFactor >= 1000.0)
                std::snprintf(buf, sizeof(buf), "x%dk", static_cast<int>(_warpFactor / 1000.0 + 0.5));
            else
                std::snprintf(buf, sizeof(buf), "x%d", static_cast<int>(_warpFactor));
            font8x8::drawShadow(_renderer, buf, pipsEnd, ly, ACCENT);

            if (_paused) {
                constexpr int PW = 94, PH = 18;
                const int px = (WIN_W - PW) / 2;
                renderPanel(px, WIN_H / 2 - 9, PW, PH, 215);
                font8x8::drawShadow(_renderer, "--- PAUSE ---", px + 7, WIN_H / 2 - 4, ACCENT);
            }

            constexpr int PR_W = 240, PR_H = 95;
            constexpr int PR_X = WIN_W - PR_W - 10, PR_Y = 10;
            renderPanel(PR_X, PR_Y, PR_W, PR_H);

            int rx = PR_X + 8;
            int ry = PR_Y + 8;

            font8x8::drawShadow(_renderer, "CONSERVATION", rx, ry, TITLE); ry += LH + 2;
            drawSep(rx, ry, PR_W - 16); ry += 6;
            font8x8::drawShadow(_renderer, "           Verlet    Euler", rx, ry, DIM); ry += LH;

            const auto fmtRow = [&](const char* label, double vVal, double eVal) {
                std::snprintf(buf, sizeof(buf), "%-9s%7.2f  %7.2f", label, vVal, eVal);
                font8x8::drawShadow(_renderer, buf, rx, ry, WHITE);
                ry += LH;
            };

            fmtRow("e MJ/kg", _verletCons.energy / 1e6, _eulerCons.energy / 1e6);
            fmtRow("r km   ", _verletCons.radius / 1e3, _eulerCons.radius / 1e3);
            if (_verletCons.energy < 0.0 && _eulerCons.energy < 0.0)
                fmtRow("T s    ", _verletCons.period, _eulerCons.period);
            if (_eulerCons.energy >= 0.0)
                font8x8::drawShadow(_renderer, "! Euler : orbite perdue", rx, ry, EULER);

            constexpr int BAR_H = 22, BAR_Y = WIN_H - BAR_H - 2;
            renderPanel(10, BAR_Y, WIN_W - 20, BAR_H, 150);

            auto drawKey = [&](int x, int y, const std::string& label) -> int {
                const int w = font8x8::textWidth(label) + 6;
                SDL_SetRenderDrawColor(_renderer, 55, 70, 105, 220);
                const SDL_Rect kr{x, y, w, 12};
                SDL_RenderFillRect(_renderer, &kr);
                SDL_SetRenderDrawColor(_renderer, 110, 140, 210, 255);
                SDL_RenderDrawRect(_renderer, &kr);
                font8x8::draw(_renderer, label, x + 3, y + 2, WHITE);
                return w;
            };

            {
                SDL_Rect er{16, BAR_Y + 7, 9, 8};
                SDL_SetRenderDrawColor(_renderer, EULER.r, EULER.g, EULER.b, 255);
                SDL_RenderFillRect(_renderer, &er);
                font8x8::drawShadow(_renderer, "Euler", 30, BAR_Y + 7, EULER);

                SDL_Rect vr{90, BAR_Y + 7, 9, 8};
                SDL_SetRenderDrawColor(_renderer, VERLET.r, VERLET.g, VERLET.b, 255);
                SDL_RenderFillRect(_renderer, &vr);
                font8x8::drawShadow(_renderer, "Verlet", 104, BAR_Y + 7, VERLET);
            }

            SDL_SetRenderDrawColor(_renderer, 60, 80, 130, 140);
            SDL_RenderDrawLine(_renderer, 160, BAR_Y + 4, 160, BAR_Y + BAR_H - 4);

            int kx = 170;
            const int ky = BAR_Y + 5;

            kx += drawKey(kx, ky, "SPC") + 3;
            font8x8::drawShadow(_renderer, "pause", kx, ky + 2, DIM);
            kx += font8x8::textWidth("pause") + 10;

            kx += drawKey(kx, ky, "^") + 1;
            kx += drawKey(kx, ky, "v") + 3;
            font8x8::drawShadow(_renderer, "warp", kx, ky + 2, DIM);
            kx += font8x8::textWidth("warp") + 10;

            kx += drawKey(kx, ky, "+") + 1;
            kx += drawKey(kx, ky, "-") + 3;
            font8x8::drawShadow(_renderer, "zoom", kx, ky + 2, DIM);
            kx += font8x8::textWidth("zoom") + 10;

            kx += drawKey(kx, ky, "R") + 3;
            font8x8::drawShadow(_renderer, "reset", kx, ky + 2, DIM);
            kx += font8x8::textWidth("reset") + 10;

            kx += drawKey(kx, ky, "ESC") + 3;
            font8x8::drawShadow(_renderer, "quitter", kx, ky + 2, DIM);

        #endif
    }

    void Application::render() {
        #ifdef ORBIT_HAS_SDL2
            renderBackground();
            renderEarth();
            renderTrail(_eulerTrail, 220, 70, 70);
            renderTrail(_verletTrail, 60, 210, 90);
            renderParticle(_eulerState, 220, 80, 80);
            renderParticle(_verletState, 70, 220, 100);
            renderHUD();
            SDL_RenderPresent(_renderer);
        #endif
    }

    int Application::run() {
        #ifdef ORBIT_HAS_SDL2
            if (!_running) return 1;
            _timeManager.start();

            while (_running) {
                SDL_Event e;
                while (SDL_PollEvent(&e)) {
                    if (e.type == SDL_QUIT) {
                        _running = false;
                    } else if (e.type == SDL_KEYDOWN) {
                        switch (e.key.keysym.sym) {
                            case SDLK_ESCAPE: _running = false; break;
                            case SDLK_SPACE: _paused = !_paused; break;
                            case SDLK_UP: _warpFactor = std::min(_warpFactor * 2.0, 128'000.0); break;
                            case SDLK_DOWN: _warpFactor = std::max(_warpFactor / 2.0, 125.0); break;
                            case SDLK_EQUALS:
                            case SDLK_PLUS: _viewScale *= 1.2; break;
                            case SDLK_MINUS: _viewScale /= 1.2; break;
                            case SDLK_r: initParticles(); break;
                            default: break;
                        }
                    }
                }

                _timeManager.update();

                if (!_paused) {
                    const double scaled = _timeManager.getUnscaledDeltaTime() * _warpFactor;
                    _simLoop.update(scaled, [&](double dt) { stepPhysics(dt); });
                }

                render();
            }

            _timeManager.stop();
            return 0;

        #else
            _timeManager.start();
            const double T_orb = Constants::TWO_PI *
                std::sqrt(ORBIT_RADIUS * ORBIT_RADIUS * ORBIT_RADIUS / Constants::MU_EARTH);

            double lastLog = -1e9;
            while (_simTime < 10.0 * T_orb) {
                _timeManager.update();
                _simLoop.update(_timeManager.getUnscaledDeltaTime() * _warpFactor,
                                [&](double dt) { stepPhysics(dt); });

                if (_simTime - lastLog >= 5000.0) {
                    lastLog = _simTime;
                    char buf[256];
                    std::snprintf(buf, sizeof(buf),
                        "t=%.0fs (%.2f orb) | Verlet e=%.4f MJ/kg r=%.0fkm"
                        " | Euler e=%.4f MJ/kg r=%.0fkm",
                        _simTime, _simTime / T_orb,
                        _verletCons.energy / 1e6, _verletCons.radius / 1e3,
                        _eulerCons.energy / 1e6, _eulerCons.radius / 1e3);
                    Logger::info(buf);
                }
            }

            _timeManager.stop();
            return 0;
        #endif
    }

}
