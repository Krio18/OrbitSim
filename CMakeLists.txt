cmake_minimum_required(VERSION 3.21)
project(OrbitSim VERSION 0.1.0 LANGUAGES CXX)

# =============================================================================
# Project Configuration
# =============================================================================

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# =============================================================================
# Build Options
# =============================================================================

option(ORBIT_BUILD_TESTS  "Build unit tests"                ON)
option(ORBIT_ENABLE_ASAN  "Enable AddressSanitizer (Debug)" OFF)

# =============================================================================
# vcpkg Integration
# =============================================================================

if(DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING "Vcpkg toolchain file")
    message(STATUS "Using vcpkg toolchain: ${CMAKE_TOOLCHAIN_FILE}")
endif()

# =============================================================================
# Sources
# =============================================================================
# Ajouter les .cpp ici au fur et à mesure de l'implémentation des phases :
#
#   Phase 0.2 : src/Core/SimulationLoop/SimulationLoop.cpp
#   Phase 0.3 : src/Core/TimeManager/TimeManager.cpp
#   Phase 0.5 : src/Core/Logger/Logger.cpp
#
# Header-only (aucun .cpp) :
#   Phase 0.1 : src/Core/State/State.hpp
#   Phase 0.4 : src/Core/Integrator/IIntegrator.hpp + sous-classes
#   Phase 0.5 : src/Core/Constants.hpp

set(ORBIT_SOURCES
    src/main.cpp
    src/Core/SimulationLoop/SimulationLoop.cpp
    src/Core/TimeManager/TimeManager.cpp
    src/Core/Logger/Logger.cpp
)

add_executable(OrbitSim ${ORBIT_SOURCES})

# =============================================================================
# Compiler Warnings & Optimizations
# =============================================================================

if(MSVC)
    target_compile_options(OrbitSim PRIVATE /W4 /WX /MP)
    target_compile_options(OrbitSim PRIVATE $<$<CONFIG:Release>:/O2 /Oi /Ot /GL>)
    target_link_options(OrbitSim PRIVATE $<$<CONFIG:Release>:/LTCG>)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(OrbitSim PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(OrbitSim PRIVATE $<$<CONFIG:Release>:-O3 -march=native>)
    target_compile_options(OrbitSim PRIVATE $<$<CONFIG:Debug>:-g3 -O0>)
endif()

if(ORBIT_ENABLE_ASAN AND CMAKE_BUILD_TYPE STREQUAL "Debug")
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(OrbitSim PRIVATE -fsanitize=address -fno-omit-frame-pointer)
        target_link_options(OrbitSim PRIVATE -fsanitize=address)
        message(STATUS "AddressSanitizer enabled")
    endif()
endif()

# =============================================================================
# Platform & Build Definitions
# =============================================================================

if(WIN32)
    add_compile_definitions(ORBIT_PLATFORM_WINDOWS)
elseif(APPLE)
    add_compile_definitions(ORBIT_PLATFORM_MACOS)
elseif(UNIX)
    add_compile_definitions(ORBIT_PLATFORM_LINUX)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_definitions(ORBIT_DEBUG)
else()
    add_compile_definitions(ORBIT_RELEASE)
endif()

# =============================================================================
# Include Directories
# =============================================================================

target_include_directories(OrbitSim PRIVATE src)

# =============================================================================
# Dependencies
# =============================================================================

# --- GLM (Phase 0 — requis dès le départ) ---
# dvec3, dquat et toute la double précision sont natifs dans GLM, aucun define requis.
find_package(glm CONFIG QUIET)
if(glm_FOUND)
    target_link_libraries(OrbitSim PRIVATE glm::glm)
    message(STATUS "GLM: found via vcpkg")
elseif(EXISTS "${CMAKE_SOURCE_DIR}/extern/glm")
    target_include_directories(OrbitSim PRIVATE extern/glm)
    message(STATUS "GLM: found in extern/glm")
else()
    message(FATAL_ERROR "GLM introuvable. Installer via vcpkg : vcpkg install glm")
endif()

# --- nlohmann/json (Phase 3 — chargement data-driven des corps célestes) ---
find_package(nlohmann_json CONFIG QUIET)
if(nlohmann_json_FOUND)
    target_link_libraries(OrbitSim PRIVATE nlohmann_json::nlohmann_json)
    message(STATUS "nlohmann/json: found")
else()
    message(STATUS "nlohmann/json: absent (requis en Phase 3 — vcpkg install nlohmann-json)")
endif()

# --- SDL2 (Phase 1.4 — rendu 2D minimal pour valider l'orbite) ---
find_package(SDL2 CONFIG QUIET)
if(SDL2_FOUND)
    target_link_libraries(OrbitSim PRIVATE SDL2::SDL2 SDL2::SDL2main)
    add_compile_definitions(ORBIT_HAS_SDL2)
    message(STATUS "SDL2: found")
else()
    message(STATUS "SDL2: absent (requis en Phase 1.4 — vcpkg install sdl2)")
endif()

# --- bgfx (Phase 11 — rendu 3D, non requis avant) ---
# Décommenter et ajouter extern/bgfx.cmake quand prêt :
# add_subdirectory(extern/bgfx.cmake)
# target_link_libraries(OrbitSim PRIVATE bgfx bx bimg)

# =============================================================================
# Tests (Google Test — Phase 1+)
# =============================================================================

if(ORBIT_BUILD_TESTS)
    find_package(GTest CONFIG QUIET)
    if(GTest_FOUND)
        enable_testing()
        # Décommenter quand tests/CMakeLists.txt existe :
        # add_subdirectory(tests)
        message(STATUS "GTest: found — tests enabled")
    else()
        message(STATUS "GTest: absent (vcpkg install gtest) — tests disabled")
    endif()
endif()

# =============================================================================
# Summary
# =============================================================================

message(STATUS "")
message(STATUS "=================================")
message(STATUS " OrbitSim Configuration Summary  ")
message(STATUS "=================================")
message(STATUS " Version:       ${PROJECT_VERSION}")
message(STATUS " Build type:    ${CMAKE_BUILD_TYPE}")
message(STATUS " Compiler:      ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "")
message(STATUS " Options:")
message(STATUS "   Build Tests:  ${ORBIT_BUILD_TESTS}")
message(STATUS "   ASAN:         ${ORBIT_ENABLE_ASAN}")
message(STATUS "")
message(STATUS " Dependencies:")
message(STATUS "   GLM:          ${glm_FOUND}")
message(STATUS "   SDL2:         ${SDL2_FOUND}")
message(STATUS "   nlohmann/json:${nlohmann_json_FOUND}")
message(STATUS "   GTest:        ${GTest_FOUND}")
message(STATUS "=================================")
message(STATUS "")
