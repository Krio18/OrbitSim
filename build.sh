#!/bin/bash

set -e

BUILD_TYPE="Release"
CLEAN=false
RUN=false

while [[ $# -gt 0 ]]; do
    case $1 in
        -c|--clean)
            CLEAN=true
            shift
            ;;
        -d|--debug)
            BUILD_TYPE="Debug"
            shift
            ;;
        -r|--run)
            RUN=true
            shift
            ;;
        -h|--help)
            echo "OrbitSim Build Script"
            echo ""
            echo "Usage: ./build.sh [options]"
            echo ""
            echo "Options:"
            echo "  -c, --clean     Clean build directory before building"
            echo "  -d, --debug     Build in Debug mode (default: Release)"
            echo "  -r, --run       Run OrbitSim after successful build"
            echo "  -h, --help      Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use -h or --help for usage information"
            exit 1
            ;;
    esac
done

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Détection automatique de vcpkg si VCPKG_ROOT n'est pas défini
if [ -z "$VCPKG_ROOT" ]; then
    if [ -f "$HOME/vcpkg/vcpkg" ]; then
        export VCPKG_ROOT="$HOME/vcpkg"
    fi
fi

TOOLCHAIN_ARG=""
if [ -n "$VCPKG_ROOT" ]; then
    TOOLCHAIN_ARG="-DCMAKE_TOOLCHAIN_FILE=${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
fi

echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}  OrbitSim Build Script${NC}"
echo -e "${BLUE}================================${NC}"
echo ""

if [ "$CLEAN" = true ]; then
    echo -e "${YELLOW}Cleaning build directory...${NC}"
    rm -rf build
    echo -e "${GREEN}Done${NC}"
    echo ""
fi

echo -e "${BLUE}Configuring CMake (${BUILD_TYPE})...${NC}"
cmake -B build -S . -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${TOOLCHAIN_ARG}
echo -e "${GREEN}Configuration complete${NC}"
echo ""

echo -e "${BLUE}Building...${NC}"
cmake --build build --config ${BUILD_TYPE} -j$(nproc)
echo -e "${GREEN}Build complete${NC}"
echo ""

echo -e "${GREEN}================================${NC}"
echo -e "${GREEN}  Build type: ${BUILD_TYPE}${NC}"
echo -e "${GREEN}  Binary:     build/bin/OrbitSim${NC}"
echo -e "${GREEN}================================${NC}"
echo ""

if [ "$RUN" = true ]; then
    echo -e "${BLUE}Launching OrbitSim...${NC}"
    echo ""
    ./build/bin/OrbitSim
fi
