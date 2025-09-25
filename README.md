# 2D Physics Engine

![2D Physics Engine GIF](/2dphysics.gif?raw=true "2D Physics Engine demo")

# Description
Simple 2D physics engine using impulse-based physics and GJK for convex polygon detection.

# How to build
On root,

    conan install . --output-folder=build --build=missing

    cd build/build/Release/generators
    cmake ../../../.. -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
    cmake --build .

