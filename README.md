# 2D Physics Engine


# How to build
On root,

    conan install . --output-folder=build --build=missing

    cd build/build/Release/generators
    cmake ../../../.. -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
    cmake --build .

