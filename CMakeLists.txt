cmake_minimum_required(VERSION 3.20)
project(eliott-physics VERSION 1.0.0 LANGUAGES CXX)
set(CMAKE_TOOLCHAIN_FILE "C:/Dev/vcpkg/scripts/buildsystems/vcpkg.cmake")
set(CMAKE_PREFIX_PATH "C:/Dev/vcpkg/installed/x64-mingw-static")


set(CMAKE_CXX_USE_RESPONSE_FILE_FOR_INCLUDES OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)




add_library(eliott-physics STATIC
    src/QuadTree.cpp
    src/PhysicsWorld.cpp
)

target_include_directories(eliott-physics PUBLIC
    include/
)

target_link_libraries(eliott-physics PUBLIC
    eliott-math
    eliott-ecs
)