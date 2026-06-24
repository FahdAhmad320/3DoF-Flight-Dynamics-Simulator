cmake_minimum_required(VERSION 3.16)
project(AerospaceFlightSim VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/W4 /permissive- /O2)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

add_library(sim_engine
    src/Aero_model.cpp
    src/Atmosphere.cpp
    src/simulator.cpp
)

target_include_directories(sim_engine PUBLIC include)

add_executable(flight_sim
    src/main.cpp
    src/csv_writer.cpp
)

target_link_libraries(flight_sim PRIVATE sim_engine)