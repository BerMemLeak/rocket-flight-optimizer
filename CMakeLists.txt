cmake_minimum_required(VERSION 3.27)
project(nir)

set(CMAKE_CXX_STANDARD 20)
# Добавление gnuplot-iostream
include_directories("./libs/gnuplot")



# Найти библиотеки Boost
find_package(Boost REQUIRED COMPONENTS iostreams system)
include_directories("./libs/boost")

add_executable(nir main.cpp
        rocket/rocket.cpp
        RocketOptimizer/RocketOptimizer.cpp)
# Линковка с библиотеками Boost
target_link_libraries(nir ${Boost_LIBRARIES})