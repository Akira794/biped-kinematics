cmake_minimum_required(VERSION 2.8)

project(PolynomialApproximation)

set(DCMAKE_BUILD_TYPE Debug)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread filesystem timer)
add_definitions(-lstdc++ -std=c++11)

include_directories(
	${Boost_INCLUDE_DIR}
	${BOOST_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)

link_directories(
	${Boost_LIBRARY_DIR}
	${BOOST_LIBRARY_DIRS}
)

add_executable(PolynomialApproximation PolynomialApproximation.cpp)

target_link_libraries(PolynomialApproximation ${Boost_LIBRARIES})
