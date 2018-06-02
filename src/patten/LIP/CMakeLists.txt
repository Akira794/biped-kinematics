cmake_minimum_required(VERSION 2.8)

project(LIP)

set(PROJECT_NAME LIP)
set(MAIN_SRC main.cpp)

set(DCMAKE_BUILD_TYPE Debug)

set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR} CACHE PATH "Eigen installation dir(Default: $EIGEN_HOME)")
find_package(Boost COMPONENTS system thread filesystem timer)
find_package(PythonLibs REQUIRED)

add_definitions(-lstdc++ -std=c++11)
add_definitions(-DEIGEN_DONT_ALIGN)

include_directories(
	${Boost_INCLUDE_DIR}
	${BOOST_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)

set ( INSTALL_TARGET_DIR
	${CMAKE_SOURCE_DIR}/samples )

add_executable(${PROJECT_NAME} ${MAIN_SRC} LinearInvertedPendlum.cpp LinearInvertedPendlum.h)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${INSTALL_TARGET_DIR})

