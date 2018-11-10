cmake_minimum_required(VERSION 2.8)

project( TEST_RungeKutta)

set(PROJECT_NAME OUTPUT)
set(MAIN_SRC SystemLIP.cpp)

set(DCMAKE_BUILD_TYPE Debug)

set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR} CACHE PATH "Eigen installation dir(Default: $EIGEN_HOME)")
find_package(Boost COMPONENTS system thread filesystem timer)
find_package(PythonLibs REQUIRED)

add_definitions(-lstdc++ -std=c++11)
add_definitions(-DEIGEN_DONT_ALIGN)

set( TOOLS_DIR
	${CMAKE_CURRENT_SOURCE_DIR}/tools
)


include_directories(
	${Boost_INCLUDE_DIR}
	${BOOST_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)

add_executable(
	${PROJECT_NAME} ${MAIN_SRC}
	${TOOLS_DIR}/RungeKutta.cpp
)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY} ${PYTHON_LIBRARIES})
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)

