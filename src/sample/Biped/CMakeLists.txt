cmake_minimum_required(VERSION 2.8)

project( kinematics_demo)

set(PROJECT_GNU demo_biped)
set(GNU_SRC mainsim.cpp)

set(DCMAKE_BUILD_TYPE Debug)

set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR} CACHE PATH "Eigen installation dir(Default: $EIGEN_HOME)")
find_package(Boost COMPONENTS system thread filesystem timer)

add_definitions(-lstdc++ -std=c++11)
add_definitions(-DEIGEN_DONT_ALIGN)

include_directories(
	${Boost_INCLUDE_DIR}
	${BOOST_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_GNU} ${GNU_SRC} Link.cpp Kinematics.cpp Jacobian.cpp plot.cpp cmd.cpp)
target_link_libraries(${PROJECT_GNU} ${Boost_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
install(TARGETS ${PROJECT_GNU} RUNTIME DESTINATION bin)

