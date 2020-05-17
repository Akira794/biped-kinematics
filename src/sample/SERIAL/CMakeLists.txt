cmake_minimum_required(VERSION 2.8)

project( SERIAL_MANIP_demo)

set(PROJECT_GNU serial_robot)


set(DCMAKE_BUILD_TYPE Debug)



find_package(GLUT)
find_package(OpenGL)

set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR} CACHE PATH "Eigen installation dir(Default: $EIGEN_HOME)")
find_package(Boost COMPONENTS system thread filesystem timer)

add_definitions(-lstdc++ -std=c++11 -lpthread)
add_definitions(-DEIGEN_DONT_ALIGN)
add_definitions(-lglut -lGLU -lGL -lm)

include_directories(
	${Boost_INCLUDE_DIR}
	${BOOST_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)

set(GNU_SRCS
	main.cpp Link.cpp Link.h LinkParameter.h Jacobian.cpp Jacobian.h Kinematics.cpp Kinematics.h cmd.cpp cmd.h MainControl.cpp MainControl.h
)

set ( INSTALL_TARGET_DIR
    ${CMAKE_SOURCE_DIR}/build )

add_executable(${PROJECT_GNU} ${GNU_SRCS} )
target_link_libraries(${PROJECT_GNU} ${Boost_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY} )
install(TARGETS ${PROJECT_GNU} RUNTIME DESTINATION ${INSTALL_TARGET_DIR})
