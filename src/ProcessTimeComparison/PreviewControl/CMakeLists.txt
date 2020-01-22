cmake_minimum_required(VERSION 2.8)

project(ggp_test)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_C_FLAGS_DEBUG "-g -Wall -fprofile-arcs -ftest-coverage -O0")

set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR} CACHE PATH "Eigen installation dir(Default: $EIGEN_HOME)")
find_package(Boost COMPONENTS system filesystem iostreams)
find_package(Boost COMPONENTS system thread filesystem timer)

add_definitions(-lstdc++ -std=c++11)
add_definitions(-DEIGEN_DONT_ALIGN)

set(USE_GANKENKUN CACHE BOOL FORCE)
IF(USE_GANKENKUN)
	add_definitions(-DGANKENKUN)
ENDIF(USE_GANKENKUN)

include_directories(
	${Boost_INCLUDE_DIR}
	${BOOST_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)

set(GGP_SRCS
	main.cpp MainControl.cpp PreviewControl.cpp FootStepPlanner.cpp GaitPatternGenerator.cpp cmd.cpp PreCalculatedPreviewControl.cpp
	MainControl.h Jacobian.cpp Link.cpp Jacobian.h Kinematics.h Link.h cmd.h PreCalculatedPreviewControl.h
)

IF(USE_GANKENKUN)
	set(GGP_SRCS ${GGP_SRCS} Kinematics_GankenKun.cpp LinkParameter_GankenKun.h)
ELSE(USE_GANKENKUN)
	set(GGP_SRCS ${GGP_SRCS} Kinematics.cpp LinkParameter.h)
ENDIF(USE_GANKENKUN)

add_executable(ggp_test
	${GGP_SRCS}
)

target_link_libraries(ggp_test ${Boost_LIBRARIES})
