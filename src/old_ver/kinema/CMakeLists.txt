cmake_minimum_required(VERSION 2.8)

set( PROJECT_NAME
		RunKinema
)

set ( TARGET_MAIN_SRC
		main.cpp
)

project( ${PROJECT_NAME})

set (INSTALL_TARGET_DIR
    ${CMAKE_SOURCE_DIR}/samples )

find_package(Boost COMPONENTS system thread filesystem)
set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR} CACHE PATH "Eigen installation dir(Default: $EIGEN_HOME)")

include_directories(
	${BOOST_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
	${EIGEN3_INCLUDE_DIR}
)

link_directories(
	${Boost_LIBRARY_DIRS}
	${Boost_LIBRARY_DIR}
)

add_executable( 
	${PROJECT_NAME} ${TARGET_MAIN_SRC}
	kinematics.cpp jacobian.cpp link.cpp 
	plot.cpp pi.cpp
	cmd.cpp
	robot.cpp

	kinematics.h jacobian.h link.h demo_biped_link.h 
	plot.h pi.h
	cmd.h
	robot.h
)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
install( TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${INSTALL_TARGET_DIR})
