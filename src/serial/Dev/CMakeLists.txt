cmake_minimum_required(VERSION 2.8)

set( PROJECT_NAME
		Run_SIM
)

set ( TARGET_MAIN_SRC
		mainsim.cpp
)

project( ${PROJECT_NAME})

set (INSTALL_TARGET_DIR
    ${CMAKE_SOURCE_DIR}/samples )

set (KINEMA_DIR
		${CMAKE_SOURCE_DIR}/../kinema )

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
	${KINEMA_DIR}/kinematics.cpp
	${KINEMA_DIR}/jacobian.cpp
	${KINEMA_DIR}/link.cpp 
	${KINEMA_DIR}/plot.cpp
	${KINEMA_DIR}/pi.cpp
	${KINEMA_DIR}/cmd.cpp

	${KINEMA_DIR}/kinematics.h
	${KINEMA_DIR}/jacobian.h
	${KINEMA_DIR}/link.h
	${KINEMA_DIR}/demo_biped_link.h 
	${KINEMA_DIR}/plot.h
	${KINEMA_DIR}/pi.h
	${KINEMA_DIR}/cmd.h

)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
install( TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${INSTALL_TARGET_DIR})
