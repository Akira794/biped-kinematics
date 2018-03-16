cmake_minimum_required(VERSION 2.8)

#option(_TEST "testKinematics"   OFF)
#option(_DEMO "demoKinematicsGL" OFF)

#if(_TEST)
#	set(project_name testKinematics )
#	set(main_src testKinematics.cpp )
#	add_definitions(-DTEST)
#endif()

find_package(Boost COMPONENTS system thread filesystem)
set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR} CACHE PATH "Eigen installation dir(Default: $EIGEN_HOME)")
find_package(GLUT)
find_package(OpenGL)
set(DCMAKE_BUILD_TYPE Debug)
add_definitions(-lstdc++ -std=c++11)
add_definitions(-DEIGEN_DONT_ALIGN)
add_definitions(-lglut -lGLU -lGL -lm)
include_directories(
	${BOOST_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
	${EIGEN3_INCLUDE_DIR}
)

#set(project_test testKinematics )
#set(test_src testKinematics.cpp )

#add_executable(${project_test} ${test_src} Kinematics.cpp Jacobian.cpp Link.cpp Kinematics.h Link.h)
#target_link_libraries(${project_test} ${Boost_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
#install(TARGETS ${project_test} RUNTIME DESTINATION bin)

set(project_demo demoKinematicsGL)
set(demo_src main.cpp )
add_executable(${project_demo} ${demo_src}
	kinematics.cpp
	jacobian.cpp
	link.cpp
	pi.cpp

	kinematics.h
	link.h
	demoManipLink.h
	pi.h
)
target_link_libraries(${project_demo} ${Boost_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
install(TARGETS ${project_demo} RUNTIME DESTINATION bin)

