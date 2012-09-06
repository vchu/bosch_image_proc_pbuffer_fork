cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


FIND_PACKAGE(OpenGL REQUIRED)
#FIND_PACKAGE(GLUT)
#FIND_PACKAGE(GLEW)

#IF(GLEW_FOUND)
#  SET(gl_LIBS ${gl_LIBS} ${GLEW_LIBRARY} )
#ENDIF(GLEW_FOUND)

#INCLUDE_DIRECTORIES(${GLEW_INCLUDE_DIRS})

IF(OPENGL_FOUND)
  SET(gl_LIBS ${gl_LIBS} ${OPENGL_LIBRARIES} )
ENDIF(OPENGL_FOUND)

#IF(GLUT_FOUND)
#  SET(gl_LIBS ${gl_LIBS} ${GLUT_LIBRARIES} )
#ENDIF(GLUT_FOUND)


MESSAGE(STATUS "${gl_LIBS}")

rosbuild_gensrv()

rosbuild_add_executable(camera_self_filter src/camera_self_filter.cpp src/robotMeshModel.cpp)
rosbuild_add_executable(camera_self_filter_server src/camera_self_filter_server.cpp src/robotMeshModel.cpp)
rosbuild_add_executable(image_view src/image_view.cpp )

TARGET_LINK_LIBRARIES(camera_self_filter ${gl_LIBS})
TARGET_LINK_LIBRARIES(camera_self_filter glut GLEW)

TARGET_LINK_LIBRARIES(camera_self_filter_server ${gl_LIBS})
TARGET_LINK_LIBRARIES(camera_self_filter_server glut GLEW)


rosbuild_add_executable(test_client test/test_client.cpp )
