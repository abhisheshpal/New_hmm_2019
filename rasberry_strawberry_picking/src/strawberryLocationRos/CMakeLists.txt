cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_FORCE_INLINES")
SET(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/lib/cmake/)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_definitions(-DCOMPILEDWITHC11)
message(STATUS "Using flag -std=c++11.")
project(strawberryLocationRos)

SET(CMAKE_INCLUDE_CURRENT_DIR ON)
##SET(CUDA_CUDART_LIBRARY /usr/local/cuda/lib64/libcudart.so)
##FIND_PACKAGE(CUDA REQUIRED)
##INCLUDE_DIRECTORIES(${CUDA_INCLUDE_DIRS})
FIND_PACKAGE(OpenMP REQUIRED)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)
#find_package(Eigen3 REQUIRED)

if(OPENMP_FOUND)
        message("OPENMP FOUND")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

#ADD_SUBDIRECTORY(3rdparty/gSLICr)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/3rdparty)

# Reconstruction visualization.
find_package(OpenGL REQUIRED)
if (NOT MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
else (NOT MSVC)
  find_package(GLEW REQUIRED)
  if (GLEW_FOUND)
    message ("-- Found GLEW: ${GLEW_INCLUDE_DIRS}")
  else (GLEW FOUND)
    message (FATAL_ERROR "-- Can't find GLEW. Please set GLEW_INCLUDE_DIR & GLEW_LIBRARY")
  endif (GLEW_FOUND)
  include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLEW_INCLUDE_DIRS} )
  list(APPEND EXTRA_GL_LIBRARIES ${GLEW_LIBRARY} ${GLEW_LIBRARIES})
endif (NOT MSVC)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set (external_libs
        ${OpenCV_INCLUDE_DIRS}
        #${EIGEN3_INCLUDE_DIR}
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/include
)


# add the binary tree to the search path for include files
# set ( ${external_libs} ${external_libs} ${PROJECT_BINARY_DIR} )

# include_directories(${external_libs})
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library (${PROJECT_NAME} SHARED
       # src/Segmentor.cpp
        src/Camera.cpp
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS}
${PROJECT_SOURCE_DIR}/3rdparty/gSLICr/build/libgSLICr_lib.a
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

##add_executable(strawberryLocationRos_demo examples/strawberryLocationRos_demo.cpp)
#target_link_libraries(strawberryLocationRos_demo ${PROJECT_NAME} ${catkin_LIBRARIES} ${CUDA_CUDART_LIBRARY})

add_executable(strawberryLocationRos_demo_gui examples/strawberryLocationRos_demo_gui.cpp)
target_link_libraries(strawberryLocationRos_demo_gui ${PROJECT_NAME} ${catkin_LIBRARIES} ${CUDA_CUDART_LIBRARY})

add_executable(strawberryLocationRos_demo_gui_xy examples/strawberryLocationRos_demo_gui_xy.cpp)
target_link_libraries(strawberryLocationRos_demo_gui_xy ${PROJECT_NAME} ${catkin_LIBRARIES} ${CUDA_CUDART_LIBRARY})

add_executable(strawberryLocationRos_xy_kinova examples/strawberryLocationRos_xy_kinova.cpp)
target_link_libraries(strawberryLocationRos_xy_kinova  ${PROJECT_NAME} ${catkin_LIBRARIES} ${CUDA_CUDART_LIBRARY})


add_executable(strawberry_image_publisher examples/strawberry_image_publisher.cpp)
target_link_libraries(strawberry_image_publisher ${PROJECT_NAME} ${catkin_LIBRARIES} ${CUDA_CUDART_LIBRARY})
