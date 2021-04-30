cmake_minimum_required(VERSION 3.0.0)
project(YDLIDAR-G2-starter VERSION 0.1.0)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)
enable_testing()

#Include directories
include_directories(
     ${CMAKE_SOURCE_DIR}
)
############## YDLIDAR SDK START#####################################
#find ydlidar_sdk package
find_package(ydlidar_sdk REQUIRED)
#Include directories
include_directories(
  ${YDLIDAR_SDK_INCLUDE_DIRS}
)

#link library directories
link_directories(${YDLIDAR_SDK_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} main.c)

#Link your project to ydlidar_sdk library.
target_link_libraries(${PROJECT_NAME} ${YDLIDAR_SDK_LIBRARIES} -lstdc++ -lm)

############## YDLIDAR SDK END#####################################

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
