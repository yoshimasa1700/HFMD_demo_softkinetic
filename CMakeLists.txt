# CMakeのバージョン
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(CMAKE_CXX_COMPILER g++)

SET(CMAKE_CXX_FLAGS_RELEASE "-Wall -O2 -fpermissive")
SET(CMAKE_CXX_FLAGS_DEBUG "-g -fpermissive")

SET(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)

PROJECT(HFMD_demo_DS325)

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Boost)
find_package(OpenMP)
#find_package(OpenGL)
#find_package(GLUT)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/include")
INCLUDE_DIRECTORIES(/opt/softkinetic/DepthSenseSDK/include)

LINK_DIRECTORIES("/usr/local/lib")
LINK_DIRECTORIES("/usr/lib")
LINK_DIRECTORIES("/opt/softkinetic/DepthSenseSDK/lib")

ADD_EXECUTABLE(HFMD_demo_DS325 ./src/main.cpp)
TARGET_LINK_LIBRARIES(HFMD_demo_DS325 HFMD_core 
				      ${Boost_LIBRARIES} 
				      DepthSense 
				      DepthSensePlugins 
				      turbojpeg 
				      ${OpenCV_LIBRARIES})

