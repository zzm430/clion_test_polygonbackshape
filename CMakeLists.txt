cmake_minimum_required(VERSION 3.15)
project(polygonbackshape)
find_package(Boost REQUIRED COMPONENTS )
find_package(OpenCV REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
set(CMAKE_CXX_STANDARD 11)


include_directories(.)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(polygonbackshape
              main.cpp
#              PreProcess/getMapData.h
              PreProcess/getMapData.cpp
#              Planning/path_common.h
              Planning/path_common.cpp
#              Planning/path_baseplan.h
              Planning/path_baseplan.cpp
              Geometry/reeds_shepp.cpp
              Geometry/reeds_shepp.h
              easylogging++.cc Planning/path_polygonplan.cpp common/math/common_math.h)
target_link_libraries(polygonbackshape ${Boost_LIBRARIES} ${OpenCV_LIBS})