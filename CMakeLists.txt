cmake_minimum_required(VERSION 3.9)
project(polygonbackshape)


# set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fext-numeric-literals")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-omit-frame-pointer")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-ignored-attributes")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=return-type")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wpedantic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wcomment")

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-sign-compare")

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wunused-function")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wunused-variable")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wcomment")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wsign-compare")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wshadow")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wconversion")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-ignored-qualifiers")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-vla")

## -------- folder relative path definition -----------
string(CONCAT CONF_FOLDER_PATH_STR ${CMAKE_CURRENT_SOURCE_DIR} "/")
message(STATUS "conf folder path : " ${CONF_FOLDER_PATH_STR})
add_definitions(-DCONF_FOLDER_PATH=\"${CONF_FOLDER_PATH_STR}\")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(Boost REQUIRED COMPONENTS )
#find_package(OpenCV REQUIRED)
find_package(CGAL REQUIRED)
#find_package(Matlab REQUIRED)
find_package(osqp REQUIRED)
#find_package(PythonLibs REQUIRED)
find_package(PythonLibs 3 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

find_package(Eigen3 QUIET)
include_directories("/usr/include/eigen3/")


if(CGAL_FOUND)
    message(STATUS "Found CGAL library")
else()
    message(FATAL_ERROR "Could not find CGAL library")
endif()

include_directories(${Boost_INCLUDE_DIRS})


# 添加CGAL库的头文件路径
include_directories(${CGAL_INCLUDE_DIRS})
include_directories(.)
#include_directories(${OpenCV_INCLUDE_DIRS})
#include_directories(${PYTHON_INCLUDE_DIRS})



add_executable(polygonbackshape
              main.cpp
#              PreProcess/getMapData.h
              PreProcess/getMapData.cpp
#              Planning/path_common.h
#              Planning/path_common.cpp
#              Planning/path_baseplan.h
        Geometry/reeds_shepp.cpp
              Geometry/reeds_shepp.h
              Geometry/dubins.cpp
              Geometry/dubins.h
              common/plot/plotter.h
              easylogging++.cc
              Planning/path_polygonplan.cpp
              Planning/path_polygonplan.h
              common/math/common_math.h
              Geometry/cornerTuring_location.h Geometry/cornerTuring_location.cpp
              common/utilpath/path_polygonPoint.h Geometry/newCornerTuring_location.h
              Geometry/newCornerTuring_location.cpp common/print/filePrint.h
              common/print/filePrint2.h Geometry/cornerTuring_FT_CPA_FS_Algorithm.h
              Geometry/cornerTuring_FT_CPA_FS_Algorithm.cpp common/print/normaPrint.h
              Planning/normalMatrix_Translate.h Planning/normalMatrix_Translate.cpp
              Planning/diffMode_Choose.h Geometry/cornerTuring_Implement_Radius.h
              Geometry/cornerTuring_Implement_Radius.cpp Geometry/cornerTuring_C_CPA_Algorithm.h
              Geometry/cornerTuring_C_CPA_Algorithm.cpp common/common_param/common_typedef.h
              Planning/curveDecisionManager.h Planning/curveDecisionManager.cpp
              Geometry/cornerTuring_FT_CPA_CC_Algorithm.h Geometry/cornerTuring_FT_CPA_CC_Algorithm.cpp
              Geometry/cornerTuring_FT_CPA_CV_ABptLocation.h Geometry/cornerTuring_FT_CPA_CV_ABptLocation.cpp
               common/plot/tractorPolygonShow.h common/plot/tractorPolygonShow.cpp common/print/tractorPolyPrint.h
              Geometry/cornerTuring_FT_CPA_CV_Algorithm.h Geometry/cornerTuring_FT_CPA_CV_Algorithm.cpp common/print/tractorPolyPrint2.h
              common/plot/curveCurvatureCalculate.h common/plot/curveCurvatureCalculate.cpp Decision/curveDecision.h
              Decision/curveDecision.cpp Decision/curveDecisionType.h Emplanner/frenet_converter.h
              Emplanner/frenet_converter.cpp Emplanner/customPJPO.h Emplanner/customPJPO.cpp
              Emplanner/piecewise_jerk_algorithm.h Emplanner/piecewise_jerk_algorithm.cpp
              Emplanner/piecewise_jerk_path_algorithm.h Emplanner/piecewise_jerk_path_algorithm.cpp
              smooth/customFemAlgorithm/fem_pos_deviation_sqp_osqp_interface.h smooth/customFemAlgorithm/fem_pos_deviation_sqp_osqp_interface.cc
              smooth/customFemAlgorithm/fem_pos_deviation_osqp_interface.h smooth/customFemAlgorithm/fem_pos_deviation_osqp_interface.cc
              smooth/customFemAlgorithm/fem_pos_smooth_manager.h
              Emplanner/curve_static_obstacles_manager.h Emplanner/curve_static_obstacles_manager.cpp common/utilpath/path_anchorPoint.h
              smooth/customFemAlgorithm/fem_pos_smooth_manager.cpp
              smooth/customFemAlgorithm/fem_pos_deviation_smoother.cc smooth/customFemAlgorithm/fem_pos_deviation_smoother.h common/plot/computePathProfile.h common/plot/computePathProfile.cpp common/utilpath/path_slPoint.h common/math/algebra.h common/math/vec_2d.h common/math/vec_2d.cpp common/math/algebra.cpp common/utilpath/discrete_match_helper.h common/math/line_segment_2d.h common/math/line_segment_2d.cpp common/utilpath/discretizedPath.h common/utilpath/discretizedPath.cpp common/utilpath/point.h common/utilpath/point.cpp common/utilpath/pt_slboundary.h
        common/print/tractorPolyPrintCurve.h
        common/print/tractorPJPOPathPrint.h
        HybridAStar/dubin_path.h
        HybridAStar/dubin_path.cpp
        HybridAStar/hybridA_param.h
        HybridAStar/node_HybridA.h
        common/math/box2d.h
        common/math/aabox2d.h
        common/math/aabox2d.cpp
        common/math/box2d.cpp
        common/math/math_utils.h
        common/math/math_utils.cpp
        common/math/polygon_2d.h
        common/math/polygon_2d.cpp
        HybridAStar/node_HybridA.cpp
        common/utilpath/path_dubins.h
        HybridAStar/grid_search.h
        HybridAStar/grid_search.cpp
        HybridAStar/hybrid_AStar_Algorithm.h
        HybridAStar/hybrid_AStar_Algorithm.cpp
        HybridAStar/hybrid_AStar_test.h
        common/plot/debugHybridAStarShow.h
)

target_include_directories(polygonbackshape PRIVATE ${Matlab_INCLUDE_DIRS})
target_link_libraries(polygonbackshape ${Boost_LIBRARIES})
#target_link_libraries(polygonbackshape ${Boost_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(polygonbackshape ${CGAL_LIBRARIES})
target_link_libraries(polygonbackshape ${Matlab_LIBRARIES})
target_link_libraries(polygonbackshape  osqp::osqp)
target_link_libraries(polygonbackshape ${PYTHON_LIBRARIES})



