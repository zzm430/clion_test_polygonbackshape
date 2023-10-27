cmake_minimum_required(VERSION 3.9)
project(polygonbackshape)
find_package(Boost REQUIRED COMPONENTS )
find_package(OpenCV REQUIRED)
find_package(CGAL REQUIRED)
find_package(Matlab REQUIRED)
#find_package(PythonLibs REQUIRED)

if(CGAL_FOUND)
    message(STATUS "Found CGAL library")
else()
    message(FATAL_ERROR "Could not find CGAL library")
endif()
include_directories(${Boost_INCLUDE_DIRS})
set(CMAKE_CXX_STANDARD 11)


# 添加CGAL库的头文件路径
include_directories(${CGAL_INCLUDE_DIRS})
include_directories(.)
include_directories(${OpenCV_INCLUDE_DIRS})
#include_directories(${PYTHON_INCLUDE_DIRS})


add_executable(polygonbackshape
              main.cpp
#              PreProcess/getMapData.h
              PreProcess/getMapData.cpp
#              Planning/path_common.h
#              Planning/path_common.cpp
#              Planning/path_baseplan.h
              Planning/path_baseplan.cpp
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
              Planning/curveDecisionManager.h Planning/curveDecisionManager.cpp Geometry/cornerTuring_FT_CPA_CC_Algorithm.h Geometry/cornerTuring_FT_CPA_CC_Algorithm.cpp Geometry/cornerTuring_FT_CPA_CV_ABptLocation.h Geometry/cornerTuring_FT_CPA_CV_ABptLocation.cpp common/plot/tractorPolygonShow.h common/plot/tractorPolygonShow.cpp common/print/tractorPolyPrint.h Geometry/cornerTuring_FT_CPA_CV_Algorithm.h Geometry/cornerTuring_FT_CPA_CV_Algorithm.cpp common/print/tractorPolyPrint2.h common/plot/curveCurvatureCalculate.h common/plot/curveCurvatureCalculate.cpp Decision/curveDecision.h Decision/curveDecision.cpp)
target_include_directories(polygonbackshape PRIVATE ${Matlab_INCLUDE_DIRS})

target_link_libraries(polygonbackshape ${Boost_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(polygonbackshape ${CGAL_LIBRARIES})
target_link_libraries(polygonbackshape ${Matlab_LIBRARIES})
#target_link_libraries(polygonbackshape ${PYTHON_LIBRARIES})



