//
// Created by zzm on 24-2-8.
//

#ifndef POLYGONBACKSHAPE_DRAWOFFLINEPLAN_H
#define POLYGONBACKSHAPE_DRAWOFFLINEPLAN_H

#include "HMI/imgui/imgui.h"
#include "HMI/imgui/imgui_impl_glfw.h"
#include "HMI/imgui/imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>
#include <algorithm>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include "HMI/implot/implot.h"
#include "math.h"

// 数据结构定义
struct TableRow {
    std::string column1;
    std::string column2;
};

// 模拟的 TableView 数据
std::vector<TableRow> tableViewData = {
        {"A1", "B1"},
        {"A2", "B2"},
        {"A3", "B3"}
};

class drawOffLinePlan {
public:

    drawOffLinePlan() ;
    virtual ~drawOffLinePlan() = default;
    void  drawInitialize();
    void  deleteDrawResource();
    void  drawProcess();
    void  createFileMenu();
    void  createCanvasshow();
    void  createCurveDataShow();
    void  createPolygonDrawShow();

    void  mouseKeypts();
    void  mouseRouting();
    void  mouseOuterboundary();

    void  showKeypts();
    void  showRouting();
    void  showOuterboundary();
    void  showCurveDataAttr();
    void  showDrawPolygon();

    void  processTractorSimulator();
    void  processTableViewWindow();
    void  processDebugDataShow();

private:
    GLFWwindow* window_;
    float canvas_scale_ = 1.0f;
    bool deleteAllCanvas_ = false;
    bool show_keypoints_ = false;
    bool show_routing_ = false;
    bool show_keyptcoords_ = false;
    bool show_outerboundary_ = false;
    bool show_curveDataAttr_ = false;
    bool show_drawPolygon_ = false;
    bool deleteAllPolygon_ = false;

    std::vector<ImVec2> polygonVertices_;
    std::vector<std::vector<double>>  storage_keypts_;
    std::vector<std::vector<double>>  storage_routing_;
    std::vector<std::vector<double>>  storage_outerboundary_;

    ImDrawList*  draw_list_;
    ImVec2 canvas_pos_ ;
    ImVec2 canvas_size_ ;

    float ridgeAndCurveIndex_[2];
};





#endif //POLYGONBACKSHAPE_DRAWOFFLINEPLAN_H
