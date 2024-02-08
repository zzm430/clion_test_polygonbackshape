#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

// GLFW和OpenGL的头文件
#include <GLFW/glfw3.h>
#include <algorithm>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include "implot.h"
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

int maintest() {
    // 初始化glfw和opengl上下文
    glfwInit();
    GLFWwindow* window = glfwCreateWindow(1280, 720, "OFFLINE PATH PLANNING FOR TRACTORS", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // 启用垂直同步

    // 初始化Dear ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();

    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    // 定义画布缩放因子和多边形顶点列表
    float canvas_scale = 1.0f;
    ImVec2 poly_points[4] = {{100, 100}, {200, 150}, {250, 300}, {150, 250}};

    bool show_polygon = false;
    bool showLineList = false;
    bool deleteAllCanvas = false;
    bool show_keypoints = false;
    bool show_routing = false;
    bool show_keyptcoords = false;
    bool show_outerboundary = false;
    bool show_curveDataAttr = false;
    bool show_drawPolygon = false;
    bool deleteAllPolygon = false;

    static float ridgeAndCurveIndex[2] = { 0.0f, 0.0f };

    std::vector<ImVec2> polygonVertices;

    std::vector<std::vector<double>>  storage_keypts;
    std::vector<std::vector<double>>  storage_routing;
    std::vector<std::vector<double>>  storage_outerboundary;

    // 主循环
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // 渲染Dear ImGui
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // 开始绘制
        // 获取主视口的工作区位置
        ImGui::DockSpaceOverViewport(ImGui::GetMainViewport(), ImGuiDockNodeFlags_PassthruCentralNode);
        ImGui::Begin("tractor simulator",nullptr);

        // 获取绘制命令列表
        ImDrawList* draw_list = ImGui::GetWindowDrawList();

        if (ImGui::BeginMainMenuBar())
        {
            if (ImGui::BeginMenu("File"))
            {
//            ShowExampleMenuFile();
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Canvasshow"))
            {
                //多边形绘制
                if (ImGui::MenuItem("show_polygonlist", "CTRL+Z",show_polygon)) {
                    show_polygon = !show_polygon;
                }

                // 菜单项：绘制矩形
                if (ImGui::MenuItem("show_lineList", NULL, showLineList)) {
                    showLineList = !showLineList; // 切换矩形的绘制状态
                }

                // 路径关键点展示
                if (ImGui::MenuItem("show_keypoints", NULL, show_keypoints)) {
                    show_keypoints = !show_keypoints; // 切换矩形的绘制状态
                }

                // 路径routing展示
                if (ImGui::MenuItem("show_routing", NULL, show_routing)) {
                    show_routing = !show_routing; // 切换矩形的绘制状态
                }

                // 展示外边界
                if (ImGui::MenuItem("show_outerboundary", NULL, show_outerboundary)) {
                    show_outerboundary = !show_outerboundary; // 切换矩形的绘制状态
                }

                //删除画布所有东西
                if (ImGui::MenuItem("deleteAllCanvas", NULL, deleteAllCanvas)) {
                        show_polygon = false;
                        showLineList = false;
                        show_keypoints = false;
                        show_routing = false;
                        show_outerboundary = false;
                }
//                if (ImGui::MenuItem("Redo", "CTRL+Y", false, false)) {}  // Disabled item
                ImGui::Separator();
                if (ImGui::MenuItem("Cut", "CTRL+X")) {}
                if (ImGui::MenuItem("Copy", "CTRL+C")) {}
                if (ImGui::MenuItem("Paste", "CTRL+V")) {}
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("CurveDataShow"))
            {
                if (ImGui::MenuItem("CurveDataAttr", NULL, show_curveDataAttr)) {
                    show_curveDataAttr = !show_curveDataAttr; // 切换矩形的绘制状态
                }

                ImGui::EndMenu();
            }

            if(ImGui::BeginMenu("PolygonDrawShow")){

                if(ImGui::MenuItem("PolygonDrawShow", NULL, show_drawPolygon)){
                    show_drawPolygon  = !show_drawPolygon;
                }

                //删除画布所有东西
                if (ImGui::MenuItem("deleteAllPolygon", NULL, deleteAllPolygon)) {
                    show_drawPolygon = false;
                    polygonVertices.clear();
                }
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }

        // 在ImGui窗口中添加一个条件语句，以便在鼠标左键按下的情况下捕获多边形的顶点
        if (show_polygon) {
            if (ImGui::IsMouseDragging(0, 0.0f)) {
                ImVec2 mouse_delta = ImGui::GetIO().MouseDelta;
                for (int i = 0; i < 4; i++) {
                    poly_points[i].x += mouse_delta.x;
                    poly_points[i].y += mouse_delta.y;
                }
            }
        }

        if(show_keypoints){
            if (ImGui::IsMouseDragging(0, 0.0f)) {
                ImVec2 mouse_delta = ImGui::GetIO().MouseDelta;
                auto size_M = storage_keypts[0].size();
                for (int i = 0; i < size_M; i++) {
                    storage_keypts[0][i] += mouse_delta.x;
                }
                for (int i = 0;i < size_M; i++){
                    storage_keypts[1][i] += mouse_delta.y;
                }
            }
        }

        if(show_routing){
            if (ImGui::IsMouseDragging(0, 0.0f)) {
                ImVec2 mouse_delta = ImGui::GetIO().MouseDelta;
                auto size_M = storage_routing[0].size();
                for (int i = 0; i < size_M; i++) {
                    storage_routing[0][i] += mouse_delta.x;
                }
                for (int i = 0; i < size_M; i++) {
                    storage_routing[1][i] += mouse_delta.y;
                }
            }
        }

        if(show_outerboundary){
            if (ImGui::IsMouseDragging(0, 0.0f)) {
                ImVec2 mouse_delta = ImGui::GetIO().MouseDelta;
                auto size_M = storage_outerboundary[0].size();
                for (int i = 0; i < size_M; i++) {
                    storage_outerboundary[0][i] += mouse_delta.x;
                }
                for (int i = 0; i < size_M; i++) {
                    storage_outerboundary[1][i] += mouse_delta.y;
                }
            }
        }

        // 获取画布的位置和尺寸
        ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
        ImVec2 canvas_size = ImGui::GetWindowContentRegionMax();
        canvas_size.x = 1250;
        canvas_size.y = 1250;

        // 绘制画布
        draw_list->AddRectFilled(canvas_pos,
                                 ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y),
                                 IM_COL32(255, 255, 255, 255));

        if(draw_list){
            // 绘制多边形
            if(show_polygon){
                ImVec2 scaled_points[4] = {{poly_points[0].x * canvas_scale,poly_points[0].y * canvas_scale},
                                           {poly_points[1].x * canvas_scale, poly_points[1].y * canvas_scale},
                                           {poly_points[2].x * canvas_scale, poly_points[2].y * canvas_scale},
                                           {poly_points[3].x * canvas_scale, poly_points[3].y * canvas_scale}};

                draw_list->AddLine(ImVec2(canvas_pos.x + scaled_points[0].x,
                                          canvas_pos.y + scaled_points[0].y),
                                   ImVec2(canvas_pos.x + scaled_points[1].x,
                                          canvas_pos.y + scaled_points[1].y),
                                   IM_COL32(255, 0, 0, 255));
                draw_list->AddLine(ImVec2(canvas_pos.x + scaled_points[1].x,
                                          canvas_pos.y + scaled_points[1].y),
                                   ImVec2(canvas_pos.x + scaled_points[2].x,
                                          canvas_pos.y + scaled_points[2].y),
                                   IM_COL32(255, 0, 0, 255));
                draw_list->AddLine(ImVec2(canvas_pos.x + scaled_points[2].x,
                                          canvas_pos.y + scaled_points[2].y),
                                   ImVec2(canvas_pos.x + scaled_points[3].x,
                                          canvas_pos.y + scaled_points[3].y),
                                   IM_COL32(255, 0, 0, 255));
                draw_list->AddLine(ImVec2(canvas_pos.x + scaled_points[3].x,
                                          canvas_pos.y + scaled_points[3].y),
                                   ImVec2(canvas_pos.x + scaled_points[0].x,
                                          canvas_pos.y + scaled_points[0].y),
                                   IM_COL32(255, 0, 0, 255));

                // 显示多边形顶点的坐标
                for (int i = 0; i < 4; i++) {
                    // 创建显示坐标的字符串
                    char coord_text[32];
                    snprintf(coord_text, sizeof(coord_text), "(%.1f, %.1f)",
                             poly_points[i].x, poly_points[i].y);
                    // 绘制坐标文本，稍微偏离顶点位置以避免重叠
                    draw_list->AddText(ImVec2(canvas_pos.x + scaled_points[i].x + 5,
                                              canvas_pos.y + scaled_points[i].y + 5),
                                       IM_COL32(0, 0, 255, 255), coord_text);
                }
            }

            if(showLineList){
                draw_list->AddRect(ImVec2(200, 200),
                                   ImVec2(300, 300),
                                   IM_COL32(0, 0, 255, 255));
            }

            if(show_keypoints){
                std::ifstream file("/home/zzm/Desktop/test_path_figure-main/src/keypoints.txt");
                std::string line;

                while(std::getline(file,line)){
                    std::istringstream  iss(line);
                    double value;
                    std::vector<double> temp;
                    while(iss >> value){
                     temp.push_back(value);
                    }
                    storage_keypts.push_back(temp);
                }

                std::vector<ImVec2> storageImPTsSp;
                for(size_t i = 0;i  < storage_keypts[0].size();i++){
                    storageImPTsSp.push_back(ImVec2(storage_keypts[0][i] * canvas_scale,
                                                    storage_keypts[1][i] * canvas_scale));
                }
                //线段展示
                for(size_t i = 0;i < storage_keypts[0].size()-1;i++){
                    draw_list->AddLine(ImVec2(canvas_pos.x + storageImPTsSp[i].x,
                                              canvas_pos.y + storageImPTsSp[i].y),
                                       ImVec2(canvas_pos.x + storageImPTsSp[i+1].x,
                                              canvas_pos.y + storageImPTsSp[i+1].y),
                                       IM_COL32(0, 255, 0, 255),
                                       1.0f);
                }
            }

            if(show_routing){
                std::ifstream file1("/home/zzm/Desktop/test_path_figure-main/src/cgal_show_ridge_path.txt");
                std::string line1;
                std::vector<double> colm,coln;
                double num1,num2;
                while(file1 >> num1 >> num2){
                    colm.push_back(num1);
                    coln.push_back(num2);
                }
                storage_routing.push_back(colm);
                storage_routing.push_back(coln);

                std::vector<ImVec2> storageImPTsSp;
                for(size_t i = 0;i  < storage_routing.size();i++){
                    storageImPTsSp.push_back(ImVec2(storage_routing[0][i] * canvas_scale,
                                                    storage_routing[1][i] * canvas_scale));
                }
                //线段展示
                for(size_t i = 0;i < storage_routing[0].size()-1;i++){
                    draw_list->AddLine(ImVec2(canvas_pos.x + storageImPTsSp[i].x,
                                              canvas_pos.y + storageImPTsSp[i].y),
                                       ImVec2(canvas_pos.x + storageImPTsSp[i+1].x,
                                              canvas_pos.y + storageImPTsSp[i+1].y),
                                       IM_COL32(165, 42, 42, 255),
                                       1.0f);
                }
            }

            if(show_outerboundary){
                std::ifstream file("/home/zzm/Desktop/test_path_figure-main/src/origin_polygon.txt");
                std::string line;

                while(std::getline(file,line)){
                    std::istringstream  iss(line);
                    double value;
                    std::vector<double> temp;
                    while(iss >> value){
                        temp.push_back(value);
                    }
                    storage_outerboundary.push_back(temp);
                }

                std::vector<ImVec2> storageImPTsSp;
                for(size_t i = 0;i  < storage_outerboundary[0].size();i++){
                    storageImPTsSp.push_back(ImVec2(storage_outerboundary[0][i] * canvas_scale,
                                                    storage_outerboundary[1][i] * canvas_scale));
                }
                //线段展示
                for(size_t i = 0;i < storage_outerboundary[0].size()-1;i++){
                    draw_list->AddLine(ImVec2(canvas_pos.x + storageImPTsSp[i].x,
                                              canvas_pos.y + storageImPTsSp[i].y),
                                       ImVec2(canvas_pos.x + storageImPTsSp[i+1].x,
                                              canvas_pos.y + storageImPTsSp[i+1].y),
                                       IM_COL32(135, 206, 235, 255),
                                       1.0f);
                }
            }

            if(show_curveDataAttr){

            }

            if(show_drawPolygon){
                //绘制一个透明的按钮覆盖整个画布区域，不过鼠标点击事件
                ImGui::InvisibleButton("Canvas", canvas_size);

                // 当InvisibleButton被按下（即鼠标点击时），记录顶点
                if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                    ImVec2 mouse_pos = ImGui::GetMousePos();
                    ImVec2 clicked_pos = ImVec2(mouse_pos.x - canvas_pos.x,
                                                mouse_pos.y - canvas_pos.y); // 转换为相对于画布的坐标
                    polygonVertices.push_back(clicked_pos);
                }

               // 绘制多边形的每条边
                for (size_t i = 0; i < polygonVertices.size(); ++i) {
                    ImVec2 p1 = ImVec2(canvas_pos.x + polygonVertices[i].x, canvas_pos.y + polygonVertices[i].y);
                    ImVec2 p2 = ImVec2(canvas_pos.x + polygonVertices[(i+1) % polygonVertices.size()].x,
                                       canvas_pos.y + polygonVertices[(i+1) % polygonVertices.size()].y);
                    draw_list->AddLine(p1, p2, IM_COL32(139,119,101, 255), 2.0f);
                }

                // 显示每个顶点的坐标
                for (size_t i = 0; i < polygonVertices.size(); ++i) {
                    ImVec2 p = ImVec2(canvas_pos.x + polygonVertices[i].x,
                                      canvas_pos.y + polygonVertices[i].y);
                    std::string coordText = "(" + std::to_string((int)polygonVertices[i].x)
                            + ", " + std::to_string((int)polygonVertices[i].y) + ")";
                    draw_list->AddText(p,
                                       IM_COL32(139,119,101, 255),
                                       coordText.c_str());
                }
            }
        }

        // 结束绘制
        ImGui::End();

        //增加程序运行变量展示
        // 新窗口用于tableView的数据展示

        ImGui::Begin("TableView Window");
       // 在新窗口中展示tableView的数据
      // 使用ImGui的表格控件（Table）或其他适合的控件来展示tableView的数据
        ImGui::Text("TableView Data:");

       // 使用Table控件展示tableView的数据
        if (ImGui::BeginTable("table1", 3,
                              ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
        {
            // 设置表头
            ImGui::TableSetupColumn("Column 1");
            ImGui::TableSetupColumn("Column 2");
            ImGui::TableHeadersRow();

            // 添加数据
            for (int row = 0; row < tableViewData.size(); row++)
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("Row %d", row);
                ImGui::TableNextColumn();
                ImGui::Text("%s", tableViewData[row].column1.c_str());
                ImGui::TableNextColumn();
                ImGui::Text("%s", tableViewData[row].column2.c_str());
            }
            ImGui::EndTable();
        }

        ImGui::End();

        ImGui::Begin("debug data show");
        ImGui::Text("program Data:");
        // 在ImGui窗口中创建两个输入框
        ImGui::InputFloat2("ridge and curveIndex", ridgeAndCurveIndex);
        if (ImPlot::BeginPlot("Curvature data show"))
        {
            // 将数据传递给 ImPlot，并绘制折线图
            float xs[100], ys[100];
            for (int i = 0; i < 100; ++i)
            {
                xs[i] = i * 0.1f;
                ys[i] = sin(xs[i]);
            }
            ImPlot::PlotLine("Curvature", xs, ys, 100);

            // 结束 ImPlot 绘制
            ImPlot::EndPlot();
        }
        ImGui::End();


        // 处理画布缩放
        ImGuiIO& io = ImGui::GetIO();
        if (io.MouseWheel != 0.0) {
            canvas_scale += io.MouseWheel * 0.1; // 根据需要调整缩放速度
            canvas_scale = std::max(canvas_scale, 0.1f); // 限制最小缩放值
        }

        // 渲染
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        ImGui::UpdatePlatformWindows();
        ImGui::RenderPlatformWindowsDefault();
        glfwSwapBuffers(window);
    }

    // 清理 ImPlot 资源
    ImPlot::DestroyContext();
    // 清理资源
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}