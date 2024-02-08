//
// Created by zzm on 24-2-8.
//
#include "drawOffLinePlan.h"

drawOffLinePlan::drawOffLinePlan():ridgeAndCurveIndex_{0.0f,0.0f}{

}

void drawOffLinePlan::drawInitialize() {
    // 初始化glfw和opengl上下文
    glfwInit();
    window_ = glfwCreateWindow(1280,
                                          720,
                                          "OFFLINE PATH PLANNING FOR TRACTORS",
                                          NULL,
                                          NULL);
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1); // 启用垂直同步

    // 初始化Dear ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();

    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 330");
}

void  drawOffLinePlan::deleteDrawResource(){
    // 清理 ImPlot 资源
    ImPlot::DestroyContext();
    // 清理资源
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window_);
    glfwTerminate();
}

void  drawOffLinePlan::drawProcess(){
    // 主循环
    while (!glfwWindowShouldClose(window_)) {
        glfwPollEvents();
        // 渲染Dear ImGui
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        // 开始绘制
        // 获取主视口的工作区位置
        ImGui::DockSpaceOverViewport(ImGui::GetMainViewport(),
                                     ImGuiDockNodeFlags_PassthruCentralNode);

        processTractorSimulator();
        processTableViewWindow();
        processDebugDataShow();
        // 处理画布缩放
        ImGuiIO& io = ImGui::GetIO();
        if (io.MouseWheel != 0.0) {
            canvas_scale_ += io.MouseWheel * 0.1; // 根据需要调整缩放速度
            canvas_scale_ = std::max(canvas_scale_, 0.1f); // 限制最小缩放值
        }
        // 渲染
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window_, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        ImGui::UpdatePlatformWindows();
        ImGui::RenderPlatformWindowsDefault();
        glfwSwapBuffers(window_);
    }
}

void  drawOffLinePlan::createFileMenu(){
    if (ImGui::BeginMenu("File"))
    {
//            ShowExampleMenuFile();
        ImGui::EndMenu();
    }
}

void  drawOffLinePlan::createCanvasshow(){
    if (ImGui::BeginMenu("Canvasshow"))
    {
        // 路径关键点展示
        if (ImGui::MenuItem("show_keypoints",
                            NULL,
                            show_keypoints_)) {
            show_keypoints_ = !show_keypoints_; // 切换矩形的绘制状态
        }

        // 路径routing展示
        if (ImGui::MenuItem("show_routing",
                            NULL,
                            show_routing_)) {
            show_routing_ = !show_routing_; // 切换矩形的绘制状态
        }

        // 展示外边界
        if (ImGui::MenuItem("show_outerboundary",
                            NULL,
                            show_outerboundary_)) {
            show_outerboundary_ = !show_outerboundary_; // 切换矩形的绘制状态
        }

        //删除画布所有东西
        if (ImGui::MenuItem("deleteAllCanvas",
                            NULL,
                            deleteAllCanvas_)) {
            show_keypoints_ = false;
            show_routing_ = false;
            show_outerboundary_ = false;
        }
//                if (ImGui::MenuItem("Redo", "CTRL+Y", false, false)) {}  // Disabled item
        ImGui::Separator();
        if (ImGui::MenuItem("Cut",
                            "CTRL+X")) {}
        if (ImGui::MenuItem("Copy",
                            "CTRL+C")) {}
        if (ImGui::MenuItem("Paste",
                            "CTRL+V")) {}
        ImGui::EndMenu();
    }
}

void  drawOffLinePlan::createCurveDataShow(){
    if (ImGui::BeginMenu("CurveDataShow"))
    {
        if (ImGui::MenuItem("CurveDataAttr",
                            NULL,
                            show_curveDataAttr_)) {
            show_curveDataAttr_ = !show_curveDataAttr_; // 切换矩形的绘制状态
        }

        ImGui::EndMenu();
    }
}

void drawOffLinePlan::createPolygonDrawShow(){
    if(ImGui::BeginMenu("PolygonDrawShow")){

        if(ImGui::MenuItem("PolygonDrawShow",
                           NULL,
                           show_drawPolygon_)){
            show_drawPolygon_  = !show_drawPolygon_;
        }

        //删除画布所有东西
        if (ImGui::MenuItem("deleteAllPolygon",
                            NULL,
                            deleteAllPolygon_)) {
            show_drawPolygon_ = false;
            polygonVertices_.clear();
        }
        ImGui::EndMenu();
    }
}

void  drawOffLinePlan::mouseKeypts(){
    if(show_keypoints_){
        if (ImGui::IsMouseDragging(0, 0.0f)) {
            ImVec2 mouse_delta = ImGui::GetIO().MouseDelta;
            auto size_M = storage_keypts_[0].size();
            for (int i = 0; i < size_M; i++) {
                storage_keypts_[0][i] += mouse_delta.x;
            }
            for (int i = 0;i < size_M; i++){
                storage_keypts_[1][i] += mouse_delta.y;
            }
        }
    }

}

void  drawOffLinePlan::mouseRouting(){
    if(show_routing_){
        if (ImGui::IsMouseDragging(0, 0.0f)) {
            ImVec2 mouse_delta = ImGui::GetIO().MouseDelta;
            auto size_M = storage_routing_[0].size();
            for (int i = 0; i < size_M; i++) {
                storage_routing_[0][i] += mouse_delta.x;
            }
            for (int i = 0; i < size_M; i++) {
                storage_routing_[1][i] += mouse_delta.y;
            }
        }
    }
}

void  drawOffLinePlan::mouseOuterboundary(){

    if(show_outerboundary_){
        if (ImGui::IsMouseDragging(0, 0.0f)) {
            ImVec2 mouse_delta = ImGui::GetIO().MouseDelta;
            auto size_M = storage_outerboundary_[0].size();
            for (int i = 0; i < size_M; i++) {
                storage_outerboundary_[0][i] += mouse_delta.x;
            }
            for (int i = 0; i < size_M; i++) {
                storage_outerboundary_[1][i] += mouse_delta.y;
            }
        }
    }
}

void  drawOffLinePlan::showKeypts(){
    if(show_keypoints_){
        std::ifstream file("/home/zzm/Desktop/test_path_figure-main/src/keypoints.txt");
        std::string line;

        while(std::getline(file,line)){
            std::istringstream  iss(line);
            double value;
            std::vector<double> temp;
            while(iss >> value){
                temp.push_back(value);
            }
            storage_keypts_.push_back(temp);
        }

        std::vector<ImVec2> storageImPTsSp;
        for(size_t i = 0;i  < storage_keypts_[0].size();i++){
            storageImPTsSp.push_back(ImVec2(storage_keypts_[0][i] * canvas_scale_,
                                            storage_keypts_[1][i] * canvas_scale_));
        }
        //线段展示
        for(size_t i = 0;i < storage_keypts_[0].size()-1;i++){
            draw_list_->AddLine(ImVec2(canvas_pos_.x + storageImPTsSp[i].x,
                                       canvas_pos_.y + storageImPTsSp[i].y),
                               ImVec2(canvas_pos_.x + storageImPTsSp[i+1].x,
                                      canvas_pos_.y + storageImPTsSp[i+1].y),
                               IM_COL32(0, 255, 0, 255),
                               1.0f);
        }
    }
}

void  drawOffLinePlan::showRouting(){
    if(show_routing_){
        std::ifstream file1("/home/zzm/Desktop/test_path_figure-main/src/cgal_show_ridge_path.txt");
        std::string line1;
        std::vector<double> colm,coln;
        double num1,num2;
        while(file1 >> num1 >> num2){
            colm.push_back(num1);
            coln.push_back(num2);
        }
        storage_routing_.push_back(colm);
        storage_routing_.push_back(coln);

        std::vector<ImVec2> storageImPTsSp;
        for(size_t i = 0;i  < storage_routing_.size();i++){
            storageImPTsSp.push_back(ImVec2(storage_routing_[0][i] * canvas_scale_,
                                            storage_routing_[1][i] * canvas_scale_));
        }
        //线段展示
        for(size_t i = 0;i < storage_routing_[0].size()-1;i++){
            draw_list_->AddLine(ImVec2(canvas_pos_.x + storageImPTsSp[i].x,
                                       canvas_pos_.y + storageImPTsSp[i].y),
                                ImVec2(canvas_pos_.x + storageImPTsSp[i+1].x,
                                       canvas_pos_.y + storageImPTsSp[i+1].y),
                                IM_COL32(165, 42, 42, 255),
                                1.0f);
        }
    }
}

void  drawOffLinePlan::showOuterboundary(){
    if(show_outerboundary_){
        std::ifstream file("/home/zzm/Desktop/test_path_figure-main/src/origin_polygon.txt");
        std::string line;

        while(std::getline(file,line)){
            std::istringstream  iss(line);
            double value;
            std::vector<double> temp;
            while(iss >> value){
                temp.push_back(value);
            }
            storage_outerboundary_.push_back(temp);
        }

        std::vector<ImVec2> storageImPTsSp;
        for(size_t i = 0;i  < storage_outerboundary_[0].size();i++){
            storageImPTsSp.push_back(ImVec2(storage_outerboundary_[0][i] * canvas_scale_,
                                            storage_outerboundary_[1][i] * canvas_scale_));
        }
        //线段展示
        for(size_t i = 0;i < storage_outerboundary_[0].size()-1;i++){
            draw_list_->AddLine(ImVec2(canvas_pos_.x + storageImPTsSp[i].x,
                                       canvas_pos_.y + storageImPTsSp[i].y),
                                ImVec2(canvas_pos_.x + storageImPTsSp[i+1].x,
                                       canvas_pos_.y + storageImPTsSp[i+1].y),
                                IM_COL32(135, 206, 235, 255),
                                1.0f);
        }
    }
}

void  drawOffLinePlan::showCurveDataAttr(){
    if(show_curveDataAttr_){

    }
}

void  drawOffLinePlan::showDrawPolygon(){

    if(show_drawPolygon_){
        //绘制一个透明的按钮覆盖整个画布区域，不过鼠标点击事件
        ImGui::InvisibleButton("Canvas", canvas_size_);

        // 当InvisibleButton被按下（即鼠标点击时），记录顶点
        if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            ImVec2 mouse_pos = ImGui::GetMousePos();
            ImVec2 clicked_pos = ImVec2(mouse_pos.x - canvas_pos_.x,
                                        mouse_pos.y - canvas_pos_.y); // 转换为相对于画布的坐标
            polygonVertices_.push_back(clicked_pos);
        }

        // 绘制多边形的每条边
        for (size_t i = 0; i < polygonVertices_.size(); ++i) {
            ImVec2 p1 = ImVec2(canvas_pos_.x + polygonVertices_[i].x,
                               canvas_pos_.y + polygonVertices_[i].y);
            ImVec2 p2 = ImVec2(canvas_pos_.x + polygonVertices_[(i+1) % polygonVertices_.size()].x,
                               canvas_pos_.y + polygonVertices_[(i+1) % polygonVertices_.size()].y);
            draw_list_->AddLine(p1, p2, IM_COL32(139,119,101, 255), 2.0f);
        }

        // 显示每个顶点的坐标
        for (size_t i = 0; i < polygonVertices_.size(); ++i) {
            ImVec2 p = ImVec2(canvas_pos_.x + polygonVertices_[i].x,
                              canvas_pos_.y + polygonVertices_[i].y);
            std::string coordText = "(" + std::to_string((int)polygonVertices_[i].x)
                                    + ", " + std::to_string((int)polygonVertices_[i].y) + ")";
            draw_list_->AddText(p,
                                IM_COL32(139,119,101, 255),
                                coordText.c_str());
        }
    }
}

void  drawOffLinePlan::processTractorSimulator(){
    ImGui::Begin("tractor simulator",
                 nullptr);

    // 获取绘制命令列表
    draw_list_ = ImGui::GetWindowDrawList();

    if (ImGui::BeginMainMenuBar())
    {
        createFileMenu();
        createCanvasshow();
        createCurveDataShow();
        createPolygonDrawShow();
        ImGui::EndMainMenuBar();
    }

    mouseKeypts();
    mouseRouting();
    mouseOuterboundary();

    // 获取画布的位置和尺寸
    canvas_pos_ = ImGui::GetCursorScreenPos();
    canvas_size_ = ImGui::GetWindowContentRegionMax();
    canvas_size_.x = 1250;
    canvas_size_.y = 1250;

    // 绘制画布
    draw_list_->AddRectFilled(canvas_pos_,
                              ImVec2(canvas_pos_.x + canvas_size_.x,
                                     canvas_pos_.y + canvas_size_.y),
                              IM_COL32(255, 255, 255, 255));

    if(draw_list_){
        showKeypts();
        showRouting();
        showOuterboundary();
        showCurveDataAttr();
        showDrawPolygon();
    }
    // 结束绘制
    ImGui::End();
}

void  drawOffLinePlan::processTableViewWindow(){
    //增加程序运行变量展示
    // 新窗口用于tableView的数据展示
    ImGui::Begin("TableView Window");
    // 在新窗口中展示tableView的数据
    // 使用ImGui的表格控件（Table）或其他适合的控件来展示tableView的数据
    ImGui::Text("TableView Data:");

        // 数据结构定义
    struct TableRow {
        std::string column1;
        std::string column2;
    };

    // 模拟的 TableView 数据
    std::vector<TableRow> tableViewData1 = {
            {"A1", "B1"},
            {"A2", "B2"},
            {"A3", "B3"}
    };


    // 使用Table控件展示tableView的数据
    if (ImGui::BeginTable("table1", 3,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
    {
        // 设置表头
        ImGui::TableSetupColumn("Column 1");
        ImGui::TableSetupColumn("Column 2");
        ImGui::TableHeadersRow();

        // 添加数据
        for (int row = 0; row < tableViewData1.size(); row++)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("Row %d", row);
            ImGui::TableNextColumn();
            ImGui::Text("%s", tableViewData1[row].column1.c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%s", tableViewData1[row].column2.c_str());
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

void  drawOffLinePlan::processDebugDataShow(){
    ImGui::Begin("debug data show");
    ImGui::Text("program Data:");
    // 在ImGui窗口中创建两个输入框
    ImGui::InputFloat2("ridge and curveIndex", ridgeAndCurveIndex_);
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
}

