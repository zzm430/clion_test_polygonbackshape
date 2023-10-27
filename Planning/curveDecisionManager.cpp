//
// Created by zzm on 23-9-26.
//

#include "curveDecisionManager.h"


curveDecisionManager::curveDecisionManager(
                     double ridgeNumber,
                     double ptIndex,
                     std::vector<polygonPoint> arriveLine,
                     std::vector<polygonPoint> leaveLine,
                     std::vector<std::vector<polygonPoint>>& cgalbackShape_keypoints,
                     std::unordered_map<polygonPoint,std::vector<polygonPoint>,polyPointHash>& backshape_fishnail_curve_path)
                     :ridgeNumber_(ridgeNumber),
                      ptIndex_(ptIndex),
                      arriveLine_(arriveLine),
                      leaveLine_(leaveLine),
                      cgalbackShape_keypoints_(cgalbackShape_keypoints),
                      backshape_fishnail_curve_path_(backshape_fishnail_curve_path){
}

void curveDecisionManager::processCurveType(){
    polygonPoint vector_1,vector_2,reference_vector;
    vector_1.x = arriveLine_[1].x - arriveLine_[0].x;
    vector_1.y = arriveLine_[1].y - arriveLine_[0].y;
    vector_2.x = leaveLine_[1].x - leaveLine_[0].x;
    vector_2.y = leaveLine_[1].y - leaveLine_[0].y;
    double angle = common::commonMath::computeTwolineAngleDu(vector_2,vector_1);
    if(angle < 0){
        angle += 360;
    }
    if(JUDGE_CLOCKWISE){
        angle = 360 - angle; //arriveline 往逆时针走偏离的度数
        angleIntManager_  = angle ;
        std::cout << "11111111111111111111111111111111111111111 angle  1 is : "
                  << " angle 2 is : "
                  << angle
                  << " the angle diff is ： "
                  << angleIntManager_
                  << " the i is ： "
                  << ridgeNumber_
                  << " the index is : "
                  << ptIndex_
                  << std::endl;

        if(ridgeNumber_ < 2){                //第1垄和第2垄弯道处理
            if(angleIntManager_> 0 && angleIntManager_ < 30){
                curveType_ = CurveDecision::CONVEX_CORNER;
            }else if(angleIntManager_ > 30  && angleIntManager_ < 150){
                curveType_ = CurveDecision::BORDERLESS_FISHNAIL;
            }else if(angleIntManager_ > 150  &&  angleIntManager_ < 210){
                curveType_ = CurveDecision::CONCAVE_CORNER;
            }else if(angleIntManager_ > 210  &&  angleIntManager_ < 330){
                curveType_ = CurveDecision::BORDERLESS_FISHNAIL;
            }else if(angleIntManager_ > 330 &&  angleIntManager_ < 360){
                curveType_ = CurveDecision::CONCAVE_CORNER;
            }
        }else{                            //当大于第2垄时弯道处理
            if(angleIntManager_> 0 && angleIntManager_ < 30){
                curveType_ = CurveDecision::CONVEX_CORNER;
            }else if(angleIntManager_ > 30  && angleIntManager_ < 150){
                curveType_ = CurveDecision::BORDERLESS_FISHNAIL;
            }else if(angleIntManager_ > 150  &&  angleIntManager_ < 210){
                curveType_ = CurveDecision::CONCAVE_CORNER;
            }else if(angleIntManager_ > 210  &&  angleIntManager_ < 330){
                curveType_ = CurveDecision::BORDERLESS_FISHNAIL;
            }else if(angleIntManager_ > 330 &&  angleIntManager_ < 360){
                curveType_ = CurveDecision::CONCAVE_CORNER;
            }
        }
    }else{
        //to do
    }
}

void curveDecisionManager::processCurvePath(){
    switch(curveType_){
        case CurveDecision::BORDERLESS_FISHNAIL: {
//            processBorderlessFishNail();
              processFTCPACV();
            break;
        }
        case CurveDecision::CONVEX_CORNER:{
            if(ridgeNumber_ == 0 || ridgeNumber_ == 1){
                processFTCPACV();
            }else{
                processCCPA();
            }
            break;
        }
        case CurveDecision::CONCAVE_CORNER:{
            if(ridgeNumber_ == 0 || ridgeNumber_ == 1){
//                processFTCPACC();
            }else{
                processCCPA();
            }
            break;
        }
        case CurveDecision::FT_CPA_CC:{
            processFTCPACC();
            break;
        }
        case CurveDecision::REED_SHEPP:{
            processREEDSHEPP();
            break;
        }
        default:{
            break;
        }
    }
}


void curveDecisionManager::processBorderlessFishNail(){
    cornerTuringLocation   cornerTuringLocationtest(arriveLine_,
                                                    leaveLine_);
    cornerTuringLocationtest.decideLpAandLpB();
    cornerTuringLocationtest.calculatePointsAandBForCurve();
    double angleInt = cornerTuringLocationtest.getCurveAngleInt();
    double arriveLineHeading = cornerTuringLocationtest.getCurveArrriveLineHeading();
    double arriveLineHeading2 = cornerTuringLocationtest.getCurveArrriveLineHeading2();
    arriveLineHeading = arriveLineHeading * M_PI / 180;
    //这里按照逆时针考虑的，所以取负
    arriveLineHeading = -arriveLineHeading;
    double F1 = cornerTuringLocationtest.getCurveaboutF1();
    double F2 = cornerTuringLocationtest.getCurveaboutF2();
    double F3 = cornerTuringLocationtest.getCurveaboutF3();

    polygonPoint pt1 =  cornerTuringLocationtest.getCurveStartPtA();
    polygonPoint pt2 =  cornerTuringLocationtest.getCurveendPtB();

    double RC2 = 6 ;
    cornerTuringTishNail cornerTuringTishNailtest(pt1,pt2,angleInt,RC2,F1,F2,F3);

    std::vector<polygonPoint>  ptAB;
    ptAB.push_back(pt1);
    ptAB.push_back(pt2);
    normalMatrixTranslate   testtemp;
    for(auto it : ptAB){
        auto pt = testtemp.reverseRotatePoint(
                                      it,
                                      cgalbackShape_keypoints_[ridgeNumber_][ptIndex_],
                                      arriveLineHeading);
//                LOG(INFO)  << "the pt transd   is : " << pt.x << " " << pt.y ;
    }

    //计算关键点对应的鱼尾路径点并一一对应存储
    auto local_C1path = cornerTuringTishNailtest.getFishNailC1path();
    auto local_C2path = cornerTuringTishNailtest.getFishNailC2path();
    auto local_C3path = cornerTuringTishNailtest.getFishNailC3path();

//  auto local_C1path = cornerTuringTishNailtest.getC1path();
//  auto local_C2path = cornerTuringTishNailtest.getC2path();
//  auto local_C3path = cornerTuringTishNailtest.getC3path();

    std::vector<polygonPoint> temp_test_C1Path ;
    std::vector<polygonPoint> temp_test_C2Path;
    std::vector<polygonPoint> temp_test_C3Path;

    //转换到世界坐标系下
    normalMatrixTranslate  normalMatrixTranslateInstance;

    std::vector<polygonPoint>  storage_origin_path;
    int num_size_C1 = local_C1path.size();
    int num_size_C2 = local_C2path.size();
    int num_size_C3 = local_C3path.size();

    for(int m = 0 ;m <  num_size_C1 - 1;m++){
        auto tempPt = normalMatrixTranslateInstance.reverseRotatePoint(
                local_C1path[m],
                cgalbackShape_keypoints_[ridgeNumber_][ptIndex_],
                arriveLineHeading);
        tempPt.pathPtType_ = pathPtType::FORWARD;
        temp_test_C1Path.push_back(tempPt);
        storage_origin_path.push_back(tempPt);
    }

    auto C1_LAST_PT = local_C1path[num_size_C1-1];
    auto tempPt = normalMatrixTranslateInstance.reverseRotatePoint(
            C1_LAST_PT,
            cgalbackShape_keypoints_[ridgeNumber_][ptIndex_],
            arriveLineHeading);
    tempPt.pathPtType_ = pathPtType::SWITCHPT;
    temp_test_C1Path.push_back(tempPt);
    storage_origin_path.push_back(tempPt);

    for(int m =0 ;m <  num_size_C2 -1;m++){
        auto tempPt = normalMatrixTranslateInstance.reverseRotatePoint(
                local_C2path[m],
               cgalbackShape_keypoints_[ridgeNumber_][ptIndex_],
                arriveLineHeading);
        tempPt.pathPtType_ = pathPtType::BACKWARD;
        temp_test_C2Path.push_back(tempPt);
        storage_origin_path.push_back(tempPt);
    }

    auto C2_LAST_PT = local_C2path[num_size_C2-1];
    auto tempPt1 = normalMatrixTranslateInstance.reverseRotatePoint(
            C2_LAST_PT,
           cgalbackShape_keypoints_[ridgeNumber_][ptIndex_],
            arriveLineHeading);
    tempPt1.pathPtType_ = pathPtType::SWITCHPT;
    temp_test_C2Path.push_back(tempPt);
    storage_origin_path.push_back(tempPt1);

    for(int m =0 ;m <  num_size_C3;m++){
        auto tempPt = normalMatrixTranslateInstance.reverseRotatePoint(
                local_C3path[m],
                cgalbackShape_keypoints_[ridgeNumber_][ptIndex_],
                arriveLineHeading);
        tempPt.pathPtType_ = pathPtType::FORWARD;
        temp_test_C3Path.push_back(tempPt);
        storage_origin_path.push_back(tempPt);
    }
    backshape_fishnail_curve_path_[cgalbackShape_keypoints_[ridgeNumber_][ptIndex_]] = storage_origin_path;

    //将A、B点转入到世界坐标系下
    std::string ptname = "/home/zzm/Desktop/test_path_figure-main/src/ptsshow.txt";
    auto & ptsshow =
            common::Singleton::GetInstance<filePrint2>(ptname);

    for(auto it : ptAB){
        auto tempfg = normalMatrixTranslateInstance.reverseRotatePoint(
                it,
                cgalbackShape_keypoints_[ridgeNumber_][ptIndex_],
                arriveLineHeading);
        ptsshow.writePt(tempfg);
    }

    if(ptIndex_ == 5 && ridgeNumber_ == 0){
        std::string C1name = "/home/zzm/Desktop/test_path_figure-main/src/C1path.txt";
        std::string C2name = "/home/zzm/Desktop/test_path_figure-main/src/C2path.txt";
        std::string C3name = "/home/zzm/Desktop/test_path_figure-main/src/C3path.txt";
        normalPrint C1file(C1name);
        normalPrint C2file(C2name);
        normalPrint C3file(C3name);
        C1file.writePts(temp_test_C1Path);
        C2file.writePts(temp_test_C2Path);
        C3file.writePts(temp_test_C3Path);
    }
}

void curveDecisionManager::processBorderlessFishNail(polygonPoint curvePt){
    auto ordered_pt = curvePt;  //这里的curvePt指的是弯道需要的映射点并不是交点

    //将arriveline 和leaveline延长一定距离，并求交点
    auto extend_arriveline = common::commonMath::findPointExtendSegment2(arriveLine_[0],arriveLine_[1],10);
    auto extend_leaveline = common::commonMath::findPointExtendSegment2(leaveLine_[0],leaveLine_[1],10);

    Segment seg1(pointbst(extend_arriveline[0].x, extend_arriveline[0].y), pointbst(extend_arriveline[1].x, extend_arriveline[1].y));
    Segment seg2(pointbst(extend_leaveline[0].x, extend_leaveline[0].y), pointbst(extend_leaveline[1].x, extend_leaveline[1].y));
    std::deque<pointbst> output1;
    boost::geometry::intersection(seg1, seg2, output1);

    if (!output1.empty()){
        std::cout << " Entrance Curve Intersection point: ("
                  << output1.front().x()
                  << ", "
                  << output1.front().y()
                  << ")"
                  << std::endl;
    }
    polygonPoint   IntersecPt(output1.front().x(),output1.front().y());

    cornerTuringLocation   cornerTuringLocationtest(extend_arriveline,
                                                    extend_leaveline);
    cornerTuringLocationtest.decideLpAandLpB();
    cornerTuringLocationtest.calculatePointsAandBForCurve();
    double angleInt = cornerTuringLocationtest.getCurveAngleInt();
    double arriveLineHeading = cornerTuringLocationtest.getCurveArrriveLineHeading();
    arriveLineHeading = arriveLineHeading * M_PI / 180;
    //这里按照逆时针考虑的，所以取负
    arriveLineHeading = -arriveLineHeading;
    double F1 = cornerTuringLocationtest.getCurveaboutF1();
    double F2 = cornerTuringLocationtest.getCurveaboutF2();
    double F3 = cornerTuringLocationtest.getCurveaboutF3();

    polygonPoint pt1 =  cornerTuringLocationtest.getCurveStartPtA();
    polygonPoint pt2 =  cornerTuringLocationtest.getCurveendPtB();

    double RC2 = CIRCLE_RIDIS_R + 0.5;
    cornerTuringTishNail cornerTuringTishNailtest(pt1,pt2,angleInt,RC2,F1,F2,F3);
    LOG(INFO) << "RC2 is : " << RC2;

    std::vector<polygonPoint>  ptAB;
    ptAB.push_back(pt1);
    ptAB.push_back(pt2);
    normalPrint   printAB("/home/zzm/Desktop/test_path_figure-main/src/testAB.txt");
    printAB.writePts(ptAB);

    normalMatrixTranslate   testtemp;
    for(auto it : ptAB){
        auto pt = testtemp.reverseRotatePoint(it,IntersecPt,
                                              arriveLineHeading);
        LOG(INFO)  << "the pt transd   is : " << pt.x << " " << pt.y ;
    }

    //计算关键点对应的鱼尾路径点并一一对应存储
    auto local_C1path = cornerTuringTishNailtest.getFishNailC1path();
    auto local_C2path = cornerTuringTishNailtest.getFishNailC2path();
    auto local_C3path = cornerTuringTishNailtest.getFishNailC3path();

//  auto local_C1path = cornerTuringTishNailtest.getC1path();
//  auto local_C2path = cornerTuringTishNailtest.getC2path();
//  auto local_C3path = cornerTuringTishNailtest.getC3path();

    //转换到世界坐标系下
    normalMatrixTranslate  normalMatrixTranslateInstance;

    std::vector<polygonPoint>  storage_origin_path;
    int num_size_C1 = local_C1path.size();
    int num_size_C2 = local_C2path.size();
    int num_size_C3 = local_C3path.size();

    for(int m = 0 ;m <  num_size_C1 - 1;m++){
        auto tempPt = normalMatrixTranslateInstance.reverseRotatePoint(
                local_C1path[m],
                IntersecPt,
                arriveLineHeading);
        tempPt.pathPtType_ = pathPtType::FORWARD;
        storage_origin_path.push_back(tempPt);
    }

    auto C1_LAST_PT = local_C1path[num_size_C1-1];
    auto tempPt = normalMatrixTranslateInstance.reverseRotatePoint(
            C1_LAST_PT,
            IntersecPt,
            arriveLineHeading);
    tempPt.pathPtType_ = pathPtType::SWITCHPT;
    storage_origin_path.push_back(tempPt);

    for(int m =0 ;m <  num_size_C2 -1;m++){
        auto tempPt = normalMatrixTranslateInstance.reverseRotatePoint(
                local_C2path[m],
                IntersecPt,
                arriveLineHeading);
        tempPt.pathPtType_ = pathPtType::BACKWARD;
        storage_origin_path.push_back(tempPt);
    }

    auto C2_LAST_PT = local_C2path[num_size_C2-1];
    auto tempPt1 = normalMatrixTranslateInstance.reverseRotatePoint(
            C2_LAST_PT,
            IntersecPt,
            arriveLineHeading);
    tempPt1.pathPtType_ = pathPtType::SWITCHPT;
    storage_origin_path.push_back(tempPt1);

    for(int m =0 ;m <  num_size_C3;m++){
        auto tempPt = normalMatrixTranslateInstance.reverseRotatePoint(
                local_C3path[m],
                IntersecPt,
                arriveLineHeading);
        tempPt.pathPtType_ = pathPtType::FORWARD;
        storage_origin_path.push_back(tempPt);
    }
    backshape_fishnail_curve_path_[curvePt] = storage_origin_path;
}



void curveDecisionManager::processCCPA(){
    //外扩arriveLine\leaveLine (垄宽/2)
    auto tempLeaveLine  = leaveLine_;
    std::cout << "leave line 0 and 1 is : "
              << leaveLine_[0].x
              << " "
              << leaveLine_[0].y
              << std::endl;

    //利用leaveline[0]计算垂足点
    auto footPt_1 = common::commonMath::computeFootPoint(
                                               leaveLine_[1],
                                               arriveLine_[0],
                                               arriveLine_[1]);

    auto footPt_2 = common::commonMath::computeFootPoint(
                                               arriveLine_[0],
                                               leaveLine_[0],
                                               leaveLine_[1]);

    std::vector<polygonPoint>   direction_line_1,direction_line_2;
    if(curveType_ == CurveDecision::CONCAVE_CORNER){
        direction_line_1.push_back(footPt_1);
        direction_line_1.push_back(leaveLine_[1]);

        direction_line_2.push_back(footPt_2);
        direction_line_2.push_back(arriveLine_[0]);
    } else if(curveType_ == CurveDecision::CONVEX_CORNER){
        direction_line_1.push_back(leaveLine_[1]);
        direction_line_1.push_back(footPt_1);

        direction_line_2.push_back(arriveLine_[0]);
        direction_line_2.push_back(footPt_2);
    }

    auto  extendArriveLine = common::commonMath::computeLineTranslationPoints(
            arriveLine_,
            direction_line_1,
            RIDGE_WIDTH_LENGTH/2);
    auto tempArriveLine = arriveLine_;

    auto extendLeaveLine = common::commonMath::computeLineTranslationPoints(
            leaveLine_,
            direction_line_2,
            RIDGE_WIDTH_LENGTH/2);
    //将extendArriveLine 和extendLeaveLine两端延长20m
    auto extendArriveLine_1 = common::commonMath::findPointExtendSegment2(
                                                                        extendArriveLine[0],
                                                                        extendArriveLine[1],
                                                                        20);
    auto  extendLeaveLine_1 =  common::commonMath::findPointExtendSegment2(
                                                                        extendLeaveLine[0],
                                                                        extendLeaveLine[1],
                                                                        20);
   
    //计算extendArriveLine 和extendLeaveLine的交点
    Segment seg1(pointbst(extendArriveLine_1[0].x, extendArriveLine_1[0].y),
                 pointbst(extendArriveLine_1[1].x, extendArriveLine_1[1].y));
    Segment seg2(pointbst(extendLeaveLine_1[0].x, extendLeaveLine_1[0].y),
                  pointbst(extendLeaveLine_1[1].x, extendLeaveLine_1[1].y));
    std::deque<pointbst> output1;
    boost::geometry::intersection(seg1, seg2, output1);
    if (!output1.empty()){
        std::cout << "Intersection point: ("
                  << output1.front().x()
                  << ", "
                  << output1.front().y()
                  << ")"
                  << std::endl;
    }
    polygonPoint  referencePt(output1.front().x(),output1.front().y());
    cornerTuringLocation   cornerTuringLocationtest(extendArriveLine,
                                                    extendLeaveLine);
    cornerTuringLocationtest.decideLpAandLpB();
    cornerTuringLocationtest.calculatePointsAandBForCurve();
    double angleInt = cornerTuringLocationtest.getCurveAngleInt();
    double tempfg = cornerTuringLocationtest.getCurveCCPAAngleInt();
    double arriveLineHeading = cornerTuringLocationtest.getCurveArrriveLineHeading();
    double arriveLineHeading2 = cornerTuringLocationtest.getCurveArrriveLineHeading2();
    arriveLineHeading = arriveLineHeading * M_PI / 180;
    //这里按照逆时针考虑的，所以取负
    arriveLineHeading = -arriveLineHeading;
    arriveLineHeading_ = arriveLineHeading;

    //添加验证C-CPA代码
    if(curveType_ == CurveDecision::CONCAVE_CORNER){
        //扩展线段显示
        if(ridgeNumber_ == 0 && ptIndex_ == 4) {
            std::ofstream temp;
            temp.open("/home/zzm/Desktop/test_path_figure-main/src/extendArriveAndLeaveline.txt",
                      std::ios::out);
//        temp << " " << extendArriveLine_1[0].x << " " << extendArriveLine_1[1].x <<
//             " " << extendLeaveLine_1[0].x << " " << extendLeaveLine_1[1].x << std::endl;
//        temp << " " << extendArriveLine_1[0].y << " " << extendArriveLine_1[1].y <<
//             " " << extendLeaveLine_1[0].y << " " << extendLeaveLine_1[1].y << std::endl;
            temp << " " << extendArriveLine[0].x << " " << extendArriveLine[1].x <<
                 " " << extendLeaveLine[0].x << " " << extendLeaveLine[1].x << std::endl;
            temp << " " << extendArriveLine[0].y << " " << extendArriveLine[1].y <<
                 " " << extendLeaveLine[0].y << " " << extendLeaveLine[1].y << std::endl;
            temp.close();

            cornerTuringImplementRadius cornerTuringImplementRadiusInstance;
            cornerTuringImplementRadiusInstance.calculateMiniTuringRadiusConsiderImplement();
            auto Rsw = cornerTuringImplementRadiusInstance.getRsw();
            cornerTuringCCPAAlgorithm cornerTuringCCPAAlgorithm1(
                    tempfg,
                    Rsw,
                    1,
                    false,
                    cgalbackShape_keypoints_[ridgeNumber_][ptIndex_],
                    referencePt,
                    arriveLineHeading);
            cornerTuringCCPAAlgorithm1.calculateAngleCC2();
            cornerTuringCCPAAlgorithm1.calculateCirclesCenter();
            cornerTuringCCPAAlgorithm1.calculateNewFieldBorder();
            cornerTuringCCPAAlgorithm1.calculatePath();
            std::cout << "the angleInt is : " << angleInt * 180 / M_PI;
            auto all_path = cornerTuringCCPAAlgorithm1.getAllPath();
            backshape_fishnail_curve_path_[cgalbackShape_keypoints_[ridgeNumber_][ptIndex_]] = all_path;

            auto allLocalPath = cornerTuringCCPAAlgorithm1.getAllLocalPath();
            //验证拖拉机姿态
            //计算拖拉机的轮廓点
            for(int  im = 1 ;im < all_path.size();im++) {
                tractorPolygonShow tractorPolygonShowInstance(im,
                                                              all_path);
                auto tractorHeadPts = tractorPolygonShowInstance.getTractorPolygonHeadPts();
                auto tractorTailPts = tractorPolygonShowInstance.getTractorPolygonTailPts();
                std::string test1 =  "/home/zzm/Desktop/test_path_figure-main/src/tractorHeadPtsStream.txt";
                auto & tractorHeadPtsStream = common::Singleton::GetInstance<tractorPolyPrint>(test1);
                tractorHeadPtsStream.writePts(tractorHeadPts,tractorTailPts);
            }
        }
    } else if(curveType_ == CurveDecision::CONVEX_CORNER) {
        if(ridgeNumber_ == 0 && ptIndex_ == 1 ){
            std::ofstream   temp;
            temp.open("/home/zzm/Desktop/test_path_figure-main/src/extendArriveAndLeaveline1.txt",
                      std::ios::out);
            temp << " " << extendArriveLine[0].x << " " << extendArriveLine[1].x <<
                 " " << extendLeaveLine[0].x << " " << extendLeaveLine[1].x << std::endl;
            temp << " " << extendArriveLine[0].y << " " << extendArriveLine[1].y <<
                 " " << extendLeaveLine[0].y << " " << extendLeaveLine[1].y << std::endl;
            temp.close();

            cornerTuringImplementRadius cornerTuringImplementRadiusInstance;
            cornerTuringImplementRadiusInstance.calculateMiniTuringRadiusConsiderImplement();
            auto Rsw = cornerTuringImplementRadiusInstance.getRsw();
            cornerTuringCCPAAlgorithm  cornerTuringCCPAAlgorithm1(
                    tempfg,
                    Rsw,
                    1,
                    true,
                    cgalbackShape_keypoints_[ridgeNumber_][ptIndex_],
                    referencePt,
                    arriveLineHeading);
            cornerTuringCCPAAlgorithm1.calculateAngleCC2();
            cornerTuringCCPAAlgorithm1.calculateCirclesCenter();
            cornerTuringCCPAAlgorithm1.calculateNewFieldBorder();
            cornerTuringCCPAAlgorithm1.calculatePath();
            std::cout << "the angleInt is : " << angleInt * 180 / M_PI;
            auto all_path = cornerTuringCCPAAlgorithm1.getAllPath();
            backshape_fishnail_curve_path_[cgalbackShape_keypoints_[ridgeNumber_][ptIndex_]] = all_path;
        }
    }
}

void curveDecisionManager::processFTCPACC(){
    if(ridgeNumber_ == 0 && ptIndex_ == 2){
        cornerTuringFTCPACCAlgorithm cornerTuringFTCPACCAlgorithmInstance(
                arriveLine_,
                leaveLine_);
        auto all_path = cornerTuringFTCPACCAlgorithmInstance.getPath();
        backshape_fishnail_curve_path_[cgalbackShape_keypoints_[ridgeNumber_][ptIndex_]] = all_path;
    }
}

void curveDecisionManager::processFTCPACV(){
    if(ridgeNumber_ == 0 && ptIndex_ == 5){
        //将arriveline、leaveLine 外扩 RIDGE_WIDTH_LENGTH/2
        //利用leaveline[0]计算垂足点
        auto footPt_1 = common::commonMath::computeFootPoint(
                leaveLine_[1],
                arriveLine_[0],
                arriveLine_[1]);

        auto footPt_2 = common::commonMath::computeFootPoint(
                arriveLine_[0],
                leaveLine_[0],
                leaveLine_[1]);

        std::vector<polygonPoint>   direction_line_1,direction_line_2;

        direction_line_1.push_back(leaveLine_[1]);
        direction_line_1.push_back(footPt_1);

        direction_line_2.push_back(arriveLine_[0]);
        direction_line_2.push_back(footPt_2);

        auto  extendArriveLine = common::commonMath::computeLineTranslationPoints(
                arriveLine_,
                direction_line_1,
                2.3);
        auto tempArriveLine = arriveLine_;
        auto extendLeaveLine = common::commonMath::computeLineTranslationPoints(
                leaveLine_,
                direction_line_2,
                2.3);
        //将extendArriveLine 和extendLeaveLine两端延长20m
        auto extendArriveLine_1 = common::commonMath::findPointExtendSegment2(
                extendArriveLine[0],
                extendArriveLine[1],
                20);
        auto  extendLeaveLine_1 =  common::commonMath::findPointExtendSegment2(
                extendLeaveLine[0],
                extendLeaveLine[1],
                20);

#ifdef  DEBUG_CPA_INFO
        std::ofstream   temp;
        temp.open("/home/zzm/Desktop/test_path_figure-main/src/extendArriveAndLeavelineFTCPACC11.txt",
                  std::ios::out);
        temp << " " << extendArriveLine_1[0].x << " " << extendArriveLine_1[1].x <<
             " " << extendLeaveLine_1[0].x << " " << extendLeaveLine_1[1].x << std::endl;
        temp << " " << extendArriveLine_1[0].y << " " << extendArriveLine_1[1].y <<
             " " << extendLeaveLine_1[0].y << " " << extendLeaveLine_1[1].y << std::endl;
        temp.close();
#endif

        //确定弯道处理的关键点
        turingFtcpacvLocation turingFtcpacvLocationInstance(
                arriveLine_,
                leaveLine_,
                ridgeNumber_);
        turingFtcpacvLocationInstance.calculatePointsAandBForCurve();
        auto pt1 = turingFtcpacvLocationInstance.getCurveStartPtA();
        auto pt2 = turingFtcpacvLocationInstance.getCurveendPtB();
        auto pt3 = turingFtcpacvLocationInstance.getCurveStartPtARobot();
        auto pt4 = turingFtcpacvLocationInstance.getCurveendPtBRobot();
        auto pt5 = turingFtcpacvLocationInstance.getCurveStartPtAIm();
        auto pt6 = turingFtcpacvLocationInstance.getCurveEndPtBIm();
        auto pt7 = turingFtcpacvLocationInstance.getCurveStartPtAWorkarea();
        auto pt8 = turingFtcpacvLocationInstance.getCurveEndPtBWorkarea();
        auto angleInt = turingFtcpacvLocationInstance.getCurveAngleInt();
        std::cout << "the real angleInt is : " << angleInt << std::endl;
        std::vector<polygonPoint>  stor_pts;
        stor_pts.push_back(pt1);
        stor_pts.push_back(pt2);
//        stor_pts.push_back(pt3);
//        stor_pts.push_back(pt4);
//        stor_pts.push_back(pt5);
//        stor_pts.push_back(pt6);
//        stor_pts.push_back(pt7);
//        stor_pts.push_back(pt8);

        auto arriveLineHeading = turingFtcpacvLocationInstance.getArriveLineHeading();
        arriveLineHeading = arriveLineHeading * M_PI / 180;
        //这里按照逆时针考虑的，所以取负
        arriveLineHeading_ = -arriveLineHeading;
        // 将坐标转换为相对于新坐标系的偏移量
        for(auto& i : stor_pts){
            double offsetX = i.x;
            double offsetY = i.y;
            // 计算逆向旋转后的坐标
            double reversedX = offsetX * cos(arriveLineHeading_) -  offsetY * sin(arriveLineHeading_);
            double reversedY = offsetY * cos(arriveLineHeading_) +  offsetX * sin(arriveLineHeading_) ;

            // 将逆向旋转后的坐标转换为原始坐标系
            polygonPoint reversedPoint;
            reversedPoint.x = reversedX + cgalbackShape_keypoints_[ridgeNumber_][ptIndex_].x;
            reversedPoint.y = reversedY + cgalbackShape_keypoints_[ridgeNumber_][ptIndex_].y;
            i.x = reversedPoint.x;
            i.y = reversedPoint.y;
        }

#ifdef   DEBUG_CPA_INFO
        std::ofstream   temp1;
        temp1.open("/home/zzm/clion_test_polygonbackshape/tools/test1111.txt",std::ios::out);
        for(auto i : stor_pts){
            temp1 << " " << i.x << " " << i.y << std::endl;
        }
        temp1.close();
        //构造FT-CPA-CV使用时的轮廓可视化
        std::vector<polygonPoint>  polygon_A_show;
        std::vector<polygonPoint>  polygon_B_show;

        polygon_A_show.push_back(arriveLine_[1]);
        polygon_A_show.push_back(stor_pts[0]);
        polygon_B_show.push_back(leaveLine_[0]);
        polygon_B_show.push_back(stor_pts[1]);

        tractorPolygonShow tractorPolygonShowInstanceA(1,polygon_A_show);
        auto pts_1 = tractorPolygonShowInstanceA.getTractorPolygonHeadPts();
        auto pts_2 = tractorPolygonShowInstanceA.getTractorPolygonTailPts();
        tractorPolygonShow tractorPolygonShowInstanceB(1,polygon_B_show);
        auto pts_3 = tractorPolygonShowInstanceB.getTractorPolygonHeadPts();
        auto pts_4 = tractorPolygonShowInstanceB.getTractorPolygonTailPts();

        std::string test1 =  "/home/zzm/Desktop/test_path_figure-main/src/tractorHeadPtsStream111.txt";
        auto & tractorHeadPtsStream = common::Singleton::GetInstance<tractorPolyPrint2>(test1);
        tractorHeadPtsStream.writePts(pts_1,pts_2);
        tractorHeadPtsStream.writePts(pts_3,pts_4);
#endif
        cornerTuringFTCPACVAlgorithm cornerTuringFTCPACVAlgorithmInstance(
                                                                  angleInt,
                                                                  cgalbackShape_keypoints_[ridgeNumber_][ptIndex_],
                                                                  arriveLineHeading_,
                                                                  stor_pts[0],
                                                                  stor_pts[1],
                                                                  arriveLine_,
                                                                  leaveLine_);
        cornerTuringFTCPACVAlgorithmInstance.computeLimitPtInPart2();
        cornerTuringFTCPACVAlgorithmInstance.computeTheAnglesForFTCPACV();
        cornerTuringFTCPACVAlgorithmInstance.SelectTheRequiredParts();
        cornerTuringFTCPACVAlgorithmInstance.computePath();
        auto pts = cornerTuringFTCPACVAlgorithmInstance.getPathAboutAll();
        auto path1 = cornerTuringFTCPACVAlgorithmInstance.getPathStraight1();
        auto path2 = cornerTuringFTCPACVAlgorithmInstance.getPathStraight2();

#ifdef DEBUG_CPA_INFO
        //对整个FT-CPA-CV的path点位进行展示
        std::vector<polygonPoint>  storageALLPath;
        std::reverse(path1.begin(),path1.end());
        std::reverse(path2.begin(),path2.end());
        for(auto i : path1){
            storageALLPath.push_back(i);
        }
        for(auto i : pts){
            storageALLPath.push_back(i);
        }
        for(auto i: path2){
            storageALLPath.push_back(i);
        }

        std::ofstream   testFTCPACV;
        testFTCPACV.open("/home/zzm/Desktop/test_path_figure-main/src/testFTCPACV.txt",std::ios::out);
        for(auto i : pts){
            testFTCPACV << " " << i.x ;
        }
        testFTCPACV << std::endl;
        for(auto j : pts){
            testFTCPACV <<" " << j.y;
        }
        testFTCPACV << std::endl;
        testFTCPACV.close();

        //记录曲率相关数据
        curveCurvatureCalculate curveCurvatureCalculateInstance(pts);
        auto kData = curveCurvatureCalculateInstance.getPathPtsR();
        std::ofstream  testFTCPACVK;
        testFTCPACVK.open("/home/zzm/Desktop/test_path_figure-main/src/testFTCPACVK.txt",std::ios::out);
        for(auto i : kData){
             testFTCPACVK << " " << i;
        }
        testFTCPACVK << std::endl;
        testFTCPACVK.close();

        //针对FT-CPA-CV算法进行姿态展示
        for (int i  = 1;i < storageALLPath.size();i++){
            tractorPolygonShow tractorPolygonShowInstanceM(i,storageALLPath);
            auto pts_1 = tractorPolygonShowInstanceM.getTractorPolygonHeadPts();
            auto pts_2 = tractorPolygonShowInstanceM.getTractorPolygonTailPts();
            std::string test1 =  "/home/zzm/Desktop/test_path_figure-main/src/tractorFTCPACVPOLYGON1.txt";
            auto & tractorHeadPtsStream = common::Singleton::GetInstance<tractorPolyPrint>(test1);
            tractorHeadPtsStream.writePts(pts_1,pts_2);
        }
#endif
    }
}

void curveDecisionManager::processCCCURVE(){

}

void curveDecisionManager::processREEDSHEPP(){

}

void curveDecisionManager::changeState(CurveDecision  state){
    curveType_ = state;
}

double curveDecisionManager::getArriveLineHeading(){
    return  arriveLineHeading_;
}