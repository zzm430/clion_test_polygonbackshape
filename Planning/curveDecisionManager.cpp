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

        angleInt_  = angle ;
        std::cout << "11111111111111111111111111111111111111111 angle  1 is : "
                  << " angle 2 is : "
                  << angle
                  << " the angle diff is ： "
                  << angleInt_
                  << " the i is ： "
                  << ridgeNumber_
                  << " the index is : "
                  << ptIndex_
                  << std::endl;

        if(ridgeNumber_ < 2){                //第1垄和第2垄弯道处理
            if(angleInt_> 0 && angleInt_ < 30){
                curveType_ = CurveDecision::CONVEX_CORNER;
            }else if(angleInt_ > 30  && angleInt_ < 150){
                curveType_ = CurveDecision::BORDERLESS_FISHNAIL;
            }else if(angleInt_ > 150  &&  angleInt_ < 210){
                curveType_ = CurveDecision::CONCAVE_CORNER;
            }else if(angleInt_ > 210  &&  angleInt_ < 330){
                curveType_ = CurveDecision::BORDERLESS_FISHNAIL;
            }else if(angleInt_ > 330 &&  angleInt_ < 360){
                curveType_ = CurveDecision::CONCAVE_CORNER;
            }
        }else{                            //当大于第2垄时弯道处理
            if(angleInt_> 0 && angleInt_ < 30){
                curveType_ = CurveDecision::CONVEX_CORNER;
            }else if(angleInt_ > 30  && angleInt_ < 150){
                curveType_ = CurveDecision::BORDERLESS_FISHNAIL;
            }else if(angleInt_ > 150  &&  angleInt_ < 210){
                curveType_ = CurveDecision::CONCAVE_CORNER;
            }else if(angleInt_ > 210  &&  angleInt_ < 330){
                curveType_ = CurveDecision::BORDERLESS_FISHNAIL;
            }else if(angleInt_ > 330 &&  angleInt_ < 360){
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
            processBorderlessFishNail();
            break;
        }
        case CurveDecision::CONVEX_CORNER:{
            processCCPA();
            break;
        }
        case CurveDecision::CONCAVE_CORNER:{
            processCCPA();
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

    if(ptIndex_ == 2){
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

//            auto local_C1path = cornerTuringTishNailtest.getC1path();
//            auto local_C2path = cornerTuringTishNailtest.getC2path();
//            auto local_C3path = cornerTuringTishNailtest.getC3path();

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
    std::reverse(tempLeaveLine.begin(),tempLeaveLine.end());
    auto  extendArriveLine = common::commonMath::computeLineTranslationPoints(
            arriveLine_,
            tempLeaveLine,
            RIDGE_WIDTH_LENGTH/2);
    auto tempArriveLine = arriveLine_;
    std::reverse(tempArriveLine.begin(),tempArriveLine.end());
    auto extendLeaveLine = common::commonMath::computeLineTranslationPoints(
            leaveLine_,
            tempArriveLine,
            RIDGE_WIDTH_LENGTH/2);
    //将extendArriveLine 和extendLeaveLine两端延长10m
    auto extendArriveLine_1 = common::commonMath::findPointExtendSegment2(
                                                                        extendArriveLine[0],
                                                                        extendArriveLine[1],
                                                                        10);
    auto extendLeaveLine_1 =  common::commonMath::findPointExtendSegment2(
                                                                        extendLeaveLine[0],
                                                                        extendLeaveLine[1],
                                                                        10);
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
    double arriveLineHeading = cornerTuringLocationtest.getCurveArrriveLineHeading();
    double arriveLineHeading2 = cornerTuringLocationtest.getCurveArrriveLineHeading2();
    arriveLineHeading = arriveLineHeading * M_PI / 180;
    //这里按照逆时针考虑的，所以取负
    arriveLineHeading = -arriveLineHeading;

    //添加验证C-CPA代码
    if(ridgeNumber_ == 0 && ptIndex_ == 2){
        cornerTuringImplementRadius cornerTuringImplementRadiusInstance;
        cornerTuringImplementRadiusInstance.calculateMiniTuringRadiusConsiderImplement();
        auto Rsw = cornerTuringImplementRadiusInstance.getRsw();
        cornerTuringCCPAAlgorithm  cornerTuringCCPAAlgorithm1(
                angleInt,
                Rsw,
                1,
                false,
                cgalbackShape_keypoints_[ridgeNumber_][ptIndex_],
                referencePt,
                arriveLineHeading);
        cornerTuringCCPAAlgorithm1.calculateAngleCC2();
        cornerTuringCCPAAlgorithm1.calculateCirclesCenter();
        cornerTuringCCPAAlgorithm1.calculatePath();
        std::cout << "the angleInt is : " << angleInt * 180 / M_PI;
        auto all_path = cornerTuringCCPAAlgorithm1.getAllPath();
        backshape_fishnail_curve_path_[cgalbackShape_keypoints_[ridgeNumber_][ptIndex_]] = all_path;
    }
}

void curveDecisionManager::processFTCPACC(){

}

void curveDecisionManager::processCCCURVE(){

}

void curveDecisionManager::processREEDSHEPP(){

}


void curveDecisionManager::changeState(CurveDecision  state){
    curveType_ = state;
}