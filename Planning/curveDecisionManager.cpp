//
// Created by zzm on 23-9-26.
//

#include "curveDecisionManager.h"

curveDecisionManager::curveDecisionManager(double angleTwoline)
                             :angleInt_(angleTwoline) {}

void curveDecisionManager::setCurvePara(
                                double angleInt,
                                double ridgeNumber,
                                double ptIndex,
                                std::vector<polygonPoint> arriveLine,
                                std::vector<polygonPoint> leaveLine,
                                aiforce::Route_Planning::pathPolygonPlan & path_frame){
    angleInt_ = angleInt;
    ridgeNumber_ = ridgeNumber;
    ptIndex_ = ptIndex;
    arriveLine_ = arriveLine;
    leaveLine_ = leaveLine;
}

void curveDecisionManager::processCurveType(){
    if(ridgeNumber_ < 2){                //第1垄和第2垄弯道处理
        if(angleInt_> 0 && angleInt_ < 150){
            curveType_ = CurveDecision::BORDERLESS_FISHNAIL;
        }else if(angleInt_ > 150 < angleInt_ < 180){
            curveType_ = CurveDecision::CONVEX_CORNER;
        }else if(angleInt_ > 180 < angleInt_ < 210){
            curveType_ = CurveDecision::CONCAVE_CORNER;
        }else if(angleInt_ > 210 < angleInt_ < 360){
            curveType_ = CurveDecision::BORDERLESS_FISHNAIL;
        }
    }else{                            //当大于第2垄时弯道处理
        if(angleInt_> 0 && angleInt_ < 150){
            curveType_ = CurveDecision::BORDERLESS_FISHNAIL;
        }else if(angleInt_ > 150 < angleInt_ < 180){
            curveType_ = CurveDecision::CONVEX_CORNER;
        }else if(angleInt_ > 180 < angleInt_ < 210){
            curveType_ = CurveDecision::CONCAVE_CORNER;
        }else if(angleInt_ > 210 < angleInt_ < 360){
            curveType_ = CurveDecision::BORDERLESS_FISHNAIL;
        }
    }
}

void curveDecisionManager::processCurvePath(
                 aiforce::Route_Planning::pathPolygonPlan & path_frame){
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


void curveDecisionManager::processBorderlessFishNail(
        aiforce::Route_Planning::pathPolygonPlan & path_frame){

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
                                      path_frame.cgalbackShape_keypoints_[ridgeNumber_][ptIndex_],
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
                path_frame.cgalbackShape_keypoints_[ridgeNumber_][ptIndex_],
                arriveLineHeading);
        tempPt.pathPtType_ = pathPtType::FORWARD;
        temp_test_C1Path.push_back(tempPt);
        storage_origin_path.push_back(tempPt);
    }

    auto C1_LAST_PT = local_C1path[num_size_C1-1];
    auto tempPt = normalMatrixTranslateInstance.reverseRotatePoint(
            C1_LAST_PT,
            path_frame.cgalbackShape_keypoints_[ridgeNumber_][ptIndex_],
            arriveLineHeading);
    tempPt.pathPtType_ = pathPtType::SWITCHPT;
    temp_test_C1Path.push_back(tempPt);
    storage_origin_path.push_back(tempPt);

    for(int m =0 ;m <  num_size_C2 -1;m++){
        auto tempPt = normalMatrixTranslateInstance.reverseRotatePoint(
                local_C2path[m],
                path_frame.cgalbackShape_keypoints_[ridgeNumber_][ptIndex_],
                arriveLineHeading);
        tempPt.pathPtType_ = pathPtType::BACKWARD;
        temp_test_C2Path.push_back(tempPt);
        storage_origin_path.push_back(tempPt);
    }

    auto C2_LAST_PT = local_C2path[num_size_C2-1];
    auto tempPt1 = normalMatrixTranslateInstance.reverseRotatePoint(
            C2_LAST_PT,
            path_frame.cgalbackShape_keypoints_[ridgeNumber_][ptIndex_],
            arriveLineHeading);
    tempPt1.pathPtType_ = pathPtType::SWITCHPT;
    temp_test_C2Path.push_back(tempPt);
    storage_origin_path.push_back(tempPt1);

    for(int m =0 ;m <  num_size_C3;m++){
        auto tempPt = normalMatrixTranslateInstance.reverseRotatePoint(
                local_C3path[m],
                path_frame.cgalbackShape_keypoints_[ridgeNumber_][ptIndex_],
                arriveLineHeading);
        tempPt.pathPtType_ = pathPtType::FORWARD;
        temp_test_C3Path.push_back(tempPt);
        storage_origin_path.push_back(tempPt);
    }
    path_frame.backshape_fishnail_curve_path_[path_frame.cgalbackShape_keypoints_[ridgeNumber_][ptIndex_]] = storage_origin_path;

    //将A、B点转入到世界坐标系下
    std::string ptname = "/home/zzm/Desktop/test_path_figure-main/src/ptsshow.txt";
    auto & ptsshow =
            common::Singleton::GetInstance<filePrint2>(ptname);

    for(auto it : ptAB){
        auto tempfg = normalMatrixTranslateInstance.reverseRotatePoint(
                it,
                path_frame.cgalbackShape_keypoints_[ridgeNumber_][ptIndex_],
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

void curveDecisionManager::processCCPA(aiforce::Route_Planning::pathPolygonPlan & path_frame){
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

    //添加验证C-CPA代码
    if(ptIndex_ == 2 && ridgeNumber_ == 0){
        cornerTuringImplementRadius cornerTuringImplementRadius1;
        cornerTuringImplementRadius1.calculateMiniTuringRadiusConsiderImplement();
        auto Rsw = cornerTuringImplementRadius1.getRsw();
        cornerTuringCCPAAlgorithm  cornerTuringCCPAAlgorithm1(
                angleInt,
                Rsw,
                15,
                false,
                path_frame.cgalbackShape_keypoints_[ridgeNumber_][ptIndex_],
                arriveLineHeading);
        cornerTuringCCPAAlgorithm1.calculateAngleCC2();
        cornerTuringCCPAAlgorithm1.calculateCirclesCenter();
        cornerTuringCCPAAlgorithm1.calculatePath();
        std::cout << "the angleInt is : " << angleInt * 180 / M_PI;

    }
}

void curveDecisionManager::processFTCPACC(aiforce::Route_Planning::pathPolygonPlan & path_frame){

}

void curveDecisionManager::processCCCURVE(aiforce::Route_Planning::pathPolygonPlan & path_frame){

}

void curveDecisionManager::processREEDSHEPP(aiforce::Route_Planning::pathPolygonPlan & path_frame){

}


void curveDecisionManager::changeState(CurveDecision  state){
    curveType_ = state;
}