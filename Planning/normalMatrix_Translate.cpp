//
// Created by zzm on 23-9-13.
//
#include "Planning/normalMatrix_Translate.h"


polygonPoint normalMatrixTranslate::rotatePoint(
        const polygonPoint &pt,
        const polygonPoint &origin,
        double angle){
        //将坐标转换为相对于原点的偏移量
        double offsetX = pt.x - origin.x;
        double offsetY = pt.y - origin.y;

        //计算旋转后的坐标
        double rotatedX = offsetX * cos(angle) + offsetY * sin(angle);
        double rotatedY = -offsetX * sin(angle) + offsetY * cos(angle);

        //将旋转后的坐标转换为新坐标系
        polygonPoint rotatedPoint;
        rotatedPoint.x = rotatedX ;
        rotatedPoint.y = rotatedY;
        return rotatedPoint;

}


polygonPoint normalMatrixTranslate::reverseRotatePoint(
         polygonPoint pt,
        polygonPoint origin,
        double angle){
    // 将坐标转换为相对于新坐标系的偏移量
    double offsetX = pt.x;
    double offsetY = pt.y;

    // 计算逆向旋转后的坐标
    double reversedX = offsetX * cos(angle) -  offsetY * sin(angle);
    double reversedY = offsetY * cos(angle) +  offsetX * sin(angle) ;

    // 将逆向旋转后的坐标转换为原始坐标系
    polygonPoint reversedPoint;
    reversedPoint.x = reversedX + origin.x;
    reversedPoint.y = reversedY + origin.y;

    return reversedPoint;

}

