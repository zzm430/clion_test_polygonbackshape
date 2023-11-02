//
// Created by zzm on 2023/10/30.
//

#ifndef POLYGONBACKSHAPE_CURVEDECISIONTYPE_H
#define POLYGONBACKSHAPE_CURVEDECISIONTYPE_H

enum  class CurveDecision : uint8_t {
      IDLE = 0,
      C_CPA_CC = 1,              //C-CPA 连续不倒车凹角，考虑边界
      C_CPA_CV = 2,              //C-CPA 连续不倒车凸角，考虑边界
      FT_CPA_CC = 3,             //带倒车凹角考虑边界
      FT_CPA_CV = 4,             //带倒车凸角考虑边界
      FT_CPA_FS = 5,             //不考虑边界的全覆盖鱼尾
      REED_SHEPP = 6             //rs算法
};

#endif //POLYGONBACKSHAPE_CURVEDECISIONTYPE_H
