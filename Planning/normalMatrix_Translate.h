//
// Created by zzm on 23-9-13.
//

#ifndef POLYGONBACKSHAPE_NORMALMATRIX_TRANSLATE_H
#define POLYGONBACKSHAPE_NORMALMATRIX_TRANSLATE_H
#include "common/utilpath/path_polygonPoint.h"
class normalMatrixTranslate{
public:
      normalMatrixTranslate() = default;
      ~normalMatrixTranslate() = default;
      polygonPoint rotatePoint(
              const polygonPoint &pt,
              const polygonPoint &origin,
              double angle);
      polygonPoint reverseRotatePoint(
               polygonPoint pt,
               polygonPoint origin,
              double angle);


private:

};
#endif //POLYGONBACKSHAPE_NORMALMATRIX_TRANSLATE_H
