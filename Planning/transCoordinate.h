#ifndef TRANSCOORDINATE_H
#define TRANSCOORDINATE_H
#include <algorithm>
#include "path_common.h"
class transCoordinate{
public:
    transCoordinate();
    ~transCoordinate();
    void  computeRelativePoints( std::vector<Point> & points);
    void  computeOriginalPoints(std::vector<Point> & points);
    std::vector<Point>  getRelativePoints();
    std::vector<Point>   getOriginalPoints();
private:
    std::vector<Point>  relativePoints_;
    std::vector<Point>  OriginalPoints_;
};
#endif //
