#ifndef BETTERTEACH_ELLIPSE_H
#define BETTERTEACH_ELLIPSE_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace TeachRepeat {

template <class T>
class Ellipse {
public:
    Ellipse(T a, T b);
    void rescale(T factor);
    Eigen::Matrix<T,2,1> curve(T theta);

private:
    T a;
    T b;
};

}


#endif //BETTERTEACH_ELLIPSE_H
