#ifndef BETTERTEACH_ELLIPSE_H
#define BETTERTEACH_ELLIPSE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "point.hpp"

namespace TeachRepeat {

    template <class T>
    class Ellipse {
    public:
        Ellipse(T a, T b);
        void rescale(T factor);
        Point<T> curve(T theta);

    private:
        T a;
        T b;
    };


    template <class T>
    Ellipse<T>::Ellipse(T a, T b) {
        a = a;
        b = b;
    }

    template <class T>
    void Ellipse<T>::rescale(T factor) {
        a *= factor;
        b *= factor;
    }

    template <class T>
    Point<T> Ellipse<T>::curve(T theta) {
        return Point<T>(a*std::cos(theta), b*std::sin(theta));
    };
}


#endif //BETTERTEACH_ELLIPSE_H
