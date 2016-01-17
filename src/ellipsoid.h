#ifndef BETTERTEACH_ELLIPSOID_H
#define BETTERTEACH_ELLIPSOID_H

#include "point.h"

namespace TeachRepeat {

    template <typename T>
    class Ellipsoid {
    public:
        Ellipsoid(T a, T b, T c);
        Point<T> parametrization(T theta, T phi);

    private:
        T a;
        T b;
        T c;
    };

    template <typename T>
    Ellipsoid<T>::Ellipsoid(T a, T b, T c) : a(a), b(b), c(c) {

    }

    template <typename T>
    Point<T> Ellipsoid<T>::parametrization(T theta, T phi) {
        return Point<T>(a*std::cos(theta)*std::cos(phi), b*std::cos(theta)*std::sin(phi), c*std::sin(phi));
    }
}




#endif //BETTERTEACH_ELLIPSOID_H
