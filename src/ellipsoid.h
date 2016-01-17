#ifndef BETTERTEACH_ELLIPSOID_H
#define BETTERTEACH_ELLIPSOID_H

#include "point.h"

namespace TeachRepeat {

    template <typename T>
    class Ellipsoid {
    public:
        Ellipsoid(T a, T b, T c);
        Point<T> stereographicParametrization(T u, T v);
        Point<T> standardParametrization(T theta, T phi);

    private:
        T a;
        T b;
        T c;
    };

    template <typename T>
    Ellipsoid<T>::Ellipsoid(T a, T b, T c) : a(a), b(b), c(c) {

    }

    template <typename T>
    Point<T> Ellipsoid<T>::stereographicParametrization(T u, T v) {
        T norm_factor = 1 + u*u + v*v;

        T x = a * (1 - u*u - v*v) / norm_factor;
        T y = 2 * b * u / norm_factor;
        T z = 2 * c * v / norm_factor;

        return Point<T>(x,y,z);
    }

    template <typename T>
    Point<T> Ellipsoid<T>::standardParametrization(T u, T v) {
        T x = a * std::cos(u) * std::sin(v);
        T y = b * std::sin(u) * std::sin(v);
        T z = c * cos(v);

        return Point<T>(x,y,z);
    }
}




#endif //BETTERTEACH_ELLIPSOID_H
