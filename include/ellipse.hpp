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
    Eigen::Matrix<T,2,1> Ellipse<T>::curve(T theta) {
        Eigen::Matrix<T,2,1> m;
        m << a*std::cos(theta), b*std::sin(theta);
        return m;
    };
}


#endif //BETTERTEACH_ELLIPSE_H
