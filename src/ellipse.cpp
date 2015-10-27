
#include "ellipse.h"

namespace TeachRepeat {

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