
#ifndef BETTERTEACH_POINT_H
#define BETTERTEACH_POINT_H

#include <Eigen/Core>

template <class T>
class Point {
public:
    Point(T x, T y);
    Point(T x, T y, T z);
    T getX();
    T getY();
    T getZ();
    Eigen::Vector3f toVector();

private:
    Eigen::Matrix<T, 4, 1>  pointRepresentation;

    void normalize();
};

template <class T>
Point<T>::Point(T x, T y) {
    pointRepresentation << x, y, 0.0, 1.0;
}

template <class T>
Point<T>::Point(T x, T y, T z) {
    pointRepresentation << x, y, z, 1.0;
}

template <class T>
T Point<T>::getX() {
    normalize();
    return pointRepresentation(0,0);
}

template <class T>
T Point<T>::getY() {
    normalize();
    return pointRepresentation(1,0);
}

template <class T>
T Point<T>::getZ() {
    normalize();
    return pointRepresentation(2,0);
}

template <class T>
void Point<T>::normalize() {
    pointRepresentation = pointRepresentation / pointRepresentation(3,0);
}

template <class T>
Eigen::Vector3f Point<T>::toVector() {
    normalize();

    Eigen::Vector3f vector;
    vector << pointRepresentation[0], pointRepresentation[1], pointRepresentation[2];

    return vector;
}


#endif //BETTERTEACH_POINT_H