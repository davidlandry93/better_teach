//
// Created by david on 06/11/15.
//

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

private:
    Eigen::Matrix<T, 3, 1>  pointRepresentation;

    void normalize();
};

template <class T>
Point<T>::Point(T x, T y) {
    pointRepresentation << x, y, 0.0, 1.0;
}

template <class T>
Point::Point(T x, T y, T z) {
    pointRepresentation << x, y, z, 1.0;
}

template <class T>
T Point<T>::getX() {
    normalize();
    return pointRepresentation(1,1);
}

template <class T>
T Point<T>::getY() {
    normalize();
    return pointRepresentation(1,2);
}

template <class T>
T Point<T>::getZ() {
    normalize();
    return pointRepresentation(1,3);
}

template <class T>
void Point<T>::normalize() {
    pointRepresentation = pointRepresentation / pointRepresentation(4,1);
}

#endif //BETTERTEACH_POINT_H

