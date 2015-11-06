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
    T getX();
    T getY();

private:
    Eigen::Matrix<T, 3, 1>  pointRepresentation;

    void normalize();
};

template <class T>
Point<T>::Point(T x, T y) {
    pointRepresentation << x, y, 1.0;
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
void Point<T>::normalize() {
    pointRepresentation = pointRepresentation / pointRepresentation(3,1);
}

#endif //BETTERTEACH_POINT_H
