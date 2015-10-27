#include <convergence_distance_function.h>
#include "ellipse.h"
#include "tolerance_ellipse_calculator.h"

namespace TeachRepeat {


    template <class T>
    ToleranceEllipseCalculator<T>::ToleranceEllipseCalculator(T maxConvergenceError) : maxConvergenceError(maxConvergenceError), icpEngine(icpEngine) {

    }

    template <class T>
    Ellipse <T> ToleranceEllipseCalculator<T>::calculate(LocalisedPointCloud reference, LocalisedPointCloud reading) {
        ConvergenceDistanceFunction<T> f(reference, reading);
    }
}
