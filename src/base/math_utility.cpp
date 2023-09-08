#include <cmath>
#include "math_utility.h"

namespace gsmpl {
Eigen::Isometry3d rotateZAxis(double theta) {
    Eigen::AngleAxisd angleAxis(theta, Eigen::Vector3d(0, 0, 1));
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.rotate(angleAxis);
    return tf;
}

// [-M_PI, M_PI]
double normalizeAngle(double theta) {
    double a = fmod(theta, 2.0 * M_PI);
    if ((-M_PI <= a) && (a <= M_PI))
        return a;
    if (a < -M_PI)
        return a + 2 * M_PI;
    else
        return a - 2 * M_PI;
}

double mappingZeroToPI(double theta) {
    double a = normalizeAngle(theta);
    if ((0 <= a) && (a <= M_PI))
        return a;
    else
        return a + M_PI;
}

double unitNBallMeasure(std::size_t N, double r) {
    double n = static_cast<double>(N);
    return std::pow(std::sqrt(M_PI) * r, n) / std::tgamma(n * 0.5 + 1.0);
}

double phsMeasure(std::size_t N, double minCost, double cost) {
    assert(minCost <= cost);
    double conjugateDiameter = std::sqrt(cost * cost - minCost * minCost);
    double lebsegueMeasure = cost * 0.5;
    for (std::size_t i = 0; i < N - 1; i++)
        lebsegueMeasure = lebsegueMeasure * conjugateDiameter * 0.5;
    lebsegueMeasure = lebsegueMeasure * unitNBallMeasure(N);
    return lebsegueMeasure;
}
} // namespace gsmpl
