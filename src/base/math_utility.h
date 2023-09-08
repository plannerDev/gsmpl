#pragma once

#include <iostream>
#include <limits>
#include <random>
#include <Eigen/Geometry>
#ifdef _WIN32
#include <corecrt_math_defines.h>
#define _USE_MATH_DEFINES
#endif

namespace gsmpl {
Eigen::Isometry3d rotateZAxis(double theta);

static double sign(double x) {
    if (x < -std::numeric_limits<double>::min())
        return -1.0;
    if (x > std::numeric_limits<double>::min())
        return 1.0;
    else
        return 0;
}

double normalizeAngle(double theta);  // [-M_PI, M_PI]
double mappingZeroToPI(double theta); // [0, M_PI]

double unitNBallMeasure(std::size_t N, double r = 1.0);
double phsMeasure(std::size_t N, double minCost, double cost);

class RNG {
public:
    /* Generate a random real within given bounds: [\e lower_bound, \e
     * upper_bound) */
    double uniformReal(double lowerBound, double upperBound) {
        assert(lowerBound <= upperBound);
        std::random_device rd;
        std::mt19937 gen(rd());
        return (upperBound - lowerBound) * uniDist_(gen) + lowerBound;
    }
    double uniformReal01() {
        std::random_device rd;
        std::mt19937 gen(rd());
        return uniDist_(gen);
    }

    // [lowerBound, upperBound]
    int uniformInt(int lowerBound, int upperBound) {
        assert(lowerBound <= upperBound);
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> distri(lowerBound, upperBound);
        return distri(gen);
    }

    std::vector<double> uniformUnitSphere(std::size_t dim) {
        std::random_device rd;
        std::mt19937 gen(rd());

        std::vector<double> v;
        double norm = 0.0;
        for (std::size_t i = 0; i < dim; i++) {
            double x = uniDist_(gen) - 0.5;
            v.push_back(x);
            norm += x * x;
        }
        norm = std::sqrt(norm);

        for (std::size_t i = 0; i < dim; i++)
            v[i] = v[i] / norm;

        return v;
    }

    std::vector<double> uniformInBall(std::size_t dim, double r) {
        std::vector<double> sphere = uniformUnitSphere(dim);
        double radiusScale =
            r * std::pow(uniformReal01(), 1.0 / static_cast<double>(dim));
        std::vector<double> ball;

        for (const auto& p : sphere)
            ball.push_back(radiusScale * p);
        return ball;
    }

private:
    std::uniform_real_distribution<> uniDist_{
        0.0, std::nextafter(1.0, std::numeric_limits<double>::max())}; // [0, 1]
};
} // namespace gsmpl
