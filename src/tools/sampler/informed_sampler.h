#pragma once

#include <Eigen/Geometry>
#include "../../base/math_utility.h"
#include "sampler.h"

namespace gsmpl {
GSMPL_CLASS_FORWARD(ProlateHyperspheroid)

struct PhsData {
    std::size_t dim;
    double minCost;
    double cost;
    double phsMeasure;
    Eigen::VectorXd xFocus1;
    Eigen::VectorXd xFocus2;
    Eigen::VectorXd xCentre;
    Eigen::MatrixXd rotation;       // C_we
    Eigen::MatrixXd transformation; // C_we * L
};

class ProlateHyperspheroid {
public:
    ProlateHyperspheroid(std::size_t dim, const double focus1[],
                         const double focus2[]);

    void setCost(double cost);
    std::vector<double> transform(const std::vector<double>& ball) const;
    // bool isInPhs(const double point[]) const;
    double getPhsMeasure(double cost);
    // double getPathLength(const double point[]) const;

private:
    Eigen::MatrixXd rotation() const;
    Eigen::MatrixXd transformation() const;

    PhsData data_;
};

class PathLengthDirectInfSampler {
public:
    PathLengthDirectInfSampler(const State& start, const State& goal)
        : dim(start.size()),
          phs_(start.size(), start.values.data(), goal.values.data()) {
        assert(start.size() == goal.size());
    }
    ~PathLengthDirectInfSampler() {
        std::cout << " PathLengthDirectInfSampler Processing time: " << time_
                  << " times: " << times_ << " averageTime: " << time_ / times_
                  << std::endl;
    }

    State sample(double cost);
    State sampleUniformBall(std::size_t dim);

    const std::size_t dim;

private:
    ProlateHyperspheroid phs_;
    double cost_;
    RNG rng_;
    mutable double time_{0};
    mutable int times_{0};
};
} // namespace gsmpl
