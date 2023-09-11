#include <chrono>
#include <math.h>
#include <assert.h>
#include "gsmpl/tools/sampler/informed_sampler.h"

namespace gsmpl {
ProlateHyperspheroid::ProlateHyperspheroid(std::size_t dim,
                                           const double focus1[],
                                           const double focus2[]) {
    data_.dim = dim;
    data_.cost = 0.0;
    data_.xFocus1 = Eigen::Map<const Eigen::VectorXd>(focus1, data_.dim);
    data_.xFocus2 = Eigen::Map<const Eigen::VectorXd>(focus2, data_.dim);
    data_.minCost = (data_.xFocus1 - data_.xFocus2).norm();
    data_.xCentre = 0.5 * (data_.xFocus1 + data_.xFocus2);
    data_.rotation = rotation();
}

void ProlateHyperspheroid::setCost(double cost) {
    assert(cost >= data_.minCost);
    if (data_.cost != cost) {
        data_.cost = cost;
        data_.transformation = transformation();
        data_.phsMeasure = phsMeasure(data_.dim, data_.minCost, data_.cost);
    }
}

std::vector<double> ProlateHyperspheroid::transform(
    const std::vector<double>& ball) const {
    Eigen::VectorXd B =
        Eigen::Map<const Eigen::VectorXd>(ball.data(), ball.size());
    Eigen::VectorXd P = data_.transformation * B + data_.xCentre;
    std::vector<double> state(&P[0], P.data() + P.cols() * P.rows());
    return state;
}

// bool ProlateHyperspheroid::isInPhs(const double point[]) const
// {
//     return getPathLength(point) < data_.cost;
// }

double ProlateHyperspheroid::getPhsMeasure(double cost) {
    assert(cost >= data_.minCost);
    if (data_.cost != cost)
        setCost(cost);
    return data_.phsMeasure;
}

// double ProlateHyperspheroid::getPathLength(const double point[]) const
// {
//     return (data_.xFocus1 - Eigen::Map<const Eigen::VectorXd>(point,
//     data_.dim)).norm() +
//            (Eigen::Map<const Eigen::VectorXd>(point, data_.dim) -
//            data_.xFocus2).norm();
// }

Eigen::MatrixXd ProlateHyperspheroid::rotation() const {
    Eigen::MatrixXd rotation;

    // if the cMin if too close to 0, we treat this as a circle.
    double circleTolerance = 1E-9;
    if (data_.minCost < circleTolerance)
        rotation = Eigen::MatrixXd::Identity(data_.dim, data_.dim);
    else {
        // rotation C_ba = U * A * V^T
        // A = diag(1, ... , 1, det(U) * det(V))
        // UAV^T = M, M = a1 * 1^T, 1: the first column of the identity matrix
        // and the transverse axis in the world frame, underspecified due to
        // symmetry
        Eigen::VectorXd a1 = (data_.xFocus2 - data_.xFocus1) / data_.minCost;
        Eigen::MatrixXd wahbaProb =
            a1 *
            Eigen::MatrixXd::Identity(data_.dim, data_.dim).col(0).transpose();
        Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::NoQRPreconditioner> svd(
            wahbaProb, Eigen::ComputeFullV | Eigen::ComputeFullU);
        Eigen::VectorXd middleA = Eigen::VectorXd::Ones(data_.dim);
        middleA(data_.dim - 1) =
            svd.matrixU().determinant() * svd.matrixV().determinant();
        rotation =
            svd.matrixU() * middleA.asDiagonal() * svd.matrixV().transpose();
    }
    return rotation;
}

Eigen::MatrixXd ProlateHyperspheroid::transformation() const {
    Eigen::VectorXd L(data_.dim);
    double conjugateDiamater =
        std::sqrt(data_.cost * data_.cost - data_.minCost * data_.minCost) *
        0.5;
    L.fill(conjugateDiamater);
    L(0) = 0.5 * data_.minCost;
    return data_.rotation * L.asDiagonal();
}
State PathLengthDirectInfSampler::sample(double cost) {
    auto startTime = std::chrono::steady_clock::now();

    phs_.setCost(cost);
    std::vector<double> xBall = rng_.uniformInBall(dim, 1.0);
    std::vector<double> xPhs = phs_.transform(xBall);
    assert(xPhs.size() == dim);

    auto endTime = std::chrono::steady_clock::now();
    time_ += std::chrono::duration<double>(endTime - startTime).count();
    times_++;
    return State(xPhs);
}
State PathLengthDirectInfSampler::sampleUniformBall(std::size_t dim) {
    std::vector<double> xBall = rng_.uniformInBall(dim, 1.0);
    return State(xBall);
}
} // namespace gsmpl
