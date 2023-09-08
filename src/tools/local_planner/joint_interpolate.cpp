#include <chrono>
#include "joint_interpolate.h"

namespace gsmpl {
State JointInterpolate::interpolateState(const State& q1, const State& q2,
                                         double time) const {
    assert(q1.size() == q2.size());
    assert((time >= 0) && (time <= 1));

    State q;
    for (std::size_t i = 0; i < q1.size(); i++)
        q.push_back((q2[i] - q1[i]) * time + q1[i]);
    return q;
}
std::optional<State> JointInterpolate::validInterpolateState(
    const State& q1, const State& q2, double time) const {
    State q = interpolateState(q1, q2, time);
    if (checker_->isValid(q))
        return q;

    return {};
}
double JointInterpolate::tcpTransDistanceL2(const State& q1,
                                            const State& q2) const {
    return (fk_->tcpPose(q1).translation() - fk_->tcpPose(q2).translation())
        .norm();
}

double JointInterpolate::calcDeltaTime(const State& q1, const State& q2,
                                       double jpsStepSize,
                                       double tcpStepSize) const {
    State qStep = q2;
    double tcpDistance = tcpTransDistanceL2(q1, qStep);
    double jpsDistance = distance_->distance(q1, qStep);
    double deltaTime = 1.0;
    while ((tcpDistance > tcpStepSize) || (jpsDistance > jpsStepSize)) {
        deltaTime *= 0.5;
        qStep = interpolateState(q1, q2, deltaTime);
        tcpDistance = tcpTransDistanceL2(q1, qStep);
        jpsDistance = distance_->distance(q1, qStep);
    }
    return deltaTime;
}
std::optional<Path> JointInterpolate::validInterpolatePath(
    const State& q1, const State& q2, double jpsStepSize,
    double tcpStepSize) const {
    auto startTime = std::chrono::steady_clock::now();

    Path path;
    double deltaTime = calcDeltaTime(q1, q2, jpsStepSize, tcpStepSize);
    unsigned int steps = floor(1.0 / deltaTime);
    // std::cout << "validInterpolatePath steps: " << steps << std::endl;
    path.push_back(q1);
    for (double i = 1; i < steps; i++) {
        auto validQ = validInterpolateState(q1, q2, i * deltaTime);
        if (!validQ)
            return {};
        path.push_back(validQ.value());
    }
    path.push_back(q2);

    auto endTime = std::chrono::steady_clock::now();
    time_ += std::chrono::duration<double>(endTime - startTime).count();
    times_++;
    return path;
}
Path JointInterpolate::interpolatePath(const State& q1, const State& q2,
                                       double jpsStepSize,
                                       double tcpStepSize) const {
    Path path;
    double deltaTime = calcDeltaTime(q1, q2, jpsStepSize, tcpStepSize);
    unsigned int steps = floor(1.0 / deltaTime);
    path.push_back(q1);
    for (double i = 1; i < steps; i++) {
        State q = interpolateState(q1, q2, i * deltaTime);
        path.push_back(q);
    }
    path.push_back(q2);
    return path;
}
} // namespace gsmpl
