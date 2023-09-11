#include "gsmpl/base/math_utility.h"
#include "gsmpl/tools/local_planner/local_planner.h"

namespace gsmpl {
State JointInterpolateSimple::interpolateState(const State& q1, const State& q2,
                                               double time) const {
    assert(q1.size() == q2.size());
    assert((time >= 0) && (time <= 1));

    State q;
    for (std::size_t i = 0; i < q1.size(); i++)
        q.push_back((q2[i] - q1[i]) * time + q1[i]);
    return q;
}
std::optional<State> JointInterpolateSimple::validInterpolateState(
    const State& q1, const State& q2, double time) const {
    State q = interpolateState(q1, q2, time);
    if (checker_->isValid(q))
        return q;

    return {};
}

std::optional<Path> JointInterpolateSimple::validInterpolatePath(
    const State& q1, const State& q2, double jpsStepSize,
    double tcpStepSize) const {
    Path path;
    unsigned int steps = floor(abs(distance_->distance(q1, q2)) / jpsStepSize);
    double deltaTime = 1.0 / (double)steps;
    path.push_back(q1);
    for (double i = 1; i < steps; i++) {
        auto validQ = validInterpolateState(q1, q2, i * deltaTime);
        if (!validQ)
            return {};
        path.push_back(validQ.value());
    }
    path.push_back(q2);
    return path;
}
Path JointInterpolateSimple::interpolatePath(const State& q1, const State& q2,
                                             double jpsStepSize,
                                             double tcpStepSize) const {
    Path path;
    unsigned int steps = floor(abs(distance_->distance(q1, q2)) / jpsStepSize);
    double deltaTime = 1.0 / (double)steps;
    path.push_back(q1);
    for (double i = 1; i < steps; i++) {
        State q = interpolateState(q1, q2, i * deltaTime);
        path.push_back(q);
    }
    path.push_back(q2);
    return path;
}
} // namespace gsmpl
