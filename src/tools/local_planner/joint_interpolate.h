#pragma once

#include <optional>
#include "../../utility/class_forward.h"
#include "../../base/path.h"
#include "../distance/distance.h"
#include "../checker/state_checker_base.h"
#include "../../robot_algo/fk.h"
#include "local_planner.h"

namespace gsmpl {
GSMPL_CLASS_FORWARD(JointInterpolate)

class JointInterpolate : public LocalPlannerBase {
public:
    JointInterpolate(const DistanceBasePtr& distance,
                     const StateCheckerBasePtr& checker, const FKBasePtr fk)
        : LocalPlannerBase(distance, checker), fk_(fk) {}

    ~JointInterpolate() {
        std::cout << " JointInterpolate Processing time: " << time_
                  << " times: " << times_ << " averageTime: " << time_ / times_
                  << std::endl;
    }

    State interpolateState(const State& q1, const State& q2,
                           double time) const override;
    std::optional<State> validInterpolateState(const State& q1, const State& q2,
                                               double time) const override;
    // stepSize: TCP pose L2 norm distance
    std::optional<Path> validInterpolatePath(const State& q1, const State& q2,
                                             double jpsStepSize,
                                             double tcpStepSize) const override;
    Path interpolatePath(const State& q1, const State& q2, double jpsStepSize,
                         double tcpStepSize) const override;

private:
    double tcpTransDistanceL2(const State& q1, const State& q2) const;
    double calcDeltaTime(const State& q1, const State& q2, double jpsStepSize,
                         double tcpStepSize) const;

    FKBasePtr fk_;
    mutable double time_{0};
    mutable int times_{0};
};
} // namespace gsmpl
