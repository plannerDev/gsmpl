#pragma once

#include <optional>
#include "gsmpl/utility/class_forward.h"
#include "gsmpl/base/path.h"
#include "gsmpl/tools/distance/distance.h"
#include "gsmpl/tools/checker/state_checker_base.h"

namespace gsmpl {
GSMPL_CLASS_FORWARD(LocalPlannerBase)
GSMPL_CLASS_FORWARD(JointInterpolateSimple)

class LocalPlannerBase {
public:
    LocalPlannerBase(const DistanceBasePtr& distance,
                     const StateCheckerBasePtr& checker)
        : dimension(distance->dimension),
          distance_(distance),
          checker_(checker) {}
    virtual ~LocalPlannerBase() = default;

    // interpolate without validity check
    virtual State interpolateState(const State& q1, const State& q2,
                                   double time) const = 0;
    virtual std::optional<State> validInterpolateState(const State& q1,
                                                       const State& q2,
                                                       double time) const = 0;
    // time[0.0, 1.0], return true, if interpolated path is valid
    virtual std::optional<Path> validInterpolatePath(
        const State& q1, const State& q2, double jpsStepSize,
        double tcpStepSize) const = 0;
    virtual Path interpolatePath(const State& q1, const State& q2,
                                 double jpsStepSize,
                                 double tcpStepSize) const = 0;

    const std::size_t dimension;

protected:
    DistanceBasePtr distance_;
    StateCheckerBasePtr checker_;
};

class JointInterpolateSimple : public LocalPlannerBase {
public:
    JointInterpolateSimple(const DistanceBasePtr& distance,
                           const StateCheckerBasePtr& checker)
        : LocalPlannerBase(distance, checker) {}

    State interpolateState(const State& q1, const State& q2,
                           double time) const override;
    std::optional<State> validInterpolateState(const State& q1, const State& q2,
                                               double time) const override;
    std::optional<Path> validInterpolatePath(
        const State& q1, const State& q2, double jpsStepSize,
        double tcpStepSize = 0) const override;
    Path interpolatePath(const State& q1, const State& q2, double jpsStepSize,
                         double tcpStepSize = 0) const override;
};
} // namespace gsmpl
