#pragma once

#include "gsmpl/utility/export.h"
#include "gsmpl/base/state.h"
#include "gsmpl/goal/goal.h"
#include "gsmpl/utility/class_forward.h"
#include "gsmpl/tools/distance/distance.h"

namespace gsmpl {
GSMPL_CLASS_FORWARD(OptiObjectiveBase)
GSMPL_CLASS_FORWARD(OptiPathLength)

enum class OptiType { PathLength };

class OptiObjectiveBase {
public:
    OptiObjectiveBase(double cost_threshold, OptiType opti_type)
        : threshold_(cost_threshold), type_(opti_type) {}
    virtual ~OptiObjectiveBase() = default;

    virtual double stateCost(const State& q) const = 0;
    virtual double costToGo(const State& q, const GoalBasePtr& g) const = 0;

    const double threshold_;
    const OptiType type_;
};

class OptiPathLength : public OptiObjectiveBase {
public:
    OptiPathLength(double threshold, const Bounds& b,
                   const DistanceBasePtr& dis)
        : OptiObjectiveBase(threshold, OptiType::PathLength), distance_(dis) {}

    double stateCost(const State& q) const override { return q[0]; } // TODO:
    double costToGo(const State& q, const GoalBasePtr& g) const override {
        switch (g->type) {
            case GoalType::JointTolerance: {
                State goal = g->sample().value();
                double distance = distance_->distance(q, goal);
                return distance < threshold_
                           ? distance
                           : std::numeric_limits<double>::max();
            }
            default:
                return std::numeric_limits<double>::max();
        }
    }

private:
    DistanceBasePtr distance_;
};
} // namespace gsmpl
