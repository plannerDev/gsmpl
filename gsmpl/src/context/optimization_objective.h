#pragma once

#include "../utility/export.h"
#include "../base/state.h"
#include "../goal/goal.h"
#include "../utility/class_forward.h"
#include "../tools/distance/distance.h"

namespace gsmpl {
GSMPL_CLASS_FORWARD(OptiObjectiveBase)
GSMPL_CLASS_FORWARD(OptiPathLength)

enum class OptiType { PathLength };

class OptiObjectiveBase
{
public:
    OptiObjectiveBase(double costThreshold, OptiType optiType)
        : threshold(costThreshold), type(optiType)
    {
    }
    virtual ~OptiObjectiveBase() = default;

    virtual double stateCost(const State& q) const = 0;
    virtual double costToGo(const State& q, const GoalBasePtr& g) const = 0;

    const double threshold;
    const OptiType type;
};

class OptiPathLength : public OptiObjectiveBase
{
public:
    OptiPathLength(double threshold, const Bounds& b, const DistanceBasePtr& dis)
        : OptiObjectiveBase(threshold, OptiType::PathLength), distance_(dis)
    {
    }

    double stateCost(const State& q) const override { return q[0]; } // TODO:
    double costToGo(const State& q, const GoalBasePtr& g) const override
    {
        switch (g->type) {
        case GoalType::JointTolerance:
        {
            State goal = g->sample().value();
            double distance = distance_->distance(q, goal);
            return distance < threshold ? distance : std::numeric_limits<double>::max();
        }
        default:
            return std::numeric_limits<double>::max();
        }
    }

private:
    DistanceBasePtr distance_;
};
} // namespace gsmpl
