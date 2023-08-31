#include "../utility/log_utility.h"
#include "goal.h"

namespace gsmpl {
GoalSingleState::GoalSingleState(const Bounds& b, const State& g, const DistanceBasePtr& dis)
    : GoalBase(b, GoalType::SingleState, dis)
{
    if (!setGoal(g))
        throw "set goal failed";
}
bool GoalSingleState::setGoal(const State& g)
{
    assert(g.size() == dimension_);
    auto q = bounds_.tryToMappingIntoBounds(g);
    if (!q)
        return false;
    q_ = q.value();
    return true;
}

GoalWithJointTolerance::GoalWithJointTolerance(const Bounds& b, double tolerance, const State& g,
                                               const DistanceBasePtr& dis)
    : GoalBase(b, GoalType::JointTolerance, dis), tolerance_(tolerance)
{
    if (!setGoal(g))
        throw "set goal failed";
}
bool GoalWithJointTolerance::setGoal(const State& g)
{
    assert(g.size() == dimension_);
    auto q = bounds_.tryToMappingIntoBounds(g);
    if (!q)
        return false;
    q_ = q.value();
    return true;
}
bool GoalWithJointTolerance::isSatisfied(const State& q) const
{
    assert(q.size() == dimension_);
    return distance_->distance(q, q_) < tolerance_;
}
} // namespace gsmpl
