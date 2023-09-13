#include <math.h>
#include "gsmpl/utility/log_utility.h"
#include "gsmpl/base/math_utility.h"
#include "gsmpl/tools/nearest_neighbor/nearest_neighbor_linear.h"
#include "gsmpl/planner/rrt/rrt.h"

namespace gsmpl {
namespace {
RRTParamConstPtr rrtParam(const PlannerContext& context) {
    return std::static_pointer_cast<const RRTParam>(context.planner_param);
}
} // namespace

RRT::RRT(const SpaceInformationBasePtr& si, const ProblemDefinition& pd,
         const PlannerContext& context, const PlannerRecord::VisualFunction& vf)
    : Planner(si, pd),
      param_(*rrtParam(context)),
      vf_(vf),
      nearest_neighbor_(std::make_shared<NearestNeighborLinear>(si_->distance)) {
}

const PlannerSolution& RRT::solve() {
    LOGD();
    assert(goal_->type == GoalType::JointTolerance);
    start_.printState("start");
    current_goal_ = goal_->sample().value();
    current_goal_.printState("goal");

    solution_ = PlannerSolution();

    if (!si_->checkers->isValid(start_)) {
        LOGD("start state is not valid");
        solution_.status = PlannerStatus::InvalidStart;
        return solution_;
    } else {
        LOGD("start state is valid");
    }

    if (!si_->checkers->isValid(current_goal_)) {
        LOGD("goal state is not valid");
        solution_.status = PlannerStatus::InvalidGoal;
        return solution_;
    } else {
        LOGD("goal state is valid");
    }

    tree_.setRoot(std::make_shared<Vertex>(nullptr, start_));
    nearest_neighbor_->build(tree_.root());
    unsigned i = 0;
    Status status = Status::Trapped;
    State qLast = start_;
    LOGD("*************************************");
    for (; i < param_.steps && status != Status::Reached; ++i) {
        State qSampled = si_->sampler->sample();
        // State qSampled = si_->sampleNear(qLast, 0.1);
        if (auto validV = extend(qSampled)) {
            const VertexPtr vNew = validV.value();
            update(vNew);
            qLast = vNew->state; // TODO:
            if (goal_->isSatisfied(vNew->state)) {
                status = Status::Reached;
                solution_.status = PlannerStatus::ExactSolution;
            } else
                status = Status::Advanced;
        } else {
            status = Status::Trapped;
        }
    }
    if (i >= param_.steps)
        solution_.status = PlannerStatus::TimeOut;

    LOGD("######## sample points: %d ", i);
    return solution_;
}

std::optional<VertexPtr> RRT::extend(const State& qSampled) {
    auto vNear = nearest_neighbor_->nearest(qSampled);
    const State& qNew = newState(qSampled, vNear->state, param_.step_size);

    if (si_->checkers->isValid(qNew)) {
        if (si_->local_planner->validInterpolatePath(
                vNear->state, qNew, param_.local_planner_step_size_jps,
                param_.local_planner_step_size_tcp))
            return std::make_shared<Vertex>(vNear.get(), qNew,
                                            opti_->stateCost(qNew));
    }
    return {};
}

void RRT::update(const VertexPtr& vNew) {
    tree_.addVertex(vNew);
    nearest_neighbor_->update(vNew);
    solution_.path = generatePath(vNew);
}
State RRT::newState(const State& qSampled, const State& qNear,
                    double step_size) const {
    assert(qSampled.size() == qNear.size());
    State qNew;
    for (size_t i = 0; i < qNear.size(); i++) {
        double delta = step_size * sign(normalizeAngle(qSampled[i] - qNear[i]));
        qNew.position.push_back(delta + qNear[i]);
    }
    return qNew;
}

Path RRT::generatePath(const VertexPtr& v) const {
    Path path;
    path.push_back(v->state);
    auto currentV = v.get();
    while (auto nextV = currentV->parent()) {
        path.push_back(nextV->state);
        currentV = nextV;
    }
    std::reverse(std::begin(path), std::end(path));
    LOGD("rrt path size: %ld", path.size());
    return path;
}

PlannerRecord RRT::plannerRecord() const {
    PlannerRecord pr;
    pr.start = start_;
    pr.goal = current_goal_;
    pr.addTree(tree_, "rrtTree");
    pr.addPath(solution_.path, "rawPath");
    return pr;
}
} // namespace gsmpl
