#include "gsmpl/planner/bi_rrt/bi_rrt.h"

#include <chrono>
#include <thread>
#include <math.h>
#include <utility>
#include "gsmpl/utility/log_utility.h"
#include "gsmpl/utility/global.h"
#include "gsmpl/base/math_utility.h"
#include "gsmpl/tools/nearest_neighbor/nearest_neighbor_linear.h"

namespace gsmpl {
namespace {
BiRRTParamConstPtr biRRTParam(const PlannerContext& context) {
    return std::static_pointer_cast<const BiRRTParam>(context.planner_param);
}
} // namespace
void BiRRT::TreeTuple::update(const VertexPtr& vNew) {
    tree->addVertex(vNew);
    nearest->update(vNew);
}

BiRRT::BiRRT(const SpaceInformationBasePtr& si, const ProblemDefinition& pd,
             const PlannerContext& context,
             const PlannerRecord::VisualFunction& vf)
    : Planner(si, pd),
      param_(*biRRTParam(context)),
      vf_(vf),
      s_nearest_(std::make_shared<NearestNeighborLinear>(si->distance)),
      g_nearest_(std::make_shared<NearestNeighborLinear>(si->distance)) {}
BiRRT::BiRRT(const SpaceInformationBasePtr& si, const ProblemDefinition& pd,
             const BiRRTParam& param, const PlannerRecord::VisualFunction& vf)
    : Planner(si, pd),
      param_(param),
      vf_(vf),
      s_nearest_(std::make_shared<NearestNeighborLinear>(si->distance)),
      g_nearest_(std::make_shared<NearestNeighborLinear>(si->distance)) {}

const PlannerSolution& BiRRT::solve() {
    auto startTime = std::chrono::steady_clock::now();
    assert(goal_->type == GoalType::SingleState);
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

    // create start TreeTuple
    s_tree_.setRoot(std::make_shared<Vertex>(nullptr, start_));
    s_nearest_->build(s_tree_.root());
    TreeTuple start(&s_tree_, s_nearest_, TreeType::Start);

    // create goal TreeTuple
    g_tree_.setRoot(std::make_shared<Vertex>(nullptr, current_goal_));
    g_nearest_->build(g_tree_.root());
    TreeTuple goal(&g_tree_, g_nearest_, TreeType::Goal);

    unsigned i = 0;
    Status status = Status::Trapped;
    LOGD("*************************************");
    for (; i < param_.steps && status != Status::Reached; ++i) {
        const State qSampled = si_->sampler->sample(
            start.type == TreeType::Start ? current_goal_ : start_);

        if (const auto validV = extend(qSampled, start.nearest)) {
            const VertexPtr& vNew = validV.value();
            start.update(vNew);
            if (auto near = connectTree(vNew, goal)) {
                if (goal.type == TreeType::Goal) {
                    VertexPtr vGoal = mergeTree(start.tree, vNew, near.value());
                    solution_.path = generatePath(vGoal);
                    record_.vStart = start.tree->root();
                    record_.vGoal = vGoal;
                    record_.tree = *start.tree;
                } else {
                    VertexPtr vGoal = mergeTree(goal.tree, near.value(), vNew);
                    solution_.path = generatePath(vGoal);
                    record_.vStart = goal.tree->root();
                    record_.vGoal = vGoal;
                    record_.tree = *goal.tree;
                }

                status = Status::Reached;
                solution_.status = PlannerStatus::ExactSolution;
                std::cout << "biRRT find a feasible path" << std::endl;
                pt.biRRTPathLength = pathLength(solution_.path, si_->distance);
            } else {
                status = Status::Advanced;
            }
        } else {
            status = Status::Trapped;
        }
        std::swap(start, goal);
    }
    if (i >= param_.steps)
        solution_.status = PlannerStatus::TimeOut;

    if (solution_.status == PlannerStatus::ExactSolution)
        generateRecord();
    else
        std::cout << "BiRRT find goal failed!!!!! samples: " << i << std::endl;

    LOGD("######## sample points: %d ", i);
    auto endTime = std::chrono::steady_clock::now();
    pt.biRRT = std::chrono::duration<float>(endTime - startTime).count();

    std::cout << "BiRRT samples points: " << i << std::endl;
    // visualize();
    return solution_;
}

std::optional<VertexPtr> BiRRT::extend(const State& qSampled,
                                       const NearestNeighborBasePtr& nearest) {
    auto vNear = nearest->nearest(qSampled);
    const State& qNew = newState(qSampled, vNear->state, param_.step_size);

    if (si_->checkers->isValid(qNew)) {
        if (si_->local_planner->validInterpolatePath(
                vNear->state, qNew, param_.local_planner_step_size_jps,
                param_.local_planner_step_size_tcp)) {
            double eCost = si_->distance->distance(vNear->state, qNew);
            double sCost = vNear->state_cost + eCost;
            return std::make_shared<Vertex>(vNear.get(), qNew, sCost, eCost);
        }
    }
    return {};
}
VertexPtr BiRRT::mergeTree(Tree* tree1, const VertexPtr& t1v,
                           const VertexPtr& t2v) {
    Vertex* v1 = t1v.get();
    Vertex* v2 = t2v.get();
    VertexPtr vNew;
    record_.v1 = t1v;
    bool first = true;

    while (v2) {
        double eCost = si_->distance->distance(v1->state, v2->state);
        double sCost = v1->state_cost + eCost;
        vNew = std::make_shared<Vertex>(v1, v2->state, sCost, eCost);
        if (first) {
            record_.v2 = vNew;
            first = false;
        }
        tree1->addVertex(vNew);
        v1 = vNew.get();
        v2 = v2->parent();
    }

    return vNew;
}
std::optional<VertexPtr> BiRRT::connectTree(const VertexPtr& vNew,
                                            TreeTuple& treeTuple) const {
    const State& qNew = vNew->state;
    auto vNear = treeTuple.nearest->nearest(qNew);
    double distance = si_->distance->distance(vNear->state, qNew);
    if (distance > param_.connection_range)
        return {};

    if (si_->local_planner->validInterpolatePath(vNear->state, qNew,
                                                param_.local_planner_step_size_jps,
                                                param_.local_planner_step_size_tcp))
        return vNear;

    return {};
}

State BiRRT::newState(const State& qSampled, const State& qNear,
                      const double step_size) const {
    assert(qSampled.size() == qNear.size());
    State qNew;

    if (si_->distance->distance(qSampled, qNear) < param_.step_size)
        return qSampled;

    for (size_t i = 0; i < qNear.size(); i++) {
        double delta = step_size * sign(normalizeAngle(qSampled[i] - qNear[i]));
        qNew.position.push_back(delta + qNear[i]);
    }
    return qNew;
}

Path BiRRT::generatePath(const VertexPtr& v) const {
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

void BiRRT::generateRecord() {
    PlannerRecord pr;
    pr.start = start_;
    pr.goal = current_goal_;
    pr.addTree(s_tree_, "startTree");
    pr.addTree(g_tree_, "goalTree");
    pr.addPath(solution_.path, "rawPath");
    pr.v1 = record_.v1->state;
    pr.v2 = record_.v2->state;
    pr.approximate = solution_.approximate;
    pr.cost = solution_.cost;
    pr.status = solution_.status;
    record_.pr = pr;
}
} // namespace gsmpl
