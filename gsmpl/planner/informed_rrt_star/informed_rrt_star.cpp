#include "gsmpl/planner/informed_rrt_star/informed_rrt_star.h"

#include <chrono>
#include <thread>
#include <math.h>
#include "gsmpl/utility/log_utility.h"
#include "gsmpl/utility/global.h"
#include "gsmpl/base/math_utility.h"
#include "gsmpl/tools/nearest_neighbor/nearest_neighbor_linear.h"

namespace gsmpl {
namespace {
InformedRRTStarParamConstPtr param(const PlannerContext& context) {
    return std::static_pointer_cast<const InformedRRTStarParam>(
        context.planner_param);
}
} // namespace

InformedRRTStar::InformedRRTStar(const SpaceInformationBasePtr& si,
                                 const ProblemDefinition& pd,
                                 const PlannerContext& context,
                                 const PlannerRecord::VisualFunction& vf,
                                 const PlannerRecord::VisualPoseFunction& vpf)
    : Planner(si, pd),
      param_(*param(context)),
      vf_(vf),
      vpf_(vpf),
      nearest_neighbor_(std::make_shared<NearestNeighborLinear>(si_->distance)),
      pruned_measure_(si_->getSpaceMeasure()),
      bi_rrt_(si, pd, toBirrtParam(param_), vf) {}

const PlannerSolution& InformedRRTStar::solve() {
    auto feasibleSolution = findFeasiblePath();

    if (!feasibleSolution)
        return solution_;

    auto startTime = std::chrono::steady_clock::now();
    tree_ = feasibleSolution.value().tree;
    VertexPtr vStart = feasibleSolution.value().vStart;
    start_ = vStart->state;
    VertexPtr vGoal = feasibleSolution.value().vGoal;
    current_goal_ = vGoal->state;
    double minCost = si_->distance->distance(start_, current_goal_);

    v1_ = feasibleSolution.value().v1->state;
    v2_ = feasibleSolution.value().v2->state;

    if (!(minCost > 0))
        return solution_;

    double goalCost = vGoal->state_cost;
    double costThreshold = minCost * param_.goalCostThreshold;
    std::cout << "minCost " << minCost << std::endl;
    PathLengthDirectInfSampler sampler(start_, current_goal_);
    initNearestNeighbor(tree_);
    // visualize();

    unsigned i = 0;
    bool approximateSolution = false;
    bool goalCostUpdated = false;
    double k_rrt = 2 * M_E;
    double isValidEdgetime = 0;

    LOGD("*************************************");
    while (i < param_.steps && !approximateSolution) {
        State xRand = sampler.sample(vGoal->state_cost);
        double cardDbl = static_cast<double>(nearest_neighbor_->size() + 1);
        unsigned int k = std::ceil(k_rrt * log(cardDbl));

        const VertexPtr vNearest = nearest_neighbor_->nearest(xRand);
        const State& xNew = steer(xRand, vNearest->state, param_.step_size);

        if (isValidEdge(vNearest->state, xNew)) {
            std::vector<VertexPtr> vNearK =
                nearest_neighbor_->nearestK(xNew, k); // nearestK
            VertexPtr vMin = vNearest;
            double eCostMin = edgeCost(vNearest->state, xNew);
            double sCostMin = vNearest->state_cost + eCostMin;
            std::vector<NearKEdgeCost> NearKEdgeCosts;

            for (const auto& vNear : vNearK) {
                auto time1 = std::chrono::steady_clock::now();
                bool validEdge = isValidEdge(vNear->state, xNew);
                auto time2 = std::chrono::steady_clock::now();
                isValidEdgetime +=
                    std::chrono::duration<double>(time2 - time1).count();
                if (validEdge) { // time cost
                    double eCost = edgeCost(vNear->state, xNew);
                    double sCost = vNear->state_cost + eCost;
                    NearKEdgeCosts.push_back({vNear, eCost});
                    if (sCost < sCostMin) {
                        vMin = vNear;
                        sCostMin = sCost;
                        eCostMin = eCost;
                    }
                }
            }
            const VertexPtr vNew =
                std::make_shared<Vertex>(vMin.get(), xNew, sCostMin, eCostMin);
            addVertex(vNew);
            // visualize(vNew->state, vMin->state);

            for (auto& nearKEdgeCost : NearKEdgeCosts) {
                double eCost = nearKEdgeCost.eCost;
                double sCost = vNew->state_cost + eCost;
                VertexPtr vNear = nearKEdgeCost.v;
                // visualize(vNew->state, vNear->state);
                if (sCost < vNear->state_cost) {
                    if (vNear->parent()->removeChild(vNear)) {
                        vNear->setParent(vNew, sCost, eCost);
                        vNew->addChild(vNear);
                        vNear->updateChildrenCost();
                        solution_.path = generatePath(vGoal);
                        // visualize(vNew->state, vNear->state);
                    } else
                        std::cout << "###### removeChild error ######"
                                  << std::endl;

                    if (vGoal->state_cost != goalCost) {
                        // std::cout << "goalCost update " << goalCost << " to "
                        // << vGoal->state_cost
                        //           << std::endl;
                        goalCost = vGoal->state_cost;
                        goalCostUpdated = true;
                        // visualize(vNew->state, vNear->state);
                        if (goalCost < costThreshold) {
                            solution_.status =
                                PlannerStatus::ApproximateSolution;
                            solution_.cost = goalCost;
                            approximateSolution = true;
                            break;
                        }
                    }
                }
            }
        }
        ++i;
        if (i % 20 == 0 && goalCostUpdated) {
            // visualize();
            prune(tree_, goalCost);
            initNearestNeighbor(tree_);
            // visualize();
            goalCostUpdated = false;
        }
    }

    if (solution_.status != PlannerStatus::ExactSolution && i >= param_.steps)
        solution_.status = PlannerStatus::TimeOut;

    LOGD("######## sample points: %d ", i);
    pt.rrtStarrSamples = i;
    auto endTime = std::chrono::steady_clock::now();
    float time = std::chrono::duration<float>(endTime - startTime).count();

    pt.informedRRTStar =
        std::chrono::duration<float>(endTime - startTime).count();

    std::cout << "informedRRT process time: " << time << " isValidEdgetime "
              << isValidEdgetime << std::endl;
    // visualize();
    return solution_;
}
void InformedRRTStar::visualize(const State& qNew, const State& qNear) const {
    vf_(plannerRecord());
    vpf_(qNew, Green, "new");
    vpf_(qNear, Purple, "near");
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
}
void InformedRRTStar::visualize() const {
    vf_(plannerRecord());
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
}
State InformedRRTStar::steer(const State& qSampled, const State& qNear,
                             double step_size) const {
    assert(qSampled.size() == qNear.size());
    State qNew;
    for (size_t i = 0; i < qNear.size(); i++) {
        double delta = step_size * sign(normalizeAngle(qSampled[i] - qNear[i]));
        qNew.position.push_back(delta + qNear[i]);
    }
    return qNew;
}
void InformedRRTStar::prune(Tree& tree, double costThreshold) {
    std::vector<VertexPtr> leaves;
    bool finished = false;

    while (!finished) {
        finished = true;
        leaves.clear();
        tree.leaves(tree.root(), leaves);
        for (const auto& v : leaves) {
            double vCost = heuristic(v);
            if (vCost > costThreshold) {
                if (tree.removeVertex(v))
                    finished = false;
                else
                    std::cout << "prune removeV Error!!!!!" << std::endl;
            }
        }
    }
}
double InformedRRTStar::heuristic(const VertexPtr& v) const {
    double costToGoal = edgeCost(v->state, current_goal_);
    return v->state_cost + costToGoal;
}
void InformedRRTStar::addVertex(const VertexPtr& v) {
    tree_.addVertex(v);
    nearest_neighbor_->update(v);
}
std::optional<BiRRT::Record> InformedRRTStar::findFeasiblePath() {
    solution_ = bi_rrt_.solve();
    if (solution_.status == PlannerStatus::ExactSolution)
        return bi_rrt_.record();
    return {};
}
bool InformedRRTStar::initialCheck() {
    assert(goal_->type == GoalType::JointTolerance);
    start_.printState("start");
    current_goal_ = goal_->sample().value();
    current_goal_.printState("goal");

    solution_ = PlannerSolution();

    if (!si_->checkers->isValid(start_)) {
        LOGD("start state is not valid");
        solution_.status = PlannerStatus::InvalidStart;
        return false;
    } else {
        LOGD("start state is valid");
    }

    if (!si_->checkers->isValid(current_goal_)) {
        LOGD("goal state is not valid");
        solution_.status = PlannerStatus::InvalidGoal;
        return false;
    } else {
        LOGD("goal state is valid");
    }

    return true;
}
void InformedRRTStar::updateNN(NearestNeighborBasePtr& nn, const VertexPtr& v) {
    nn->update(v);
    for (const auto& child : v->children())
        updateNN(nn, child);
}
void InformedRRTStar::initNearestNeighbor(const Tree& tree) {
    nearest_neighbor_->build(tree.root());
    for (const auto& child : tree.root()->children())
        updateNN(nearest_neighbor_, child);
}
bool InformedRRTStar::isValidEdge(const State& q1, const State& q2) const {
    if (si_->checkers->isValid(q2)) {
        if (si_->local_planner->validInterpolatePath(
                q1, q2, param_.local_planner_step_size_jps,
                param_.local_planner_step_size_tcp))
            return true;
    }
    return false;
}
double InformedRRTStar::edgeCost(const State& q1, const State& q2) const {
    return si_->distance->distance(q1, q2);
}
Path InformedRRTStar::generatePath(const VertexPtr& v) const {
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

PlannerRecord InformedRRTStar::plannerRecord() const {
    PlannerRecord pr;
    pr.start = start_;
    pr.goal = current_goal_;
    pr.addTree(tree_, "rrtTree");
    pr.addPath(solution_.path, "rawPath");
    pr.v1 = v1_;
    pr.v2 = v2_;
    return pr;
}

double InformedRRTStar::kRRT(double rewireFactor) const {
    // k_rrt > 2^(d + 1) * e * (1 + 1 / d).  K-nearest RRT*
    const double dim = static_cast<double>(si_->dimension);
    return rewireFactor * (std::pow(2, dim + 1) * M_E * (1.0 + 1.0 / dim));
}
double InformedRRTStar::rRRT(double rewireFactor) const {
    // r_rrt > (2*(1+1/d))^(1/d)*(measure/ballvolume)^(1/d)
    // If we're not using the informed measure, pruned_measure_ will be set to
    // si_->getSpaceMeasure();
    const double dim = static_cast<double>(si_->dimension);
    return rewireFactor *
           std::pow(2 * (1.0 + 1.0 / dim) *
                        (pruned_measure_ / unitNBallMeasure(si_->dimension)),
                    1.0 / dim);
}
BiRRTParam InformedRRTStar::toBirrtParam(
    const InformedRRTStarParam& param) const {
    return BiRRTParam(param_.biRRTParam);
}
} // namespace gsmpl
