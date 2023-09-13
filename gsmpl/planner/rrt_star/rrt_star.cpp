#include <thread>
#include <math.h>
#include "gsmpl/utility/log_utility.h"
#include "gsmpl/base/math_utility.h"
#include "gsmpl/tools/nearest_neighbor/nearest_neighbor_linear.h"
#include "gsmpl/planner/rrt_star/rrt_star.h"

namespace gsmpl {
namespace {
RRTStarParamConstPtr param(const PlannerContext &context) {
    return std::static_pointer_cast<const RRTStarParam>(context.planner_param);
}
} // namespace

RRTStar::RRTStar(const SpaceInformationBasePtr &si, const ProblemDefinition &pd,
                 const PlannerContext &context,
                 const PlannerRecord::VisualFunction &vf,
                 const PlannerRecord::VisualPoseFunction &vpf)
    : Planner(si, pd),
      param_(*param(context)),
      vf_(vf),
      vpf_(vpf),
      nearest_neighbor_(std::make_shared<NearestNeighborLinear>(si_->distance)),
      pruned_measure_(si_->getSpaceMeasure()) {}

const PlannerSolution &RRTStar::solve() {
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
    std::vector<VertexPtr> goals;
    VertexPtr bestGoal = nullptr;
    double bestCost = std::numeric_limits<double>::max();
    bool haveSolution = false;
    double k_rrt = kRRT(1.2); // kRRT(1.2); // TODO: rewireFactor > 1
    double r_rrt = rRRT(1.2);

    solution_.status = PlannerStatus::Initialized;

    LOGD("*************************************");
    for (; i < param_.steps; ++i) {
        State xRand = si_->sampler->sample();
        double cardDbl = static_cast<double>(nearest_neighbor_->size() + 1);
        // unsigned int k = std::ceil(k_rrt * log(cardDbl));
        double r =
            std::max(param_.step_size,
                     r_rrt * std::pow(log(cardDbl) / cardDbl,
                                      1 / static_cast<double>(si_->dimension)));
        r = 0.5;

        const VertexPtr vNearest = nearest_neighbor_->nearest(xRand);

        visualize(xRand, vNearest->state); // visualize

        const State &xNew = steer(xRand, vNearest->state, param_.step_size);

        visualize(xNew, vNearest->state); // visualize

        if (isValidEdge(vNearest->state, xNew)) {
            std::vector<VertexPtr> vNearK =
                nearest_neighbor_->nearestR(xNew, r); // nearestK
            std::cout << "R_rrt " << r_rrt << " cardDbl " << cardDbl
                      << " vNearK " << vNearK.size() << " r " << r << std::endl;
            VertexPtr vMin = vNearest;
            double eCostMin = edgeCost(vNearest->state, xNew);
            double sCostMin = vNearest->state_cost + eCostMin;
            for (const auto &vNear : vNearK) {
                if (isValidEdge(vNear->state, xNew)) {
                    double eCost = edgeCost(vNear->state, xNew);
                    double sCost = vNear->state_cost + eCost;
                    if (sCost < sCostMin) {
                        vMin = vNear;
                        sCostMin = sCost;
                        eCostMin = eCost;
                    }
                }
            }
            const VertexPtr vNew =
                std::make_shared<Vertex>(vMin.get(), xNew, sCostMin, eCostMin);
            tree_.addVertex(vNew);
            nearest_neighbor_->update(vNew);

            visualize(xNew, vMin->state); // visualize

            if (!haveSolution && goal_->isSatisfied(vNew->state)) {
                std::cout << "goal satisfied " << std::endl;
                if (isValidEdge(vNew->state, current_goal_)) {
                    double eCost = edgeCost(vNew->state, current_goal_);
                    double sCost = vNew->state_cost + eCost;
                    const VertexPtr vGoal = std::make_shared<Vertex>(
                        vNew.get(), current_goal_, sCost, eCost);
                    tree_.addVertex(vGoal);
                    nearest_neighbor_->update(vGoal);
                    bestGoal = vGoal;
                    bestCost = bestGoal->state_cost;
                    solution_.status = PlannerStatus::ExactSolution;
                    haveSolution = true;

                    // visualize
                    solution_.path = generatePath(bestGoal);
                    visualize(xNew, vMin->state);

                    std::cout << "find a solution" << std::endl;
                }
            }

            for (auto &vNear : vNearK) {
                if (isValidEdge(xNew, vNear->state)) {
                    double eCost = edgeCost(xNew, vNear->state);
                    double sCost = vNew->state_cost + eCost;
                    if (sCost < vNear->state_cost) {
                        std::cout << "remove vertex" << std::endl;
                        vNear->parent()->removeChild(vNear);
                        vNear->setParent(vNew, sCost, eCost);
                        vNew->addChild(vNear);
                        vNear->updateChildrenCost();
                        if (bestGoal) {
                            solution_.path = generatePath(bestGoal);
                            if (bestGoal->state_cost != bestCost) {
                                bestCost = bestGoal->state_cost;
                                std::cout << "goal cost updated" << std::endl;
                            }
                        } else
                            solution_.path = generatePath(vNew);
                        visualize(vNew->state, vNear->state);
                    }
                }
            }
        }
    }

    if (solution_.status != PlannerStatus::ExactSolution && i >= param_.steps)
        solution_.status = PlannerStatus::TimeOut;

    LOGD("######## sample points: %d ", i);
    visualize();
    return solution_;
}

void RRTStar::update(const VertexPtr &vNew) {
    tree_.addVertex(vNew);
    nearest_neighbor_->update(vNew);
    solution_.path = generatePath(vNew);
}
State RRTStar::steer(const State &qSampled, const State &qNear,
                     double step_size) const {
    assert(qSampled.size() == qNear.size());
    State qNew;
    for (size_t i = 0; i < qNear.size(); i++) {
        double delta = step_size * sign(normalizeAngle(qSampled[i] - qNear[i]));
        qNew.position.push_back(delta + qNear[i]);
    }
    return qNew;
}

Path RRTStar::generatePath(const VertexPtr &v) const {
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

PlannerRecord RRTStar::plannerRecord() const {
    PlannerRecord pr;
    pr.start = start_;
    pr.goal = current_goal_;
    pr.addTree(tree_, "rrtTree");
    pr.addPath(solution_.path, "rawPath");
    return pr;
}

double RRTStar::kRRT(double rewireFactor) const {
    // k_rrt > 2^(d + 1) * e * (1 + 1 / d).  K-nearest RRT*
    const double dim = static_cast<double>(si_->dimension);
    return rewireFactor * (std::pow(2, dim + 1) * M_E * (1.0 + 1.0 / dim));
}
double RRTStar::rRRT(double rewireFactor) const {
    // r_rrt > (2*(1+1/d))^(1/d)*(measure/ballvolume)^(1/d)
    // If we're not using the informed measure, pruned_measure_ will be set to
    // si_->getSpaceMeasure();
    const double dim = static_cast<double>(si_->dimension);
    return rewireFactor *
           std::pow(2 * (1.0 + 1.0 / dim) *
                        (pruned_measure_ / unitNBallMeasure(si_->dimension)),
                    1.0 / dim);
}
void RRTStar::visualize() const { vf_(plannerRecord()); }
void RRTStar::visualize(const State &qNew, const State &qNear) const {
    std::cout << "rrtstar visualize states" << std::endl;
    vf_(plannerRecord());
    std::cout << "rrtstar visualize states qNew" << std::endl;
    vpf_(qNew, Green, "new");
    std::cout << "rrtstar visualize states qNear" << std::endl;
    vpf_(qNear, Purple, "near");
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
}
} // namespace gsmpl
