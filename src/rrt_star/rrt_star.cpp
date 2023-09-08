#include <thread>
#include <math.h>
#include "../utility/log_utility.h"
#include "../base/math_utility.h"
#include "../tools/nearest_neighbor/nearest_neighbor_linear.h"
#include "rrt_star.h"

namespace gsmpl {
namespace {
RRTStarParamConstPtr param(const PlannerContext &context) {
    return std::static_pointer_cast<const RRTStarParam>(context.plannerParam);
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
      nearestNeighbor_(std::make_shared<NearestNeighborLinear>(si_->distance)),
      prunedMeasure_(si_->getSpaceMeasure()) {}

const PlannerSolution &RRTStar::solve() {
    LOGD();
    assert(goal_->type == GoalType::JointTolerance);
    start_.printState("start");
    currentGoal_ = goal_->sample().value();
    currentGoal_.printState("goal");

    solution_ = PlannerSolution();

    if (!si_->checkers->isValid(start_)) {
        LOGD("start state is not valid");
        solution_.status = PlannerStatus::InvalidStart;
        return solution_;
    } else {
        LOGD("start state is valid");
    }

    if (!si_->checkers->isValid(currentGoal_)) {
        LOGD("goal state is not valid");
        solution_.status = PlannerStatus::InvalidGoal;
        return solution_;
    } else {
        LOGD("goal state is valid");
    }

    tree_.setRoot(std::make_shared<Vertex>(nullptr, start_));
    nearestNeighbor_->build(tree_.root());
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
        double cardDbl = static_cast<double>(nearestNeighbor_->size() + 1);
        // unsigned int k = std::ceil(k_rrt * log(cardDbl));
        double r =
            std::max(param_.stepSize,
                     r_rrt * std::pow(log(cardDbl) / cardDbl,
                                      1 / static_cast<double>(si_->dimension)));
        r = 0.5;

        const VertexPtr vNearest = nearestNeighbor_->nearest(xRand);

        visualize(xRand, vNearest->state); // visualize

        const State &xNew = steer(xRand, vNearest->state, param_.stepSize);

        visualize(xNew, vNearest->state); // visualize

        if (isValidEdge(vNearest->state, xNew)) {
            std::vector<VertexPtr> vNearK =
                nearestNeighbor_->nearestR(xNew, r); // nearestK
            std::cout << "R_rrt " << r_rrt << " cardDbl " << cardDbl
                      << " vNearK " << vNearK.size() << " r " << r << std::endl;
            VertexPtr vMin = vNearest;
            double eCostMin = edgeCost(vNearest->state, xNew);
            double sCostMin = vNearest->stateCost + eCostMin;
            for (const auto &vNear : vNearK) {
                if (isValidEdge(vNear->state, xNew)) {
                    double eCost = edgeCost(vNear->state, xNew);
                    double sCost = vNear->stateCost + eCost;
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
            nearestNeighbor_->update(vNew);

            visualize(xNew, vMin->state); // visualize

            if (!haveSolution && goal_->isSatisfied(vNew->state)) {
                std::cout << "goal satisfied " << std::endl;
                if (isValidEdge(vNew->state, currentGoal_)) {
                    double eCost = edgeCost(vNew->state, currentGoal_);
                    double sCost = vNew->stateCost + eCost;
                    const VertexPtr vGoal = std::make_shared<Vertex>(
                        vNew.get(), currentGoal_, sCost, eCost);
                    tree_.addVertex(vGoal);
                    nearestNeighbor_->update(vGoal);
                    bestGoal = vGoal;
                    bestCost = bestGoal->stateCost;
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
                    double sCost = vNew->stateCost + eCost;
                    if (sCost < vNear->stateCost) {
                        std::cout << "remove vertex" << std::endl;
                        vNear->parent()->removeChild(vNear);
                        vNear->setParent(vNew, sCost, eCost);
                        vNew->addChild(vNear);
                        vNear->updateChildrenCost();
                        if (bestGoal) {
                            solution_.path = generatePath(bestGoal);
                            if (bestGoal->stateCost != bestCost) {
                                bestCost = bestGoal->stateCost;
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
    nearestNeighbor_->update(vNew);
    solution_.path = generatePath(vNew);
}
State RRTStar::steer(const State &qSampled, const State &qNear,
                     double stepSize) const {
    assert(qSampled.size() == qNear.size());
    State qNew;
    for (size_t i = 0; i < qNear.size(); i++) {
        double delta = stepSize * sign(normalizeAngle(qSampled[i] - qNear[i]));
        qNew.push_back(delta + qNear[i]);
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
    pr.goal = currentGoal_;
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
    // If we're not using the informed measure, prunedMeasure_ will be set to
    // si_->getSpaceMeasure();
    const double dim = static_cast<double>(si_->dimension);
    return rewireFactor *
           std::pow(2 * (1.0 + 1.0 / dim) *
                        (prunedMeasure_ / unitNBallMeasure(si_->dimension)),
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
