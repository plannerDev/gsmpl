#pragma once

#include <optional>
#include "../context/planner.h"
#include "../tools/nearest_neighbor/nearest_neighbor.h"
#include "../planner_data/planner_record.h"
#include "../bi_rrt/bi_rrt.h"
#include "../tools/sampler/informed_sampler.h"
#include "informed_rrt_star_param.h"

namespace gsmpl {
class InformedRRTStar : public Planner
{
public:
    enum Status { Reached, Advanced, Trapped };

    InformedRRTStar(const SpaceInformationBasePtr& si, const ProblemDefinition& pd,
                    const PlannerContext& context, const PlannerRecord::VisualFunction& vf,
                    const PlannerRecord::VisualPoseFunction& vpf = nullptr);

    const PlannerSolution& solve() override;
    PlannerRecord plannerRecord() const override;
    void visualize() const;

private:
    struct NearKEdgeCost
    {
        NearKEdgeCost(const VertexPtr& vp, double cost) : v(vp), eCost(cost) {}

        VertexPtr v{nullptr};
        double eCost{0};
    };

    void prune(Tree& tree, double costThreshold);
    double heuristic(const VertexPtr& v) const;
    void addVertex(const VertexPtr& v);
    std::optional<BiRRT::Record> findFeasiblePath();
    bool initialCheck();
    void initNearestNeighbor(const Tree& tree);
    void updateNN(NearestNeighborBasePtr& nn, const VertexPtr& v);
    Path generatePath(const VertexPtr& v) const;
    State steer(const State& qSampled, const State& qNear, double stepSize) const;
    bool isValidEdge(const State& q1, const State& q2) const;
    double edgeCost(const State& q1, const State& q2) const;
    double kRRT(double rewireFactor) const;
    double rRRT(double rewireFactor) const;
    BiRRTParam toBirrtParam(const InformedRRTStarParam& param) const;
    void visualize(const State& qNew, const State& qNear) const;

    InformedRRTStarParam param_;
    NearestNeighborBasePtr nearestNeighbor_;
    Tree tree_;
    State currentGoal_;
    State v1_;
    State v2_;
    PlannerRecord::VisualFunction vf_;
    PlannerRecord::VisualPoseFunction vpf_;
    double prunedMeasure_;
    BiRRT biRRT_;
};
} // namespace gsmpl
