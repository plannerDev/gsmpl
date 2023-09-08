#include <chrono>
#include <memory>
#include "assert.h"
#include "base/tree.h"
#include "planner_interface.h"
#include "utility/global.h"

namespace gsmpl {
PlannerTimes pt;
PlannerInterface::PlannerInterface(const PlannerContext &context,
                                   const ProblemDefinition &pd,
                                   const PlannerRecord::VisualFunction &vf,
                                   const PlannerRecord::VisualPoseFunction &vpf)
    : param_(context.generalParam), pd_(pd), vf_(vf), vpf_(vpf) {
    bool success = allocPlanner(context);
    assert(success);
}

void PlannerInterface::allocSi(const PlannerContext &context) {
    auto samplerParam = std::static_pointer_cast<const SampleWithBiasParam>(
        context.samplerParam);
    si_ = std::make_shared<SpaceInformation>(
        context.generalParam, *samplerParam, pd_, context.checkers, context.fk);
}

bool PlannerInterface::allocPlanner(const PlannerContext &context) {
    assert(context.plannerParam != nullptr);
    assert(context.samplerParam != nullptr);

    switch (context.plannerType) {
        case PlannerType::RRT: {
            allocSi(context);
            planner_ = std::make_shared<RRT>(si_, pd_, context, vf_);
            return true;
        }
        case PlannerType::BiRRT: {
            allocSi(context);
            planner_ = std::make_shared<BiRRT>(si_, pd_, context, vf_);
            return true;
        }
        case PlannerType::RRTStar: {
            allocSi(context);
            planner_ = std::make_shared<RRTStar>(si_, pd_, context, vf_, vpf_);
            return true;
        }
        case PlannerType::InformedRRTStar: {
            allocSi(context);
            planner_ =
                std::make_shared<InformedRRTStar>(si_, pd_, context, vf_, vpf_);
            return true;
        }
    }
    return false;
}
bool PlannerInterface::plan(PlannerSolution &solution) {
    auto startTime1 = std::chrono::steady_clock::now();
    solution = planner_->solve();
    auto endTime1 = std::chrono::steady_clock::now();
    record_ = planner_->plannerRecord();

    if (solution.isValid()) {
        auto startTime2 = std::chrono::steady_clock::now();
        auto simplifierSolution =
            si_->simplifyPath(solution.path, param_.pathSimplifier);
        auto endTime2 = std::chrono::steady_clock::now();

        solution.trajectory = simplifierSolution.simplified_2;
        solution.path = simplifierSolution.simplified_2.waypoints;

        auto endTime3 = std::chrono::steady_clock::now();
        pt.planner =
            std::chrono::duration<float>(endTime3 - startTime1).count();
        pt.rrtSolver =
            std::chrono::duration<float>(endTime1 - startTime1).count();
        pt.pathProcessing =
            std::chrono::duration<float>(endTime2 - startTime2).count();
        pt.print();
        record_.addPath(simplifierSolution.simplified.waypoints, "simplified");
        record_.addPath(simplifierSolution.smoothed.waypoints, "smoothed");
        record_.addPath(simplifierSolution.simplified_2.waypoints,
                        "simplified_2");
        vf_(record_);
        return true;
    }

    return false;
}
} // namespace gsmpl
