#include <chrono>
#include <memory>
#include "assert.h"
#include "gsmpl/base/tree.h"
#include "gsmpl/planner_interface.h"
#include "gsmpl/utility/global.h"

namespace gsmpl {
PlannerTimes pt;
PlannerInterface::PlannerInterface(const PlannerContext &context,
                                   const ProblemDefinition &pd,
                                   const PlannerRecord::VisualFunction &vf,
                                   const PlannerRecord::VisualPoseFunction &vpf)
    : param_(context.general_param), pd_(pd), vf_(vf), vpf_(vpf) {
    bool success = allocPlanner(context);
    assert(success);
}

void PlannerInterface::allocSi(const PlannerContext &context) {
    auto sampler_param = std::static_pointer_cast<const SampleWithBiasParam>(
        context.sampler_param);
    si_ = std::make_shared<SpaceInformation>(context.general_param,
                                             *sampler_param, pd_,
                                             context.checkers, context.fk);
}

bool PlannerInterface::allocPlanner(const PlannerContext &context) {
    assert(context.planner_param != nullptr);
    assert(context.sampler_param != nullptr);

    switch (context.planner_type) {
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
// bool PlannerInterface::plan(PlannerSolution &solution) {
//     auto startTime1 = std::chrono::steady_clock::now();
//     solution = planner_->solve();
//     auto endTime1 = std::chrono::steady_clock::now();
//     record_ = planner_->plannerRecord();
//     std::cout << "solution.isValid() " << solution.isValid() << std::endl;

//     if (solution.isValid()) {
//         auto startTime2 = std::chrono::steady_clock::now();
//         auto simplifierSolution =
//             si_->simplifyPath(solution.path, param_.path_simplifier);
//         auto endTime2 = std::chrono::steady_clock::now();

//         std::cout << "PlannerInterface Trajectory start" << std::endl;
//         solution.trajectory = Trajectory(
//             si_->local_planner, simplifierSolution.simplified_2,
//             param_.traj_processing.step_size_jps,
//             param_.traj_processing.stepSizeTcp, param_.traj_processing.dt);
//         std::cout << "PlannerInterface Trajectory end" << std::endl;
        
//         solution.path = simplifierSolution.simplified_2;

//         auto endTime3 = std::chrono::steady_clock::now();
//         pt.planner =
//             std::chrono::duration<float>(endTime3 - startTime1).count();
//         pt.rrtSolver =
//             std::chrono::duration<float>(endTime1 - startTime1).count();
//         pt.pathProcessing =
//             std::chrono::duration<float>(endTime2 - startTime2).count();
//         pt.print();
//         record_.addPath(simplifierSolution.simplified, "simplified");
//         record_.addPath(simplifierSolution.smoothed, "smoothed");
//         record_.addPath(simplifierSolution.simplified_2, "simplified_2");
//         record_.addPath(solution.trajectory.dense_waypoints, "dense_waypoints");
//         // vf_(record_);
//         return true;
//     }

//     return false;
// }
// bool PlannerInterface::plan(PlannerSolution &solution) {
//     auto startTime1 = std::chrono::steady_clock::now();
//     solution = planner_->solve();
//     auto endTime1 = std::chrono::steady_clock::now();
//     record_ = planner_->plannerRecord();
//     std::cout << "solution.isValid() " << solution.isValid() << std::endl;

//     if (solution.isValid()) {
//         auto startTime2 = std::chrono::steady_clock::now();
//         pt.rrtSolver =
//             std::chrono::duration<float>(endTime1 - startTime1).count();

//         pt.print();
//         record_.addPath(solution.path, "solution.path");
//         // vf_(record_);
//         return true;
//     }

//     return false;
// }
bool PlannerInterface::plan(PlannerSolution &solution) {
    auto startTime1 = std::chrono::steady_clock::now();
    solution = planner_->solve();
    auto endTime1 = std::chrono::steady_clock::now();
    record_ = planner_->plannerRecord();
    std::cout << "solution.isValid() " << solution.isValid() << std::endl;

    if (solution.isValid()) {
        auto startTime2 = std::chrono::steady_clock::now();
        auto simplifierSolution =
            si_->simplifyPath(solution.path, param_.path_simplifier);
        auto endTime2 = std::chrono::steady_clock::now();

        std::cout << "PlannerInterface Trajectory start" << std::endl;
        solution.trajectory = Trajectory(
            si_->local_planner, simplifierSolution.simplified_2,
            param_.traj_processing.step_size_jps,
            param_.traj_processing.stepSizeTcp, param_.traj_processing.dt);
        std::cout << "PlannerInterface Trajectory end" << std::endl;
        
        solution.path = simplifierSolution.simplified_2;

        auto endTime3 = std::chrono::steady_clock::now();
        pt.planner =
            std::chrono::duration<float>(endTime3 - startTime1).count();
        pt.rrtSolver =
            std::chrono::duration<float>(endTime1 - startTime1).count();
        pt.pathProcessing =
            std::chrono::duration<float>(endTime2 - startTime2).count();
        pt.print();
        record_.addPath(simplifierSolution.simplified, "simplified");
        record_.addPath(simplifierSolution.smoothed, "smoothed");
        record_.addPath(simplifierSolution.simplified_2, "simplified_2");
        record_.addPath(solution.trajectory.dense_waypoints, "dense_waypoints");
        // vf_(record_);
        return true;
    }

    return false;
}
} // namespace gsmpl
