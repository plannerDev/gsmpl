#include "gsmpl/tools/checker/state_checker_group.h"
#include "gsmpl/utility/global.h"
#include "gsmpl/context/space_information.h"

namespace gsmpl {
std::optional<State> SpaceInformationBase::validSample() {
    for (unsigned int i = 0; i < sampler->attempts; i++) {
        State q = sampler->sample();
        if (checkers->isValid(q))
            return q;
    }
    return {};
}

SpaceInformationBase::FlexibleTools SpaceInformation::creatFlexibleTools(
    const PlannerGeneralParamters& param,
    const SampleWithBiasParam& sampler_param, const ProblemDefinition& pd,
    const StateCheckerGroupPtr& checkers, const FKBasePtr& fk) // TODO
{
    SpaceInformationBase::FlexibleTools tools;
    tools.distance = std::make_shared<DistanceL2>(param.bounds);
    tools.sampler = std::make_shared<SampleWithBias>(
        param.bounds, pd.goal->sample().value(), sampler_param);
    // tools.sampler = std::make_shared<SampleUniform>(param.bounds,
    // sampler_param.attempts);
    tools.checkers = checkers;
    tools.local_planner = std::make_shared<JointInterpolate>(
        std::make_shared<DistanceLInfinity>(param.bounds), tools.checkers, fk);
    return tools;
}
PathSimplifier::SolutionRecord SpaceInformation::simplifyPath(
    const Path& rawPath, const PathSimplifierParamters& param) {
    PathSimplifier::SolutionRecord solution;
    pt.rrtStarPathLength = pathLength(rawPath, distance);
    pt.rrtStarPathSize = rawPath.size();
    Path simplifiedPath = path_simplifier.reduceVertices(
        rawPath, param.step_size_jps, param.stepSizeTcp,
        param.reduceVerticesMaxSteps, param.maxEmptySteps,
        param.reduceVerticesRangeRatio);
    solution.simplified = path_simplifier.generateTrajector(
        simplifiedPath, param.step_size_jps, param.stepSizeTcp);
    LOGD("simplified waypoints size:%d",
         static_cast<int>(solution.simplified.waypoints.size()));
    std::cout << "simplified waypoints size: "
              << solution.simplified.waypoints.size() << " length: "
              << pathLength(solution.simplified.waypoints, distance)
              << std::endl;
    auto smoothedPath = path_simplifier.smoothBSpline(
        solution.simplified.waypoints, param.smoothBSplineMaxSteps,
        param.smoothBSplineMinChange, param.step_size_jps, param.stepSizeTcp);
    solution.smoothed = path_simplifier.generateTrajector(
        smoothedPath, param.step_size_jps, param.stepSizeTcp);
    LOGD("smoothed waypoints size:%d",
         static_cast<int>(solution.smoothed.waypoints.size()));
    std::cout << "smoothed waypoints size: "
              << solution.smoothed.waypoints.size() << " length: "
              << pathLength(solution.smoothed.waypoints, distance) << std::endl;

    Path simplifiedPath_2 = smoothedPath;
    int i = 0;
    while (simplifiedPath_2.size() > 10 && i < 10) {
        simplifiedPath_2 = path_simplifier.reduceVertices(
            solution.smoothed.waypoints, param.step_size_jps, param.stepSizeTcp,
            param.reduceVerticesMaxSteps * 15, param.maxEmptySteps,
            param.reduceVerticesRangeRatio); // * 15
        std::cout << "simplifyed once more!!!" << std::endl;
        i++;
    }
    if (simplifiedPath_2.size() > 15 &&
        simplifiedPath_2.size() > simplifiedPath.size()) {
        simplifiedPath_2 = path_simplifier.reduceVertices(
            simplifiedPath, param.step_size_jps, param.stepSizeTcp,
            param.reduceVerticesMaxSteps * 15, param.maxEmptySteps,
            param.reduceVerticesRangeRatio); // * 15
        std::cout << "reduceVertices twice!!!" << std::endl;
    }

    solution.simplified_2 = path_simplifier.generateTrajector(
        simplifiedPath_2, param.step_size_jps, param.stepSizeTcp);
    pt.finalPathLength = pathLength(solution.simplified_2.waypoints, distance);
    std::cout << "simplified_2 waypoints size: "
              << solution.simplified_2.waypoints.size() << " length: "
              << pathLength(solution.simplified_2.waypoints, distance)
              << std::endl;
    pt.pathSize = solution.simplified_2.waypoints.size();

    return solution;
}
} // namespace gsmpl
