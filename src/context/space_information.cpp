#include "../tools/checker/state_checker_group.h"
#include "../utility/global.h"
#include "space_information.h"

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
    const SampleWithBiasParam& samplerParam, const ProblemDefinition& pd,
    const StateCheckerGroupPtr& checkers, const FKBasePtr& fk) // TODO
{
    SpaceInformationBase::FlexibleTools tools;
    tools.distance = std::make_shared<DistanceL2>(param.bounds);
    tools.sampler = std::make_shared<SampleWithBias>(
        param.bounds, pd.goal->sample().value(), samplerParam);
    // tools.sampler = std::make_shared<SampleUniform>(param.bounds,
    // samplerParam.attempts);
    tools.checkers = checkers;
    tools.localPlanner = std::make_shared<JointInterpolate>(
        std::make_shared<DistanceLInfinity>(param.bounds), tools.checkers, fk);
    return tools;
}
PathSimplifier::SolutionRecord SpaceInformation::simplifyPath(
    const Path& rawPath, const PathSimplifierParamters& param) {
    PathSimplifier::SolutionRecord solution;
    pt.rrtStarPathLength = pathLength(rawPath, distance);
    pt.rrtStarPathSize = rawPath.size();
    Path simplifiedPath = pathSimplifier.reduceVertices(
        rawPath, param.stepSizeJps, param.stepSizeTcp,
        param.reduceVerticesMaxSteps, param.maxEmptySteps,
        param.reduceVerticesRangeRatio);
    solution.simplified = pathSimplifier.generateTrajector(
        simplifiedPath, param.stepSizeJps, param.stepSizeTcp);
    LOGD("simplified waypoints size:%d",
         static_cast<int>(solution.simplified.waypoints.size()));
    std::cout << "simplified waypoints size: "
              << solution.simplified.waypoints.size() << " length: "
              << pathLength(solution.simplified.waypoints, distance)
              << std::endl;
    auto smoothedPath = pathSimplifier.smoothBSpline(
        solution.simplified.waypoints, param.smoothBSplineMaxSteps,
        param.smoothBSplineMinChange, param.stepSizeJps, param.stepSizeTcp);
    solution.smoothed = pathSimplifier.generateTrajector(
        smoothedPath, param.stepSizeJps, param.stepSizeTcp);
    LOGD("smoothed waypoints size:%d",
         static_cast<int>(solution.smoothed.waypoints.size()));
    std::cout << "smoothed waypoints size: "
              << solution.smoothed.waypoints.size() << " length: "
              << pathLength(solution.smoothed.waypoints, distance) << std::endl;

    Path simplifiedPath_2 = smoothedPath;
    int i = 0;
    while (simplifiedPath_2.size() > 10 && i < 10) {
        simplifiedPath_2 = pathSimplifier.reduceVertices(
            solution.smoothed.waypoints, param.stepSizeJps, param.stepSizeTcp,
            param.reduceVerticesMaxSteps * 15, param.maxEmptySteps,
            param.reduceVerticesRangeRatio); // * 15
        std::cout << "simplifyed once more!!!" << std::endl;
        i++;
    }
    if (simplifiedPath_2.size() > 15 &&
        simplifiedPath_2.size() > simplifiedPath.size()) {
        simplifiedPath_2 = pathSimplifier.reduceVertices(
            simplifiedPath, param.stepSizeJps, param.stepSizeTcp,
            param.reduceVerticesMaxSteps * 15, param.maxEmptySteps,
            param.reduceVerticesRangeRatio); // * 15
        std::cout << "reduceVertices twice!!!" << std::endl;
    }

    solution.simplified_2 = pathSimplifier.generateTrajector(
        simplifiedPath_2, param.stepSizeJps, param.stepSizeTcp);
    pt.finalPathLength = pathLength(solution.simplified_2.waypoints, distance);
    std::cout << "simplified_2 waypoints size: "
              << solution.simplified_2.waypoints.size() << " length: "
              << pathLength(solution.simplified_2.waypoints, distance)
              << std::endl;
    pt.pathSize = solution.simplified_2.waypoints.size();

    return solution;
}
} // namespace gsmpl
