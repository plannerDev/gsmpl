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
    std::cout << "simplifyPath 1" << std::endl;
    pt.rrtStarPathLength = pathLength(rawPath, distance);
    std::cout << "simplifyPath 2" << std::endl;
    pt.rrtStarPathSize = rawPath.size();
    std::cout << "simplifyPath 3" << std::endl;
    solution.simplified = path_simplifier.reduceVertices(
        rawPath, param.step_size_jps, param.stepSizeTcp,
        param.reduceVerticesMaxSteps, param.maxEmptySteps,
        param.reduceVerticesRangeRatio);
    
    std::cout << "simplifyPath 4" << std::endl;
    LOGD("simplified waypoints size:%d",
         static_cast<int>(solution.simplified.size()));
    std::cout << "simplified waypoints size: "
              << solution.simplified.size() << " length: "
              << pathLength(solution.simplified, distance)
              << std::endl;
    solution.smoothed = path_simplifier.smoothBSpline(
        solution.simplified, param.smoothBSplineMaxSteps,
        param.smoothBSplineMinChange, param.step_size_jps, param.stepSizeTcp);
    std::cout << "simplifyPath 5" << std::endl;
    LOGD("smoothed waypoints size:%d",
         static_cast<int>(solution.smoothed.size()));
    std::cout << "smoothed waypoints size: "
              << solution.smoothed.size() << " length: "
              << pathLength(solution.smoothed, distance) << std::endl;

    solution.simplified_2 = solution.smoothed;
    std::cout << "simplifyPath 6" << std::endl;
    int i = 0;
    while (solution.simplified_2.size() > 10 && i < 10) {
        solution.simplified_2 = path_simplifier.reduceVertices(
            solution.simplified_2, param.step_size_jps, param.stepSizeTcp,
            param.reduceVerticesMaxSteps * 15, param.maxEmptySteps,
            param.reduceVerticesRangeRatio); // * 15
        std::cout << "simplifyed once more!!!" << std::endl;
        i++;
    }
    std::cout << "simplifyPath 7" << std::endl;
    
    pt.finalPathLength = pathLength(solution.simplified_2, distance);
    std::cout << "simplified_2 size: "
              << solution.simplified_2.size() << " length: "
              << pathLength(solution.simplified_2, distance)
              << std::endl;
    pt.pathSize = solution.simplified_2.size();
    std::cout << "simplifyPath 8" << std::endl;
    return solution;
}
} // namespace gsmpl
