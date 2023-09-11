#pragma once

#include <optional>
#include <memory>
#include "gsmpl/tools/sampler/sampler.h"
#include "gsmpl/tools/checker/state_checker_group.h"
#include "gsmpl/planner_data/planner_param.h"
#include "gsmpl/utility/class_forward.h"
#include "gsmpl/tools/local_planner/joint_interpolate.h"
#include "gsmpl/tools/distance/distance.h"
#include "gsmpl/tools/distance/distanceSe3.h"
#include "gsmpl/tools/path_simplifier/path_simplifier.h"
#include "gsmpl/robot_algo/fk.h"
#include "gsmpl/context/problem_definition.h"

namespace gsmpl {
GSMPL_CLASS_FORWARD(SpaceInformationBase)

class SpaceInformationBase {
public:
    struct FlexibleTools {
        DistanceBasePtr distance;
        StateCheckerGroupPtr checkers;
        SamplerBasePtr sampler;
        LocalPlannerBasePtr local_planner;
    };
    SpaceInformationBase(std::size_t dim, const FlexibleTools& tools)
        : dimension(dim),
          distance(tools.distance),
          checkers(tools.checkers),
          sampler(tools.sampler),
          local_planner(tools.local_planner),
          path_simplifier(local_planner, distance) {}
    virtual ~SpaceInformationBase() = default;

    std::optional<State> validSample();
    virtual PathSimplifier::SolutionRecord simplifyPath(
        const Path& rawPath, const PathSimplifierParamters& param) = 0;
    virtual double getSpaceMeasure() const = 0;

public:
    std::size_t dimension;
    DistanceBasePtr distance;
    StateCheckerGroupPtr checkers;
    SamplerBasePtr sampler;
    LocalPlannerBasePtr local_planner;
    PathSimplifier path_simplifier;
};

class SpaceInformation : public SpaceInformationBase {
public:
    SpaceInformation(const PlannerGeneralParamters& param,
                     const SampleWithBiasParam& sampler_param,
                     const ProblemDefinition& pd,
                     const StateCheckerGroupPtr& checkers, const FKBasePtr& fk)
        : SpaceInformationBase(
              param.dimension,
              creatFlexibleTools(param, sampler_param, pd, checkers, fk)),
          bounds_(param.bounds) {}

    PathSimplifier::SolutionRecord simplifyPath(
        const Path& rawPath, const PathSimplifierParamters& param) override;
    double getSpaceMeasure() const override { return bounds_.getMeasure(); }

private:
    SpaceInformationBase::FlexibleTools creatFlexibleTools(
        const PlannerGeneralParamters& param,
        const SampleWithBiasParam& sampler_param, const ProblemDefinition& pd,
        const StateCheckerGroupPtr& checkers, const FKBasePtr& fk);
    Bounds bounds_;
};
} // namespace gsmpl
