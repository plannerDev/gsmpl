#pragma once

#include <optional>
#include <memory>
#include "../tools/sampler/sampler.h"
#include "../tools/checker/state_checker_group.h"
#include "../planner_data/planner_param.h"
#include "../utility/class_forward.h"
#include "../tools/local_planner/joint_interpolate.h"
#include "../tools/distance/distance.h"
#include "../tools/distance/distanceSe3.h"
#include "../tools/path_simplifier/path_simplifier.h"
#include "../robot_algo/fk.h"
#include "problem_definition.h"

namespace gsmpl {
GSMPL_CLASS_FORWARD(SpaceInformationBase)

class SpaceInformationBase
{
public:
    struct FlexibleTools
    {
        DistanceBasePtr distance;
        StateCheckerGroupPtr checkers;
        SamplerBasePtr sampler;
        LocalPlannerBasePtr localPlanner;
    };
    SpaceInformationBase(std::size_t dim, const FlexibleTools& tools)
        : dimension(dim), distance(tools.distance), checkers(tools.checkers),
          sampler(tools.sampler), localPlanner(tools.localPlanner),
          pathSimplifier(localPlanner, distance)
    {
    }
    virtual ~SpaceInformationBase() = default;

    std::optional<State> validSample();
    virtual PathSimplifier::SolutionRecord simplifyPath(const Path& rawPath,
                                                        const PathSimplifierParamters& param) = 0;
    virtual double getSpaceMeasure() const = 0;

public:
    std::size_t dimension;
    DistanceBasePtr distance;
    StateCheckerGroupPtr checkers;
    SamplerBasePtr sampler;
    LocalPlannerBasePtr localPlanner;
    PathSimplifier pathSimplifier;
};

class SpaceInformation : public SpaceInformationBase
{
public:
    SpaceInformation(const PlannerGeneralParamters& param, const SampleWithBiasParam& samplerParam,
                     const ProblemDefinition& pd, const StateCheckerGroupPtr& checkers,
                     const FKBasePtr& fk)
        : SpaceInformationBase(param.dimension,
                               creatFlexibleTools(param, samplerParam, pd, checkers, fk)),
          bounds_(param.bounds)
    {
    }

    PathSimplifier::SolutionRecord simplifyPath(const Path& rawPath,
                                                const PathSimplifierParamters& param) override;
    double getSpaceMeasure() const override { return bounds_.getMeasure(); }

private:
    SpaceInformationBase::FlexibleTools creatFlexibleTools(const PlannerGeneralParamters& param,
                                                           const SampleWithBiasParam& samplerParam,
                                                           const ProblemDefinition& pd,
                                                           const StateCheckerGroupPtr& checkers,
                                                           const FKBasePtr& fk);
    Bounds bounds_;
};
} // namespace gsmpl
