#pragma once

#include "gsmpl/planner_data/planner_param.h"
#include "gsmpl/base/state.h"
#include "gsmpl/tools/checker/state_checker_group.h"
#include "gsmpl/robot_algo/fk.h"

namespace gsmpl {
enum class PlannerType { RRT, BiRRT, RRTStar, InformedRRTStar };

struct PlannerContext {
    PlannerContext(PlannerType type, SamplerParamBasePtr sParam,
                   PlannerParamBasePtr pParam,
                   const PlannerGeneralParamters& gParam,
                   StateCheckerGroupPtr checkers, FKBasePtr fkPtr)
        : planner_type(type),
          sampler_param(sParam),
          planner_param(pParam),
          general_param(gParam),
          checkers(checkers),
          fk(fkPtr) {}

    PlannerType planner_type;
    SamplerParamBasePtr sampler_param;
    PlannerParamBasePtr planner_param;
    PlannerGeneralParamters general_param;
    StateCheckerGroupPtr checkers;
    FKBasePtr fk;
};
} // namespace gsmpl
