#pragma once

#include "planner_param.h"
#include "../base/state.h"
#include "../tools/checker/state_checker_group.h"
#include "../robot_algo/fk.h"

namespace gsmpl {
enum class PlannerType { RRT, BiRRT, RRTStar, InformedRRTStar };

struct PlannerContext {
    PlannerContext(PlannerType type, SamplerParamBasePtr sParam,
                   PlannerParamBasePtr pParam,
                   const PlannerGeneralParamters& gParam,
                   StateCheckerGroupPtr checkers, FKBasePtr fkPtr)
        : plannerType(type),
          samplerParam(sParam),
          plannerParam(pParam),
          generalParam(gParam),
          checkers(checkers),
          fk(fkPtr) {}

    PlannerType plannerType;
    SamplerParamBasePtr samplerParam;
    PlannerParamBasePtr plannerParam;
    PlannerGeneralParamters generalParam;
    StateCheckerGroupPtr checkers;
    FKBasePtr fk;
};
} // namespace gsmpl
