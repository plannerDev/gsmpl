#pragma once

#include <cstddef>
#include "gsmpl/utility/class_forward.h"
#include "gsmpl/planner_data/planner_param.h"
#include "gsmpl/planner/bi_rrt/bi_rrt_param.h"

namespace gsmpl {
GSMPL_STRUCT_FORWARD(InformedRRTStarParam)

struct InformedRRTStarParam : public PlannerParamBase {
public:
    InformedRRTStarParam(std::size_t dim, unsigned int stepsP, double stepSizeP,
                         double lpStepSizeJps, double lpStepSizeTcp, double gct,
                         const BiRRTParam& biRRTP)
        : dimension(dim),
          steps(stepsP),
          step_size(stepSizeP),
          local_planner_step_size_jps(lpStepSizeJps),
          local_planner_step_size_tcp(lpStepSizeTcp),
          goalCostThreshold(gct),
          biRRTParam(biRRTP) {}

    const std::size_t dimension;
    const unsigned int steps;
    const double step_size;
    const double local_planner_step_size_jps;
    const double local_planner_step_size_tcp;
    const double goalCostThreshold;
    BiRRTParam biRRTParam;
};
} // namespace gsmpl
namespace informed_rrt_star {
constexpr unsigned int RRT_MAX_STEPS = 50;
constexpr double LOCAL_PLANNER_SETP_SIZE_TCP = 0.01;  // 1cm
constexpr double LOCAL_PLANNER_SETP_SIZE_JPS = 0.035; // 0.035rad // 2deg
constexpr double RRT_STEP_SIZE = LOCAL_PLANNER_SETP_SIZE_JPS * 5;
constexpr double GOAL_BIAS = 0.05; // 0.05
constexpr double COST_THRESHOLD = 50.0;
constexpr double CONE_THRESHOLD = 0.3; // 0.3rad = 17deg
constexpr double GOAL_COST_THRESHOLD = 1.0;

constexpr unsigned int PATH_SIMPLIFIER_MAX_EMPTY_STEPS = 10;
constexpr unsigned int PATH_SIMPLIFIER_REDUCE_VERTICES_MAX_STEPS = 15;
constexpr double PATH_SIMPLIFIER_REDUCE_VERTICES_RANGE_RATIO = 0.33;
constexpr unsigned int PATH_SIMPLIFIER_SMOOTH_BSPLINE_MAX_STEPS = 10;
constexpr double PATH_SIMPLIFIER_SMOOTH_BSPLINE_MIN_CHANGE =
    LOCAL_PLANNER_SETP_SIZE_JPS * 0.05;
constexpr unsigned int PATH_SIMPLIFIER_COLLAPSE_CLOSE_VERTICES_MAX_STEPS = 0;
constexpr double DT = 0.01;

constexpr unsigned int SAMPLER_ATTEMPTS = 10;
} // namespace informed_rrt_star
