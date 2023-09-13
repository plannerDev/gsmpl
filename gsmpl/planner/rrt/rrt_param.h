#pragma once

#include <cstddef>
#include "gsmpl/utility/class_forward.h"
#include "gsmpl/planner_data/planner_param.h"

namespace gsmpl {
GSMPL_STRUCT_FORWARD(RRTParam)

struct RRTParam : public PlannerParamBase {
public:
    RRTParam(std::size_t dim, unsigned int stepsP, double stepSizeP,
             double lpStepSizeJps, double lpStepSizeTcp, double gt, double gb)
        : dimension(dim),
          steps(stepsP),
          step_size(stepSizeP),
          local_planner_step_size_jps(lpStepSizeJps),
          local_planner_step_size_tcp(lpStepSizeTcp),
          goal_threshold(gt),
          goal_bias(gb) {}

    std::size_t dimension;
    unsigned int steps;
    double step_size;
    double local_planner_step_size_jps;
    double local_planner_step_size_tcp;
    double goal_threshold;
    double goal_bias; // The fraction of time the goal is picked as the state to
                     // expand towards (if such a state is available)
};
} // namespace gsmpl
namespace rrt {
constexpr unsigned int RRT_MAX_STEPS = 50000;
constexpr double LOCAL_PLANNER_SETP_SIZE_TCP = 0.01;  // 1cm
constexpr double LOCAL_PLANNER_SETP_SIZE_JPS = 0.035; // 0.035; // 2deg
constexpr double RRT_STEP_SIZE = LOCAL_PLANNER_SETP_SIZE_JPS * 20;
constexpr double GOAL_THRESHOLD = 0.05;
constexpr double GOAL_BIAS = 0.05;
constexpr double COST_THRESHOLD = 50.0;
constexpr double CONE_THRESHOLD = 0.3; // 0.3rad = 17deg

constexpr unsigned int PATH_SIMPLIFIER_MAX_EMPTY_STEPS = 10;
constexpr unsigned int PATH_SIMPLIFIER_REDUCE_VERTICES_MAX_STEPS = 15;
constexpr double PATH_SIMPLIFIER_REDUCE_VERTICES_RANGE_RATIO = 0.33;
constexpr unsigned int PATH_SIMPLIFIER_SMOOTH_BSPLINE_MAX_STEPS = 10;
constexpr double PATH_SIMPLIFIER_SMOOTH_BSPLINE_MIN_CHANGE =
    LOCAL_PLANNER_SETP_SIZE_JPS * 0.05;
constexpr unsigned int PATH_SIMPLIFIER_COLLAPSE_CLOSE_VERTICES_MAX_STEPS = 0;
constexpr double DT = 0.01;

constexpr unsigned int SAMPLER_ATTEMPTS = 10;
} // namespace rrt
