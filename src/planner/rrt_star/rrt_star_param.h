#pragma once

#include <cstddef>
#include "../../utility/class_forward.h"
#include "../../planner_data/planner_param.h"

namespace gsmpl {
GSMPL_STRUCT_FORWARD(RRTStarParam)

struct RRTStarParam : public PlannerParamBase {
public:
    RRTStarParam(std::size_t dim, unsigned int stepsP, double stepSizeP,
                 double lpStepSizeJps, double lpStepSizeTcp, double gt,
                 double gb)
        : dimension(dim),
          steps(stepsP),
          stepSize(stepSizeP),
          localPlannerStepSizeJps(lpStepSizeJps),
          localPlannerStepSizeTcp(lpStepSizeTcp),
          goalThreshold(gt),
          goalBias(gb) {}

    std::size_t dimension;
    unsigned int steps;
    double stepSize;
    double localPlannerStepSizeJps;
    double localPlannerStepSizeTcp;
    double goalThreshold;
    double goalBias; // The fraction of time the goal is picked as the state to
                     // expand towards (if such a state is available)
};
} // namespace gsmpl
namespace rrt_star {
constexpr unsigned int RRT_MAX_STEPS = 50000;
constexpr double LOCAL_PLANNER_SETP_SIZE_TCP = 0.01;  // 1cm
constexpr double LOCAL_PLANNER_SETP_SIZE_JPS = 0.035; // 0.035; // 2deg
constexpr double RRT_STEP_SIZE = LOCAL_PLANNER_SETP_SIZE_JPS * 5;
constexpr double GOAL_THRESHOLD = RRT_STEP_SIZE * 5;
constexpr double GOAL_BIAS = 0.1; // 0.05
constexpr double COST_THRESHOLD = 50.0;
constexpr double CONE_THRESHOLD = 0.3; // 0.3rad = 17deg

constexpr unsigned int PATH_SIMPLIFIER_MAX_EMPTY_STEPS = 10;
constexpr unsigned int PATH_SIMPLIFIER_REDUCE_VERTICES_MAX_STEPS = 15;
constexpr double PATH_SIMPLIFIER_REDUCE_VERTICES_RANGE_RATIO = 0.33;
constexpr unsigned int PATH_SIMPLIFIER_SMOOTH_BSPLINE_MAX_STEPS = 10;
constexpr double PATH_SIMPLIFIER_SMOOTH_BSPLINE_MIN_CHANGE =
    LOCAL_PLANNER_SETP_SIZE_JPS * 0.05;
constexpr unsigned int PATH_SIMPLIFIER_COLLAPSE_CLOSE_VERTICES_MAX_STEPS = 0;

constexpr unsigned int SAMPLER_ATTEMPTS = 10;
} // namespace rrt_star
