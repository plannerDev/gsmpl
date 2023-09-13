#pragma once

#include "gsmpl/utility/class_forward.h"
#include "gsmpl/base/bounds.h"

namespace gsmpl {
GSMPL_STRUCT_FORWARD(PlannerParamBase)
GSMPL_STRUCT_FORWARD(SamplerParamBase)

struct PathSimplifierParamters {
    double step_size_jps;
    double stepSizeTcp;
    unsigned int maxEmptySteps;
    unsigned int reduceVerticesMaxSteps;
    double reduceVerticesRangeRatio;
    unsigned int smoothBSplineMaxSteps;
    double smoothBSplineMinChange;
    unsigned int collapseCloseVerticesMaxSteps;
};
struct TrajectoryProcessingParamters {
    double step_size_jps;
    double stepSizeTcp;
    double dt;
};

struct PlannerGeneralParamters {
    std::size_t dimension;
    PathSimplifierParamters path_simplifier;
    TrajectoryProcessingParamters traj_processing;
    Bounds bounds;
    double coneThreshold; // rad
};

struct SamplerParamBase {
    SamplerParamBase(unsigned int sampleAttempts) : attempts(sampleAttempts) {}
    virtual ~SamplerParamBase() = default;

    unsigned int attempts;
};
struct PlannerParamBase {
    virtual ~PlannerParamBase() = default;
};
} // namespace gsmpl
