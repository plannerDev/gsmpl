#pragma once

#include "../utility/class_forward.h"
#include "../base/bounds.h"

namespace gsmpl {
GSMPL_STRUCT_FORWARD(PlannerParamBase)
GSMPL_STRUCT_FORWARD(SamplerParamBase)

struct PathSimplifierParamters
{
    double stepSizeJps;
    double stepSizeTcp;
    unsigned int maxEmptySteps;
    unsigned int reduceVerticesMaxSteps;
    double reduceVerticesRangeRatio;
    unsigned int smoothBSplineMaxSteps;
    double smoothBSplineMinChange;
    unsigned int collapseCloseVerticesMaxSteps;
};

struct PlannerGeneralParamters
{
    std::size_t dimension;
    PathSimplifierParamters pathSimplifier;
    Bounds bounds;
    double coneThreshold; // rad
};

struct SamplerParamBase
{
    SamplerParamBase(unsigned int sampleAttempts) : attempts(sampleAttempts) {}
    virtual ~SamplerParamBase() = default;

    unsigned int attempts;
};
struct PlannerParamBase
{
    virtual ~PlannerParamBase() = default;
};
} // namespace gsmpl
