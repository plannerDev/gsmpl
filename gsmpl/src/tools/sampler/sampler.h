#pragma once

#include <optional>
#include "../../base/math_utility.h"
#include "../../base/state.h"
#include "../../base/bounds.h"
#include "../../utility/class_forward.h"
#include "../../planner_data/planner_param.h"

namespace gsmpl {
GSMPL_CLASS_FORWARD(SamplerBase)
GSMPL_CLASS_FORWARD(SampleUniform)
GSMPL_CLASS_FORWARD(SampleWithBias)
GSMPL_STRUCT_FORWARD(SampleWithBiasParam)

class SamplerBase
{
public:
    SamplerBase(const Bounds& b, unsigned int sampleAttempts) : attempts(sampleAttempts), bounds(b)
    {
    }
    virtual ~SamplerBase() = default;

    virtual State sample() = 0;
    virtual State sample(const State& goal) { return {}; }
    virtual State sampleNear(const State& qNear, double distance) = 0;

    const unsigned int attempts;
    const Bounds bounds;

protected:
    RNG rng_;
};

class SampleUniform : public SamplerBase
{
public:
    SampleUniform(const Bounds& bounds, unsigned int sampleAttempts = 1)
        : SamplerBase(bounds, sampleAttempts)
    {
    }

    State sample() override;
    State sampleNear(const State& qNear, double distance) override;
};

struct SampleWithBiasParam : public SamplerParamBase
{
    SampleWithBiasParam(unsigned int sampleAttempts, double b)
        : SamplerParamBase(sampleAttempts), bias(b)
    {
    }

    double bias;
};

class SampleWithBias : public SampleUniform
{
public:
    SampleWithBias(const Bounds& bounds, const State& center, const SampleWithBiasParam& param)
        : SampleUniform(bounds, param.attempts), bias(param.bias), qCenter_(center)
    {
    }

    State sample() override;
    State sample(const State& goal) override;
    State sampleNear(const State& qNear, double distance) override;

    const double bias;

protected:
    State qCenter_;
};
} // namespace gsmpl
