#include <math.h>
#include <assert.h>
#include "sampler.h"

namespace gsmpl {
State SampleUniform::sample()
{
    State q;
    for (size_t i = 0; i < bounds.size(); i++) {
        double d = rng_.uniformReal(-bounds[i].bound, bounds[i].bound);
        q.push_back(d);
    }
    return bounds.mappingBack(q);
}

State SampleUniform::sampleNear(const State& qNear, double distance)
{
    assert(qNear.size() == bounds.size());
    State q;
    auto nearMapping = bounds.mapping(qNear);

    assert(nearMapping);

    for (size_t i = 0; i < bounds.size(); i++) {
        double l = std::max(nearMapping.value()[i] - distance, -bounds[i].bound);
        double h = std::min(nearMapping.value()[i] + distance, bounds[i].bound);
        double d = rng_.uniformReal(l, h);
        q.push_back(d);
    }
    return bounds.mappingBack(q);
}

State SampleWithBias::sample()
{
    if (rng_.uniformReal01() < bias)
        return qCenter_;
    return SampleUniform::sample();
}

State SampleWithBias::sample(const State& goal)
{
    if (rng_.uniformReal01() < bias)
        return goal;
    return SampleUniform::sample();
}

State SampleWithBias::sampleNear(const State& qNear, double distance)
{
    if (rng_.uniformReal01() < bias)
        return qCenter_;
    return SampleUniform::sampleNear(qNear, distance);
}
} // namespace gsmpl
