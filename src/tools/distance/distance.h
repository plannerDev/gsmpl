#pragma once

#include "../../utility/class_forward.h"
#include "../../base/bounds.h"
#include "../../base/state.h"
#include "../../utility/export.h"

namespace gsmpl {
GSMPL_CLASS_FORWARD(DistanceBase)
GSMPL_CLASS_FORWARD(DistanceLInfinity)
GSMPL_CLASS_FORWARD(DistanceL2)

class EXPORT DistanceBase {
public:
    DistanceBase(const Bounds& b) : dimension(b.size()), bounds(b) {}
    virtual ~DistanceBase() = default;

    virtual bool isEquivalent(const State& q1, const State& q2) const = 0;
    virtual double distance(const State& q1, const State& q2) const = 0;

    const std::size_t dimension;
    const Bounds bounds;

protected:
    double rWeight_{1};
    double pWeight_{1};
};

class EXPORT DistanceLInfinity : public DistanceBase {
public:
    DistanceLInfinity(const Bounds& b) : DistanceBase(b) {}

    double distance(const State& q1, const State& q2) const override;
    bool isEquivalent(const State& q1, const State& q2) const override;
};

class EXPORT DistanceL2 : public DistanceLInfinity {
public:
    DistanceL2(const Bounds& b) : DistanceLInfinity(b) {}

    double distance(const State& q1, const State& q2) const override;
};
} // namespace gsmpl
