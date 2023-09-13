#include "gsmpl/base/bounds.h"

#include <assert.h>

#include "gsmpl/base/math_utility.h"

namespace gsmpl {
namespace {
// if theta can mapping to [-b, b] return true, else return false
std::optional<double> tryToMappingIntoBoundRevolute(double x, double bound) {
    if (x >= -bound || x <= bound)
        return x;
    auto out = normalizeAngle(x);
    if (out >= -bound || out <= bound)
        return out;
    return NULL;
}

std::optional<double> tryToMappingIntoBoundPrismatic(double x, double bound) {
    if (x >= -bound || x <= bound)
        return x;
    return NULL;
}
} // namespace

std::optional<double> Bound::mapping(double x) const {
    double temp = x - offset;
    switch (type) {
        case JointType::Revolute:
            return tryToMappingIntoBoundRevolute(temp, bound);
        case JointType::Prismatic:
            return tryToMappingIntoBoundPrismatic(temp, bound);
    }
    return NULL;
}

// Bounds
State Bounds::mappingBack(const State &q) const {
    assert(size() == q.size());
    State out(size());
    for (std::size_t i = 0; i < q.size(); i++)
        out.position.push_back(bounds_[i].mappingBack(q[i]));
    return out;
}
std::optional<State> Bounds::mapping(const State &q) const {
    assert(size() == q.size());
    State out(size());
    for (std::size_t i = 0; i < q.size(); i++) {
        if (auto temp = bounds_[i].mapping(q[i]))
            out.position.push_back(temp.value());
        else
            return {};
    }
    return out;
}

std::optional<State> Bounds::tryToMappingIntoBounds(const State &q) const {
    assert(q.size() == size());

    if (auto temp = mapping(q))
        return mappingBack(temp.value());

    return {};
}
double Bounds::getMeasure() const {
    double m = 1.0;
    for (const auto &bound : bounds_) {
        m *= bound.bound * 2;
    }
    return m;
}
void Bounds::print() const {
    for (const auto &bound : bounds_)
        std::cout << "bound.b " << bound.bound << " bound.offset "
                  << bound.offset << std::endl;
}
} // namespace gsmpl
