#pragma once

#include <vector>
#include <memory>
#include <assert.h>
#include <iostream>
#include <optional>
#include "state.h"

namespace gsmpl {
struct Bound
{
    Bound() = default;
    Bound(double l, double h, JointType t) : bound(calcBound(l, h)), offset(calcOffset(l, h)), type(t) {}

    // [-b, b] ---> [l, h]
    double mappingBack(double x) const { return x + offset; }
    // [l, h] ---> [-b, b]
    std::optional<double> mapping(double x) const;

    double bound;
    double offset;
    JointType type;

private:
    static double calcBound(double l, double h)
    {
        assert(l < h);
        return (h - l) * 0.5;
    }
    static double calcOffset(double l, double h)
    {
        assert(l < h);
        return (l + h) * 0.5;
    }
};

class Bounds
{
public:
    Bounds(const std::vector<Bound>& bounds) : bounds_(bounds) {}
    Bounds(){}

    void push_back(const Bound& bound) { bounds_.push_back(bound); }
    const Bound& operator[](std::size_t i) const { return bounds_[i]; }
    std::size_t size() const { return bounds_.size(); }
    std::optional<State> tryToMappingIntoBounds(const State& q) const;
    // [-b, b] ---> [l, h]
    State mappingBack(const State& q) const;
    // [l, h] ---> [-b, b]
    std::optional<State> mapping(const State& q) const;
    double getMeasure() const;

    void print() const;

private:
    std::vector<Bound> bounds_;
};
} // namespace gsmpl
