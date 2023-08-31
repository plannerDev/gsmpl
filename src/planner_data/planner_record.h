#pragma once

#include <map>
#include <functional>
#include "../base/tree.h"
#include "../utility/export.h"
#include "../utility/class_forward.h"
#include "planner_solution.h"

namespace gsmpl {
struct RGBA
{
    RGBA(float red, float green, float blue, float alpha) : r(red), g(green), b(blue), a(alpha) {}

    float r;
    float g;
    float b;
    float a;
};

const RGBA Green(0.0, 1.0, 0.0, 1.0);
const RGBA Red(1.0, 0.0, 0.0, 1.0);
const RGBA Blue(0.0, 0.0, 1.0, 1.0);
const RGBA Yellow(1.0, 1.0, 0.0, 1.0);
const RGBA Purple(1.0, 0.0, 1.0, 1.0);
const RGBA Gold(1.0, 0.843, 0, 1.0);
const RGBA Orange(1.0, 0.647, 0, 1.0);

GSMPL_CLASS_FORWARD(PlannerRecord)

class EXPORT PlannerRecord
{
public:
    using VisualFunction = std::function<void(const PlannerRecord&)>;
    using VisualPoseFunction = std::function<void(const State&, const RGBA&, const std::string&)>;

    State start;
    State goal;
    State v1;
    State v2;
    std::vector<std::pair<Tree, std::string>> trees;

    PlannerStatus status = PlannerStatus::Initialized;
    Path path;
    double cost = 0.0;
    bool approximate = false;

    Tree pathToTree(const Path& path) const;
    void addPath(const Path& p, const std::string& desc)
    {
        path = p;
        trees.emplace_back(pathToTree(path), desc);
    }
    void addTree(const Tree& tree, const std::string& desc) { trees.emplace_back(tree, desc); }
};

struct EXPORT PlannerTimes
{
    float planner;
    float rrtSolver;
    float pathProcessing;
    float biRRT;
    float informedRRTStar;
    int rrtStarrSamples;
    double biRRTPathLength;
    double rrtStarPathLength;
    int rrtStarPathSize;
    double finalPathLength;
    int pathSize;

    void print() const
    {
        std::cout << "planner: " << planner << std::endl;
        std::cout << "rrtSolver: " << rrtSolver << std::endl;
        std::cout << "pathProcessing: " << pathProcessing << std::endl;
        std::cout << "biRRT: " << biRRT << std::endl;
        std::cout << "informedRRTStar: " << informedRRTStar << std::endl;
        std::cout << "rrtStarrSamples: " << rrtStarrSamples << std::endl;
        std::cout << "biRRTPathLength: " << biRRTPathLength << std::endl;
        std::cout << "rrtStarPathLength: " << rrtStarPathLength << std::endl;
        std::cout << "rrtStarPathSize: " << rrtStarPathSize << std::endl;
        std::cout << "finalPathLength: " << finalPathLength << std::endl;
        std::cout << "pathSize: " << pathSize << std::endl;
    }
};
} // namespace gsmpl
