#include <map>
#include "path_simplifier.h"

namespace gsmpl {
Path PathSimplifier::subdividePath(const Path& path) const {
    if (path.size() < 2)
        return path;
    Path newPath;
    newPath.push_back(path[0]);
    for (std::size_t i = 1; i < path.size(); i++) {
        State q = localPlanner_->interpolateState(path[i - 1], path[i], 0.5);
        newPath.push_back(q);
        newPath.push_back(path[i]);
    }
    return newPath;
}
Path PathSimplifier::reduceVertices(const Path& rawPath, double stepSizeJps,
                                    double stepSizeTcp, unsigned int maxSteps,
                                    unsigned int maxEmptySteps,
                                    double rangeRatio) {
    if (rawPath.size() < 3)
        return rawPath;
    if (maxSteps == 0)
        maxSteps = rawPath.size();

    Path newPath = rawPath;
    unsigned int emptySteps = 0;

    for (std::size_t i = 0; i < maxSteps && emptySteps < maxEmptySteps;
         i++, emptySteps++) {
        int count = newPath.size();
        int maxN = count - 1;
        int range = 1 + static_cast<int>(floor(
                            0.5 + static_cast<double>(count) * rangeRatio));

        int p1 = rng_.uniformInt(0, maxN);
        int p2 = rng_.uniformInt(std::max(p1 - range, 0),
                                 std::min(maxN, p1 + range));
        if (abs(p1 - p2) < 2) {
            if (p1 < maxN - 1)
                p2 = p1 + 2;
            else if (p1 > 1)
                p2 = p1 - 2;
            else
                continue;
        }
        if (p1 > p2)
            std::swap(p1, p2);
        if (localPlanner_->validInterpolatePath(newPath[p1], newPath[p2],
                                                stepSizeJps, stepSizeTcp)) {
            newPath.erase(newPath.begin() + p1 + 1, newPath.begin() + p2);
            LOGD("erase %d", p2 - p1);
            emptySteps = 0;
        }
    }
    return newPath;
}
Path PathSimplifier::collapseCloseVertices(const Path& rawPath,
                                           double stepSizeJps,
                                           double stepSizeTcp,
                                           unsigned int maxSteps,
                                           unsigned int maxEmptySteps) const {
    if (rawPath.size() < 3)
        return rawPath;
    if (maxSteps == 0)
        maxSteps = rawPath.size();
    Path newPath = rawPath;

    std::map<std::pair<const State*, const State*>, double> distanceMap;
    for (unsigned int i = 0; i < newPath.size(); i++)
        for (unsigned int j = i + 2; j < newPath.size(); j++)
            distanceMap[std::make_pair(&newPath[i], &newPath[j])] =
                distance_->distance(newPath[i], newPath[j]);

    maxEmptySteps = std::min(maxEmptySteps, (unsigned int)newPath.size());
    unsigned int emptySteps = 0;
    for (unsigned int s = 0; s < maxSteps && emptySteps < maxEmptySteps; s++) {
        double minDistance = std::numeric_limits<double>::max();
        int p1 = -1;
        int p2 = -2;
        for (unsigned int i = 0; i < newPath.size(); i++) {
            for (unsigned int j = i + 2; j < newPath.size(); j++) {
                double d =
                    distanceMap[std::make_pair(&newPath[i], &newPath[j])];
                if (d < minDistance) {
                    minDistance = d;
                    p1 = i;
                    p2 = j;
                }
            }

            if (p1 >= 0 && p2 >= 0) {
                if (localPlanner_->validInterpolatePath(
                        newPath[p1], newPath[p2], stepSizeJps, stepSizeTcp)) {
                    newPath.erase(newPath.begin() + p1 + 1,
                                  newPath.begin() + p2);
                    emptySteps = 0;
                } else {
                    distanceMap[std::make_pair(&newPath[p1], &newPath[p2])] =
                        std::numeric_limits<double>::max();
                }
            } else {
                break;
            }
        }
    }
    return newPath;
}

Path PathSimplifier::smoothBSpline(const Path& rawPath, unsigned int maxSteps,
                                   double minChange, double stepSizeJps,
                                   double stepSizeTcp) const {
    if (rawPath.size() < 3)
        return rawPath;
    Path newPath = rawPath;
    State temp1, temp2, qNew;
    for (unsigned int step = 0; step < maxSteps; step++) {
        newPath = subdividePath(newPath);
        unsigned int i = 2, length = newPath.size() - 1;
        bool modified = false;
        while (i < length) {
            auto temp1 = localPlanner_->validInterpolateState(newPath[i - 1],
                                                              newPath[i], 0.5);
            auto temp2 = localPlanner_->validInterpolateState(
                newPath[i], newPath[i + 1], 0.5);
            if (temp1 && temp2) {
                if (auto qNew = localPlanner_->validInterpolateState(
                        temp1.value(), temp2.value(), 0.5)) {
                    if (localPlanner_->validInterpolatePath(
                            newPath[i - 1], qNew.value(), stepSizeJps,
                            stepSizeTcp) &&
                        localPlanner_->validInterpolatePath(
                            qNew.value(), newPath[i + 1], stepSizeJps,
                            stepSizeTcp)) {
                        if (distance_->distance(newPath[i], qNew.value()) >
                            minChange) {
                            newPath[i] = qNew.value();
                            modified = true;
                        }
                    }
                }
            }
            i += 2;
        }
        if (!modified)
            break;
    }
    return newPath;
}

Trajectory PathSimplifier::generateTrajector(const Path& path,
                                             double stepSizeJps,
                                             double stepSizeTcp) const {
    assert(path.size() > 1);

    Trajectory trajectory;
    trajectory.waypoints = path;
    for (int i = 1; i < path.size(); i++) {
        Trajectory::Segment segment;
        segment = localPlanner_->interpolatePath(path[i - 1], path[i],
                                                 stepSizeJps, stepSizeTcp);
        trajectory.segments.push_back(segment);
    }
    return trajectory;
}
} // namespace gsmpl
