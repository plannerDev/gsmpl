#include "gsmpl/utility/log_utility.h"
#include "gsmpl/planner_data/planner_record.h"

namespace gsmpl {
Tree PlannerRecord::pathToTree(const Path& path) const {
    Tree tree;
    if (path.size() < 1)
        return tree;

    tree.setRoot(std::make_shared<Vertex>(nullptr, path[0]));
    VertexPtr vLast = tree.root();

    for (std::size_t i = 1; i < path.size(); i++) {
        auto vNext = std::make_shared<Vertex>(vLast.get(), path[i]);
        tree.addVertex(vNext);
        vLast = vNext;
    }
    return tree;
}
} // namespace gsmpl
