#pragma once

#include <vector>
#include <memory>

#include "state.h"
#include "../utility/class_forward.h"

namespace gsmpl {
GSMPL_STRUCT_FORWARD(Vertex)

struct Vertex {
public:
    Vertex(Vertex* parent, const State& q, double sc = 0.0, double ec = 0.0)
        : state(q), parent_(parent), stateCost(sc), edgeCost(ec) {}

    Vertex(const Vertex&) = delete;
    Vertex& operator=(const Vertex&) = delete;

    void addChild(const VertexPtr& v) { children_.push_back(v); }
    bool removeChild(const VertexPtr& v) {
        for (int i = 0; i < children_.size(); i++) {
            VertexPtr child = children_[i];
            if (child.get() == v.get()) {
                children_.erase(children_.begin() + i);
                v->setParent(nullptr, 0.0, 0.0);
                return true;
            }
        }
        std::cout << "removeChild Error!!!!!" << std::endl;
        return false;
    }
    bool hasChild(const VertexPtr& v) const {
        for (auto it = children_.begin(); it != children_.end(); it++) {
            if (it->get() == v.get()) {
                return true;
            }
        }
        return false;
    }
    void updateChildrenCost() {
        for (auto& child : children_) {
            child->stateCost = stateCost + child->edgeCost;
            child->updateChildrenCost();
        }
    }
    Vertex* parent() { return parent_; }
    void setParent(const VertexPtr& p, double sCost, double eCost) {
        parent_ = p.get();
        stateCost = sCost;
        edgeCost = eCost;
    }
    const Vertex* cParent() const { return parent_; }
    const std::vector<VertexPtr>& children() const { return children_; }

    const State state;
    double stateCost;
    double edgeCost;

private:
    Vertex* parent_;
    std::vector<VertexPtr> children_;
};

struct Edge {
    Edge(const Vertex* o, const Vertex* i) : out(o), in(i) {}

    // out---->in
    const Vertex* out;
    const Vertex* in;
    double cost = 0.0;
};

class Tree {
public:
    void addVertex(const VertexPtr& v) { v->parent()->addChild(v); }

    bool removeVertex(const VertexPtr& v) {
        if (v->parent())
            return v->parent()->removeChild(v);
        std::cout << "v->parent() is null!!!!!" << std::endl;
        return false;
    }

    void setRoot(VertexPtr root) { root_ = std::move(root); }
    const VertexPtr& root() const { return root_; }
    void leaves(VertexPtr v, std::vector<VertexPtr>& out) const {
        if (v->children().size() > 0) {
            for (const auto& child : v->children())
                leaves(child, out);
        } else
            out.push_back(v);
    }

    bool check(VertexPtr v) const {
        for (auto& child : v->children())
            if (!child->parent()->hasChild(child))
                return false;
        return true;
    }

private:
    VertexPtr root_;
};
} // namespace gsmpl
