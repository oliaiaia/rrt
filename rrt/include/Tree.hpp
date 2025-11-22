#pragma once
#include <memory>
#include <vector>
#include <functional>
#include <limits>
#include <queue>

#include "Environment.hpp"

struct Node {
    State value;
    std::shared_ptr<Node> parentPtr;
    std::vector<std::shared_ptr<Node>> children;

    Node() = default;
    Node(State state) : value(state) {}
};

class Tree {
private:
    std::shared_ptr<Node> root;

public:
    std::function<double(const State&, const State&)> distanceFunction;

    Tree() = default;
    Tree(State rootState, std::function<double(const State&, const State&)> distanceFunction = nullptr) {
        root = std::make_shared<Node>(rootState);
        
        if (!distanceFunction) {
            this->distanceFunction = [this](const State& q1, const State& q2) {
                return manhattanDistance(q1, q2);
            };
        } else {
            this->distanceFunction = distanceFunction;
        }
    }

    double manhattanDistance(const State& q1, const State& q2) const;

    std::shared_ptr<Node> findNearestRecursive(const State& target);

    std::shared_ptr<Node> findNearest(const State& target);

    std::shared_ptr<Node> addNode(std::shared_ptr<Node> parent, const State& state);

    std::shared_ptr<Node> getRoot() const {
        return root;
    }
    std::shared_ptr<Node> findNearestRecursiveHelper(const std::shared_ptr<Node>& current,
                                                    const State& target,
                                                    std::shared_ptr<Node> bestNode,
                                                    double bestDistance);

    // BFS
    std::shared_ptr<Node> findNearestBFS(const State& target);
};