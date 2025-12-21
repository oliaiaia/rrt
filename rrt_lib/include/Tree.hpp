#pragma once
#include <memory>
#include <vector>
#include <functional>
#include <limits>
#include <queue>

#include "Environment.hpp"

struct Node
{
    State value;
    std::shared_ptr<Node> parentPtr;
    std::vector<std::shared_ptr<Node>> children;

    Node() = default;
    Node(State state) : value(state) {}
};

class Tree
{
private:
    std::shared_ptr<Node> root;
    std::vector<double>weights = {1.0, 1.0, 1.0, 1.0};

public:

    Tree() = default;
    Tree(State rootState)
    {
        root = std::make_shared<Node>(rootState);

    }

    void setWeight1(double w) { weights[0] = w; }
    void setWeight2(double w) { weights[1] = w; }
    void setWeight3(double w) { weights[2] = w; }
    void setWeight4(double w) { weights[3] = w; }

    void setWeights(double w1, double w2, double w3, double w4) {
        weights[0] = w1;
        weights[1] = w2;
        weights[2] = w3;
        weights[3] = w4;
    }

    double weightedDistance(const State &q1, const State &q2) const;

    std::shared_ptr<Node> findNearest(const State &target);

    std::shared_ptr<Node> addNode(std::shared_ptr<Node> parent, const State &state);

    std::shared_ptr<Node> getRoot() const
    {
        return root;
    }

    std::shared_ptr<Node> findNearestBFS(const State &target);
};