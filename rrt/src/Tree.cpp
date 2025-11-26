#include "Tree.hpp"
#include <iostream>

double Tree::weightedDistance(const State &q1, const State &q2) const
{
    double distance = 0.0;

    for (int i = 0; i < 4; ++i)
    {
        double diff = std::abs(q1.angleVector(i) - q2.angleVector(i));
        if (diff > 180.0) {
            diff = 360.0 - diff;
        }
        
        distance += diff * weights[i];
    }
    return distance;
}


std::shared_ptr<Node> Tree::findNearest(const State &target)
{
    return findNearestBFS(target);
}

std::shared_ptr<Node> Tree::addNode(std::shared_ptr<Node> parent, const State &state)
{
    auto newNode = std::make_shared<Node>(state);
    newNode->parentPtr = parent;
    if (parent)
    {
        parent->children.push_back(newNode);
    }
    return newNode;
}

std::shared_ptr<Node> Tree::findNearestBFS(const State &target)
{
    if (!root)
        return nullptr;

    std::shared_ptr<Node> bestNode = root;
    double bestDistance = weightedDistance(root->value, target);

    std::queue<std::shared_ptr<Node>> queue;
    queue.push(root);

    while (!queue.empty())
    {
        auto current = queue.front();
        queue.pop();

        double currentDistance = weightedDistance(current->value, target);
        if (currentDistance < bestDistance)
        {
            bestDistance = currentDistance;
            bestNode = current;
        }

        for (const auto &child : current->children)
        {
            queue.push(child);
        }
    }

    return bestNode;
}
