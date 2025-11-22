#include "Tree.hpp"

double Tree::manhattanDistance(const State &q1, const State &q2) const
{
    double distance = 0.0;
    for (int i = 1; i <= 4; ++i)
    {
        for (int j = 0; j <= 1; ++j)
        {
            distance += abs(q1.jointsMatrix(i, j) - q2.jointsMatrix(i, j));
        }
    }
    return distance;
}

std::shared_ptr<Node> Tree::findNearestRecursive(const State &target)
{
    return findNearestRecursiveHelper(root, target, root, std::numeric_limits<double>::max());
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

std::shared_ptr<Node> Tree::findNearestRecursiveHelper(const std::shared_ptr<Node> &current,
                                                       const State &target,
                                                       std::shared_ptr<Node> bestNode,
                                                       double bestDistance)
{
    if (!current)
        return bestNode;

    double currentDistance = distanceFunction(current->value, target);
    if (currentDistance < bestDistance)
    {
        bestDistance = currentDistance;
        bestNode = current;
    }

    for (const auto &child : current->children)
    {
        bestNode = findNearestRecursiveHelper(child, target, bestNode, bestDistance);
        bestDistance = distanceFunction(bestNode->value, target);
    }

    return bestNode;
}

// BFS
std::shared_ptr<Node> Tree::findNearestBFS(const State &target)
{
    if (!root)
        return nullptr;

    std::shared_ptr<Node> bestNode = root;
    double bestDistance = distanceFunction(root->value, target);

    std::queue<std::shared_ptr<Node>> queue;
    queue.push(root);

    while (!queue.empty())
    {
        auto current = queue.front();
        queue.pop();

        double currentDistance = distanceFunction(current->value, target);
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
