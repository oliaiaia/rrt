#include "RRT.hpp"

static double normalizeAngle(double a)
{
    a = fmod(a + 180.0, 360.0);
    if (a < 0)
        a += 360.0;
    return a - 180.0;
}

State RRT::sample()
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(-180.0, 180.0);

    std::vector<double> angles{dist(gen), dist(gen), dist(gen), dist(gen)};
    return State(angles);
}

State RRT::smartSample()
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(-180.0, 180.0);
    std::uniform_real_distribution<double> prob_dist(0.0, 1.0);

    if (prob_dist(gen) < goalBiasProbability && !goalState.angleVector.isZero())
    {
        return goalState;
    }

    std::vector<double> angles{dist(gen), dist(gen), dist(gen), dist(gen)};
    return State(angles);
}

bool RRT::launch(State stateSTART, State stateGOAL)
{
    Tree *newTree = new Tree(stateSTART);
    tree = *newTree;
    delete newTree;

    goalState = stateGOAL;
    startState = stateSTART;

    int iter = 0;

    while (true)
    {
        iter++;

        State q_rand = smartSample();

        std::shared_ptr<Node> q_near = tree.findNearest(q_rand);
        State q_new = steer(q_rand, q_near->value);

        std::shared_ptr<Node> q_new_node = nullptr;

        // check collision along the path
        if (checkStatesBtw(q_new, q_near->value))
        {
            q_new_node = tree.addNode(q_near, q_new);
            // std::cout << "q_near" << "\n"
                    //   << q_near->value.angleVector << "\n";
            // std::cout << "q_new_node" << "\n"
                    //   << q_new_node->value.angleVector << "\n";
            // std::cout << "add new node to the tree" << "\n";
        }


        if (q_new_node && checkGoalTreashold(q_new, stateGOAL))
        {
            nodeGOAL = tree.addNode(q_new_node, stateGOAL);
            std::cout << "=========================================================" << "\n";
            std::cout << "Find the path!" << "\n";
            std::cout << "=========================================================" << "\n";
            std::cout << "Iteration: " << iter << "\n";
            return true;
        }

        if (iter > 200000)
            return false;
        // std::cout << "ITER: " << iter << "\n";
    }
}

State RRT::steer(const State &q_rand, const State &q_near)
{
    std::vector<double> angles(4);
    bool needReduce = false;
    double maxDiff = treasholdOneStepDegree;

    for (int i = 0; i < q_near.joints; i++)
    {
        double delta = normalizeAngle(q_rand.angleVector(i) - q_near.angleVector(i));

        if (delta >= maxDiff)
        {
            maxDiff = delta;
        }

        if (std::fabs(delta) > treasholdOneStepDegree)
        {
            needReduce = true;
        }
    }

    if (needReduce)
    {
        double fractionOfTheDifference = treasholdOneStepDegree / maxDiff;

        for (int i = 0; i < q_near.joints; i++)
        {
            double delta = normalizeAngle(q_rand.angleVector(i) - q_near.angleVector(i));
            angles[i] = normalizeAngle(q_near.angleVector(i) + fractionOfTheDifference * delta);
        }
    }
    return State(angles);
}

void RRT::generateStatesBtw(const State &q_new, const State &q_near, std::vector<State> &out)
{
    out.reserve(onHowManyStepsDevideAngleDistanceBtw2States + 1);

    for (size_t t = 0; t <= onHowManyStepsDevideAngleDistanceBtw2States; t++)
    {
        std::vector<double> angles(4);
        double alpha = double(t) / double(onHowManyStepsDevideAngleDistanceBtw2States);

        for (int i = 0; i < q_new.joints; i++)
        {
            double delta = normalizeAngle(q_new.angleVector(i) - q_near.angleVector(i));
            angles[i] = normalizeAngle(q_near.angleVector(i) + delta * alpha);
        }

        out.emplace_back(angles);
    }
}

std::vector<State> RRT::getStatesBtw(const State &q_new, const State &q_near)
{
    std::vector<State> vec;
    generateStatesBtw(q_new, q_near, vec);
    return vec;
}

bool RRT::checkStatesBtw(const State &q_new, const State &q_near)
{
    std::vector<State> vec;
    generateStatesBtw(q_new, q_near, vec);

    for (const auto &s : vec)
    {
        if (env.checkCollision(s))
            return false;
    }
    return true;
}

bool RRT::checkGoalTreashold(const State &q_new, const State &q_goal)
{
    return tree.weightedDistance(q_new, q_goal) <= treasholdToTheGoalDegree;
}

std::vector<State> RRT::getPlan()
{
    std::vector<State> plan;
    if (!nodeGOAL)
        return plan;

    std::shared_ptr<Node> cur = nodeGOAL;
    while (cur)
    {
        plan.push_back(cur->value);
        cur = cur->parentPtr;
    }

    std::reverse(plan.begin(), plan.end());
    std::cout << "Plan length: " << plan.size() << "\n";

    return plan;
}
