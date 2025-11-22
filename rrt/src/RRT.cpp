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

bool RRT::launch(State stateSTART, State stateGOAL)
{
    Tree *newTree = new Tree(stateSTART);
    tree = *newTree;
    delete newTree;

    int iter = 0;

    while (true)
    {
        iter++;

        State q_rand = sample();
        std::shared_ptr<Node> q_near = tree.findNearest(q_rand);
        State q_new = steer(q_rand, q_near->value);

        std::shared_ptr<Node> q_new_node = nullptr;

        if (checkStatesBtw(q_new, q_near->value))
        {
            q_new_node = tree.addNode(q_near, q_new);
        }

        if (q_new_node && checkGoalTreashold(q_new, stateGOAL))
        {
            nodeGOAL = tree.addNode(q_new_node, stateGOAL);
            return true;
        }

        if (iter > 20000)
            return false;
    }
}

State RRT::steer(const State &q_rand, const State &q_near)
{
    std::vector<double> angles(4);

    double localStep = step;

    bool needReduce = false;

    for (int i = 0; i < q_near.joints; i++)
    {
        double delta = normalizeAngle(q_rand.angleVector(i) - q_near.angleVector(i));
        double a = q_near.angleVector(i) + delta * localStep;

        if (std::fabs(a) > treashold)
        {
            needReduce = true;
        }
    }

    if (needReduce)
    {
        while (true)
        {
            localStep /= 2.0;

            bool ok = true;
            for (int i = 0; i < q_near.joints; i++)
            {
                double delta = normalizeAngle(q_rand.angleVector(i) - q_near.angleVector(i));
                double a = q_near.angleVector(i) + delta * localStep;

                if (std::fabs(a) > treashold)
                    ok = false;
            }

            if (ok)
                break;
            if (localStep < 1e-6)
                break;
        }
    }

    for (int i = 0; i < q_near.joints; i++)
    {
        double delta = normalizeAngle(q_rand.angleVector(i) - q_near.angleVector(i));
        angles[i] = q_near.angleVector(i) + delta * localStep;
    }

    return State(angles);
}

void RRT::generateStatesBtw(const State &q_new, const State &q_near, std::vector<State> &out)
{
    out.reserve(stepBtw + 1);

    for (size_t t = 0; t <= stepBtw; t++)
    {
        std::vector<double> angles(4);
        double alpha = double(t) / double(stepBtw);

        for (int i = 0; i < q_new.joints; i++)
        {
            double delta = normalizeAngle(q_new.angleVector(i) - q_near.angleVector(i));
            angles[i] = q_near.angleVector(i) + delta * alpha;
        }

        out.emplace_back(angles);
    }
}

bool RRT::checkStatesBtw(const State &q_new, const State &q_near)
{
    std::vector<State> vec;
    generateStatesBtw(q_new, q_near, vec);

    for (const auto &s : vec)
    {
        if (env.checkCollision(s, 0.05))
            return false;
    }
    return true;
}

bool RRT::checkGoalTreashold(const State &q_new, const State &q_goal)
{
    return tree.distanceFunction(q_new, q_goal) <= treashold;
}
