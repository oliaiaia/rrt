#include <RRT.hpp>

int main()
{
    std::vector<std::vector<double>> obstaclesDouble = {{2.5, 1.0, 0.5}, {1.5, -0.8, 0.5}, {-2.0, -0.7, 0.5}, {-2.0, 1.5, 0.5}, {0.0, 1.2, 0.5}, {-0.3, -2.0, 0.7}};
    std::vector<Obstacle> obstaclesVector;
    for (const auto &obstacle : obstaclesDouble)
    {
        obstaclesVector.push_back(Obstacle(obstacle));
    }
    Environment env = Environment(obstaclesVector);
    State startState = State({0.0, 0.0, 0.0, 0.0});
    State goalState = State({-180.0, -60.0, 72.0, -60.0});


    // RRT rrt(env);
    // if(rrt.launch(startState, goalState)) {
    //     rrt.getPlan().size();
    // }
    return 0;
}