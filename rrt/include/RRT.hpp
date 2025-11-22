#pragma once

#include "Tree.hpp"
#include <random>
#include <cmath>

class RRT {
public:
    RRT(Environment env): env(env) {}
    bool launch(State stateSTART, State stateGOAL);

private:
    Tree tree;
    Environment env;
    double step = 0.5;
    double stepBtw = 10;
    double treashold = 10;
    std::shared_ptr<Node> nodeGOAL;

    State sample();
    State steer(const State& stateRANDOM, const State& stateNEAR);
    void generateStatesBtw(const State& stateNEW, const State& stateNEAR, std::vector<State> &statesVector);
    bool checkStatesBtw(const State& stateNEW, const State& stateNEAR);
    bool checkGoalTreashold(const State& stateNEW, const State& stateGOAL);
};
