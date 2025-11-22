#pragma once

#include "Tree.hpp"
#include <random>

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
    State steer(State stateRANDOM, State stateNEAR);
    void generateStatesBtw(State stateNEW, State stateNEAR, std::vector<State> &statesVector);
    bool checkStatesBtw(State stateNEW, State stateNEAR);
    bool checkGoalTreashold(State stateNEW, State stateGOAL);

};