#pragma once

#include "Tree.hpp"
#include <random>
#include <cmath>
#include <algorithm>
#include <iostream>

class RRT
{
public:

    

    void setFractionOfTheDifferenceInAngles(double fractionOfTheDifferenceInAngles) {
        this->fractionOfTheDifferenceInAngles = fractionOfTheDifferenceInAngles;
    }
    void setOnHowManyStepsDevideAngleDistanceBtw2States(double onHowManyStepsDevideAngleDistanceBtw2States) {
        this->onHowManyStepsDevideAngleDistanceBtw2States = onHowManyStepsDevideAngleDistanceBtw2States;
    }
    void setTreasholdToTheGoal(double treasholdToTheGoalDegree) {
        this->treasholdToTheGoalDegree = treasholdToTheGoalDegree;
    }
    void setTreasholdOneStepDegree(double treasholdOneStepDegree) {
        this->treasholdOneStepDegree = treasholdOneStepDegree;
    }
    void setWeights(double w1, double w2, double w3, double w4) {
        tree.setWeights(w1, w2, w3, w4);
    }
    void setGoalBiasProbability(double probability) {
        goalBiasProbability = probability;
    }
    RRT() = default;

    RRT(Environment env) : env(env) {
        std::cout << "RRT has been created" << "\n";
    }
    RRT(Environment env, double fractionOfTheDifferenceInAngles, double onHowManyStepsDevideAngleDistanceBtw2States, double treasholdToTheGoalDegree) : env(env), fractionOfTheDifferenceInAngles(fractionOfTheDifferenceInAngles), onHowManyStepsDevideAngleDistanceBtw2States(onHowManyStepsDevideAngleDistanceBtw2States), treasholdToTheGoalDegree(treasholdToTheGoalDegree){
        std::cout << "RRT has been created" << "\n";
    }
    bool launch(State stateSTART, State stateGOAL);
    std::vector<State> getPlan();
    State sample();
    std::vector<State> getStatesBtw(const State &stateNEW, const State &stateNEAR);

private:
    Tree tree;
    Environment env;
    std::shared_ptr<Node> nodeGOAL;

    double fractionOfTheDifferenceInAngles = 0.5; //devide by it in steer
    double onHowManyStepsDevideAngleDistanceBtw2States = 10; //steps between two states to check obstacles between them
    double treasholdToTheGoalDegree = 100; //dist to goal 
    double treasholdOneStepDegree = 10; 
    double goalBiasProbability = 0.1;

    State steer(const State &stateRANDOM, const State &stateNEAR);
    void generateStatesBtw(const State &stateNEW, const State &stateNEAR, std::vector<State> &statesVector);
    bool checkGoalTreashold(const State &stateNEW, const State &stateGOAL);
    bool checkStatesBtw(const State &stateNEW, const State &stateNEAR);

};
