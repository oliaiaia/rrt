#include "RRT.hpp"

    double normalizeAngle(double angleDegree) {
        while(angleDegree > 180) {
            angleDegree -= 360;
        }
        while(angleDegree < -180) {
            angleDegree += 360;
        }
        return angleDegree;
    }
    State RRT::sample() {

        std::random_device rd;
        std::mt19937 gen(rd());
        double min = -180.0;
        double max = 180.0;
        std::uniform_real_distribution<double> dist(min, max);
        std::vector<double> angles {dist(gen), dist(gen), dist(gen), dist(gen)};

        return State(angles);
    
    }

    bool RRT::launch(State stateSTART, State stateGOAL){
        Tree *newTree = new Tree(stateSTART);
        tree = *newTree; 
        delete newTree;

        while(true) {

            State stateRANDOM = sample();
            std::shared_ptr<Node> nodeNEAR = tree.findNearest(stateRANDOM);
            State stateNEW = steer(stateRANDOM, nodeNEAR->value);
            
            std::shared_ptr<Node> nodeNEW;

            if(checkStatesBtw(stateNEW, nodeNEAR->value)) {
                nodeNEW = tree.addNode(nodeNEAR, stateNEW);
            }

            if(checkGoalTreashold(stateNEW, stateGOAL))
            {
                nodeGOAL = tree.addNode(nodeNEW, stateGOAL);
                return true;
            }
        }
    }


    State RRT::steer(State stateRANDOM, State stateNEAR){
            std::vector<double> angles(4);
            for (size_t i = 0; i < stateNEAR.joints; i++)
            {
                /* code */
                double angle = stateNEAR.angleVector(i) + normalizeAngle(stateRANDOM.angleVector(i) - stateNEAR.angleVector(i)) * step;
                bool changeStep = false;

                while(angle > treashold) {
                    step /= 2; 
                    angle = stateNEAR.angleVector(i) + normalizeAngle(stateRANDOM.angleVector(i) - stateNEAR.angleVector(i)) * step;
                    changeStep = true;
                }

                angles[i] = angle;
                if(changeStep && i != 0) i = -1; // restart for all joints
            }

            step = 0.5;
            return State(angles);
    }

    void RRT::generateStatesBtw(State stateNEW, State stateNEAR, std::vector<State> &statesVector){
        for(size_t t = 0; t <= stepBtw; t++) {
            std::vector<double> angles(4);
            for (size_t i = 0; i < stateNEW.joints; i++)
            {
                /* code */
                double angle = stateNEAR.angleVector(i) + t * normalizeAngle(stateNEW.angleVector(i) - stateNEAR.angleVector(i)) / stepBtw;
                angles[i] = angle;
            }
            State stateBTW = State(angles);
            statesVector.push_back(stateBTW);
        }
    }

    bool RRT::checkStatesBtw(State stateNEW, State stateNEAR){
        std::vector<State> statesVector;
        generateStatesBtw(stateNEW, stateNEAR, statesVector);
        for(const auto& state: statesVector) {
            if(env.checkCollision(state, 0.05)) return false;
        }
        return true;
    }

    bool RRT::checkGoalTreashold(State stateNEW, State stateGOAL){
        if(tree.distanceFunction(stateNEW, stateGOAL) <= treashold) return true;
        return false;
    }


