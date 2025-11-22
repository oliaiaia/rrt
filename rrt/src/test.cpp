#include <iostream>
#include <vector>
#include "RRT.hpp"

#define OK(msg) std::cout << "[OK]   " << msg << "\n"
#define FAIL(msg) std::cout << "[FAIL] " << msg << "\n"

bool approx(double a, double b, double eps = 1e-6){
    return fabs(a-b) < eps;
}

void test_forward_kinematics(){
    State s({0,0,0,0});
    bool pass = true;

    pass &= approx(s.jointsMatrix(1,0), 1.0);
    pass &= approx(s.jointsMatrix(2,0), 2.0);
    pass &= approx(s.jointsMatrix(3,0), 3.0);
    pass &= approx(s.jointsMatrix(4,0), 4.0);

    if(pass) OK("test_forward_kinematics");
    else FAIL("test_forward_kinematics");
}

void test_collision_end_effector(){
    Environment env;
    env.obstacles.push_back({4.0, 0.0, 0.2});

    State s({0,0,0,0});
    if(env.checkCollision(s, 0.05))
        OK("test_collision_end_effector");
    else
        FAIL("test_collision_end_effector");
}

void test_collision_segment(){
    Environment env;
    env.obstacles.push_back({1.0, 0.3, 0.2});

    State s({0,0,0,0});
    if(env.checkCollision(s, 0.05))
        OK("test_collision_segment");
    else
        FAIL("test_collision_segment");
}


int main(){
    test_forward_kinematics();
    test_collision_end_effector();
    test_collision_segment();
    return 0;
}
