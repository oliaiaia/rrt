#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>

#include "RRT.hpp"
#include "Tree.hpp"
#include "Environment.hpp"

namespace py = pybind11;

std::vector<std::vector<double>> py_list_to_vector2d_double(const py::list &list)
{
    std::vector<std::vector<double>> result;
    for (size_t i = 0; i < list.size(); ++i)
    {
        py::list row = list[i].cast<py::list>();
        std::vector<double> row_vec;
        for (size_t j = 0; j < row.size(); ++j)
        {
            row_vec.push_back(row[j].cast<double>());
        }
        result.push_back(row_vec);
    }
    return result;
}

std::vector<double> py_list_to_vector1d_double(const py::list &list)
{
    std::vector<double> result;
    for (size_t i = 0; i < list.size(); ++i)
    {
        double element = list[i].cast<double>();
        result.push_back(element);
    }
    return result;
}

py::list state_to_angles_list(State state) {
    py::list angles;
    for(const auto &angle: state.angleVector) {
        angles.append(angle);
    }
    return angles;
}

class PyRRT
{
public:
    PyRRT(const py::list& obstacles)
    {
        setEnv(obstacles);
        rrt = RRT(env);
    }

    void setEnv(const py::list& obstacles)
    {
        std::vector<std::vector<double>> obstaclesDouble = py_list_to_vector2d_double(obstacles);
        std::vector<Obstacle> obstaclesVector;
        for (const auto &obstacle : obstaclesDouble)
        {
            obstaclesVector.push_back(Obstacle(obstacle));
        }
        env = Environment(obstaclesVector);
    }


     void setStepSize(double fractionOfTheDifferenceInAngles) {
        rrt.setFractionOfTheDifferenceInAngles(fractionOfTheDifferenceInAngles);
    }

    void setIntermediateSteps(double onHowManyStepsDevideAngleDistanceBtw2States) {
        rrt.setOnHowManyStepsDevideAngleDistanceBtw2States(onHowManyStepsDevideAngleDistanceBtw2States);
    }

    void setGoalThreshold(double treasholdToTheGoalDegree) {
        rrt.setTreasholdToTheGoal(treasholdToTheGoalDegree);
    }

    void setStepThreshold(double treasholdOneStepDegree) {
        rrt.setTreasholdOneStepDegree(treasholdOneStepDegree);
    }
    void setWeights(double w1, double w2, double w3, double w4) {
        rrt.setWeights(w1, w2, w3, w4);
    }

    void setStartGoal(const py::list& startStateAngles, const py::list& goalStateAngles)
    {
        std::vector<double> startAngles;
        for (size_t i = 0; i < startStateAngles.size(); ++i)
        {
            startAngles.push_back(startStateAngles[i].cast<double>());
        }
        stateSTART = State(startAngles);

        std::vector<double> goalAngles;
        for (size_t i = 0; i < goalStateAngles.size(); ++i)
        {
            goalAngles.push_back(goalStateAngles[i].cast<double>());
        }
        stateGOAL = State(goalAngles);
    }

    bool launch()
    {
        return rrt.launch(stateSTART, stateGOAL);
    }

    py::list sample()
    {
        State stateRANDOM = rrt.sample();
        return state_to_angles_list(stateRANDOM);
    }

    py::list getPlan()
    {
        py::list fullPlan;

        std::vector<State> plan = rrt.getPlan();
        for(const auto &step: plan) {
            py::list aglesFromTheStep = state_to_angles_list(step);
            fullPlan.append(aglesFromTheStep);
        }
        return fullPlan;
    }

    py::list generateStatesBtw2States(const py::list &state1PY, const py::list &state2PY)
    {
        py::list fullPlan;
        
        State state1S = py_list_to_vector1d_double(state1PY);
        State state2S = py_list_to_vector1d_double(state2PY);
        std::vector<State> stateBTW = rrt.getStatesBtw(state1S, state2S);

        for(const auto &step: stateBTW) {
            py::list aglesFromTheStep = state_to_angles_list(step);
            fullPlan.append(aglesFromTheStep);
        }
        return fullPlan;
    }

    bool checkCollisioin(const py::list& stateAngle, double treashold) {
        State s = State(py_list_to_vector1d_double(stateAngle));
        return env.checkCollision(s, treashold);
    }

private:
    Environment env;
    RRT rrt;
    State stateSTART;
    State stateGOAL;
};


// -------------------------------------------------------------------------------------------------------------------
PYBIND11_MODULE(rtt_planning_lib, m)
{
    m.doc() = "RRT path planning algorithm for robotic manipulator";

    py::class_<PyRRT>(m, "RRT")
        .def(py::init<const py::list &>(),
             py::arg("obstacles"),
             "Initialize RRT planner with obstacles\n"
             "Args:\n"
             "    obstacles: list of obstacles (format depends on your Obstacle class)")

        .def("set_env", &PyRRT::setEnv,
             py::arg("obstacles"),
             "Set environment obstacles\n"
             "Args:\n"
             "    obstacles: list of obstacles")

        .def("set_step_size", &PyRRT::setStepSize,
             py::arg("step_size"),
             "Set step size for RRT expansion\n"
             "Args:\n"
             "    step_size: fraction of angle difference for steering (0.0 - 1.0)\n"
             "               smaller values = smaller steps, more precision")
        
        .def("set_intermediate_steps", &PyRRT::setIntermediateSteps,
             py::arg("num_steps"),
             "Set number of intermediate steps for collision checking\n"
             "Args:\n"
             "    num_steps: number of steps between states for collision checking\n"
             "               higher values = more collision checks, safer but slower")
        
        .def("set_goal_threshold", &PyRRT::setGoalThreshold,
             py::arg("threshold"),
             "Set goal acceptance threshold\n"
             "Args:\n"
             "    threshold: distance threshold for considering goal reached (degrees)\n"
             "               smaller values = more precise goal reaching")
        
        .def("set_step_threshold", &PyRRT::setStepThreshold,
             py::arg("threshold"),
             "Set step validation threshold\n"
             "Args:\n"
             "    threshold: maximum allowed angle change per step (degrees)\n"
             "               ensures smooth motion between states")
        .def("set_weights_in_distant_func", &PyRRT::setWeights,
             py::arg("w1"),
             py::arg("w2"),
             py::arg("w3"),
             py::arg("w4"),
             "Set weights to dist func\n")

        .def("set_start_goal", &PyRRT::setStartGoal,
             py::arg("start_angles"),
             py::arg("goal_angles"),
             "Set start and goal configurations\n"
             "Args:\n"
             "    start_angles: list of start joint angles\n"
             "    goal_angles: list of goal joint angles")

        .def("launch", &PyRRT::launch,
             "Run RRT planning algorithm"
            "Returns:\n"
             "    bool: is path availible")

        .def("sample", &PyRRT::sample,
             "Sample a random state\n"
             "Returns:\n"
             "    list: random state as list of angles")

        .def("get_plan", &PyRRT::getPlan,
             "Get the planned path\n"
             "Returns:\n"
             "    list: list of waypoints, each waypoint is list of angles")

        .def("generate_states_btw", &PyRRT::generateStatesBtw2States,
             py::arg("state1"),
             py::arg("state2"),
             "Generate intermediate states between two states\n"
             "Args:\n"
             "    state1: first state as list of angles\n"
             "    state2: second state as list of angles\n"
             "Returns:\n"
             "    list: list of intermediate states")

        .def("check_collision", &PyRRT::checkCollisioin,
             py::arg("state_angles"),
             py::arg("threshold"),
             "Check if a configuration is in collision\n"
             "Args:\n"
             "    state_angles: list of joint angles\n"
             "    threshold: collision threshold\n"
             "Returns:\n"
             "    bool: True if collision detected, False otherwise")

        .def("__repr__", [](const PyRRT &)
             { return "<PyRRT planner>"; });

    m.def("state_to_angles_list", &state_to_angles_list,
          py::arg("state"),
          "Convert State object to list of angles (for internal use)");

    m.def("py_list_to_vector2d", &py_list_to_vector2d_double,
          py::arg("list"),
          "Convert 2D Python list to vector of vectors (for internal use)");

    m.def("py_list_to_vector1d", &py_list_to_vector1d_double,
          py::arg("list"),
          "Convert Python list to vector (for internal use)");
}