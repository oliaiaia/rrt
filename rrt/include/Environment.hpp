#pragma once

#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <limits>

struct Obstacle
{
    Obstacle() = default;
    Obstacle(double x_, double y_, double R_) : x(x_), y(y_), R(R_) {}
    Obstacle(std::vector<double> obs) : x(obs[0]), y(obs[1]), R(obs[2]) {}
    double x = 0.0;
    double y = 0.0;
    double R = 0.0;
};

struct State
{
    Eigen::Vector4d angleVector; // degrees
    Eigen::Matrix<double, 5, 2> jointsMatrix;
    int joints = 4;

    State() = default;

    State(const std::vector<double> &angles)
    {
        joints = 4;
        angleVector.setZero();
        jointsMatrix.setZero();
        fillAngles(angles);
        jointsMatrix = calculateJointsByAngles(angles);
    }

    void fillAngles(const std::vector<double> &angles)
    {
        for (int t = 0; t < (int)angles.size() && t < 4; ++t)
            angleVector(t) = angles[t];
    }

    static Eigen::Matrix3d formSE2(double x, double y, double angleDeg)
    {
        double angleRad = angleDeg * M_PI / 180.0;

        Eigen::Matrix3d T;
        T << std::cos(angleRad), -std::sin(angleRad), x,
            std::sin(angleRad), std::cos(angleRad), y,
            0.0, 0.0, 1.0;
        return T;
    }

    static Eigen::Matrix<double, 5, 2> calculateJointsByAngles(
        const std::vector<double> &angles,
        double jointLength = 1.0)
    {
        Eigen::Matrix<double, 5, 2> seg;
        seg.setZero();

        // initial pos (0, 0)
        seg(0, 0) = 0.0;
        seg(0, 1) = 0.0;

        Eigen::Matrix3d T1 = formSE2(0, 0, angles[0]);
        Eigen::Matrix3d T2 = formSE2(jointLength, 0, angles[1]);
        Eigen::Matrix3d T3 = formSE2(jointLength, 0, angles[2]);
        Eigen::Matrix3d T4 = formSE2(jointLength, 0, angles[3]);
        Eigen::Matrix3d T5 = formSE2(jointLength, 0, 0);

        Eigen::Vector3d p0(0, 0, 1);

        Eigen::Vector3d p = T1 * T2 * p0;
        seg(1, 0) = p(0);
        seg(1, 1) = p(1);

        p = T1 * T2 * T3 * p0;
        seg(2, 0) = p(0);
        seg(2, 1) = p(1);

        p = T1 * T2 * T3 * T4 * p0;
        seg(3, 0) = p(0);
        seg(3, 1) = p(1);

        p = T1 * T2 * T3 * T4 * T5 * p0;
        seg(4, 0) = p(0);
        seg(4, 1) = p(1);

        return seg;
    }
};

struct Environment
{
    std::vector<Obstacle> obstacles;
    double collision_threshold = 0.0;

    Environment() = default;
    Environment(const std::vector<Obstacle> &obs, double threshold = 0.1)
        : obstacles(obs), collision_threshold(threshold) {}

    void addObstacle(const Obstacle &o) { obstacles.push_back(o); }

    bool checkCollision(const State &s) const
    {
        for (const auto &obs : obstacles)
        {
            Eigen::Vector2d p0(obs.x, obs.y);
            double r = obs.R + collision_threshold;

            for (int i = 0; i < 4; ++i)
            {
                Eigen::Vector2d p1 = s.jointsMatrix.row(i);
                Eigen::Vector2d p2 = s.jointsMatrix.row(i + 1);

                if ((p0 - p1).norm() <= r || (p0 - p2).norm() <= r)
                    return true;

                Eigen::Vector2d line = p2 - p1;
                double L2 = line.squaredNorm();

                if (L2 < std::numeric_limits<double>::epsilon())
                    continue;

                double t = (p0 - p1).dot(line) / L2;

                if (t < 0.0)
                {
                    if ((p0 - p1).norm() <= r)
                        return true;
                }
                else if (t > 1.0)
                {
                    if ((p0 - p2).norm() <= r)
                        return true;
                }
                else
                {
                    Eigen::Vector2d projection = p1 + t * line;
                    if ((p0 - projection).norm() <= r)
                        return true;
                }
            }
        }
        return false;
    }
};