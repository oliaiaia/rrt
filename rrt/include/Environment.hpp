#pragma once

#include <vector>
#include <Eigen/Dense>
#include <cmath>

struct Obstacle
{
    double x, y;
    double R;
};

struct State
{
    Eigen::Vector4d angleVector;
    Eigen::Matrix<double, 5, 2> jointsMatrix;
    int joints;

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
        for (int t = 0; t < angles.size(); t++)
            angleVector(t) = angles[t];
    }

    static Eigen::Matrix3d formSE2(double x, double y, double angleDeg)
    {
        double angleRad = angleDeg * M_PI / 180.0;

        Eigen::Matrix3d T;
        T << std::cos(angleRad), -std::sin(angleRad), x,
             std::sin(angleRad),  std::cos(angleRad), y,
             0.0,                 0.0,                1.0;
        return T;
    }

    static Eigen::Matrix<double, 5, 2> calculateJointsByAngles(
        const std::vector<double> &angles,
        double jointLength = 1.0)
    {
        Eigen::Matrix<double, 5, 2> seg;
        seg.setZero();

        Eigen::Vector3d p0(0, 0, 1);

        Eigen::Matrix3d T1 = formSE2(0, 0, angles[0]);
        Eigen::Matrix3d T2 = formSE2(jointLength, 0, angles[1]);
        Eigen::Matrix3d T3 = formSE2(jointLength, 0, angles[2]);
        Eigen::Matrix3d T4 = formSE2(jointLength, 0, angles[3]);
        Eigen::Matrix3d T5 = formSE2(jointLength, 0, 0);

        seg(0, 0) = 0.0;
        seg(0, 1) = 0.0;

        Eigen::Vector3d p = T1 * p0;
        seg.row(1) = p.head<2>();

        p = T1 * T2 * p0;
        seg.row(2) = p.head<2>();

        p = T1 * T2 * T3 * p0;
        seg.row(3) = p.head<2>();

        p = T1 * T2 * T3 * T4 * T5 * p0;
        seg.row(4) = p.head<2>();

        return seg;
    }
};

struct Environment
{
    std::vector<Obstacle> obstacles;

    bool checkCollision(const State &s, double collisionThreshold)
    {
        for (const auto &obs : obstacles)
        {
            Eigen::Vector2d p0(obs.x, obs.y);
            double r = obs.R + collisionThreshold;

            for (int i = 0; i < 4; i++)
            {
                Eigen::Vector2d p1 = s.jointsMatrix.row(i);
                Eigen::Vector2d p2 = s.jointsMatrix.row(i + 1);

                if ((p0 - p1).norm() <= r || (p0 - p2).norm() <= r)
                    return true;

                Eigen::Vector2d line = p2 - p1;
                double L2 = line.squaredNorm();
                if (L2 == 0)
                    continue;

                double t = (p0 - p1).dot(line) / L2;
                if (t < 0.0 || t > 1.0)
                    continue;

                Eigen::Vector2d p4 = p1 + t * line;
                if ((p0 - p4).norm() <= r)
                    return true;
            }
        }

        return false;
    }
};
