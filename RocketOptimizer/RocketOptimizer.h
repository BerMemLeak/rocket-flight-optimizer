// RocketOptimizer.h

#pragma once

#include "../rocket/rocket.h"
#include <iostream>

class RocketOptimizer {
public:
    RocketOptimizer(double step_size, double st_vel, double velocity_range, double st_mass, double weight_range,
                    double Cx, double Cx_range, double Cz, double Cz_range, double tetta, double tetta_range,
                    double psi, double psi_range, double x, double x_range, double y, double y_range,
                    double z, double z_range, double d, double d_range, double G, double G_range,
                    double booster_weight, double booster_weight_range, double time_to_undock, double time_to_undock_range,
                    double Px_boost, double Py_boost, double Pz_boost, double Px = 0, double Py = 0, double Pz = 0);

    void findOptimalRocket();
    rocket* roc_getter();

private:
    rocket roc;
    double Px;
    double Py;
    double Pz;
    double Px_boost;
    double Py_boost;
    double Pz_boost;
    double step_size;
    double st_vel, velocity_range;
    double st_mass, weight_range;
    double Cx, Cx_range;
    double Cz, Cz_range;
    double tetta, tetta_range;
    double psi, psi_range;
    double x, x_range;
    double y, y_range;
    double z, z_range;
    double d, d_range;
    double G, G_range;
    double booster_weight, booster_weight_range;
    double time_to_undock, time_to_undock_range;
};

