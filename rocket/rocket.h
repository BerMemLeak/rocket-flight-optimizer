//
// Created by user on 08.10.2024.
//

#pragma once

#include "iostream"
#include <vector>
#include <map>
#include <string>
#include <random>
#include <fstream>

using namespace std;

const double pi = 3.141592653589793238463;

class rocket {
public:
    rocket(double time_to_undock,double booster_weight, double st_vel, double st_mass,  double Cx,
           double Cz, double tetta, double psi,
           double x, double y, double z, double G,
           double d, double dt,double Px_boost,double Py_boost, double Pz_boost, double Px = 0, double Py = 0, double Pz = 0);

    void print();
    void printBooster();
    void print_first_end();
    void printRocketParameters();
    map<string, vector<double>>& get_data();
    map<string, vector<double>>& get_booster_data();

private:
    double d;
    double st_vel; // скорость
    double st_mass; // стартовая масса
    double Cx; // коэф сопр по х
    double Cz; // коэф сопр по z
    double tetta; // угол тангажа
    double psi; // угол пути
    double x;
    double y;
    double z;
    double G; // расход
    double Sm; // площадь миделя
    double Px;
    double Py;
    double Pz;
    double Px_boost;
    double Py_boost;
    double Pz_boost;
    double dt;
    double H; // высота в момент разделения
    map<string, vector<double>> data; // данные решенной системы для ракеты
    map<string, vector<double>> boosterData; // данные решенной системы для разгонной ду

    vector<double> atmosphere();
    double i_form() const;
    double Fx_43();
    double Fx_to_cx();
    double zeta(double M);
    double Fx_Siacci(double M);
    double cx_43();
    double cy();

    void derivatives(double P_x, double P_y, double P_z, double V, double Theta, double Psi, double x, double y, double z, double m, double t,
                     double& dV, double& dTheta, double& dPsi, double& dx, double& dy, double& dz, double& dm);
    void runge_kutta_step();
    void runge_kutta_step_for_booster();
    void derivatives_booster(double V, double Theta, double Psi, double x, double y, double z, double m, double t,
                     double& dV, double& dTheta, double& dPsi, double& dx, double& dy, double& dz, double& dm);

};
