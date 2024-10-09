//
// Created by user on 08.10.2024.
//

#include "rocket.h"

#include "rocket.h"

rocket::rocket(double time_to_undock, double booster_weight,double st_vel, double st_mass,  double Cx,
               double Cz, double tetta, double psi,
               double x, double y, double z, double G,
               double d, double dt,double Px_boost,double Py_boost, double Pz_boost, double Px, double Py, double Pz)
        : st_vel(st_vel), st_mass(st_mass), Cx(Cx),
          Cz(Cz), tetta(tetta), psi(psi), x(x), y(y), z(z),
          G(G), Px_boost(Px_boost),Py_boost(Py_boost), Pz_boost(Pz_boost), Px(Px), Py(Py), Pz(Pz), dt(dt), d(d) {

    this->Sm = 3.14 * d * d / 4;

    // заполним начальными данными
    this->data["V"].push_back(st_vel);
    this->data["tetta"].push_back(tetta);
    this->data["psi"].push_back(psi);
    this->data["x"].push_back(x);
    this->data["y"].push_back(y);
    this->data["z"].push_back(z);
    this->data["m"].push_back(st_mass);
    this->data["t"].push_back(0);
    this->data["Px"].push_back(Px_boost);
    this->data["Py"].push_back(Py_boost);
    this->data["Pz"].push_back(Pz_boost);

// решаем систему
    bool undocked = false; // Переменная для отслеживания состояния отделения


    while (this->data["y"].back() >= 0) {
        // Проверяем, нужно ли отделить двигатель
        if (!undocked && this->data["t"].back() >= time_to_undock) {
            this->H = data["x"].back();
            this->data["m"].back() -= booster_weight; // Отделяем двигатель
            undocked = true; // Устанавливаем флаг, что отделение произошло
            this->boosterData["V"].push_back(this->data["V"].back());
            this->boosterData["tetta"].push_back(this->data["tetta"].back());
            this->boosterData["psi"].push_back(this->data["psi"].back());
            this->boosterData["x"].push_back(this->data["x"].back());
            this->boosterData["y"].push_back(this->data["y"].back());
            this->boosterData["z"].push_back(this->data["z"].back());
            this->boosterData["m"].push_back(booster_weight);
            this->boosterData["t"].push_back(this->data["t"].back());
            this->boosterData["Px"].push_back(0);
            this->boosterData["Py"].push_back(0);
            this->boosterData["Pz"].push_back(0);
            //тут выключение разгонного , теперь у нас только маршевый
            this->data["Px"].push_back(Px);
            this->data["Py"].push_back(Py);
            this->data["Pz"].push_back(Pz);
            while (this->boosterData["y"].back() >= 0) {
                this->runge_kutta_step_for_booster(); // Выполняем шаг Рунге-Кутты для разгонного двигателя

            }
        }
        this->runge_kutta_step(); // Выполняем шаг Рунге-Кутты
    }
}



void rocket::printRocketParameters()  {
    std::cout << "Текущие параметры ракеты:" << std::endl;
    std::cout << "Диаметр миделя (d): " << d << std::endl;
    std::cout << "Скорость (st_vel): " << st_vel << std::endl;
    std::cout << "Стартовая масса (st_mass): " << st_mass << std::endl;
    std::cout << "Коэффициент сопротивления по X (Cx): " << Cx << std::endl;
    std::cout << "Коэффициент сопротивления по Z (Cz): " << Cz << std::endl;
    std::cout << "Угол тангажа (tetta): " << tetta * 180 / M_PI << " градусов" << std::endl;
    std::cout << "Угол пути (psi): " << psi * 180 / M_PI << " градусов" << std::endl;
    std::cout << "Координата X: " << x << std::endl;
    std::cout << "Координата Y: " << y << std::endl;
    std::cout << "Координата Z: " << z << std::endl;
    std::cout << "Расход топлива (G): " << G << std::endl;
    std::cout << "Площадь миделя (Sm): " << Sm << std::endl;
    std::cout << "Проекция скорости по X (Px): " << Px << std::endl;
    std::cout << "Проекция скорости по Y (Py): " << Py << std::endl;
    std::cout << "Проекция скорости по Z (Pz): " << Pz << std::endl;
    std::cout << "Шаг по времени (dt): " << dt << std::endl;
    std::cout << "Высота в момент разделения (H): " << H << std::endl;
}

void rocket::print() {
    for (auto it = this->data.begin(); it != this->data.end(); it++) {
        cout << it->first << ": " << endl;
        for (auto num = it->second.begin(); num != it->second.end(); num++) {
            cout << *num << " ";
        }
        cout << endl;
    }
}

void rocket::printBooster() {
    for (auto it = this->boosterData.begin(); it != this->boosterData.end(); it++) {
        cout << it->first << ": " << endl;
        for (auto num = it->second.begin(); num != it->second.end(); num++) {
            cout << *num << " ";
        }
        cout << endl;
    }
}

void rocket::print_first_end() {
    for (auto it = this->data.begin(); it != this->data.end(); it++) {
        cout << it->first << ": " << *(it->second.begin()) << " ";
        cout << *(it->second.end() - 1) << " ";
        cout << endl;
    }
}

map<string, vector<double>>& rocket::get_data() {
    return this->data;
}

map<string, vector<double>>& rocket::get_booster_data(){
    return this->boosterData;
};


vector<double> rocket::atmosphere() {
    double p = 101325;
    double T = 288.15;
    double rho = 1.225;
    double a = 340.294;
    double g = 9.80665;

    vector<double> atmo = {p, T, rho, a, g};
    return atmo;
}

double rocket::i_form() const {
    return 4.2 * (1.67 * this->d + 1.0326);
}

double rocket::Fx_43() {
    double a = atmosphere()[3];
    double M = (this->data["V"].back()) / 340.294;

    if (M >= 0.1 && M <= 0.7)
        return zeta(0.7) * Fx_Siacci(0.7) * M * a / (0.7 * a);
    else if (M > 0.7 && M <= 3.3)
        return zeta(M) * Fx_Siacci(M);
    else
        return zeta(3.3) * Fx_Siacci(3.3) * M * a / (3.3 * a);
}

double rocket::Fx_to_cx() {
    double rho = atmosphere()[2];
    double a = atmosphere()[3];
    return 8000 * this->Fx_43() / (pi * rho * pow(((this->data["V"].back()) / 340.294 * a), 2));
}

double rocket::zeta(double M) {
    double b0 = 0.652;
    double b1 = -0.1133;
    double b2 = -0.001;
    double b3 = 0.0063;
    return b0 + M * (b1 + M * (b2 + M * b3));
}

double rocket::Fx_Siacci(double M) {
    double a = atmosphere()[3];
    return 0.2002 * M * a - 48.05 + sqrt(pow(0.1648 * M * a - 47.95, 2) + 9.6) +
           0.0442 * M * a * (M * a - 300) / (371 + pow(M * a / 200, 10));
}

double rocket::cx_43() {
    return i_form() * Fx_to_cx();
}

double rocket::cy() {
    return 0.1;
}

void rocket::derivatives(double P_x,double P_y, double P_z, double V, double Theta, double Psi, double x, double y, double z, double m, double t,
                         double& dV, double& dTheta, double& dPsi, double& dx, double& dy, double& dz, double& dm) {
    double rho = atmosphere()[2];
    double a = atmosphere()[3];
    double g = atmosphere()[4];
    dV = 1 / m * (P_x - cx_43() * rho * pow(V, 2) / 2 * Sm - m * g * sin(Theta));
    dTheta = 1 / (m * V) * (P_y + cy() * rho * pow(V, 2) / 2 * Sm - m * g * cos(Theta));
    dPsi = - (P_z + cy() * rho * pow(V, 2) / 2 * Sm) / (m * V * cos(Theta));
    dx = V * cos(Theta) * cos(Psi);
    dy = V * sin(Theta);
    dz = - V * cos(Theta) * sin(Psi);
    dm = -G;
}


void rocket::derivatives_booster(double V, double Theta, double Psi, double x, double y, double z, double m, double t,
                         double& dV, double& dTheta, double& dPsi, double& dx, double& dy, double& dz, double& dm) {
    double rho = atmosphere()[2];
    double a = atmosphere()[3];
    double g = atmosphere()[4];

    dV = 1 / m * (0 - cx_43() * rho * pow(V, 2) / 2 * Sm - m * g * sin(Theta));
    dTheta = 1 / (m * V) * (0 + cy() * rho * pow(V, 2) / 2 * Sm - m * g * cos(Theta));
    dPsi = - (0 + cy() * rho * pow(V, 2) / 2 * Sm) / (m * V * cos(Theta));
    dx = V * cos(Theta) * cos(Psi);
    dy = V * sin(Theta);
    dz = - V * cos(Theta) * sin(Psi);
    dm = 0;
}


void rocket::runge_kutta_step() {
    double V_ = this->data["V"].back();
    double Theta_ = this->data["tetta"].back();
    double Psi_ = this->data["psi"].back();
    double x_ = this->data["x"].back();
    double y_ = this->data["y"].back();
    double z_ = this->data["z"].back();
    double m_ = this->data["m"].back();
    double t_ = this->data["t"].back();
    double P_x = this->data["Px"].back();
    double P_y = this->data["Py"].back();
    double P_z = this->data["Pz"].back();

    double dV1, dTheta1, dPsi1, dx1, dy1, dz1, dm1;
    double dV2, dTheta2, dPsi2, dx2, dy2, dz2, dm2;
    double dV3, dTheta3, dPsi3, dx3, dy3, dz3, dm3;
    double dV4, dTheta4, dPsi4, dx4, dy4, dz4, dm4;

    // K1
    derivatives(P_x, P_y, P_z, V_, Theta_, Psi_, x_, y_, z_, m_, t_, dV1, dTheta1, dPsi1, dx1, dy1, dz1, dm1);

    // K2
    derivatives(P_x, P_y, P_z,V_ + dV1 * dt / 2, Theta_ + dTheta1 * dt / 2,
                Psi_ + dPsi1 * dt / 2, x_ + dx1 * dt / 2,
                y_ + dy1 * dt / 2, z_ + dz1 * dt / 2,
                m_ + dm1 * dt / 2, t_ + dt / 2,
                dV2, dTheta2, dPsi2, dx2, dy2, dz2, dm2);

    // K3
    derivatives(P_x, P_y, P_z,V_ + dV2 * dt / 2, Theta_ + dTheta2 * dt / 2,
                Psi_ + dPsi2 * dt / 2, x_ + dx2 * dt / 2,
                y_ + dy2 * dt / 2, z_ + dz2 * dt / 2,
                m_ + dm2 * dt / 2, t_ + dt / 2,
                dV3, dTheta3, dPsi3, dx3, dy3, dz3, dm3);

    // K4
    derivatives(P_x, P_y, P_z,V_ + dV3 * dt, Theta_ + dTheta3 * dt,
                Psi_ + dPsi3 * dt, x_ + dx3 * dt,
                y_ + dy3 * dt, z_ + dz3 * dt,
                m_ + dm3 * dt, t_ + dt,
                dV4, dTheta4, dPsi4, dx4, dy4, dz4, dm4);

    // Update values
    this->data["V"].push_back(V_ + (dV1 + 2 * dV2 + 2 * dV3 + dV4) * dt / 6);
    this->data["tetta"].push_back(Theta_ + (dTheta1 + 2 * dTheta2 + 2 * dTheta3 + dTheta4) * dt / 6);
    this->data["psi"].push_back(Psi_ + (dPsi1 + 2 * dPsi2 + 2 * dPsi3 + dPsi4) * dt / 6);
    this->data["x"].push_back(x_ + (dx1 + 2 * dx2 + 2 * dx3 + dx4) * dt / 6);
    this->data["y"].push_back(y_ + (dy1 + 2 * dy2 + 2 * dy3 + dy4) * dt / 6);
    this->data["z"].push_back(z_ + (dz1 + 2 * dz2 + 2 * dz3 + dz4) * dt / 6);
    this->data["m"].push_back(m_ + (dm1 + 2 * dm2 + 2 * dm3 + dm4) * dt / 6);
    this->data["t"].push_back(t_ + dt);
    this->data["Px"].push_back(P_x);
    this->data["Py"].push_back(P_y);
    this->data["Pz"].push_back(P_z);
}

//в падлу разделять на 2 класса
void rocket::runge_kutta_step_for_booster() {
    double V_ = this->boosterData["V"].back();
    double Theta_ = this->boosterData["tetta"].back();
    double Psi_ = this->boosterData["psi"].back();
    double x_ = this->boosterData["x"].back();
    double y_ = this->boosterData["y"].back();
    double z_ = this->boosterData["z"].back();
    double m_ = this->boosterData["m"].back();
    double t_ = this->boosterData["t"].back();

    double dV1, dTheta1, dPsi1, dx1, dy1, dz1, dm1;
    double dV2, dTheta2, dPsi2, dx2, dy2, dz2, dm2;
    double dV3, dTheta3, dPsi3, dx3, dy3, dz3, dm3;
    double dV4, dTheta4, dPsi4, dx4, dy4, dz4, dm4;

    // K1
    derivatives_booster(V_, Theta_, Psi_, x_, y_, z_, m_, t_, dV1, dTheta1, dPsi1, dx1, dy1, dz1, dm1);

    // K2
    derivatives_booster(V_ + dV1 * dt / 2, Theta_ + dTheta1 * dt / 2,
                Psi_ + dPsi1 * dt / 2, x_ + dx1 * dt / 2,
                y_ + dy1 * dt / 2, z_ + dz1 * dt / 2,
                m_ + dm1 * dt / 2, t_ + dt / 2,
                dV2, dTheta2, dPsi2, dx2, dy2, dz2, dm2);

    // K3
    derivatives_booster(V_ + dV2 * dt / 2, Theta_ + dTheta2 * dt / 2,
                Psi_ + dPsi2 * dt / 2, x_ + dx2 * dt / 2,
                y_ + dy2 * dt / 2, z_ + dz2 * dt / 2,
                m_ + dm2 * dt / 2, t_ + dt / 2,
                dV3, dTheta3, dPsi3, dx3, dy3, dz3, dm3);

    // K4
    derivatives_booster(V_ + dV3 * dt, Theta_ + dTheta3 * dt,
                Psi_ + dPsi3 * dt, x_ + dx3 * dt,
                y_ + dy3 * dt, z_ + dz3 * dt,
                m_ + dm3 * dt, t_ + dt,
                dV4, dTheta4, dPsi4, dx4, dy4, dz4, dm4);

    // Update values
    this->boosterData["V"].push_back(V_ + (dV1 + 2 * dV2 + 2 * dV3 + dV4) * dt / 6);
    this->boosterData["tetta"].push_back(Theta_ + (dTheta1 + 2 * dTheta2 + 2 * dTheta3 + dTheta4) * dt / 6);
    this->boosterData["psi"].push_back(Psi_ + (dPsi1 + 2 * dPsi2 + 2 * dPsi3 + dPsi4) * dt / 6);
    this->boosterData["x"].push_back(x_ + (dx1 + 2 * dx2 + 2 * dx3 + dx4) * dt / 6);
    this->boosterData["y"].push_back(y_ + (dy1 + 2 * dy2 + 2 * dy3 + dy4) * dt / 6);
    this->boosterData["z"].push_back(z_ + (dz1 + 2 * dz2 + 2 * dz3 + dz4) * dt / 6);
    this->boosterData["m"].push_back(m_ + (dm1 + 2 * dm2 + 2 * dm3 + dm4) * dt / 6);
    this->boosterData["t"].push_back(t_ + dt);
    this->boosterData["Px"].push_back(0);
    this->boosterData["Py"].push_back(0);
    this->boosterData["Pz"].push_back(0);
}