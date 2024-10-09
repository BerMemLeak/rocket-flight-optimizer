//
// Created by user on 08.10.2024.
//

#include "RocketOptimizer.h"


RocketOptimizer::RocketOptimizer(double step_size, double st_vel, double velocity_range, double st_mass, double weight_range,
                                 double Cx, double Cx_range, double Cz, double Cz_range, double tetta, double tetta_range,
                                 double psi, double psi_range, double x, double x_range, double y, double y_range,
                                 double z, double z_range, double d, double d_range, double G, double G_range,
                                 double booster_weight, double booster_weight_range, double time_to_undock, double time_to_undock_range,
                                 double Px_boost, double Py_boost, double Pz_boost, double Px, double Py, double Pz)
        : roc(0, 0, st_vel, st_mass, Cx, Cz, tetta, psi, x, y, z, G, d, 0.1, 0, 0, 0), // Инициализация rocket
          step_size(step_size), st_vel(st_vel), velocity_range(velocity_range), st_mass(st_mass), weight_range(weight_range),
          Cx(Cx), Cx_range(Cx_range), Cz(Cz), Cz_range(Cz_range), tetta(tetta), tetta_range(tetta_range),
          psi(psi), psi_range(psi_range), x(x), x_range(x_range), y(y), y_range(y_range),
          z(z), z_range(z_range), d(d), d_range(d_range), G(G), G_range(G_range),
          booster_weight(booster_weight), booster_weight_range(booster_weight_range),
          time_to_undock(time_to_undock), time_to_undock_range(time_to_undock_range),
          Px_boost(Px_boost), Py_boost(Py_boost), Pz_boost(Pz_boost), Px(Px), Py(Py), Pz(Pz) {}

void RocketOptimizer::findOptimalRocket() {
    double bestDistance = 0;
    rocket* bestRocket = nullptr;

    // Перебор всех вариантов параметров ракеты с заданным шагом
    for (double vel = st_vel - velocity_range; vel <= st_vel + velocity_range; vel += step_size) {
        for (double mass = st_mass - weight_range; mass <= st_mass + weight_range; mass += step_size) {
            for (double cx = Cx - Cx_range; cx <= Cx + Cx_range; cx += step_size) {
                for (double cz = Cz - Cz_range; cz <= Cz + Cz_range; cz += step_size) {
                    for (double angle_tetta = tetta - tetta_range; angle_tetta <= tetta + tetta_range; angle_tetta += step_size) {
                        for (double angle_psi = psi - psi_range; angle_psi <= psi + psi_range; angle_psi += step_size) {
                            for (double coord_x = x - x_range; coord_x <= x + x_range; coord_x += step_size) {
                                for (double coord_y = y - y_range; coord_y <= y + y_range; coord_y += step_size) {
                                    for (double coord_z = z - z_range; coord_z <= z + z_range; coord_z += step_size) {
                                        for (double diameter = d - d_range; diameter <= d + d_range; diameter += step_size) {
                                            for (double fuel_G = G - G_range; fuel_G <= G + G_range; fuel_G += step_size) {
                                                for (double boosterW = booster_weight - booster_weight_range; boosterW <= booster_weight + booster_weight_range; boosterW += step_size) {
                                                    for (double undockTime = time_to_undock - time_to_undock_range; undockTime <= time_to_undock + time_to_undock_range; undockTime += step_size) {

                                                        // Динамическое создание ракеты
                                                        rocket* currentRocket = new rocket(undockTime, boosterW, vel, mass, cx, cz,
                                                                                           angle_tetta, angle_psi, coord_x, coord_y, coord_z,
                                                                                           fuel_G, diameter, 0.1, Px_boost, Py_boost, Pz_boost, Px, Py, Pz);

                                                        // Симуляция полета и получение дальности
                                                        double distance = currentRocket->get_data()["x"].back();

                                                        // Если дальность лучше, сохраняем результат
                                                        if (distance > bestDistance) {
                                                            bestDistance = distance;
                                                            if (bestRocket) {
                                                                delete bestRocket;  // Освобождаем предыдущий лучший объект
                                                            }
                                                            bestRocket = currentRocket;
                                                        } else {
                                                            delete currentRocket;  // Освобождаем неудачный объект
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    if (bestRocket) {
        std::cout << "Наилучший результат полета: " << bestDistance << " метров.\n";
        bestRocket->printRocketParameters();  // Вывод параметров лучшей ракеты
    } else {
        std::cout << "Не удалось найти подходящую ракету.\n";
    }

    this->roc = *bestRocket;  // Копируем объект в член класса
    delete bestRocket;  // Освобождаем память после копирования
}

rocket* RocketOptimizer::roc_getter() {
    return &this->roc;
}
