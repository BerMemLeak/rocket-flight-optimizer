// 123.cpp : Defines the entry point for the console application.
//
#include "iostream"
#include "./rocket/rocket.h"
#include "./RocketOptimizer/RocketOptimizer.h"
#include "gnuplot-iostream.h"

using namespace std;

void vizualize(rocket* roc);
void visualize3D(rocket* roc);
void visualizeSeparateGraphs(rocket* roc);

int main()
{
    // Параметры для создания объекта rocket
    double st_vel = 50.0;  // Начальная скорость
    double velocity_range = 5; // разбежка скорости

    double st_mass = 35.0;       // Начальная масса
    double weight_range = 0.7; // разбежка массы

    double Cx = 1.422;              // Коэффициент сопротивления по x
    double Cx_range = 0.02;         //разбежка сопротивления по x

    double Cz = 0;                  // Коэффициент сопротивления по z
    double Cz_range = 0.02;         //разбежка сопротивления по z


    double tetta = 12 * pi / 180; // Угол тангажа (в радианах)
    double tetta_range = 7 * pi / 180; //разбежка Угол тангажа

    double psi = 0;          // Угол пути
    double psi_range = 0.02;//разбежка Угол пути


    double x = 1;        // Начальная координата y
    double x_range = 1;//разбежка по высоте

    double y = 1;        // Начальная координата y
    double y_range = 1;//разбежка по высоте

    double z = 0;           // Начальная координата z
    double z_range = 0;           // Начальная координата z

    double d = 0.2;          // Диаметр миделя
    double d_range = 0.002; //разбежка Диаметр миделя

    double dt = 0.1;              // шаг по Время

    double G = 1 * dt;    // Расход топлива
    double G_range = 0.01 * dt;    // разбежка Расход топлива

    double booster_weight = 5; // вес разгонного двигателя
    double booster_weight_range = 0.5; //разбежка разгонного двигателя

    double time_to_undock = 0.5; // время до отстыковки
    double time_to_undock_range = 0.1; // разбежка время до отстыковки

    double iter_count = 1; //шаг по каждому параметру

    //сначала тяга у нас больше из-за стартовика, а потом меньше
    double Px_boost = 1500;
    double Py_boost = 100;
    double Pz_boost = 0;

    double Px = 1000;
    double Py = 50;
    double Pz = 0;


//    rocket roc( time_to_undock ,booster_weight,st_vel,  st_mass,    Cx,
//                Cz,  tetta,  psi,
//                x,  y,  z,  G,
//                d,  dt,  Px_boost,Py_boost, Pz_boost,Px,Py,Pz );
//    roc.print();

    RocketOptimizer optimizer(iter_count, st_vel, velocity_range, st_mass, weight_range,Cx , Cx_range, Cz, Cz_range,
                              tetta, tetta_range, psi, psi_range, x, x_range, y, y_range, z, z_range, d, d_range,
                              G, G_range, booster_weight, booster_weight_range, time_to_undock, time_to_undock_range,
                              Px_boost,Py_boost, Pz_boost,Px,Py,Pz);

    optimizer.findOptimalRocket();

//    здесь нужно перебрать все варианты, если есть вариант с наилучшими показателями(дальность полета),
//    то выбрать его и сохранить те параметры, с которыми нужно работать
// также нужно сохранить точки всех вариаций попадания на график

    vizualize(optimizer.roc_getter());
    visualize3D(optimizer.roc_getter());
    visualizeSeparateGraphs(optimizer.roc_getter());

//    vizualize(&roc);
//    visualize3D(&roc);
//    visualizeSeparateGraphs(&roc);
    return 0;
}

void vizualize(rocket* roc) {
    Gnuplot gp;

    // Настройки графика
    gp << "set title 'График зависимости y от x для ракеты и разгонного двигателя'\n";
    gp << "set xlabel 'x'\n";
    gp << "set ylabel 'y'\n";
    gp << "set grid\n";

    // Выводим два графика на одном изображении: данные ракеты и данные бустера
    gp << "plot '-' with lines title 'Rocket y(x)', '-' with lines title 'Booster y(x)'\n";

    // График 1: y(x) для ракеты
    std::vector<std::pair<double, double>> rocket_data;
    for (size_t i = 0; i < roc->get_data()["x"].size(); ++i) {
        rocket_data.emplace_back(roc->get_data()["x"][i], roc->get_data()["y"][i]);
    }
    gp.send1d(rocket_data);

    // График 2: y(x) для разгонного движка
    std::vector<std::pair<double, double>> booster_data;
    for (size_t i = 0; i < roc->get_booster_data()["x"].size(); ++i) {
        booster_data.emplace_back(roc->get_booster_data()["x"][i], roc->get_booster_data()["y"][i]);
    }
    gp.send1d(booster_data);
}


void visualize3D(rocket* roc) {
    Gnuplot gp;

    // Настройки графика
    gp << "set title '3D график траекторий ракеты и разгонного двигателя'\n";
    gp << "set xlabel 'x, м'\n";  // Ось x - горизонтальная
    gp << "set ylabel 'z, м'\n";  // Ось z - вертикальная
    gp << "set zlabel 'y, м'\n";  // Ось y - глубина

    // Устанавливаем диапазоны, чтобы начало координат было в точке (0,0,0)
    gp << "set xrange [0:*]\n";   // Ось x: диапазон начинается с 0
    gp << "set yrange [0:*]\n";   // Ось z: диапазон начинается с 0
    gp << "set zrange [0:*]\n";   // Ось y: диапазон начинается с 0

    gp << "set grid\n";
    gp << "set hidden3d\n";  // Для скрытия невидимых линий в 3D
    gp << "$rocket_data << EOD\n";

    // Данные для ракеты
    for (size_t i = 0; i < roc->get_data()["t"].size(); ++i) {
        gp << roc->get_data()["x"][i] << " "
           << roc->get_data()["z"][i] << " "
           << roc->get_data()["y"][i] << "\n";
    }
    gp << "EOD\n";

    gp << "$booster_data << EOD\n";

    // Данные для разгонного двигателя
    for (size_t i = 0; i < roc->get_booster_data()["t"].size(); ++i) {
        gp << roc->get_booster_data()["x"][i] << " "
           << roc->get_booster_data()["z"][i] << " "
           << roc->get_booster_data()["y"][i] << "\n";
    }
    gp << "EOD\n";

    // Построение графиков
    gp << "splot '$rocket_data' with lines title 'Rocket Trajectory', "
       << "'$booster_data' with lines title 'Booster Trajectory'\n";
}


void visualizeSeparateGraphs(rocket* roc) {
    Gnuplot gp1, gp2, gp3,gp4,gp5, gp6 ;

    // График 1: Зависимость tetta от времени t
    gp1 << "set title 'Зависимость tetta от времени t'\n";
    gp1 << "set xlabel 't, с'\n";
    gp1 << "set ylabel 'tetta'\n";
    gp1 << "set grid\n";
    std::vector<std::pair<double, double>> tetta_data;
    for (size_t i = 0; i < roc->get_data()["t"].size(); ++i) {
        tetta_data.emplace_back(roc->get_data()["t"][i], roc->get_data()["tetta"][i]);
    }
    gp1 << "plot '-' with lines title 'tetta(t)'\n";
    gp1.send1d(tetta_data);

    // График 2: Зависимость psi от времени t
    gp2 << "set title 'Зависимость psi от времени t'\n";
    gp2 << "set xlabel 't, с'\n";
    gp2 << "set ylabel 'psi'\n";
    gp2 << "set grid\n";
    std::vector<std::pair<double, double>> psi_data;
    for (size_t i = 0; i < roc->get_data()["t"].size(); ++i) {
        psi_data.emplace_back(roc->get_data()["t"][i], roc->get_data()["psi"][i]);
    }
    gp2 << "plot '-' with lines title 'psi(t)'\n";
    gp2.send1d(psi_data);

    // График 3: Зависимость V от времени t
    gp3 << "set title 'Зависимость скорости V от времени t'\n";
    gp3 << "set xlabel 't, с'\n";
    gp3 << "set ylabel 'V, м/с'\n";
    gp3 << "set xrange [0:*]\n";   // Ось x: диапазон начинается с 0
    gp3 << "set yrange [0:*]\n";   // Ось z: диапазон начинается с 0
    gp3 << "set grid\n";
    std::vector<std::pair<double, double>> V_data;
    for (size_t i = 0; i < roc->get_data()["t"].size(); ++i) {
        V_data.emplace_back(roc->get_data()["t"][i], roc->get_data()["V"][i]);
    }
    gp3 << "plot '-' with lines title 'V(t)'\n";
    gp3.send1d(V_data);



    // График 4: Зависимость tetta от времени t у бустера
    gp1 << "set title 'Зависимость tetta от времени у бустера t'\n";
    gp1 << "set xlabel 't, с'\n";
    gp1 << "set ylabel 'tetta'\n";
    gp1 << "set grid\n";
    std::vector<std::pair<double, double>> tetta_data_boost;
    for (size_t i = 0; i < roc->get_booster_data()["t"].size(); ++i) {
        tetta_data_boost.emplace_back(roc->get_booster_data()["t"][i], roc->get_booster_data()["tetta"][i]);
    }
    gp1 << "plot '-' with lines title 'tetta(t)'\n";
    gp1.send1d(tetta_data_boost);

    // График 2: Зависимость psi от времени t у бустера
    gp2 << "set title 'Зависимость psi от времени у бустера t'\n";
    gp2 << "set xlabel 't, с'\n";
    gp2 << "set ylabel 'psi'\n";
    gp2 << "set grid\n";
    std::vector<std::pair<double, double>> psi_data_boost;
    for (size_t i = 0; i < roc->get_booster_data()["t"].size(); ++i) {
        psi_data_boost.emplace_back(roc->get_booster_data()["t"][i], roc->get_booster_data()["psi"][i]);
    }
    gp2 << "plot '-' with lines title 'psi(t)'\n";
    gp2.send1d(psi_data_boost);

    // График 3: Зависимость V от времени t у бустера
    gp3 << "set title 'Зависимость скорости V от времени у бустера t'\n";
    gp3 << "set xlabel 't, с'\n";
    gp3 << "set ylabel 'V, м/с'\n";
    gp3 << "set xrange [0:*]\n";   // Ось x: диапазон начинается с 0
    gp3 << "set yrange [0:*]\n";   // Ось z: диапазон начинается с 0
    gp3 << "set grid\n";
    std::vector<std::pair<double, double>> V_data_boost;
    for (size_t i = 0; i < roc->get_booster_data()["t"].size(); ++i) {
        V_data_boost.emplace_back(roc->get_booster_data()["t"][i], roc->get_booster_data()["V"][i]);
    }
    gp3 << "plot '-' with lines title 'V(t)'\n";
    gp3.send1d(V_data_boost);
}





