#include <iostream>
#include <random>
#include<ranges>
#include "ParticleLife.hpp"
#include "Display_Particle.hpp"

#include <QApplication>
#include <QMainWindow>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QMainWindow window;
    window.resize(1600, 800);
    window.grabGesture(Qt::PanGesture);
    window.grabGesture(Qt::PinchGesture);
    window.show();

    return a.exec();
}