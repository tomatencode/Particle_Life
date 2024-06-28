#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "settings.h"
#include "particle_visualization.h"

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow();
    Settings *settings;

private:
    particle_visualization *visualization;
};

#endif // MAINWINDOW_H
