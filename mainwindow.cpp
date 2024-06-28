#include "mainwindow.h"
#include "settings.h"

MainWindow::MainWindow() {
    particle_visualization *visualization = new class particle_visualization(this);
    setCentralWidget(visualization);

    settings = new Settings(this, visualization);
    settings->move(50,50);  // Set the size and position of the settings widget

    settings->show();

}
