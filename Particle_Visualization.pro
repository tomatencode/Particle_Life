QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    force_table.cpp \
    main.cpp \
    mainwindow.cpp \
    particle.cpp \
    particle_life.cpp \
    particle_visualization.cpp \
    settings.cpp

HEADERS += \
    force_table.h \
    mainwindow.h \
    particle.h \
    particle_life.h \
    particle_visualization.h \
    settings.h

FORMS += \
    settings.ui

INCLUDEPATH += /usr/include/eigen3

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    settings_resurces.qrc
