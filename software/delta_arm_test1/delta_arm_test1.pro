CONFIG += c++11 console
CONFIG -= app_bundle

DEFINES += QT_DEPRECATED_WARNINGS

SOURCES += \
        arm_kinematics.cpp \
        main.cpp

HEADERS += \
    arm_kinematics.h

INCLUDEPATH += /usr/include/eigen3

QT -= gui
QT += serialport
