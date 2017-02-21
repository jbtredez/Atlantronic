#-------------------------------------------------
#
# Project created by QtCreator 2017-02-01T22:39:46
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Simulator
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    settingswindow.cpp \
    openglwidget.cpp

HEADERS  += mainwindow.h \
    settingswindow.h \
    openglwidget.h

FORMS    += mainwindow.ui \
    settingswindow.ui

QT += opengl

