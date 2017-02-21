#include "mainwindow.h"
#include "settingswindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	MainWindow w;
	w.start();
	a.exec();
	return 0;
}
