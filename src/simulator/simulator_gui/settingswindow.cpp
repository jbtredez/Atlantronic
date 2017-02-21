#include "settingswindow.h"
#include "ui_settingswindow.h"
#include <QRect>
#include <QDesktopWidget>

SettingsWindow::SettingsWindow(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::SettingsWindow)
{
	ui->setupUi(this);

	QDesktopWidget desktop;
	int height=desktop.geometry().height();
	int width=desktop.geometry().width();
	move(width - 200, height / 2 - 240);
}

SettingsWindow::~SettingsWindow()
{
	delete ui;
}
