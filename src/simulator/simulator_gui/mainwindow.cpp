#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QCloseEvent>

MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow)
{
	ui->setupUi(this);
	m_settingsWin = new SettingsWindow();

}

MainWindow::~MainWindow()
{
	delete ui;
}

void MainWindow::start()
{
	show();
	m_settingsWin->show();
}


void MainWindow::closeEvent(QCloseEvent *event)
{
	m_settingsWin->close();
}
