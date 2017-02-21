#include "mainwindow.h"
#include <QCloseEvent>
#include <QDebug>
#include "ui_mainwindow.h"

void MainWindow::start()
{
	show();
	m_settingsWin->show();
}

MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent)
{
	setupUi(this);
	m_settingsWin = new SettingsWindow();
	m_settingsMenu = findChild<QAction*>("actionSettings");

	connect(m_settingsMenu, SIGNAL(triggered(bool)), this, SLOT(handleSettingMenu(bool)));
}

MainWindow::~MainWindow()
{
	delete m_settingsWin;
}

void MainWindow::handleSettingMenu(bool)
{
	m_settingsWin->show();
}


void MainWindow::closeEvent(QCloseEvent *event)
{
	m_settingsWin->hide();
}
