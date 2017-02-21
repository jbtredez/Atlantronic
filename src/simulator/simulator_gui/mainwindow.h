#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QAction>
#include "ui_mainwindow.h"
#include "settingswindow.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow, Ui::MainWindow
{
	Q_OBJECT
public:
	void start();

public:
	explicit MainWindow(QWidget *parent = 0);
	~MainWindow();

private slots:
	void handleSettingMenu(bool);

private:
	void closeEvent(QCloseEvent *event);

private:
	SettingsWindow *m_settingsWin;
	QAction *m_settingsMenu;
};

#endif // MAINWINDOW_H
