#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "PoseWindow.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();


	PoseWindow *poseWindow;



private slots:
	void on_connectButton_clicked();
	void on_disconnectButton_clicked();

    void on_armButton_clicked();
	void on_disarmButton_clicked();

	void on_throttleSlider_valueChanged(int val);



private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
