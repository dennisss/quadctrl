#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "PoseWindow.h"

#include "net.h"

#include <iostream>
#include <thread>


using namespace std;

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow){

	ui->setupUi(this);



	// In the method you are creating/calling a QTOgreWindow:

	poseWindow = new PoseWindow();
	QWidget* renderingContainer = QWidget::createWindowContainer(poseWindow);

	// As an example, the below method places the QTOgreWindow we just created inside of a QTabWidget as a tab.

	ui->tabWidget->addTab(renderingContainer, tr("Scene"));


}

MainWindow::~MainWindow(){
    delete ui;
}





static MainWindow *win = NULL;
static thread pose_thread;
void pose_listener(packet *pkt, int datalen){

	if(pkt->type == PACKET_POSE){
		float *quat = (float *)pkt->data;
		win->poseWindow->axesNode->setOrientation(quat[0], quat[1], quat[2], quat[3]);
	}

}
void pose_requester(){
	while(win != NULL){
		net_request_pose();
		usleep(50000);
	}
}


void MainWindow::on_connectButton_clicked(){
	win = this;

	net_init();
	net_setlistener(pose_listener);



	// TODO: Start up another thread for listening to the xbee
	pose_thread = std::thread(pose_requester);


}

void MainWindow::on_disconnectButton_clicked(){
	win = NULL;
	pose_thread.join();

	net_destroy();
}

void MainWindow::on_armButton_clicked(){

	net_arm();

}

void MainWindow::on_disarmButton_clicked(){

	net_disarm();

}

/*
void MainWindow::on_pushButton_clicked(){

    ui->throttleSlider->setValue(100);

}
*/

void MainWindow::on_throttleSlider_valueChanged(int val){

	float v = val / 100.0f;

	cout << v << endl;

	net_setthrottle(v);
}


/*



*/
