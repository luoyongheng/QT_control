/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#include <QImage>
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/gui_subscriber/main_window.hpp"
//#include"../../gui_publisher/include/gui_publisher/main_window.hpp"
#include <QDebug>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace enc = sensor_msgs::image_encodings;
namespace gui_subscriber {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
  ,qnode(argc,argv)
{
  ui.setupUi(this);
  //LCD----
  ui.lcdNumber->setDigitCount(5);
  ui.lcdNumber->setMode(QLCDNumber::Dec);
  ui.lcdNumber->setSmallDecimalPoint(true);
  ui.lcdNumber->setSegmentStyle(QLCDNumber::Flat);
  ui.lcdNumber->setStyleSheet("border:2px solid green; color: green; background: red;");
  on_lcdNumber_overflow();

  ui.lcdNumber_2->setDigitCount(5);
  ui.lcdNumber_2->setMode(QLCDNumber::Dec);
  ui.lcdNumber_2->setSmallDecimalPoint(true);
  ui.lcdNumber_2->setSegmentStyle(QLCDNumber::Flat);
  ui.lcdNumber_2->setStyleSheet("border:2px solid green; color: green; background: red;");
  on_lcdNumber_2_overflow();

  ui.lcdNumber_3->setDigitCount(5);
  ui.lcdNumber_3->setMode(QLCDNumber::Dec);
  ui.lcdNumber_3->setSmallDecimalPoint(true);
  ui.lcdNumber_3->setSegmentStyle(QLCDNumber::Flat);
  ui.lcdNumber_3->setStyleSheet("border:2px solid green; color: green; background: red;");
  on_lcdNumber_3_overflow();
  timer = new QTimer();
  timer->setInterval(10);
  timer->start();
  connect(timer, SIGNAL(timeout()), this, SLOT(on_lcdNumber_overflow()));
  connect(timer, SIGNAL(timeout()), this, SLOT(on_lcdNumber_2_overflow()));
  connect(timer, SIGNAL(timeout()), this, SLOT(on_lcdNumber_3_overflow()));
  //LCD---
  //Be sure register the message type
  qRegisterMetaType<cv::Mat>("cv::Mat");
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));

  ReadSettings();
  setWindowIcon(QIcon(":/images/icon.png"));
  ui.tab_manager->setCurrentIndex(0);
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  //Next is the key siganl and slot
  QObject::connect(&qnode,SIGNAL(imageSignal(cv::Mat)),this,SLOT(displayMat(cv::Mat)));
	/*********************
	** Logging
	**********************/
  ui.view_logging->setModel(qnode.loggingModel());
  ui.view_logging_sub->setModel(qnode.loggingModel_sub()); //add
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
  QObject::connect(&qnode,SIGNAL(loggingUpdated_sub()),this,SLOT(updateLoggingView_sub())); //add
  //QObject::connect(ui.right_vel, SIGNAL(valueChanged(int)), this, SLOT(on_right_vel_sliderMoved(int)));
  //实现槽函数
    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/
void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}
void MainWindow::updateLoggingView_sub(){// add
        ui.view_logging_sub-> scrollToBottom();
}
/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "gui_subscriber");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "gui_subscriber");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

void MainWindow::displayMat(cv::Mat image)
{
  cv::Mat rgb;
      QImage img;
      if(image.channels()==3)
      {
          //cvt Mat BGR 2 QImage RGB
          cv::cvtColor(image,rgb,CV_BGR2RGB);
          img =QImage((const unsigned char*)(rgb.data),
                      rgb.cols,rgb.rows,
                      rgb.cols*rgb.channels(),
                      QImage::Format_RGB888);
      }
      else
      {
          img =QImage((const unsigned char*)(image.data),
                      image.cols,image.rows,
                      image.cols*image.channels(),
                      QImage::Format_RGB888);
      }
      ui.label_11->setPixmap(QPixmap::fromImage(img));
      ui.label_11->resize(ui.label_11->pixmap()->size());
}

void MainWindow::on_lcdNumber_overflow()
{
    ui.lcdNumber->display(angular_value);
    std::cout<<"angular value is"<<angular_value<<std::endl;
}

void MainWindow::on_lcdNumber_2_overflow()
{
    ui.lcdNumber_2->display(linear_value);
    std::cout<<"linear Valueis"<<linear_value<<std::endl;
}

void MainWindow::on_lcdNumber_3_overflow()
{
    ui.lcdNumber_3->display((linear_value-linear_value_old)/0.1);
}

}  // namespace gui_subscriber

void gui_subscriber::MainWindow::on_forward_clicked()
{
    qnode.forward();
}

void gui_subscriber::MainWindow::on_backward_clicked()
{
    qnode.backward();
}

void gui_subscriber::MainWindow::on_left_clicked()
{
    qnode.left();
}
void gui_subscriber::MainWindow::on_right_clicked()
{
    qnode.right();
}

void gui_subscriber::MainWindow::on_right_vel_sliderMoved(int slider_value)
{
  angular_value = slider_value*0.1;//slider 值0-100,转化为0-10弧度
  qnode.turtle_angular = angular_value;//将值传给qnode.cpp中turtle角速度
  std::cout<<"Angular Value: "<<angular_value<<std::endl;
}

void gui_subscriber::MainWindow::on_left_vel_sliderMoved(int slider_value)
{
  angular_value = slider_value*0.1;//slider 值0-100,转化为0-10弧度
  qnode.turtle_angular = angular_value;//将值传给qnode.cpp中turtle角速度
  std::cout<<"Angular Value: "<<angular_value<<std::endl;
}

void gui_subscriber::MainWindow::on_forward_vel_sliderMoved(int slider_value)
{
  linear_value_old = linear_value;
  linear_value = slider_value*0.1;//slider 值0-100,转化为0-10弧度
  qnode.turtle_linear = linear_value;//将值传给qnode.cpp中turtle角速度
  std::cout<<"linear Value: "<<linear_value<<std::endl;
}

void gui_subscriber::MainWindow::on_backward_vel_sliderMoved(int slider_value)
{
  linear_value_old = linear_value;
  linear_value = slider_value*0.1;//slider 值0-100,转化为0-10弧度
  qnode.turtle_linear = linear_value;//将值传给qnode.cpp中turtle角速度
  std::cout<<"linear Value: "<<linear_value<<std::endl;
}

void gui_subscriber::MainWindow::on_rviz_clicked()
{
    system("gnome-terminal -x bash -c 'rosrun rviz rviz'");
    exit(0);
}

void gui_subscriber::MainWindow::on_gazebo_clicked()
{
    system("gnome-terminal -x bash -c 'gazebo'");
    exit(0);
}
