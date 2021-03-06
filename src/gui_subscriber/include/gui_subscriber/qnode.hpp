/**
 * @file /include/gui_subscriber/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef gui_subscriber_QNODE_HPP_
#define gui_subscriber_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
//#include "main_window.hpp"
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace gui_subscriber {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
    void run();
  //void myCallback(const std_msgs::Float64& message_holder);
  void myCallback_img(const sensor_msgs::ImageConstPtr& msg);
  cv::Mat img;
	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
    QStringListModel* loggingModel_sub(){return &logging_model_sub; } //add
    //void log( const LogLevel &level, const std_msgs::Float64 &msg);
    void log( const LogLevel &level, const std::string &msg);

  void log_sub(const LogLevel &level,const std::string& msg); //add
  void Callback(const std_msgs::String &submsg);// add
  void forward();
  void backward();
  void left();
  void right();
  float turtle_angular=0;
  float turtle_linear=0;
Q_SIGNALS:
  void loggingUpdated();
  void loggingUpdated_sub(); //add
  void rosShutdown();
  void imageSignal(cv::Mat);

private:
	int init_argc;
	char** init_argv;
  //ros::Subscriber chatter_subscriber;
    ros::Publisher chatter_publisher;//add
    ros::Subscriber chatter_subscriber; //add

    ros::Publisher vel_publisher;
  image_transport::Subscriber image_sub;
  QStringListModel logging_model;

  QStringListModel logging_model_sub; //add
};

}  // namespace gui_subscriber

#endif /* gui_subscriber_QNODE_HPP_ */
