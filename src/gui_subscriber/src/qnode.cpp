/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

#include "../include/gui_subscriber/main_window.hpp"
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/gui_subscriber/qnode.hpp"
#include <ros/ros.h>
#include <ros/network.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/image_encodings.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <iostream>
#include <geometry_msgs/Twist.h>

namespace enc = sensor_msgs::image_encodings;

namespace gui_subscriber {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"gui_subscriber");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
  //chatter_subscriber = n.subscribe("chatter",1000,&QNode::myCallback,this);
  image_transport::ImageTransport it(n);
  image_transport::Subscriber image_sub;
  //image_sub = it.subscribe("node_a",1,&QNode::myCallback_img,this);//图片尝试
  image_sub = it.subscribe("/cam0/image_raw",1000,&QNode::myCallback_img,this);//图片尝试
  chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);//add-
  chatter_subscriber = n.subscribe("chatter",1000,&QNode::Callback,this); //add
  vel_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",100);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"gui_subscriber");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
  //chatter_subscriber = n.subscribe("chatter",1000,&QNode::myCallback,this);
  image_transport::ImageTransport it(n);
  image_transport::Subscriber image_sub;
  //image_sub = it.subscribe("node_a",1,&QNode::myCallback_img,this);//subscribe image
  image_sub = it.subscribe("/cam0/image_raw",1000,&QNode::myCallback_img,this);//图片尝试
  chatter_subscriber = n.subscribe("chatter",1000,&QNode::Callback,this); //add
  chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);//add-
  vel_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",100);
  start();
	return true;
}

void QNode::run()
{
  log(Info,"start runing");
  ros::Rate loop_rate(20);
  ros::NodeHandle n;
  chatter_subscriber = n.subscribe("chatter",1000,&QNode::Callback,this);//add-
  image_transport::ImageTransport it(n);
  image_transport::Subscriber image_sub;
  //image_sub = it.subscribe("node_a",1,&QNode::myCallback_img,this);
  image_sub = it.subscribe("/cam0/image_raw",1000,&QNode::myCallback_img,this);//图片尝试

  int count=0;
  while(ros::ok())
  {
      std_msgs::String msg;
      std::stringstream ss;
      ss << "hello world " << count;
      count++;
      msg.data = ss.str();
      chatter_publisher.publish(msg);
      //log(Info,std::string("I sent: ")+msg.data);
      ros::spinOnce();
      loop_rate.sleep();
  }

  std::cout<<"ROS shutdown,proceding to clode the gui."  <<std::endl;
  Q_EMIT rosShutdown();
}

void QNode::forward() {

ros::Rate loop_rate(10);
if( ros::ok() )
{
    geometry_msgs::Twist msg;
    msg.linear.x = turtle_linear;
    msg.angular.z = 0.0;
    vel_publisher.publish(msg);
    log(Info,std::string("send linear velocity is")+std::to_string(msg.linear.x));
    ros::spinOnce();
    loop_rate.sleep();
}
}

void QNode::backward() {

ros::Rate loop_rate(1);
if( ros::ok() )
{
    geometry_msgs::Twist msg;
    msg.linear.x = -turtle_linear;
    msg.angular.z = 0.0;
    vel_publisher.publish(msg);
    log(Info,std::string("send linear velocity is")+std::to_string(msg.linear.x));
    ros::spinOnce();
    loop_rate.sleep();
}
}

void QNode::left() {

ros::Rate loop_rate(1);
if ( ros::ok() )
{
    geometry_msgs::Twist msg;
    msg.linear.x = 0.0;
    msg.angular.z = turtle_angular;
    vel_publisher.publish(msg);
    log(Info,std::string("send angular velocity is")+std::to_string(msg.angular.z));
    ros::spinOnce();
    loop_rate.sleep();
}
}

void QNode::right() {

ros::Rate loop_rate(1);
if ( ros::ok() )
{
    geometry_msgs::Twist msg;
    msg.linear.x = 0.0;
    msg.angular.z = -turtle_angular;
    vel_publisher.publish(msg);
    log(Info,std::string("send angular velocity is")+std::to_string(msg.angular.z));
    ros::spinOnce();
    loop_rate.sleep();
}
}

//void QNode::log( const LogLevel &level, const std_msgs::Float64 &msg) {
void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
        logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: "<< msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: "<< msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
        logging_model_msg << "[WARN] [" << ros::Time::now() << "]: "<< msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
        logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: "<< msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
        logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " <<"recevied valude is:"<< msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::log_sub(const LogLevel& level,const std::string& msg){// add
    logging_model_sub.insertRows(logging_model_sub.rowCount(),1);
    std::stringstream logging_model_msg;
    switch (level){
        case(Debug):{
                ROS_DEBUG_STREAM(msg);
                logging_model_msg <<"[DEBUG] ["<<ros::Time::now()<<"]:"<< msg;
                break;
        }
    case(Info):{
                ROS_INFO_STREAM(msg);
                logging_model_msg <<"[INFO] ["<<ros::Time::now()<<"]:"<< msg;
                break;
        }
    case(Warn):{
                ROS_WARN_STREAM(msg);
                logging_model_msg <<"[WARN] ["<< ros :: Time :: now()<<"]:"<< msg;
                break;
        }
    case(Error):{
                ROS_ERROR_STREAM(msg);
                logging_model_msg <<"[ERROR] ["<< ros :: Time :: now()<<"]:"<< msg;
                break;
        }
    case(Fatal):{
                ROS_FATAL_STREAM(msg);
                logging_model_msg <<"[FATAL] ["<< ros :: Time :: now()<<"]:"<< msg;
                break;
        }
    }
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model_sub.setData(logging_model_sub.index(logging_model_sub.rowCount()-1),new_row);
    Q_EMIT loggingUpdated_sub(); //用于重新调整滚动条
}


//callback send message
void QNode::myCallback_img(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {   /*change to CVImage*/
  cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
  //cv::imshow("gui_subscriber listener from node_a",cv_ptr->image);
  img = cv_ptr->image;
  Q_EMIT imageSignal(img);
  //cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
   ROS_ERROR("cv_bridge exception is %s", e.what());
      return;
  }

}

//void QNode::myCallback(const std_msgs::Float64 &message_holder)
//{
//  log(Info,message_holder);
//}

void QNode::Callback(const std_msgs::String &submsg)// add
{
    log_sub(Info,std::string("current linear velocity is: ")+ std::to_string(turtle_linear));
    log_sub(Info,std::string("current angular velocity is: ")+ std::to_string(turtle_angular));
}

}  // namespace gui_subscriber
