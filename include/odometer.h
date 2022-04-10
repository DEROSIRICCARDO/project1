#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "ros/ros.h"

#include <std_msgs/Float64.h>

#define NAME_OF_THIS_NODE "odometer"


class odometer  //header of the class
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Subscriber input_subscriber;
    ros::Publisher output_publisher, clock_publisher;
    
    /* Parameters from ROS parameter server */
    

    /* ROS topic callbacks */
    void input_MessageCallback(const std_msgs::Float64::ConstPtr& msg);

    /* Estimator periodic task */
    void PeriodicTask(void);
    
    /* Node state variables */
  
    

  public:
    void Prepare(void);
    
    void RunPeriodically(void);
    
    void Shutdown(void);

};

#endif