#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "ros/ros.h"

#include "geometry_msgs/TwistStamped.h"
#include <project1/Reset_Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <project1/integration_methodsConfig.h>

#define NAME_OF_THIS_NODE "odometer"


class odometer  //header of the class
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Subscriber input_subscriber;
    ros::Publisher output_publisher;
    
    /* Ros service*/
    ros::ServiceServer server;
    
    
    /* Parameters from ROS parameter server */
    double loopRate;
    

    /* ROS topic callbacks */
    void input_MessageCallback(const geometry_msgs::TwistStamped::ConstPtr& cmd_vel);
    
    /* ROS service callbacks */
    bool reset_callback(project1::Reset::Request &req, project1::Reset::Response &res);
    
    /* dynamics reconfigure callback*/
    bool int_method_callback(project1::integration_methodConfig &config, uint32_t level)
     
    /*auxiliary functions*/
    void integrate(void);
    void publish(void);
    
    
    /* Node state variables */
    ros::Time current_time, past_time;
    
    int integration_method;
    
    double x, y, theta;
    double vel_x, vel_y, omega;
  
    

  public:
    void Prepare(void);
    
    void RunPeriodically(void);
    
    void Shutdown(void);

};

#endif
