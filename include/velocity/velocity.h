#ifndef VELOCITY_H
#define VELOCITY_H

#include "ros/ros.h"

#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/velocity.h"


#define NAME_OF_THIS_NODE "velocity"


class velocity  //header of the class
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Subscriber input_subscriber;
    ros::Publisher output_publisher;

    /* ROS topic callbacks */
    void input_MessageCallback(const sensor_msgs::JointState::ConstPtr& cmd_vel);
    
    /* dynamics reconfigure callback*/
    void int_method_callback(project1::integration_methodsConfig &config, uint32_t level);
     
    /*auxiliary functions*/
    void integrate(void);
    void publish(void);
    
    
    /* Node state variables */
    ros::Time current_time, past_time;
    
    int integration_method;
    
    double position;
  
    

  public:
    void Prepare(void);
    
    void RunPeriodically(void);
    
    void Shutdown(void);

};

#endif
