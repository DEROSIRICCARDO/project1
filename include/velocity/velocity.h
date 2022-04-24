#ifndef VELOCITY_H
#define VELOCITY_H

#include "ros/ros.h"

#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"


#define NAME_OF_THIS_NODE "velocity"


class velocity  //header of the class
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Subscriber input_subscriber;
    ros::Publisher output_publisher;

    /* ROS topic callbacks */
    void input_MessageCallback(const sensor_msgs::JointState::ConstPtr& wheel_state);
     
    /*auxiliary functions*/
    void compute_velocity(void);
    void publish(void);
    
    
    /* Node state variables */
    
    double LoopRate;
    ros::Time current_time, past_time;
    
    double position_curr[4], position_past[4];
    double vel[3];
    double l, w, r, T, N;
    

  public:
    void Prepare(void);
    
    void RunPeriodically(void);
    
    void Shutdown(void);

};

#endif
