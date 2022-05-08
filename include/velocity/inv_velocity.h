#ifndef INV_VELOCITY_H
#define INV_VELOCITY_H

#include "ros/ros.h"

#include "geometry_msgs/TwistStamped.h"
#include "project1/Wrpm.h"
#include "std_msgs/Float64MultiArray.h"
#include <dynamic_reconfigure/server.h>
#include <project1/calibrationConfig.h>

#define NAME_OF_THIS_NODE "inv_velocity"


class inv_velocity  //header of the class
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Subscriber input_subscriber2;
    ros::Subscriber input_subscriber;
    ros::Publisher output_publisher;

    /* Dynamic reconfigure server*/
    dynamic_reconfigure::Server<project1::calibrationConfig> dynServer;

    /* ROS topic callbacks */
    void input_MessageCallback(const geometry_msgs::TwistStamped::ConstPtr& cmd_vel);
    void input_ValueParameter(const std_msgs::Float64MultiArray::ConstPtr& value);

    /* dynamics reconfigure callback*/
    /*void robot_params_callback(project1::calibrationConfig &config, uint32_t level);*/
     
    /*auxiliary functions*/
    void compute_inv_velocity(void);
    void publish(void);
    
    
    /* Node state variables */
    
    double vel_x, vel_y, omega;
    double ome_fl, ome_fr, ome_rl, ome_rr;
    double l, w, r;
    int T;
    double N;

  public:
    void Prepare(void);
    
    void RunPeriodically(void);
    
    void Shutdown(void);

};

#endif
