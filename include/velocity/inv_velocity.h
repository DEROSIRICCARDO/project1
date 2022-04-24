#ifndef INV_VELOCITY_H
#define INV_VELOCITY_H

#include "ros/ros.h"

#include "geometry_msgs/TwistStamped.h"
#include "project1/Wrpm.h"


#define NAME_OF_THIS_NODE "inv_velocity"


class inv_velocity  //header of the class
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Subscriber input_subscriber;
    ros::Publisher output_publisher;

    /* ROS topic callbacks */
    void input_MessageCallback(const geometry_msgs::TwistStamped::ConstPtr& cmd_vel);
     
    /*auxiliary functions*/
    void compute_inv_velocity(void);
    void publish(void);
    
    
    /* Node state variables */
    
    double vel_x, vel_y, omega;
    double ome_fl, ome_fr, ome_rl, ome_rr;
    double l, w, r, T, N;
    

  public:
    void Prepare(void);
    
    void RunPeriodically(void);
    
    void Shutdown(void);

};

#endif
