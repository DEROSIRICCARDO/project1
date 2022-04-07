#ifndef KYNEMATICS_H_
#define KYNEMATICS_H_

#include "ros/ros.h"

#include <std_msgs/Float64.h>



#define NAME_OF_THIS_NODE "kynematics_simulator"


class kyn_simulator  //header of the class
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Subscriber input_subscriber;
    ros::Publisher output_publisher, clock_publisher;
    
    /* Parameters from ROS parameter server */

    double l, w, r;

    /* ROS topic callbacks */
    void input_MessageCallback(const std_msgs::Float64::ConstPtr& msg);

    /* Estimator periodic task */
    void PeriodicTask(void);
    

    

  public:
    void Prepare(void);
    
    void RunPeriodically(void);
    
    void Shutdown(void);

};

#endif 