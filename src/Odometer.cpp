#include "odometry/odometry.h"


#include <nav_msgs/Odometry.h>

void odometer::Prepare(void)
{
    /* Retrieve parameters from ROS parameter server */
    std::string FullParamName, PartialName = "omnirobot";

    // Model initial state
    FullParamName = PartialName+"/l";
    if (false == Handle.getParam(FullParamName, l))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/w";
    if (false == Handle.getParam(FullParamName, w))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    
    FullParamName = ros::this_node::getName()+"/r";
    if (false == Handle.getParam(FullParamName, r))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    
    /* ROS topics */
    this->input_subscriber = this->Handle.subscribe("/cmd_vel", 1, &odometer::input_MessageCallback, this);
    this->output_publisher = this->Handle.advertise<nav_msgs::Odometry>("/odom", 1);
    this->server = this->Handle.advertiseService<project1::Reset::Request, project1::Reset::Response>("reset odometry",&odometer::reset_callback, this);

    /* Initialize node state */
    this->vel = 0;
    this->omega = 0;
    
    this->x = 0;
    this->y = 0;
    this->theta = 0;

    

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void odometer::RunPeriodically(void)
{
    ROS_INFO("Node %s running.", ros::this_node::getName().c_str());

    // Wait other nodes start
    sleep(1.0);
    
    ros::Rate loop_rate(10);


    while (ros::ok())
    {

        ros::spinOnce();
        loop_rate.sleep();

    }
}

void odometer::Shutdown(void)
{

    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void odometer::input_MessageCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    /* Read message and store information */
    this->vel = msg->data.at(1);
    this->omega = msg->data(2);
    
    integrate();
    
}

void odometer::reset_callback(const nav_msgs::Odometry::ConstPtr& msg){
    this->x = msg->data(1);
    this->y = msg.data(2);
    this->theta = msg.data(3);
    
    odometer::publish();
}

void odometer::integrate(void){
    
}

void odometer::publish(void){
    
}

 
