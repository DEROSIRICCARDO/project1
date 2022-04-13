#include "odometry/odometry.h"
#include "nav_msgs/Odometry.h"
#include <project1/Reset_Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <project1/integration_methodsConfig.h>



void odometer::Prepare(void)
{
    /* Retrieve parameters from ROS parameter server */
    std::string FullParamName;

    
    FullParamName = ros::this_node::getName()+"/loopRate";
    if (false == Handle.getParam(FullParamName, loopRate))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    
    /* ROS topics */
    this->input_subscriber = this->Handle.subscribe("/cmd_vel", 1, &odometer::input_MessageCallback, this);
    this->output_publisher = this->Handle.advertise<nav_msgs::Odometry>("/odom", 1);
    
    /*Ros services*/
    this->server = this->Handle.advertiseService<project1::Reset_Odometry::Request, project1::Reset_Odometry::Response>("reset odometry",&odometer::reset_callback, this);
    
    /*dynamic reconfigure*/
    dynamic_reconfigure::Server<project1::integration_methodsConfig> dynServer;
    dynamic_reconfigure::Server<project1::integration_methodsConfig>::CallbackType &int_method_callback;
    dynServer.setCallback(&int_method_callback);
    

    /* Initialize node state */
    this->vel = 0.0;
    this->omega = 0.0;
    
    this->x = 0.0;
    this->y = 0.0;
    this->theta = 0.0;
    this->integration_method = 0.0;

    

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void odometer::RunPeriodically(void)
{
    ROS_INFO("Node %s running.", ros::this_node::getName().c_str());

    // Wait other nodes start
    sleep(1.0);
    
    ros::Rate loop_rate(this->loopRate);


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

void odometer::input_MessageCallback(const geometry_msgs::TwistedStamped::ConstPtr& msg)
{
    /* Read message and store information */
    this->vel_x = msg.linear.x;
    this->vel_y = msg.linear.y;
    this->omega = msg.angular.z;
    
    integrate();
    
}

bool odometer::reset_callback(project1::Reset_Odometry::Request  &req, project1::Reset_Odometry::Response &res){
    res.x = this->x;
    res.y = this->y;
    res.theta = this->theta;
    
    this->x = req.x;
    this->y = req.y;
    this->theta = req.theta;
    
    ROS_INFO("Request to reset the pose of the odometry to [%f,%f,%f]  - Responding with old pose: [%f,%f,%f]",
        (double)req.x, (double)req.y, (double)req.theta, (double)res.x, (double)res.y, (double)res.theta);
    
    return true;
}

void odometer::integrate(void){
    
    switch (integration_method) {
        case <#constant#>:
            <#statements#>
            break;
            
        case <#constant#>:
            <#statements#>
            break;
            
        
    }
    
    
    
}

void odometer::publish(void){
    nav_msgs::Odometry odom_msg;
    
    
    
    output_publisher.publish(odom_msg);
}

 
