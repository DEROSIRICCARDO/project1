#include "odometer.h"


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
    input_subscriber = Handle.subscribe("/robot_input", 1, &uni_kyn_simulator::input_MessageCallback, this);
    output_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/robot_state", 1);
    clock_publisher  = Handle.advertise<rosgraph_msgs::Clock>("/clock", 1);
    /* Initialize node state */

    

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void kyn_simulator::RunPeriodically(void)
{
    ROS_INFO("Node %s running.", ros::this_node::getName().c_str());

    // Wait other nodes start
    sleep(1.0);

    while (ros::ok())
    {
        PeriodicTask();

        ros::spinOnce();

        usleep(1000);
    }
}

void kyn_simulator::Shutdown(void)
{

    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void kyn_simulator::input_MessageCallback(const sensor_msgs::Float64::ConstPtr& msg)
{
    /* Read message and store information */
    simulator->setInputValues(msg->data.at(1), msg->data(2));
}

void kyn_simulator::PeriodicTask(void)
{
    


    /* Publish vehicle velocities */
    geometry_msgs::TwistStamped VehicleVelocityMsg;
    outputMsg.data.clear();
    outputMsg.data.push_back(v);
    outputMsg.data.push_back(omega);
    output_publisher.publish(VehicleStateMsg);

    
}

