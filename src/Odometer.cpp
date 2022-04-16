#include "odometry/odometry.h"





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
    this->server = this->Handle.advertiseService("reset odometry", &odometer::reset_callback, this);
    
    /*dynamic reconfigure*/
    dynamic_reconfigure::Server<project1::integration_methodsConfig> dynServer;
    dynamic_reconfigure::Server<project1::integration_methodsConfig>::CallbackType f;
    f = boost::bind(&odometer::int_method_callback, this, _1, _2);
    dynServer.setCallback(f);
    

    /* Initialize node state */
    this->current_time = ros::Time::now();
    this->past_time = ros::Time::now();
    
    this->x = 0.0;
    this->y = 0.0;
    this->theta = 0.0;
    
    this->vel_x = 0.0;
    this->vel_y = 0.0;
    this->omega = 0.0;
    
    this->integration_method = 0;

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

void odometer::input_MessageCallback(const geometry_msgs::TwistStamped::ConstPtr& cmd_vel)
{
    /* Read message and store information */
    this->vel_x = cmd_vel->twist.linear.x;
    this->vel_y = cmd_vel->twist.linear.y;
    this->omega = cmd_vel->twist.angular.z;
    
    integrate();
    
}

bool odometer::reset_callback(project1::Reset_Odometry::Request&  req, project1::Reset_Odometry::Response& res){
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

void odometer::int_method_callback(project1::integration_methodsConfig &config, uint32_t level){
    ROS_INFO("Reconfigure request: %d - Level %d",
             config.integration_method, level);
    
    this->integration_method = config.integration_method;
}

void odometer::integrate(void){
    
    this-> current_time = ros::Time::now();
    double Ts = (this->current_time - this->past_time).toSec();
    double delta_x, delta_y, delta_theta;
    
    switch (this->integration_method) {
        case '0':
            delta_x = vel_x*Ts*std::cos(theta) - vel_y*Ts*std::sin(theta);
            delta_y = vel_x*Ts*std::sin(theta) + vel_y*Ts*std::cos(theta);
            delta_theta = omega * Ts;
            break;
            
        case '1':
            delta_x = vel_x*Ts*std::cos(theta+omega*Ts/2) - vel_y*Ts*std::sin(theta+omega*Ts/2);
            delta_y = vel_y*Ts*std::sin(theta+omega*Ts/2) - vel_y*Ts*std::cos(theta+omega*Ts/2);
            delta_theta = omega * Ts;
            break;
            
    }
    
    this->x += delta_x;
    this->y += delta_y;
    this->theta += delta_theta;
    
    this->past_time = this->current_time;
}

void odometer::publish(void){
    nav_msgs::Odometry odom_msg;
    
    
    
    output_publisher.publish(odom_msg);
}

 
