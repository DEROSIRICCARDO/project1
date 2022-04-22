#include "odometry/odometry.h"


void odometer::Prepare(void)
{
    /* ROS topics */
    this->input_subscriber = this->Handle.subscribe("/cmd_vel", 1, &odometer::input_MessageCallback, this);
    this->output_publisher = this->Handle.advertise<nav_msgs::Odometry>("/odom", 1);
    
    /*Ros services*/
    this->server = this->Handle.advertiseService("reset_odometry", &odometer::reset_callback, this);
    
    /* dynamic reconfigure*/
    
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
    
    ros::spin();
}

void odometer::Shutdown(void)
{

    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void odometer::input_MessageCallback(const geometry_msgs::TwistStamped::ConstPtr& cmd_vel)
{
    this->current_time = cmd_vel->header.stamp;
    
    odometer::integrate();
    odometer::publish();
    
    this->vel_x = cmd_vel->twist.linear.x;
    this->vel_y = cmd_vel->twist.linear.y;
    this->omega = cmd_vel->twist.angular.z;
    
    this->past_time = this->current_time;
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
        case 0:
            delta_x = vel_x*Ts*std::cos(theta) - vel_y*Ts*std::sin(theta);
            delta_y = vel_x*Ts*std::sin(theta) + vel_y*Ts*std::cos(theta);
            delta_theta = omega * Ts;
            break;
            
        case 1:
            delta_x = vel_x*Ts*std::cos(theta+omega*Ts/2) - vel_y*Ts*std::sin(theta+omega*Ts/2);
            delta_y = vel_y*Ts*std::sin(theta+omega*Ts/2) - vel_y*Ts*std::cos(theta+omega*Ts/2);
            delta_theta = omega * Ts;
            break;
            
    }
    
    this->x += delta_x;
    this->y += delta_y;
    this->theta += delta_theta;

    this->past_time = this->current_time;

    ROS_INFO("supposed pose is [%f,%f,%f], integrated with method %d", (double)this->x, (double)this->y, (double)this->theta, this->integration_method);
}

/*void odometer::publish(void){
    //We create a quaternion based on the the yaw of the robot
    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0,0,this->theta);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = this->current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = this->x;
    odom_trans.transform.translation.y = this->y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation.x = odom_quat.x();
    odom_trans.transform.rotation.y = odom_quat.y();
    odom_trans.transform.rotation.z = odom_quat.z();
    odom_trans.transform.rotation.w = odom_quat.w();

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = this->current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = this->x;
    odom.pose.pose.position.y = this->y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_trans.transform.rotation;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = this->vel_x;
    odom.twist.twist.linear.y = this->vel_y;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = this->omega;

    //publish the message
    output_publisher.publish(odom);
    
}

 
*/