#include "velocity/inv_velocity.h"

void inv_velocity::Prepare(void)
{
    /* Retrieve parameters from ROS parameter server 

    std::string FullParamName;
    std::string PartialName = "omnirobot";
    
    
    FullParamName = PartialName+"/l";
    if (false == Handle.getParam(FullParamName, l))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    
    FullParamName = PartialName+"/w";
    if (false == Handle.getParam(FullParamName, w))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    
    FullParamName = PartialName+"/r";
    if (false == Handle.getParam(FullParamName, r))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    
    FullParamName = PartialName+"/T";
    if (false == Handle.getParam(FullParamName, T))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    
    FullParamName = PartialName+"/N";
    if (false == Handle.getParam(FullParamName, N))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    
    */
    
    /* ROS topics */
    this->input_subscriber = this->Handle.subscribe("/cmd_vel", 1, &inv_velocity::input_MessageCallback, this);
    this->output_publisher = this->Handle.advertise<project1::Wrpm>("/wheels_rpm", 1);
    
    /* dynamic parameters */

    dynamic_reconfigure::Server<project1::calibrationConfig>::CallbackType f;
    f = boost::bind(&inv_velocity::robot_params_callback, this, _1, _2);
    dynServer.setCallback(f);

    /* Initialize node state */
    this->vel_x = 0;
    this->vel_y = 0;
    this->omega = 0;
    
    this->ome_fl = 0;
    this->ome_fr = 0;
    this->ome_rl = 0;
    this->ome_rr = 0;
    
    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
};

void inv_velocity::RunPeriodically(void)
{
    ROS_INFO("Node %s running.", ros::this_node::getName().c_str());

    // Wait other nodes start
    sleep(1.0);
    ros::spin();
   
};

void inv_velocity::Shutdown(void)
{
    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
};

void inv_velocity::input_MessageCallback(const geometry_msgs::TwistStamped::ConstPtr& cmd_vel)
{
    /* Read message and store information */
    this->vel_x = cmd_vel->twist.linear.x;
    this->vel_y = cmd_vel->twist.linear.y;
    this->omega = cmd_vel->twist.angular.z;

    inv_velocity::compute_inv_velocity();
    inv_velocity::publish();
};

void inv_velocity::robot_params_callback(project1::calibrationConfig &config, uint32_t level){
    ROS_INFO("Reconfigure request: r=%f, l=%f, w=%f, T=%d, N=%d - Level %d",
             config.r, config.l, config.w, config.T, config.N, level);
    
    this->r = config.r;
    this->l = config.l;
    this->w = config.w;
    this->T = config.T;
    this->N = config.N;
}

void inv_velocity::compute_inv_velocity(void){
    
    this->ome_fl = T/r * (vel_x - vel_y - omega * (l+w));
    this->ome_fr = T/r * (vel_x + vel_y + omega * (l+w));
    this->ome_rl = T/r * (vel_x + vel_y - omega * (l+w));
    this->ome_rr = T/r * (vel_x - vel_y + omega * (l+w));

    ROS_INFO("supposed velocities of the wheels are [%f,%f,%f,%f]", (double)this->ome_fl, (double)this->ome_fr, (double)this->ome_rl, (double)this->ome_rr);
};
    
    
void inv_velocity::publish(void){
    project1::Wrpm wheels_rpm;
     
    wheels_rpm.rpm_fl = this->ome_fl;
    wheels_rpm.rpm_fr = this->ome_fr;
    wheels_rpm.rpm_rr = this->ome_rr;
    wheels_rpm.rpm_rl = this->ome_rl;
    
    output_publisher.publish(wheels_rpm);
};
    


 
