#include "velocity/velocity.h"

void velocity::Prepare(void)
{
    /* Retrieve parameters from ROS parameter server */
    //std::string FullParamName;

    
    /*FullParamName = ros::this_node::getName()+"/loopRate";
    if (false == Handle.getParam(FullParamName, loopRate))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    */
    
    
    /* ROS topics */
    this->input_subscriber = this->Handle.subscribe("/wheel_state", 1000, &velocity::input_MessageCallback, this);
    this->output_publisher = this->Handle.advertise<nav_msgs::velocity>("/velocity", 1000);
    

    /* Initialize node state */
    this->current_time = ros::Time::now();
    this->past_time = ros::Time::now();
    
    this->position_curr = 0.0;
    this->position_past = 0.0;
    double r = 0.07, lx = 0.2, ly = 0.169, T = 5, N = 42;

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void velocity::RunPeriodically(void)
{
    ROS_INFO("Node %s running.", ros::this_node::getName().c_str());

    // Wait other nodes start
    sleep(1.0);
    
    ros::spinOnce();
}

void velocity::Shutdown(void)
{

    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void velocity::input_MessageCallback(const sensor_msgs::JointState::ConstPtr& wheel_state)
{
    /* Read message and store information */
    this->position_curr[] = wheel_state->position[];
    
    integrate();
    
}

void velocity::integrate(void){
    
    this-> current_time = ros::Time::now();
    double Ts = (this->current_time - this->past_time).toSec();
    double ticks[] = this->position_curr[] - this->position_past[];
    double wheel_vel_fr[4];

    wheel_vel[] = ticks[] / Ts / N / T * 2 * 3.14;
    this->vel[] = r/4 * [1 1 1 1; -1 1 1 -1; -1/(lx+ly) 1/(lx+ly) -1/(lx+ly) 1/(lx+ly)] * 		wheel_vel[];


    this->past_time = this->current_time;
    this->position_past = this->position_curr;
}


 
