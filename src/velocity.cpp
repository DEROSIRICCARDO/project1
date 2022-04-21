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
    
    double i = 0;
    while(i<4){
        this->position_curr[i] = 0.0;
        this->position_past[i] = 0.0;
        i = i + 1;
    }
    double r = 0.07, lx = 0.2, ly = 0.169, T = 5, N = 42;

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void velocity::RunPeriodically(void)
{
    ROS_INFO("Node %s running.", ros::this_node::getName().c_str());

    // Wait other nodes start
    sleep(1.0);

    ros::Rate r(10);

    while (ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
}

void velocity::Shutdown(void)
{

    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void velocity::input_MessageCallback(const sensor_msgs::JointState::ConstPtr& wheel_state)
{
    /* Read message and store information */
    double i = 0;
    while(i<4){
        this->position_curr[i] = wheel_state->position[i];
        i = i + 1;
    }
    
    compute_velocity();
}

void velocity::compute_velocity(void){
    
    this-> current_time = ros::Time::now();
    double Ts = (this->current_time - this->past_time).toSec();
    double i = 0;
    while (i < 4){
        double ticks[i] = this->position_curr[i] - this->position_past[i];
        i = i + 1;
    }
    double wheel_vel[4], mat[3][4] = r/4 * [1 1 1 1; -1 1 1 -1; -1/(lx+ly) 1/(lx+ly) -1/(lx+ly) 1/(lx+ly)];

    double i = 0;
    while (i < 3){
        double j = 0;
        while (j < 4){
            wheel_vel[j] = ticks[j] / Ts / N / T * 2 * 3.14;
            this->vel[i] = this->vel[i] + mat[i][j] * wheel_vel[j];
            j = j + 1;
        }
        i = i + 1;
    }

    ROS_INFO("supposed velocity is [%f,%f,%f]", (double)this->vel[0], (double)this->vel[1], (double)this->vel[2]);


    this->past_time = this->current_time;
    this->position_past = this->position_curr;
}