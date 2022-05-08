#include "velocity/velocity.h"

void velocity::Prepare(void)
{
    /* Retrieve parameters from ROS parameter server */
    std::string FullParamName;
    std::string PartialName = "omnirobot";
    
    FullParamName = ros::this_node::getName()+"/LoopRate";
    if (false == Handle.getParam(FullParamName, LoopRate))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    
    /* FullParamName = PartialName+"/l";
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
    this->input_subscriber = this->Handle.subscribe("/wheel_states", 1, &velocity::input_MessageCallback, this);
    this->output_publisher = this->Handle.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1);
    this->output_publisher2 = this->Handle.advertise<std_msgs::Float64MultiArray>("/value", 1);

    /* dynamic parameters */

    dynamic_reconfigure::Server<project1::calibrationConfig>::CallbackType f;
    f = boost::bind(&velocity::robot_params_callback, this, _1, _2);
    dynServer.setCallback(f);
    

    /* Initialize node state */
    this->current_time = ros::Time::now();
    this->past_time = ros::Time::now();
    this->T = 5;
    
    for(int i = 0; i<4; i++){
        this->vel[i] = 0;
    };
    
    for(int i=0; i<4; i++){
        this->position_curr[i] = 0.0;
        this->position_past[i] = 0.0;
    };

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
};

void velocity::RunPeriodically(void)
{
    ROS_INFO("Node %s is running.", ros::this_node::getName().c_str());

    ros::Rate LoopRate(this->LoopRate);

    // Wait other nodes start
    sleep(1.0);

    // used to initialize ticks
    ros::spinOnce();
    for(int i=0; i<4; i++)
        this->position_past[i] = this->position_curr[i];

    sleep(0.5);

    while (ros::ok()){
        
        ros::spinOnce();
        velocity::compute_velocity();
        velocity::publish();
        LoopRate.sleep();
    };
};

void velocity::Shutdown(void)
{

    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
};

void velocity::input_MessageCallback(const sensor_msgs::JointState::ConstPtr& wheel_state)
{
    /* Read message and store information */
    for(int i=0; i<4; i++){
        this->position_curr[i] = wheel_state->position[i];
    };
};

void velocity::robot_params_callback(project1::calibrationConfig &config, uint32_t level){
    ROS_INFO("Reconfigure request: r=%f, l=%f, w=%f,  N=%f - Level %d",
             config.r, config.l, config.w, config.N, level);
    
    this->r = config.r;
    this->l = config.l;
    this->w = config.w;
    this->N = config.N;

    velocity::publish_value();
}

void velocity::compute_velocity(void){
    
    this-> current_time = ros::Time::now();
    double Ts = (this->current_time - this->past_time).toSec();
    double ticks[4];
    for (int i = 0; i < 4; i++){
        ticks[i] = this->position_curr[i] - this->position_past[i];
    };
    double wheel_vel[4];
    double mat[3][4] = {{r/4, r/4, r/4, r/4},
                     {-r/4, r/4, r/4, -r/4},
                     {-r/4/(l+w), r/4/(l+w), -r/4/(l+w), r/4/(l+w)}};

    
    for(int i=0; i<3; i++)
        vel[i] = 0;

    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 4; j++){
            wheel_vel[j] = ticks[j] / Ts / N / T * 2 * 3.14;
            this->vel[i] = this->vel[i] + mat[i][j] * wheel_vel[j];
        };
    };

    ROS_INFO("supposed velocity is [%f,%f,%f]", (double)this->vel[0], (double)this->vel[1], (double)this->vel[2]);


    this->past_time = this->current_time;

    for(int i = 0; i<4; i++){
        this->position_past[i] = this->position_curr[i];
    };
};
    
    
void velocity::publish(){
    geometry_msgs::TwistStamped cmd_vel;
    cmd_vel.header.stamp = this->current_time;
    cmd_vel.header.frame_id = "robot";
        
    cmd_vel.twist.linear.x = this->vel[0];
    cmd_vel.twist.linear.y = this->vel[1];
    cmd_vel.twist.linear.z = 0.0;
        
    cmd_vel.twist.angular.x = 0.0;
    cmd_vel.twist.angular.y = 0.0;
    cmd_vel.twist.angular.z = this->vel[2];
        
    output_publisher.publish(cmd_vel);
};
    
void velocity::publish_value(){

    std_msgs::Float64MultiArray value;
    value.data[0] = this->r;
    value.data[1] = this->l;
    value.data[2] = this->w;
    value.data[3] = this->N;
    
    output_publisher2.publish(value);
};