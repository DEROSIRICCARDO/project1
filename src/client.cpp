#include "ros/ros.h"
#include "project1/Reset_Odometry.h"
#include "geometry_msgs/PoseStamped.h"


class Reset {
private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::ServiceClient client;
    project1::Reset_Odometry srv;
    
    geometry_msgs::PoseStamped true_pose;
    
    bool published;
    
    void countCallback(const geometry_msgs::PoseStamped::ConstPtr& true_pose) {
        
        this->true_pose = true_pose;
        
        resetFunction();
    }
    
    
    void resetFunction(){
        srv.request.x = this->true_pose.pose.point.x;
        srv.request.y = this->true_pose.pose.point.y;
        
        tf::Pose pose;
        tf::poseMsgToTF(true_pose.pose, pose);
        
        srv.request.theta = tf::getYaw(pose.getRotation());
        
        if (client.call(srv))
        {
          ROS_INFO("Old pose: [%f, %f, %f]", (double)srv.response.x, (double)srv.response.y, (double)srv.response.theta);
            this->published = true;
        }
        else
        {
          ROS_ERROR("Failed to call service reset_odometry, retry");
        }
    }
    
    
    
public:
    Reset() { // class constructor
      // all initializations here
        ros::NodeHandle n;
        this->sub = this->n.subscribe("/robot/pose", 1, &Reset::countCallback, this);
        this->client = n.serviceClient<project1:Reset_Odometry>("reset_odometry");
    }
    
    main_loop(){
        sleep(1.0);
        ros::Rate rate(20);
        
        while(!published){
            
            ros::spinOnce()
            rate.sleep();
            
        }
        
    }
    
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "reset_client");
    
    Reset my_reset;
    
    my_reset.main_loop();
    
    ROS_INFO("node reset_client is shutting down")

  return 0;
}
