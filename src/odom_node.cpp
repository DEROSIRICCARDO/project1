#include "odometer.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);

  odometer odom_node;

  odom_node.Prepare();

  odom_node.RunPeriodically();

  odom_node.Shutdown();

  return (0);
}


