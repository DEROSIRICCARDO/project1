#include "velocity/inv_velocity.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);

  inv_velocity inv_velocity_node;

  inv_velocity_node.Prepare();

  inv_velocity_node.RunPeriodically();

  inv_velocity_node.Shutdown();

  return (0);
}
