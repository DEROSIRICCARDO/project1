#include "velocity/velocity.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);

  velocity velocity_node;

  velocity_node.Prepare();

  velocity_node.RunPeriodically();

  velocity_node.Shutdown();

  return (0);
}
