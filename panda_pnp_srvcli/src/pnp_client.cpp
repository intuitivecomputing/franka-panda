#include "ros/ros.h"
#include "panda_pnp_srvcli/PnpRequest.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "panda_pnp_client");
  // if (argv[1] != "long" || argv[1] != "short")
  // {
  //   std::cout<< argv[1] <<std::endl;
  //   ROS_INFO("options: long or short");
  //   return 1;
  // }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<panda_pnp_srvcli::PnpRequest>("panda_pnp_service");
  panda_pnp_srvcli::PnpRequest srv;
  srv.request.pipe = argv[1];
  if (client.call(srv))
  {
    ROS_INFO("pnp result: ", srv.response.result);
  }
  else
  {
    ROS_ERROR("Failed to call service panda_pnp_server");
    return 1;
  }

  return 0;
}