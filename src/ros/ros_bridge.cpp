#include <grl/ros/KukaLBRiiwaROSPlugin.hpp>

using namespace grl::ros;


int main(int argc, char **argv) {

  ros::init("kuka_lbr_ros_bridge",argc,argv);

  KukaLBRiiwaROSPlugin plugin;
  plugin.construct();


  plugin.run_one();

  return 0;
}
