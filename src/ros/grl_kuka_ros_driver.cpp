#include <grl/ros/KukaLBRiiwaROSPlugin.hpp>

using namespace grl::ros;


int main(int argc, char **argv) {

  ros::init(argc,argv,"kuka_lbr_ros_bridge");

  if(!ros::master::check())
  {
    std::cerr << "WARNING: roscore does not appear to be running\n";
  }

  std::shared_ptr<KukaLBRiiwaROSPlugin> plugin(std::make_shared<KukaLBRiiwaROSPlugin>());
  plugin->construct();

  ::ros::Rate rate(100);
  while (::ros::ok()) {
    ::ros::spinOnce();

    plugin->run_one();

    rate.sleep();
  }

  return 0;
}
