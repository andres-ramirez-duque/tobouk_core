#include <string>
#include <ros/ros.h>

#include "tobo_core.hpp"

using namespace std;

int main(int argc, char* argv[])
{
    //ROS initialization
    ros::init(argc, argv, "tobo_core_node");
    ros::NodeHandle _private_node("~");
    ros::Rate hz(30);
    // load parameters
    string arguments;
    
    _private_node.getParam("model_loc", arguments);
    
    stringstream ss(arguments);
    istream_iterator<string> begin(ss);
    istream_iterator<string> end;
    std::vector<string> v_arguments(begin, end);
    
    ROS_INFO_STREAM("Initializing the Tobo_core node");
    ToboCore Tobo(_private_node,v_arguments);
    ROS_INFO("Tobo_core is ready");
    ros::spin();

    return 0;
}
