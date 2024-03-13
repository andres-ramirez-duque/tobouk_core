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
    std::string logs_path;
    std::string log_name;
    ros::param::get("logs_path", logs_path);
    ros::param::get("core_log", log_name);
    
    ROS_INFO("Initializing the Tobo_core node");
    ToboCore Tobo(_private_node);
    ROS_INFO("Tobo_core is ready");
    ros::spin();
    std::cout <<" ==== Tobo_Core_Node ended ===== \n";
    Tobo.logStream << "@[" << currentDateTime() << "]@ Tobo_Core_Node ended " << std::endl;
    for (auto itr = Tobo.actions_count.begin(); itr != Tobo.actions_count.end(); ++itr){
      
      std::cout <<"-- Number of Action " << itr->first.c_str() <<" executed: "<< itr->second <<"\n";
      Tobo.logStream << "@[" << currentDateTime() << "]@ -- Number of Action " << itr->first.c_str() <<" executed: "<< itr->second << std::endl;
    }
    std::string logContent = Tobo.logStream.str();
    std::cout << logs_path +"/"+ log_name << std::endl;
    std::ofstream logFile(logs_path +"/"+ log_name);
    if (!logFile.is_open()) {
        std::cerr << "Failed to open the log file" << std::endl;
        return 1;
    }
    logFile << logContent;
    logFile.close();
    return 0;
}
