#ifndef TOBO_CORE_H
#define TOBO_CORE_H

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <ctime>
// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <tobo_planner/action_chain.h>


#include <naoqi_bridge_msgs/SetString.h>
#include <naoqi_bridge_msgs/SetFloat.h>

using namespace std;

class ToboCore
{
public:

    ToboCore(ros::NodeHandle& rosNode);
    std::vector<std::string> actions_command;
    std::map<std::string,int> actions_count;
    std::ostringstream logStream;
    
private: 

    ros::NodeHandle& rosNode;
    ros::Publisher tobo_speech_pub;
    ros::Publisher tobo_action_executed_pub;
    ros::Subscriber tobo_action_sub; //std_msgs::Header
    
    std_msgs::String speech_command;
    tobo_planner::action_chain action_executed_msg;
    
    std::vector<std::string> get_action_command;
    ros::Time get_action_stamp;
    std::vector<std::string> last_action_command;
    bool publish_speech;
    bool publish_request;
    string multimodal_command;
    string request_command;
    
    string rate =R"( \RSPD=65\ )";
    string pause =R"( \PAU=300\ )";
    string run = " ^call(ALBehaviorManager.runBehavior(\"ToDo\")) ";
    string stand = " ^call(ALRobotPosture.goToPosture(\"Stand\", 0.7)) ";
    
    std::map<std::string,std::vector<string>>::iterator it;
    std::map<std::string,std::vector<string>>::iterator itd;
    std::map<std::string,std::vector<string>> dialog;
    std::map<std::string,std::vector<string>> actions;
    std::map<std::string,std::vector<string>> request_dialog;
    
    naoqi_bridge_msgs::SetString srv_wakeup;
    naoqi_bridge_msgs::SetString srv_leds;
    naoqi_bridge_msgs::SetString srv_animation;
    naoqi_bridge_msgs::SetString srv_aliveness;
    naoqi_bridge_msgs::SetFloat srv_language;
    
    ros::ServiceClient service_language_client; //setup ALMotion wakeup rest
    ros::ServiceClient service_wakeup_client; //setup ALMotion wakeup rest
    ros::ServiceClient service_leds_client; //setup ALLeds on off reset [String]
    ros::ServiceClient service_animation_client; //setup ALAnimationPlayer runTag [String]
    ros::ServiceClient service_aliveness_client; //setup ALAutonomousLife setAutonomousAbilityEnabled [String bool]
        
    ros::Timer timer;
    
    void tobo_init(); // Init the robot state
    void tobo_config(); // Config the robot state
    void tobo_action_callback(const tobo_planner::action_chain& msg);
    void tobo_multimodal_output(std::string action_type, std::string activity);
    void tobo_request_output(std::string r_type, std::string opt_1, std::string opt_2);
    void replace_key(std::map<std::string,std::vector<string>>& d, std::string all, std::string to);
};

std::string currentDateTime();
#endif // TOBO_CORE_H