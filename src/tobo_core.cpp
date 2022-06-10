#include "tobo_core.hpp"


using namespace std;

ToboCore::ToboCore(ros::NodeHandle& rosNode, std::vector<string> &arguments) :
            rosNode(rosNode)
{
    tobo_action_sub = rosNode.subscribe("/next_action", 1000,&ToboCore::tobo_action_callback,this);
    
    tobo_speech_pub = rosNode.advertise<tobo_planner::action_chain>("/animated_speech", 10);
    //tobo_action_executed_pub = rosNode.advertise<tobo_planner::action_chain>("/action_executed", 10);

    service_wakeup_client = rosNode.serviceClient<naoqi_bridge_msgs::SetString>("/naoqi_driver/set_wakeup");
    service_leds_client = rosNode.serviceClient<naoqi_bridge_msgs::SetString>("/naoqi_driver/set_leds");
    service_animation_client = rosNode.serviceClient<naoqi_bridge_msgs::SetString>("/naoqi_driver/set_animation");
    service_aliveness_client = rosNode.serviceClient<naoqi_bridge_msgs::SetString>("/naoqi_driver/set_aliveness");
    service_language_client = rosNode.serviceClient<naoqi_bridge_msgs::SetFloat>("/naoqi_driver/set_language");
    
    while(!ros::param::has("/nao_state"))
    {
    ROS_INFO("Waiting for nao_state initialization");
    }
    
    tobo_init();
    ros::Rate hz(30);
}

void ToboCore::tobo_init()
{    
    ros::param::set("/nao_state", "bussy");
    publish_speech = false;
    
    actions_command.push_back("intronau");
    actions_command.push_back("ivdescription");
    actions_command.push_back("leadmeditation");
    actions_command.push_back("taichi");
    actions_command.push_back("dance");
    actions_command.push_back("quiz");
    actions_command.push_back("reward");
    actions_command.push_back("byenau");
    
    ros::param::get("/dialog/intronau", dialog[actions_command[0]]);
    ros::param::get("/dialog/ivdescription", dialog[actions_command[1]]);
    ros::param::get("/dialog/leadmeditation", dialog[actions_command[2]]);
    ros::param::get("/dialog/taichi", dialog[actions_command[3]]);
    ros::param::get("/dialog/dance", dialog[actions_command[4]]);
    ros::param::get("/dialog/quiz", dialog[actions_command[5]]);
    ros::param::get("/dialog/reward", dialog[actions_command[6]]);
    ros::param::get("/dialog/byenau", dialog[actions_command[7]]);
    ros::param::get("/actions/leadmeditation", actions[actions_command[2]]);
    ros::param::get("/actions/taichi", actions[actions_command[3]]);
    ros::param::get("/actions/dance", actions[actions_command[4]]);
    ros::param::get("/actions/reward", actions[actions_command[6]]);
     
    for(int i = 0; i < actions_command.size(); i++)
    {
        it = dialog.find(actions_command[i]);
        std::size_t found = 0;
        for( std::size_t j = 0 ; j < it->second.size() ; ++j )
        {
            while (found!=std::string::npos)
            {
                found = it->second[j].find(",",found+1);
                if (found!=std::string::npos)
                    it->second[j].replace(found, 1, pause);
            }
        }
    }
          
    srv_aliveness.request.data = "AutonomousBlinking 0";
    service_aliveness_client.waitForExistence();
    service_aliveness_client.call(srv_aliveness);
    
    srv_language.request.data = 50.0;
    service_language_client.call(srv_language);
    
    srv_wakeup.request.data = "wakeUp";
    service_wakeup_client.call(srv_wakeup);
    
    srv_aliveness.request.data = "BasicAwareness 0";
    service_aliveness_client.call(srv_aliveness);
    
    srv_leds.request.data = "reset AllLeds";
    service_leds_client.call(srv_leds);
    
    srv_aliveness.request.data = "ListeningMovement 0";
    service_aliveness_client.call(srv_aliveness);
    
    srv_animation.request.data = " ";
    service_animation_client.call(srv_animation);
       
    //srv_aliveness.request.data = "SpeakingMovement 0";
    //service_aliveness_client.call(srv_aliveness);
}

void ToboCore::tobo_config()
{   
    ros::param::set("/nao_state", "bussy");
    srv_animation.request.data = " ";
      
    service_wakeup_client.call(srv_wakeup);
    service_leds_client.call(srv_leds);
    service_animation_client.call(srv_animation);
    service_aliveness_client.call(srv_aliveness);
    ros::param::set("/nao_state", "available");
}

void ToboCore::tobo_multimodal_output()
{
    multimodal_command = rate;
    
    int command_type = 100;
    for(int i = 0; i < actions_command.size(); i++) {
        if (get_action_command.find(actions_command[i]) != std::string::npos)
        {
            command_type = i;
            ROS_INFO("%s", actions_command[i].c_str());
        }
    }
    if (command_type != 100)
    {
        it = dialog.find(actions_command[command_type]);
        if (it != dialog.end())
            multimodal_command += it->second[0];
            
        it = actions.find(actions_command[command_type]);
        if (it != actions.end())
            multimodal_command += it->second[0];
        multimodal_command += stand;
        publish_speech = true;
    }else{
        multimodal_command = "";
        publish_speech = false;
        ROS_INFO("Transition action or bad get_action_input");
    }
    //ROS_INFO("String command: %s", multimodal_command.c_str());
}
void ToboCore::tobo_action_callback(const tobo_planner::action_chain& msg)
{
    diagnostic_msgs::DiagnosticStatus nao_stat;
    nao_stat.name="NAO_executed_status";
    diagnostic_msgs::KeyValue nao_value;
    
    
    action_executed_msg.caller_id = 1;
    action_executed_msg.plan_step = msg.plan_step;
    string action_type = msg.action_type;
    action_executed_msg.parameters = msg.parameters;
    
    if (action_type.find("doactivity") != std::string::npos){
      get_action_command = msg.parameters[0];
      tobo_multimodal_output();
    
      if (publish_speech){
        nao_stat.message="Speech command Sent";
        nao_value.key="Wrong get_action_input";
        nao_value.value = "False";
        nao_stat.values.push_back(nao_value);
        nao_stat.level = diagnostic_msgs::DiagnosticStatus::OK;
      }
      else{
        nao_stat.message="Speech command Sent";
        nao_value.key="Wrong get_action_input";
        nao_value.value = "True";
        nao_stat.values.push_back(nao_value);
        nao_stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
      }
      action_executed_msg.execution_status = nao_stat;
      action_executed_msg.speech_cmd=string(multimodal_command);  
      tobo_speech_pub.publish(action_executed_msg);
    }
    srv_aliveness.request.data = "BackgroundMovement 1";
    service_aliveness_client.call(srv_aliveness);
    publish_speech=false;
}
/*
bool ToboCore::reset_service(meri_local::Reset::Request  &req, meri_local::Reset::Response &res)
{
    if (req.a == 2){
    clnf_model.Reset();
    }
    tracker_init=false;
    ROS_INFO("Tracker reseted");
    res.b=req.a;
    return true;
}
"\\style=joyful\\ \\rspd=80\\ Hello my friends \\pau=1000\\ \\vol=30\\how are you ^start(animations/Stand/Gestures/Hey_1) this is the number "
[{"name":"AutonomousBlinking","enabled":true,"running":true},{"name":"BackgroundMovement","enabled":true,"running":true},{"name":"BasicAwareness","enabled":false,"running":false},{"name":"ListeningMovement","enabled":false,"running":false},{"name":"SpeakingMovement","enabled":true,"running":false}]
*/
