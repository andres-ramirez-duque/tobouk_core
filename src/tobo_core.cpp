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
       
    tobo_init();
    ros::Rate hz(30);
}

void ToboCore::replace_key(std::map<std::string,std::vector<string>>& d, std::string all, std::string to){
    it = d.begin();
    while (it != d.end()) {
        std::size_t found = 0;
        for( std::size_t j = 0 ; j < it->second.size() ; ++j )
        {
            while (found!=std::string::npos)
            {
                
                found = it->second[j].find(all,found+1);
                if (found!=std::string::npos)
                    it->second[j].replace(found, all.length(), to);
            }
        }
        it++;
    }
}
void ToboCore::tobo_init()
{    
    publish_speech = false;
    publish_request = false;
    /*
    intro bye ivdescription song1 song2 dance1 dance2 leadmeditation taichi story quiz magic bowout strategyenforce
    */
    actions_command.push_back("intro");
    actions_command.push_back("ivdescription");
    actions_command.push_back("leadmeditation");
    actions_command.push_back("taichi");
    actions_command.push_back("dance");
    actions_command.push_back("song");
    actions_command.push_back("bye");
    actions_command.push_back("quiz");
    actions_command.push_back("story");
    actions_command.push_back("magic");
    actions_command.push_back("bowout");
    actions_command.push_back("strategyenforce");
    actions_command.push_back("ivdebrief_song");
    
    ros::param::get("/dialogs/intro", dialog[actions_command[0]]);
    ros::param::get("/dialogs/ivdescription", dialog[actions_command[1]]);
    ros::param::get("/dialogs/leadmeditation", dialog[actions_command[2]]);
    ros::param::get("/dialogs/taichi", dialog[actions_command[3]]);
    ros::param::get("/dialogs/dance", dialog[actions_command[4]]);
    ros::param::get("/dialogs/song", dialog[actions_command[5]]);
    ros::param::get("/dialogs/bye", dialog[actions_command[6]]);
    ros::param::get("/dialogs/quiz", dialog[actions_command[7]]);
    ros::param::get("/dialogs/story", dialog[actions_command[8]]);
    ros::param::get("/dialogs/magic", dialog[actions_command[9]]);
    ros::param::get("/dialogs/bowout", dialog[actions_command[10]]);
    ros::param::get("/dialogs/strategyenforce", dialog[actions_command[11]]);
    ros::param::get("/dialogs/ivdebrief_song", dialog[actions_command[12]]);

    ros::param::get("/actions/leadmeditation", actions[actions_command[2]]);
    ros::param::get("/actions/taichi", actions[actions_command[3]]);
    ros::param::get("/actions/dance", actions[actions_command[4]]);
    ros::param::get("/actions/song", actions[actions_command[5]]);
    ros::param::get("/actions/magic", actions[actions_command[9]]);
    ros::param::get("/actions/ivdebrief_song", actions[actions_command[12]]);
    
    ros::param::get("/request/preference/", request_dialog["preference"]);
    
    for (auto c : actions_command)
      actions_count.emplace(c, 0);
    
    string child_name;
    string hospital_name;
    ros::param::param<std::string>("/child_name", child_name, "Andres");
    ros::param::param<std::string>("/hospital_name", hospital_name, "Sick Kids hospital");
     
    replace_key(dialog, ",", pause);
    replace_key(dialog, "name", child_name);
    replace_key(dialog, "hospital", hospital_name);
    replace_key(request_dialog, ",", pause);
              
    srv_aliveness.request.data = "AutonomousBlinking 0";
    service_aliveness_client.waitForExistence(ros::Duration(30.0));
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
    srv_animation.request.data = " ";
      
    service_wakeup_client.call(srv_wakeup);
    service_leds_client.call(srv_leds);
    service_animation_client.call(srv_animation);
    service_aliveness_client.call(srv_aliveness);
}

void ToboCore::tobo_multimodal_output()
{
    multimodal_command = "";
    string activity = get_action_command[0];
    
    if (std::find_if(activity.begin(), activity.end(), ::isdigit) != activity.end())
        activity.erase(std::find_if(activity.begin(), activity.end(), ::isdigit));
        
    it = dialog.find(activity);
    if (it != dialog.end()){
      multimodal_command = rate;
      multimodal_command += it->second[actions_count[activity]];
      it = actions.find(activity);
      if (it != actions.end())
        multimodal_command += it->second[actions_count[activity]];
      publish_speech = true;
      actions_count[activity]++;
    }
}

void ToboCore::tobo_request_output()
{
    request_command = rate;
    string activity_1 = "Calm";
    string activity_2 = "Active";
    replace_key(request_dialog, "acti_1", activity_1);
    replace_key(request_dialog, "acti_2", activity_2);
    request_command += request_dialog["preference"][0];
    
    publish_request = true;
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
    string hierarchy_name = "/action_hierarchy/" + action_type;
    string hierarchy_value;
    ros::param::get(hierarchy_name, hierarchy_value);
    
    if (hierarchy_value.find("doactivity") != std::string::npos){
      if (action_type.find("ivdebrief") != std::string::npos){
        get_action_command = msg.parameters;
        get_action_command[0] = "ivdebrief_" + get_action_command[0];
      }else{
        get_action_command = msg.parameters;
      }
      ROS_INFO("String command: %s", get_action_command[0].c_str());
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
    }else if (hierarchy_value.find("qtypepreference") != std::string::npos){
      tobo_request_output();
    
      if (publish_request){
        nao_stat.message="Request command Sent";
        nao_value.key="Wrong  qtypepreference input";
        nao_value.value = "False";
        nao_stat.values.push_back(nao_value);
        nao_stat.level = diagnostic_msgs::DiagnosticStatus::OK;
      }
      else{
        nao_stat.message="Request command Sent";
        nao_value.key="Wrong  qtypepreference input";
        nao_value.value = "True";
        nao_stat.values.push_back(nao_value);
        nao_stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
      }
      action_executed_msg.execution_status = nao_stat;
      action_executed_msg.speech_cmd=string(request_command);  
      tobo_speech_pub.publish(action_executed_msg);
    }
    
    srv_aliveness.request.data = "BackgroundMovement 1";
    service_aliveness_client.call(srv_aliveness);
    publish_speech=false;
    publish_request=false;
}
