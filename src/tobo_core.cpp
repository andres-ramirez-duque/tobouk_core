#include "tobo_core.hpp"

using namespace std;

std::string currentDateTime() {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    char buffer[20];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H:%M:%S", std::localtime(&time));
    return buffer;
}

ToboCore::ToboCore(ros::NodeHandle& rosNode) :
            rosNode(rosNode)
{
    tobo_action_sub = rosNode.subscribe("/next_action", 1,&ToboCore::tobo_action_callback,this);
    
    tobo_speech_pub = rosNode.advertise<tobo_planner::action_chain>("/animated_speech", 1);
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
   /*
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
    
    actions_command.push_back("bruno");
    actions_command.push_back("look");
    actions_command.push_back("bam");
    actions_command.push_back("shakeit");
    actions_command.push_back("macarena");
    actions_command.push_back("armdance");
    actions_command.push_back("happy");
    actions_command.push_back("calmdown");
    actions_command.push_back("babyshark");
    actions_command.push_back("oldmacdonald");
    actions_command.push_back("fivemonkeys");
    actions_command.push_back("happyandyouknow");
    actions_command.push_back("saxophone");
    actions_command.push_back("guitar");
    actions_command.push_back("belly");
    actions_command.push_back("cookies");
    actions_command.push_back("chocolate");
    actions_command.push_back("twinklestar");
    */
    ros::param::get("/dialogs/intro", dialog["intro"]);
    ros::param::get("/dialogs/ivdescription", dialog["ivdescription"]);
    ros::param::get("/dialogs/chocolate", dialog["chocolate"]);
    ros::param::get("/dialogs/cookies", dialog["cookies"]);
    ros::param::get("/dialogs/belly", dialog["belly"]);
    ros::param::get("/dialogs/taichi", dialog["taichi"]);
    ros::param::get("/dialogs/twinklestar", dialog["twinklestar"]);
    ros::param::get("/dialogs/dance", dialog["dance"]);
    ros::param::get("/dialogs/bye", dialog["bye"]);
    ros::param::get("/dialogs/quiz", dialog["quiz"]);
    ros::param::get("/dialogs/story", dialog["story"]);
    ros::param::get("/dialogs/magic", dialog["magic"]);
    ros::param::get("/dialogs/bowout", dialog["bowout"]);
    ros::param::get("/dialogs/strategyenforce", dialog["strategyenforce"]);
    ros::param::get("/dialogs/ivdebrief", dialog["ivdebrief"]);
    
    ros::param::get("/actions/belly", actions["belly"]);
    ros::param::get("/actions/cookies", actions["cookies"]);
    ros::param::get("/actions/chocolate", actions["chocolate"]);
    ros::param::get("/actions/taichi", actions["taichi"]);
    ros::param::get("/actions/twinklestar", actions["twinklestar"]);
    ros::param::get("/actions/bruno", actions["bruno"]);
    ros::param::get("/actions/look", actions["look"]);
    ros::param::get("/actions/bam", actions["bam"]);
    ros::param::get("/actions/shakeit", actions["shakeit"]);
    ros::param::get("/actions/macarena", actions["macarena"]);
    ros::param::get("/actions/armdance", actions["armdance"]);
    ros::param::get("/actions/happy", actions["happy"]);
    ros::param::get("/actions/calmdown", actions["calmdown"]);
    ros::param::get("/actions/babyshark", actions["babyshark"]);
    ros::param::get("/actions/oldmacdonald", actions["oldmacdonald"]);
    ros::param::get("/actions/fivemonkeys", actions["fivemonkeys"]);
    ros::param::get("/actions/happyandyouknow", actions["happyandyouknow"]);
    ros::param::get("/actions/saxophone", actions["saxophone"]);
    ros::param::get("/actions/guitar", actions["guitar"]);
    ros::param::get("/actions/asitwas", actions["asitwas"]);
    ros::param::get("/actions/whatdoyoumean", actions["whatdoyoumean"]);
    ros::param::get("/actions/idontcare", actions["idontcare"]);
    ros::param::get("/actions/blindinglights", actions["blindinglights"]);
    ros::param::get("/actions/story", actions["story"]);
    
    ros::param::get("/request/preference", request_dialog["preference"]);
    ros::param::get("/request/activity_preference", request_dialog["activity_preference"]);
    actions_count.insert({"preference",0});
    actions_count.insert({"activity_preference",0});

    //for (auto c : actions_command)
    //  actions_count.emplace(c, 0);
    
    string child_name;
    string hospital_name;
    ros::param::param<std::string>("/child_name", child_name, "Andres");
    ros::param::param<std::string>("/hospital_name", hospital_name, "SickKidshospital");
    
    replace_key(request_dialog, ",", pause);
    replace_key(request_dialog, "<name>", child_name);

    replace_key(dialog, ",", pause);
    replace_key(dialog, "<name>", child_name);
    replace_key(dialog, "<hospitalname>", hospital_name);
    
    srv_aliveness.request.data = "AutonomousBlinking 0";
    
    if (! service_aliveness_client.waitForExistence(ros::Duration(30.0))){
      ROS_INFO("Stoping Node for NaoQi inactivity");
      logStream << "@[" << currentDateTime() << "]@ Stoping Node for NaoQi inactivity" << std::endl;
      ros::shutdown();
    }
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

void ToboCore::tobo_multimodal_output(std::string action_type, std::string activity)
{
    multimodal_command = "";
    multimodal_command = rate;
    actions_count.insert({activity, 0});
    if (action_type.find("ivdebrief") != std::string::npos){
      it = dialog.find(action_type);  
    }else{
      it = dialog.find(activity);
    }
    if (it != dialog.end()){
      multimodal_command += it->second[actions_count[activity]];
    }else{
      itd = dialog.find("dance");
      actions_count.insert({"dance", 0});
      if (itd != dialog.end()){
        multimodal_command += itd->second[actions_count["dance"]];
        actions_count["dance"]++;
      }
    }
    it = actions.find(activity);
    if (it != actions.end()){
      multimodal_command += it->second[actions_count[activity]];
    }
    publish_speech = true;
    actions_count[activity]++;
    ROS_INFO("Output of: %s", activity.c_str());
}

void ToboCore::tobo_request_output(std::string r_type, std::string opt_1, std::string opt_2)
{
    request_command = rate;
    if(r_type.find("qtypepreference") != std::string::npos){
      request_command += request_dialog["preference"][actions_count["preference"]];
      actions_count["preference"]++;
      publish_request = true;
    }else{
      std::vector<string> activity_1 = actions[opt_1];
      std::vector<string> activity_2 = actions[opt_2];
      std::map<std::string,std::vector<string>> temp_request;
      std::vector<string> temp_dialog;
      temp_dialog.push_back(request_dialog["activity_preference"][actions_count["activity_preference"]]);
      temp_request["activity_preference"] = temp_dialog;
      replace_key(temp_request, "act_1", activity_1[1]);
      replace_key(temp_request, "act_2", activity_2[1]);
      request_command += temp_request["activity_preference"][0];
      actions_count["activity_preference"]++;
      publish_request = true;
    }
    ROS_INFO("Output of: %s", r_type.c_str());
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
    logStream << "@[" << currentDateTime() << "]@ action type received: " << action_type << " Index: " << msg.plan_step <<std::endl;
    ROS_INFO("Action type received: %s, Index: %d", action_type.c_str(), msg.plan_step);

    if (hierarchy_value.find("doactivity") != std::string::npos){
      logStream << "@[" << currentDateTime() << "]@ action doactivity found " << std::endl;
      get_action_command = msg.parameters;
      ROS_INFO("String command: %s", get_action_command[0].c_str());
      logStream << "@[" << currentDateTime() << "]@ action parameters " << get_action_command[0].c_str() << std::endl;
      tobo_multimodal_output(action_type, get_action_command[0]);
    
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
      logStream << "@[" << currentDateTime() << "]@ Nao speech cmd published " << action_executed_msg.speech_cmd << std::endl;  
      tobo_speech_pub.publish(action_executed_msg);
    }else if (hierarchy_value.find("qtypepreference") != std::string::npos){
      ROS_INFO("Action type qtypepreference started");
      tobo_request_output("qtypepreference","None", "None");
      logStream << "@[" << currentDateTime() << "]@ action qtypepreference found " << std::endl;
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
      logStream << "@[" << currentDateTime() << "]@ Nao speech cmd published " << action_executed_msg.speech_cmd << std::endl;  
      tobo_speech_pub.publish(action_executed_msg);
    }else if (hierarchy_value.find("qactivitypreference") != std::string::npos){
      ROS_INFO("Action type qactivitypreference started");
      tobo_request_output("qactivitypreference", msg.parameters[0], msg.parameters[1]);
      logStream << "@[" << currentDateTime() << "]@ action qactivitypreference found " << std::endl;
      if (publish_request){
        nao_stat.message="Request command Sent";
        nao_value.key="Wrong  qactivitypreference input";
        nao_value.value = "False";
        nao_stat.values.push_back(nao_value);
        nao_stat.level = diagnostic_msgs::DiagnosticStatus::OK;
      }
      else{
        nao_stat.message="Request command Sent";
        nao_value.key="Wrong  qactivitypreference input";
        nao_value.value = "True";
        nao_stat.values.push_back(nao_value);
        nao_stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
      }
      action_executed_msg.execution_status = nao_stat;
      action_executed_msg.speech_cmd=string(request_command);
      logStream << "@[" << currentDateTime() << "]@ Nao speech cmd published " << action_executed_msg.speech_cmd << std::endl;  
      tobo_speech_pub.publish(action_executed_msg);
    }
    
    srv_aliveness.request.data = "BackgroundMovement 1";
    service_aliveness_client.call(srv_aliveness);
    publish_speech=false;
    publish_request=false;
}
