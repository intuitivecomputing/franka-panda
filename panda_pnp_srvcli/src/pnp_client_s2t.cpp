#include "ros/ros.h"
#include "panda_pnp_srvcli/PnpRequest.h"
#include "std_msgs/String.h"
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <ros/package.h>





class SubscribeAndPublish
{

private:
  ros::NodeHandle n;
  // ros::Subscriber sub;
  // ros::Publisher pub;

public:

  ros::Subscriber sub;
  ros::Publisher pub;
  int pipe_count = 0;
  int joint_count = 0;
  int condition = 0;

  //constructor
  SubscribeAndPublish()
  {
    //Topic you want to publish
    // pub_ = n_.advertise<PUBLISHED_MESSAGE_TYPE>("/published_topic", 1);
    pub = n.advertise<std_msgs::String>("text_to_speech", 100);

    //Topic you want to subscribe
    // sub_ = n_.subscribe("/subscribed_topic", 1, &SubscribeAndPublish::callback, this);
    sub = n.subscribe("speech_text", 100, &SubscribeAndPublish::callback, this);
  }

  bool containsAny(const std::string& message, const std::vector<std::string>& keywords) {
    for (const auto& keyword : keywords) {
        if (message.find(keyword) != std::string::npos) {
            return true;
        }
    }
    return false;
  }

  bool containsAll(const std::string& message, const std::vector<std::string>& keywords) {
      for (const auto& keyword : keywords) {
          if (message.find(keyword) == std::string::npos) {
              return false;
          }
      }
      return true;
  }

  void callback(const std_msgs::String::ConstPtr& msg)
  {
    std::vector<std::string> short_keywords = {"short", "yellow"};
    std::vector<std::string> long_keywords = {"long", "lone", "green", "lom"};
    std::vector<std::string> pipe_keywords = {"pipe", "type", "pie", "pita", "eye", "high"};
    std::vector<std::string> joint_keywords = {"joint", "join"};
    std::vector<std::string> report_keywords = {"report", "reported", "reporting", "reports"};
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<panda_pnp_srvcli::PnpRequest>("panda_pnp_service");
    panda_pnp_srvcli::PnpRequest srv;
    srv.request.pipe = "";

    std::string message = msg->data.c_str();

    // turn all string into lowercase
    transform(message.begin(), message.end(), message.begin(), ::tolower);

        // 0 ~ 14
    int Functional = rand() % 14;
    // 14 ~ 25 (offset + rand() % range)
    int Social = 14 + (rand() % 12);
    std_msgs::String pub_msg;


    if (containsAny(message, short_keywords) && containsAny(message, pipe_keywords)) {
    
      srv.request.pipe = "short";
      pub_msg.data = "p";

      ROS_INFO("The message is: [%s]", msg->data.c_str());
      ROS_INFO("The requested type is: [%s]", srv.request.pipe.c_str());

      
      if (condition == 0){
        pub_msg.data.append(std::to_string(Functional));
      }
      else{
        pub_msg.data.append(std::to_string(Social));
      }
      pub.publish(pub_msg);

      if (client.call(srv))
      {
        ROS_INFO("pnp result: short pipe delivered");


        pipe_count++;
      }
      else
      {
        ROS_ERROR("Failed to call service panda_pnp_server");
      }
    }
    if (containsAny(message, long_keywords) && containsAny(message, pipe_keywords)) {
      srv.request.pipe = "long";
      pub_msg.data = "p";

      ROS_INFO("The message is: [%s]", msg->data.c_str());
      ROS_INFO("The requested type is: [%s]", srv.request.pipe.c_str());

      if (condition == 0){
        pub_msg.data.append(std::to_string(Functional));
      }
      else{
        pub_msg.data.append(std::to_string(Social));
      }

      pub.publish(pub_msg);

      if (client.call(srv))
      {
        ROS_INFO("pnp result: long pipe delivered");

        pipe_count++;
      }
      else
      {
        ROS_ERROR("Failed to call service panda_pnp_server");
      }
    }
    if (containsAny(message, joint_keywords) && containsAny(message, report_keywords)) {
      srv.request.pipe = "joint";
      pub_msg.data = "FAULTY";

      ROS_INFO("The message is: [%s]", msg->data.c_str());
      pub.publish(pub_msg);
      if (client.call(srv))
      {
        ROS_INFO("pnp result: joint delivered");
        // joint_count++;
      }
      else
      {
        ROS_ERROR("Failed to call service panda_pnp_server");
      }
    }
  }

};//End of class SubscribeAndPublish




int main(int argc, char **argv)
{
  ros::init(argc, argv, "panda_pnp_client");
  // if (argv[1] != "long" || argv[1] != "short")
  // {
  //   std::cout<< argv[1] <<std::endl;
  //   ROS_INFO("options: long or short");
  //   return 1;
  // }
  srand (time(NULL));
  SubscribeAndPublish SAP;
  std::string main_share_dir = ros::package::getPath("panda_pnp_srvcli");

  std::ifstream config_file(main_share_dir + "/json/poses.json");
  if (!config_file.is_open())
  {
      std::cout<<"failed to read config json file"<<std::endl;
      return false;
  }
  Json::Reader reader;
  Json::Value pose_config;
  reader.parse(config_file, pose_config);
  SAP.condition = pose_config["condition"].asFloat();

  // ros::NodeHandle n;
  // ros::Subscriber sub = n.subscribe("speech_text", 100, Callback);
  // ros::Publisher pub = n.advertise<std_msgs::String>("text_to_speech", 1000);

  if(SAP.pipe_count==3){
    std_msgs::String msg;
    msg.data = "r0";
    SAP.pub.publish(msg);
  }
  else if(SAP.pipe_count==11){
    std_msgs::String msg;
    msg.data = "r1";
    SAP.pub.publish(msg);
  }
  else if(SAP.pipe_count==17){
    std_msgs::String msg;
    msg.data = "r2";
    SAP.pub.publish(msg);
  }
  else if(SAP.pipe_count==24){
    std_msgs::String msg;
    msg.data = "r3";
    SAP.pub.publish(msg);
  }
  ros::spin();
  return 0;
}