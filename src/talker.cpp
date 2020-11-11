/** @file talker.cpp
* @brief Publisher file 
* @Copyright MIT License 2020 Vishnuu
*/
#include <ros/console.h>
#include <beginner_tutorials/changeMsg.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <sstream>

std::string pubMsg;
/** 
* @brief Ros service server to change ros msg being published
* @param req  Standard variable of type changeMsg::Request 
* defined in the header file
* @param res - Standard variable of type changeMsg::Response
* defined in the header file
* @return bool value
*/
bool changeMyMsg(beginner_tutorials::changeMsg::Request &req,
           beginner_tutorials::changeMsg::Response &res) {
  ROS_INFO("Request recieved to change string to %s", req.newMsg);
  try {
    pubMsg = req.newMsg;
    ROS_WARN("Successfully changed the string");
    res.resp = true;
    return true;
  }
  catch (const std::exception&) {
    ROS_ERROR("Couldn't change the string");
    res.resp = false;
  }
  return false;
}
/**
 * @brief This tutorial demonstrates simple sending of messages over the ROS system.
 * @param argc argv
 * @return None
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("changeMsg", changeMyMsg);
  // ros::ServiceServer service = n.advertiseService("changeMsg", changeMsg);
  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  int freq = 2;
  bool result = n.getParam("freq", freq);
  if (result) {
    ROS_INFO("Successfully got the param");
  }
  ros::Rate loop_rate(freq);
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  ROS_DEBUG_STREAM_ONCE("This is a Debug Stream" << " Message");
  tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(1, 2, 0));
  tf::Quaternion q;
  q.setRPY(0, 0, 1);
  transform.setRotation(q);
  int count = 0;
  pubMsg = "808x defualt message";
  while (ros::ok()) {
    std_msgs::String msg;
    std::stringstream ss;
    ss << pubMsg << count;
    msg.data = ss.str();

    ROS_INFO("Talker : %s", msg.data.c_str());

    chatter_pub.publish(msg);
    br.sendTransform(tf::StampedTransform(transform,
             ros::Time::now(), "world", "talk"));
    ROS_WARN_STREAM_ONCE("Talker : This is a Warn Stream" << " Message");
    ROS_INFO_STREAM_ONCE("Talker : This is a Info Stream" << " Message");
    ROS_ERROR_STREAM_ONCE("Talker : This is a Error Stream" << " Message");
    ROS_FATAL_STREAM_ONCE("Talker : This is a Fatal Stream" << " Message");
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}
