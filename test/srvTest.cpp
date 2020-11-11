#include <gtest/gtest.h>
#include <ros/console.h>
#include <beginner_tutorials/changeMsg.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <bits/stdc++.h>
#include <sstream>


TEST(TestSuite, testCase1) {
	ROS_INFO("Started TEST");
	ros::NodeHandle nh;
	ros::ServiceClient client =
  		nh.serviceClient<beginner_tutorials::changeMsg>("changeMsg");
  	beginner_tutorials::changeMsg srv;
  	srv.request.newMsg = "I changed the message from another node";
  	ros::Rate sleep_rate(10);
  	if (client.call(srv)) {
    	ROS_INFO("Response received : %d", static_cast<bool>(srv.response.resp));
    	sleep_rate.sleep();
    	ros::Duration dur(10);
    	EXPECT_EQ(static_cast<bool>(srv.response.resp), true);
  	} else {
    	ROS_ERROR("Failed to call service ");
  }
}

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	std::cout << "Started test node" << std::endl;
	ros::init(argc, argv, "srvTest");
	ros::NodeHandle nh;
	return RUN_ALL_TESTS();
}