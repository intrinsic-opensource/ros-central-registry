#include <gtest/gtest.h>
#include <ur_msgs/msg/analog.hpp>
#include <ur_msgs/msg/digital.hpp>
#include <ur_msgs/msg/io_states.hpp>
#include <ur_msgs/srv/set_io.hpp>
#include <ur_msgs/action/tool_contact.hpp>

TEST(UrMsgsTest, TestMessageInstantiation) {
  ur_msgs::msg::Analog msg;
  msg.pin = 1;
  msg.domain = ur_msgs::msg::Analog::VOLTAGE;
  msg.state = 3.14f;
  EXPECT_EQ(msg.pin, 1);
  EXPECT_EQ(msg.domain, ur_msgs::msg::Analog::VOLTAGE);
  EXPECT_FLOAT_EQ(msg.state, 3.14f);
}

TEST(UrMsgsTest, TestServiceInstantiation) {
  ur_msgs::srv::SetIO::Request req;
  ur_msgs::srv::SetIO::Response resp;
  req.fun = 1;
  req.pin = 2;
  req.state = 3.5;
  resp.success = true;
  EXPECT_EQ(req.fun, 1);
  EXPECT_EQ(req.pin, 2);
  EXPECT_DOUBLE_EQ(req.state, 3.5);
  EXPECT_TRUE(resp.success);
}

TEST(UrMsgsTest, TestActionInstantiation) {
  ur_msgs::action::ToolContact::Goal goal;
  // ToolContact action is empty, but we can instantiate its Goal
  (void)goal;
}
