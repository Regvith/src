#include <fastbot_interfaces/action/waypoint.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using Waypoint = fastbot_interfaces::action::Waypoint;
using GoalHandleWaypoint = rclcpp_action::ClientGoalHandle<Waypoint>;
using namespace std::chrono_literals;

class WaypointActionTest : public ::testing::Test {
protected:
  static void SetUpTestCase() {
    rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("test_waypoint_node");
    client_ = rclcpp_action::create_client<Waypoint>(node_, "/fastbot_as");

    ASSERT_TRUE(client_->wait_for_action_server(10s))
        << "Action server not available";
  }

  static void TearDownTestCase() { rclcpp::shutdown(); }

  void sendAndCheckGoal(double x, double y, double z = 0.0) {
    auto goal_msg = Waypoint::Goal();
    goal_msg.position.x = x;
    goal_msg.position.y = y;
    goal_msg.position.z = z;

    // Send goal and get handle
    auto future_goal_handle = client_->async_send_goal(goal_msg);
    auto goal_handle_result =
        rclcpp::spin_until_future_complete(node_, future_goal_handle, 10s);
    ASSERT_EQ(goal_handle_result, rclcpp::FutureReturnCode::SUCCESS)
        << "Timed out waiting for goal handle";
    auto goal_handle = future_goal_handle.get();
    ASSERT_TRUE(goal_handle) << "Goal was rejected by server";

    // Wait for result
    auto result_future = client_->async_get_result(goal_handle);
    auto result_code =
        rclcpp::spin_until_future_complete(node_, result_future, 200s);
    ASSERT_EQ(result_code, rclcpp::FutureReturnCode::SUCCESS)
        << "Timed out waiting for waypoint action result";

    auto result = result_future.get();
    ASSERT_TRUE(result.result->success)
        << "Waypoint action to (" << x << ", " << y << ") reported failure";

   // rclcpp::sleep_for(2s); // small delay between goals
  }

  static rclcpp::Node::SharedPtr node_;
  static rclcpp_action::Client<Waypoint>::SharedPtr client_;
};

rclcpp::Node::SharedPtr WaypointActionTest::node_ = nullptr;
rclcpp_action::Client<Waypoint>::SharedPtr WaypointActionTest::client_ =
    nullptr;

TEST_F(WaypointActionTest, SecondWaypoint) { sendAndCheckGoal(0.0, 0.055); }
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
