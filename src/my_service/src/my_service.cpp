#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/twist.hpp"


#include <memory>

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
auto twist = geometry_msgs::msg::Twist();
rclcpp::Rate loop_rate(1);

void move(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  twist.linear.x = 0.3;
  for (int i = 1; (i < 5) && rclcpp::ok(); ++i){
    publisher->publish(twist);
    loop_rate.sleep();
  }

  twist.linear.x = 0.0;
  publisher->publish(twist);
  response->success = true;
  response->message = "Successfully called service";
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Incoming response\n");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%s]", response->message);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("my_service_node");

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv = node->create_service<std_srvs::srv::Trigger>("my_service", &move);

  publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Service Started");

  rclcpp::spin(node);
  rclcpp::shutdown();
}