#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

// This namespace allows us to write times in our code easily ie. we can write 5s to mean 5 seconds.
using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc,argv);

  if (argc != 3)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"usage: add_two_ints_client X Y ");
  }

  // We are create a shared ptr to a node we just constructed called add_two_ints_client
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");

  // We are creating a client shared_ptr by calling the create_client method of the Node class
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client = 
    node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints_client");

  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);

  // This while gives the client one second to see if there are any services in the network. If
  // it doesn't find any then it will keep waiting. If the client is cancelled by you pressing 
  // Ctrl + C then the error message below will print and this node will return 0; 
  while (!client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Interrupted while waiting for service. Exiting...");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
}