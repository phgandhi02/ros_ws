/*
The client node here will send a request to the server and then shuts down after it gets a response
or fails. If we want the node to persist then we would use the rclcpp::spin(node)
*/
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
    return 1;
  }

  // We are create a shared ptr to a node we just constructed called add_two_ints_client
  /* 
  We are creating a node here that is run by an executable and we can do more than just have it 
  a client. We attach a client to the node and then handle if the response is a success
  any errors that may come up from the
  response.
  */
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");

  // We are creating a client shared_ptr by calling the create_client method of the Node class
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client = 
    node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();

  // This parses the arguments we pass when 
  // starting the executable ie. ros2 run cpp_srvcli client argv[1] argv[2]
  
  request->a = atoll(argv[1]); //arg 1 when the client sends a request
  request->b = atoll(argv[2]); //arg 2 when the client sends a request

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
  if (rclcpp::FutureReturnCode::SUCCESS == rclcpp::spin_until_future_complete(node, result))
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Sum: %ld", result.get()->sum);
    //client->remove_pending_request(future);
    // handle timeout
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    // handle_response(future.get());
  }

  rclcpp::shutdown();
  return 0;
}