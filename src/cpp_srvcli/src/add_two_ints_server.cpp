/*
We are including ROS2 C++ lib so that we can use ROS2 functions in our
code. We are also using the example interfaces lib so that we can get the service message that is 
written in that library. Oftentimes we will not write our own srv message and instead will use one
that comes from another package. 
*/
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <memory>

/*
Here we define the function that we want to use ie. we want to add two ints together. This function
takes two arguments: request and response. The request should contain all the information needed to 
provide the response. The request is a shared_ptr which means that we can ensure the objects created
by this node are destroyed by only one thread ie. the last resource using the object. 

We access the request methods a and b to get the ints they contain and add them together. The result
of the addition is then assigned to the response sum method which will store the sum of the response
vars. 

Structure:
// function to perform some action
void func(const std::shared_ptr<pkg::srv::srv_message::Request> request,
            std::shared_ptr<pkg::srv::srv_message::Response> response)
{
    // send response based on request attributes
}
*/
void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
            std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
{
    response->sum = request->a + request->b;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" "b: %ld",
        request->a, request->b);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back response: [%ld]",
        (long int)response->sum);
}
/*
We call the main function and accept any args that might be passed if this function is called. Then 
we do the same for the rclcpp::init function which allows us to pass forward the same arguments for
this node. We allocate memory to construct a node and then return a shared_ptr to the object. We 
then create a service from the node by doing the following:

    service = create_service method<service_type>("service_name", &service_function).
We save the service created in the service variable which is also a shared_ptr. Lastly, we publish 
an info message and then spin the node up. Once the node lifecycle ends then the shutdown function 
is called. 
*/
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server"); // Shared_ptr to node variable which is a shared_ptr<rclcpp::Node> type. 

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service = 
        node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

    rclcpp::spin(node);
    rclcpp::shutdown();
}