#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

// This namespace allows us to write times in our code easily ie. we can write 5s to mean 5 seconds.
using namespace std::chrono_literals;

