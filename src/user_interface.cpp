/*
✓Implement a simple textual interface to retrieve the user command
(i.e., you can use cin (c++) or input (python). The user should be able to
select the robot they want to control (turtle1 or turtle2), and the velocity
of the robot.

✓The command should be sent for 1 second, and then the robot should
stop, and the user should be able again to insert the command.
*/


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <string>
#include <iostream>
#include <memory>

class TurtleController : public rclcpp::Node
{
public:
    TurtleController() : Node("ui_node")
    {
        // Publisher per turtle1 e turtle2
        pub_turtle1_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        pub_turtle2_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 10);
    }

    // Loop user input
    void user_input_loop()
    {
        while (rclcpp::ok()) //Check rclcpp’s status
        {
            std::string turtle_name;
            float linear_vel, angular_vel;

            std::cout << "Select which turtle to control (turtle1/turtle2): ";
            std::cin >> turtle_name;
            if (turtle_name != "turtle1" && turtle_name != "turtle2")
            {
                std::cout << "Name not valid!" << std::endl;
                continue; // restart loop
            }


            std::cout << "Linear velocity: ";
            std::cin >> linear_vel; // geomtry_msg::Twist float64
            if (std::cin.fail())
            {
                std::cin.clear();       // error flag clearing
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // discard input
                std::cout << "Input not valid!" << std::endl;
                continue;
            }

            std::cout << "Angular velocity: ";
            std::cin >> angular_vel;
            if (std::cin.fail())
            {
                std::cin.clear();             // error flag clearing
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // discard input
                std::cout << "Input not valid!" << std::endl;
                continue;
            }

            // Create message
            auto msg = geometry_msgs::msg::Twist();
            msg.linear.x = linear_vel;
            msg.angular.z = angular_vel;

            // Publish message
            if (turtle_name == "turtle1")
            {
                pub_turtle1_->publish(msg);
            }
            else if (turtle_name == "turtle2")
            {
                pub_turtle2_->publish(msg);
            }

            // wait 1 second
            rclcpp::sleep_for(std::chrono::seconds(1));

            // send stop command by publishing zero velocities
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;

            if (turtle_name == "turtle1")
            {
                pub_turtle1_->publish(msg);
            }
            else if (turtle_name == "turtle2")
            {
                pub_turtle2_->publish(msg);
            }
            std::cout << "Command executed for 1 second. Turtle stopped.\n";
        }
    }

private:
    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_turtle1_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_turtle2_;
};

//Everything under private is NOT accessible from outside 
//so only the class itself can see these variables or functions.

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); //initialize ROS2
    auto node = std::make_shared<TurtleController>(); //create node
    node->user_input_loop(); //start user input loop
    rclcpp::shutdown(); //shutdown ROS2
    return 0;
}
