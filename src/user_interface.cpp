/*
✓Implement a simple textual interface to retrieve the user command 
(i.e., you can use cin (c++) or input (python). The user should be able to 
select the robot they want to control (turtle1 or turtle2), and the velocity 
of the robot. 

✓The command should be sent for 1 second, and then the robot should 
stop, and the user should be able again to insert the command. 

1) chiedere all'utente quale tartaruga controllare

2) utente inserisce velocità 

3) invio il comando per 1s e poi la tarta si ferma

4) l'interfaccia torna all'utente

inizializza nodo ROS
crea publisher su /turtle1/cmd_vel e /turtle2/cmd_vel

loop infinito:
    chiedi all’utente quale tartaruga controllare
    chiedi velocità lineare e angolare
    crea messaggio Twist
    pubblica comando
    aspetta 1 secondo
    ferma la tartaruga (Twist a zero)
*/

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <string>
#include <iostream>
#include <memory>

class TurtleController : public rclcpp::Node
{
public:
    TurtleController(): Node("user_input_turtle_controller")
    {
        // Publisher per turtle1 e turtle2
        pub_turtle1_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        pub_turtle2_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 10);

        // Subscriber per turtle1 e turtle2
        sub_turtle1_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10,
            std::bind(&TurtleController::pose_callback_turtle1, this, std::placeholders::_1));

        sub_turtle2_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle2/pose", 10,
            std::bind(&TurtleController::pose_callback_turtle2, this, std::placeholders::_1));

        // Timer per leggere input utente in loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&TurtleController::user_input_loop, this));
    }

private:
    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_turtle1_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_turtle2_;

    // Subscriber
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_turtle1_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_turtle2_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Stato delle tartarughe (vuoto, aggiornalo dopo input)
    double x1_, y1_, theta1_;
    double x2_, y2_, theta2_;

    // ======================
    // Funzioni Subscriber
    // ======================
    void pose_callback_turtle1(const turtlesim::msg::Pose::SharedPtr msg)
    {
        x1_ = msg->x;
        y1_ = msg->y;
        theta1_ = msg->theta;
    }

    void pose_callback_turtle2(const turtlesim::msg::Pose::SharedPtr msg)
    {
        x1_ = msg->x;
        y2_ = msg->y;
        theta_2 = msg->theta;
    }

    // ======================
    // Loop input utente
    // ======================
    void user_input_loop()
    {
        static bool waiting_input = true;
        if(!waiting_input) return; 
        // Serve per evitare che il timer richiami la funzione più volte mentre stiamo 
        // ancora leggendo l’input dall’utente. Se waiting_input è false, significa che stiamo già leggendo.
        // In questo caso la funzione esce subito, evitando conflitti con altre chiamate del timer e setta a false.
        // cosi la funzione non viene più chiamata

        waiting_input = false;

        std::string turtle_name;
        double linear_vel, angular_vel;
        std::string direction;

        // --- Input utente ---
        std::cout << "Seleziona la tartaruga da controllare (turtle1/turtle2): ";
        std::cin >> turtle_name;

        std::cout << "Direzione da muovere (es: move_right, move_left, etc.): ";
        std::cin >> direction;

        std::cout << "Velocità lineare: ";
        std::cin >> linear_vel;

        std::cout << "Velocità angolare: ";
        std::cin >> angular_vel;

        
        // --- Pubblica velocità ---
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = linear_vel;
        msg.angular.z = angular_vel;

        if(turtle_name == "turtle1") {
            pub_turtle1_->publish(msg);
        
        } else if(turtle_name == "turtle2") {
            pub_turtle2_->publish(msg);

        } else {
            std::cout << "Nome tartaruga non valido!" << std::endl;
        }

        // Dopo l’invio del comando si può fermare o attendere 1 secondo
        // TODO: logica per stop o durata comando

        waiting_input = true; // riattiva il timer per nuovo input
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

