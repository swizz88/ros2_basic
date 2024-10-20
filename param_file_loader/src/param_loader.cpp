#include "rclcpp/rclcpp.hpp"

class MyRobotNode : public rclcpp::Node
{
public:
    MyRobotNode() : Node("param_loader_node")
    {
        // Khai báo và lấy tham số
        this->declare_parameter<double>("speed", 1.0);
        this->declare_parameter<std::string>("name", "default_robot");
        this->declare_parameter<bool>("is_autonomous", false);

        double speed = this->get_parameter("speed").as_double();
        std::string name = this->get_parameter("name").as_string();
        bool is_autonomous = this->get_parameter("is_autonomous").as_bool();

        RCLCPP_INFO(this->get_logger(), "Speed: %f", speed);
        RCLCPP_INFO(this->get_logger(), "Name: %s", name.c_str());
        RCLCPP_INFO(this->get_logger(), "Is Autonomous: %s", is_autonomous ? "True" : "False");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyRobotNode>());
    rclcpp::shutdown();
    return 0;
}
