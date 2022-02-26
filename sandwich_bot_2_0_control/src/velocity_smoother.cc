#include "rclcpp/rclcpp.hpp"
#include "filters/transfer_function.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

std::string topic_pub = "/cmd_vel";
std::string topic_sub = "/cmd_vel_smoother";

rclcpp::Node::SharedPtr make_node_with_params(const std::vector<double> & a, const std::vector<double> & b)
{
    rclcpp::NodeOptions options;
    options.parameter_overrides().emplace_back("dummy.prefix.a", a);
    options.parameter_overrides().emplace_back("dummy.prefix.b", b);
    return std::make_shared<rclcpp::Node>("transfer_function_test", options);
}

class VelocitySmoother : public rclcpp::Node
{
  public:
    VelocitySmoother()
    : Node("velocity_smoother")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        topic_sub, 10, std::bind(&VelocitySmoother::topic_callback, this, _1));
        node = std::make_shared<rclcpp::Node>("velocity_publisher");
        velocity_publisher =  node->create_publisher<geometry_msgs::msg::Twist>(topic_pub,10);

        rclcpp::Node::SharedPtr node_;
        rclcpp::NodeOptions options;

        double epsilon = 1e-4;

        auto node = make_node_with_params(
            {1.0, -0.509525449494429},
            {0.245237275252786, 0.245237275252786});
            
        linear_velocity_filter = std::make_shared<filters::SingleChannelTransferFunctionFilter<double>>();
        angular_velocity_filter = std::make_shared<filters::SingleChannelTransferFunctionFilter<double>>();

        linear_velocity_filter->configure( "dummy.prefix", "LowPass",
        node->get_node_logging_interface(), node->get_node_parameters_interface());

        angular_velocity_filter->configure( "dummy.prefix", "LowPass",
        node->get_node_logging_interface(), node->get_node_parameters_interface());

        timer_ = this->create_wall_timer(10ms, std::bind(&VelocitySmoother::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        geometry_msgs::msg::Twist data;
        linear_velocity_filter->update(_msg.linear.x, data.linear.x);
        // std::cout<<"linear: "<<data.linear.x<<std::endl;
        angular_velocity_filter->update(_msg.angular.z, data.angular.z);
        // std::cout<<"angular: "<<data.angular.z<<std::endl;
        velocity_publisher->publish(data);
    }
    
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr _msg)
    {
        this->_msg = *_msg;
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist _msg;
    std::shared_ptr<filters::FilterBase<double>> linear_velocity_filter;
    std::shared_ptr<filters::FilterBase<double>> angular_velocity_filter;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocitySmoother>());
    return 0;
}