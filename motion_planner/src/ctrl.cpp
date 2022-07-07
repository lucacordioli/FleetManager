#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

class Controller : public rclcpp::Node {
    public:
    explicit Controller() : rclcpp::Node("controller")
    {
        pub = create_publisher<geometry_msgs::msg::Pose2D>("goal", rclcpp::SystemDefaultsQoS());

        geometry_msgs::msg::Pose2D msg;

        char inp = '0';
        float x, y, th;
        do{
            std::cout << "x: ";
            std::cin >> x;

            std::cout << "y: ";
            std::cin >> y;

            std::cout << "th: ";
            std::cin >> th;

            std::cout << "q to quit: ";
            std::cin >> inp;

            msg.x = x;  msg.y = y;  msg.theta = th;
            pub->publish(msg);
        } while(inp != 'q');
    }

    private:
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Controller>();

    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();

    return 0;
}