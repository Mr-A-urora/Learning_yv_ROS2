#include "rclcpp/rclcpp.hpp"

class PersonNode : public rclcpp::Node
{
private:
    std::string name_values;
    int age_values;

public:
    PersonNode(const std::string &node_name, const std::string &name, const int &age)
        /*调用父类的构造函数，等同于python中的 super().__init__()*/
        : Node(node_name)
    {
        this->name_values = name;
        this->age_values = age;
    };

    void eat(const std::string &food_name)
    {
        RCLCPP_WARN(this->get_logger(), "%d岁的%s正在吃%s", this->age_values, this->name_values.c_str(), food_name.c_str());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PersonNode>("person_node", "zhangsan", 18);
    RCLCPP_INFO(node->get_logger(), "节点已经创建成功了");
    node->eat("鱼香ROS");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
