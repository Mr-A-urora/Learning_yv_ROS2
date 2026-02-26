#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp" // 提供消息接口
#include "tf2/LinearMath/Quaternion.hpp"           // 提供 tf2::Quaternion类
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // 消息类型转换函数
#include "tf2_ros/transform_listener.hpp"          // 坐标监听器类
#include "tf2_ros/buffer.hpp"                      // 提供Buffer
#include "tf2/utils.hpp"                           // 提供了四元数转欧拉角的函数
#include "chrono"

using namespace std::chrono_literals; // s/ms

class TFListener : public rclcpp::Node
{
private:
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> buffer_;

public:
    TFListener() : Node("tf_listener")
    {
        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        this->listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, this);
        timer_ = this->create_wall_timer(1s, std::bind(&TFListener::getTransform, this));
    }
    void getTransform()
    {
        // 到 buffer_ 里查询坐标关系
        try
        {
            // 查询坐标关系
            const auto transform = buffer_->lookupTransform("base_link", "target_point", this->get_clock()->now(), rclcpp::Duration::from_seconds(1.0f));
            // 获取查询结果
            auto translation = transform.transform.translation;
            auto rotation = transform.transform.rotation;
            double y, p, r;

            tf2::getEulerYPR(rotation, y, p, r);
            RCLCPP_INFO(get_logger(), "平移：%f, %f, %f", translation.x, translation.y, translation.z);
            RCLCPP_INFO(get_logger(), "旋转：%f, %f, %f", y, p, r);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(get_logger(), "%s", e.what());
        }
    }
};

int main(int argv, char **argc)
{
    rclcpp::init(argv, argc);
    auto node = std::make_shared<TFListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
