#include "rclcpp/rclcpp.hpp"
#include "chapt4_interfaces/srv/partrol.hpp"
#include <chrono>
#include <ctime>

using Partrol = chapt4_interfaces::srv::Partrol;
using namespace std::chrono_literals; // 可以使用10s，100ms

class PartrolClient : public rclcpp::Node
{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<Partrol>::SharedPtr partrol_client_;

public:
    PartrolClient() : Node("turtle_controller")
    {
        srand(time(NULL));   // 初始化随机数种子
        partrol_client_ = this->create_client<Partrol>("partrol");
        timer_ = this->create_wall_timer(10s, [&]()->void {
            // 检测服务器是否上线
            while (!this->partrol_client_->wait_for_service(1s))
            {
                if(!rclcpp::ok()){
                    RCLCPP_ERROR(this->get_logger(), "等待服务上线过程中，rclcpp挂了， 我退下了");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "等待服务上线中......");
            }
            // 构造请求的对象
            auto request = std::make_shared<Partrol::Request>();
            request->target_x = rand() % 15;
            request->target_y = rand() % 15;
            RCLCPP_INFO(this->get_logger(), "准备好目标点%f,%f", request->target_x, request->target_y);
            // 发送请求
            this->partrol_client_->async_send_request(request, [&](rclcpp::Client<Partrol>::SharedFuture result_future) -> void{
                auto response = result_future.get();
                if (response -> result == Partrol::Response::SUCCESS){
                    RCLCPP_INFO(this->get_logger(), "请求巡逻目标点成功！");
                }
                if (response->result == Partrol::Response::FAIL){
                    RCLCPP_INFO(this->get_logger(), "请求巡逻目标点失败！");
                }
            });
        });
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PartrolClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
