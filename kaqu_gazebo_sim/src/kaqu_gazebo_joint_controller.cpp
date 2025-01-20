#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "chrono"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

/*
 과제
1. kaqu_gazebo_joint_controller.cpp 작성
    1. 계산이 끝난 각도 값을 가진 토픽을 구독해서 gazebo가 인식할 수 있는 토픽으로 변환하여 발행해주는 코드
2. cmakelist.txt, package.xml 파일 작성
    1. 빌드를 제대로 할 수 있도록 하는 설정 파일임. 혼자 작성해보고 빌드 제대로 되는지 확인해보기
*/

// using namespace std::chrono_literals;
using std::placeholders::_1;

class KaquGazeboJointCtrl : public rclcpp::Node
{
public:
    KaquGazeboJointCtrl()
        : Node("Kaqu_gazebo_joint_ctrl_node")
    {
        // kaqu_jointController/commands : hyperdogd에서는 inverse_kinematic_node에서 publish. 아직은 실행 불가
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "kaqu_jointController/commands", 30, std::bind(&KaquGazeboJointCtrl::topic_callback, this, _1));

        // gazebo에서 인식할 수 있는 토픽으로 발행
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("gazebo_joint_controller/commands", 30);
    }

private:
    void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg_rx) const
    {
        auto joint_angles = std_msgs::msg::Float64MultiArray();
        std::vector<double> angles[12];
        for (float ang : msg_rx->data)
        {
            joint_angles.data.push_back(double(ang * M_PI / 180));
        }
        publisher_->publish(joint_angles);
    }
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KaquGazeboJointCtrl>());
    rclcpp::shutdown();
    return 0;
}
