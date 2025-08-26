#include <chrono>
#include <cmath>
#include <vector>
#include <mutex>
#include <thread>
#include <functional>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals; // 시간 단위 리터럴을 사용할 수 있게 해주는 namespace

class DataHub : public rclcpp::Node 
{
public:
    DataHub() : Node("data_hub"), t_(0.0f) {
        pub_ints_  = this->create_publisher<std_msgs::msg::Int32MultiArray>("/ints", rclcpp::QoS(10));
        pub_float_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/floats", rclcpp::QoS(10));

        cbg_timer_1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cbg_timer_2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        timer_     = this->create_wall_timer(100ms, std::bind(&DataHub::onTimer, this), cbg_timer_1_);
        timer_2_   = this->create_wall_timer(100ms, std::bind(&DataHub::onTimer_2, this), cbg_timer_2_);
    }

private:
    static size_t tid() { return std::hash<std::thread::id>{}(std::this_thread::get_id()); }

    void onTimer() {

        const double ts = this->get_clock()->now().seconds();
        RCLCPP_INFO(get_logger(), "[T%zu][%.3f] onTimer Start", tid(), ts);

        std::this_thread::sleep_for(120ms);

        std_msgs::msg::Int32MultiArray imsg;
        imsg.data.resize(10);

        for (int i=0; i<10; ++i) imsg.data[i] = i;

        imsg.layout.dim.resize(1);
        imsg.layout.dim[0].label = "n";
        imsg.layout.dim[0].size = static_cast<uint32_t>(imsg.data.size());
        imsg.layout.dim[0].stride = static_cast<uint32_t>(imsg.data.size());

        pub_ints_->publish(imsg);

        std_msgs::msg::Float32MultiArray fmsg;
        float s1 = std::sin(t_);
        float s2 = std::cos(t_);
        float s3 = std::sin(2.0f * t_);

        fmsg.data = { s1, s2, s3 };

        fmsg.layout.dim.resize(1);
        fmsg.layout.dim[0].label = "n";
        fmsg.layout.dim[0].size = static_cast<uint32_t>(fmsg.data.size());
        fmsg.layout.dim[0].stride = static_cast<uint32_t>(fmsg.data.size());

        pub_float_->publish(fmsg);

        {
            std::lock_guard<std::mutex> lock(m_);
            last_floats_ = { s1, s2, s3 };
        }

        RCLCPP_INFO(this->get_logger(), "[T%zu][%.3f] onTimer  END  (%.3f, %.3f, %.3f)", tid(), this->get_clock()->now().seconds(), s1, s2, s3);

        t_ += 0.05f;
        RCLCPP_DEBUG(this->get_logger(), "published arrays");
    }

    void onTimer_2() {
        const double ts = this->get_clock()->now().seconds();
        std::array<float, 3> copy;
        {
            std::lock_guard<std::mutex> lock(m_);
            copy = last_floats_;
        }
        RCLCPP_INFO(get_logger(), "[T%zu][%.3f] onTimer2 READ (%.3f, %.3f, %.3f)", tid(), ts, copy[0], copy[1], copy[2]);
    }

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_ints_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_float_;

    rclcpp::TimerBase::SharedPtr timer_, timer_2_;

    rclcpp::CallbackGroup::SharedPtr cbg_timer_1_;
    rclcpp::CallbackGroup::SharedPtr cbg_timer_2_;
    
    std::array<float,3> last_floats_{0.0f, 0.0f, 0.0f};
    std::mutex m_;

    float t_;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DataHub>();

    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(node);
    exec.spin();


    rclcpp::shutdown();

    return 0;
}