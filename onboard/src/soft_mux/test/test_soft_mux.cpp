#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <custom_interfaces/msg/pwms.hpp>
#include "soft_mux.hpp"

using namespace std::chrono_literals;
class TestSoftMuxInterface : public::testing::Test {
    protected:
        std::shared_ptr<SoftMux> mux;
       
        void SetUp() override {
            // Initialize ROS2
            if (!rclcpp::ok()) {
                rclcpp::init(0, nullptr);
            }
        }
        void TearDown() override {
            mux.reset();
        }
        void createMux() {
            mux = std::make_shared<SoftMux>();
        }
        /*  @brief Helper to publish PWM message and process it
            @param msg The PWM message to publish
        */
        void publish_and_process(custom_interfaces::msg::Pwms::UniquePtr msg, std::string s) {
            auto publisher = mux->create_publisher<custom_interfaces::msg::Pwms>(s, 10);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            publisher->publish(*std::move(msg));
            rclcpp::spin_some(mux);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        void subscribe_and_process(custom_interfaces::msg::Pwms::UniquePtr msg) {
            rclcpp::Subscription<custom_interfaces::msg::Pwms>::SharedPtr pwm_cmd_subscriber = mux->create_subscription<custom_interfaces::msg::Pwms>("pwm_cmd", 10, std::bind(&TestSoftMuxInterface::pwm_cmd_callback, this, std::placeholders::_1));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            (void) msg;
        }
        void pwm_cmd_callback(custom_interfaces::msg::Pwms::UniquePtr pwm) {
            (void) pwm;
        }
};
TEST_F(TestSoftMuxInterface, MuxConstruction) {
    ASSERT_NO_THROW({
        createMux();
    });
    ASSERT_NE(mux, nullptr);
    EXPECT_EQ(mux->get_name(), std::string("SoftMux"));
}
TEST_F(TestSoftMuxInterface, MuxSendChosenPwms) {
    createMux();
   
    auto msg1 = std::make_unique<custom_interfaces::msg::Pwms>();
    auto msg2 = std::make_unique<custom_interfaces::msg::Pwms>();
    for (int i = 0; i < 8; i++) {
         msg1->pwms[i] = 2 * (i + 1);
    }
    for (int i = 0; i < 8; i++) {
         msg2->pwms[i] = 8 * (i + 7);
    }
    publish_and_process(std::move(msg1), "pwm_ctrl");
    publish_and_process(std::move(msg2), "pwm_cli");
    auto msgR = std::make_unique<custom_interfaces::msg::Pwms>();
    mux->is_matlab_mode = true;
    ASSERT_NE(mux, nullptr);
    mux->pwm_ctrl_callback();
    mux->pwm_cli_callback();
    subscribe_and_process(std::move(msgR));
    ASSERT_NE(msgR, nullptr);
    for (int i = 0; i < 8; i++) {
        EXPECT_EQ(msgR->pwms[i], msg1->pwms[1])
    }
    mux->is_matlab_mode = false;
    mux->pwm_ctrl_callback();
    mux->pwm_cli_callback();
    ASSERT_NE(mux, nullptr);
    subscribe_and_process(std::move(msgR));
    ASSERT_NE(msgR, nullptr);
    for (int i = 0; i < 8; i++) {
        EXPECT_EQ(msgR->pwms[i], msg2->pwms[1])
    }
}
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
