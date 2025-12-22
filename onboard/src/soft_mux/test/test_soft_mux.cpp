#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <custom_interfaces/msg/pwms.hpp>
#include "soft_mux.hpp"


using namespace std::chrono_literals;
class TestSoftMuxInterface : public::testing::Test {
    protected:
        std::shared_ptr<SoftMux> mux;
        rclcpp::Subscription<custom_interfaces::msg::Pwms>::SharedPtr pwm_cmd_subscriber;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr current_control_mode_subscriber;
        custom_interfaces::msg::Pwms most_recent_msg;
        std_msgs::msg::Bool most_recent_mode;
        std_msgs::msg::Bool most_recent_mux_hb;
       
        void SetUp() override {
            // Initialize ROS2
            if (!rclcpp::ok()) {
                rclcpp::init(0, nullptr);
            }
        }
        void TearDown() override {
            mux.reset();
            rclcpp::shutdown();
        }
        void createMux() {
            mux = std::make_shared<SoftMux>();
        }

        /*
        *   Helper function to subscribe to the pwm_cmd topic to get the PWMs sent through
        */
        void subscribe() {
            pwm_cmd_subscriber = mux->create_subscription<custom_interfaces::msg::Pwms>("pwm_cmd", 10, std::bind(&TestSoftMuxInterface::pwm_cmd_callback, this, std::placeholders::_1));
        }
        
        /*
        *   Helper function for the callback for pwm_cmd
        */
        void pwm_cmd_callback(custom_interfaces::msg::Pwms::UniquePtr pwm) {
            for (int i = 0; i < 8; i++) {
                most_recent_msg.pwms[i] = pwm->pwms[i];
            }
        }

        /*
        *   Helper function to subscribe to the current_mode topic and get the current mode
        */
        void subscribe_control_mode() {
            current_control_mode_subscriber = mux->create_subscription<std_msgs::msg::Bool>("current_mode", 10, std::bind(&TestSoftMuxInterface::control_mode_callback, this, std::placeholders::_1));
        }

        /*
        *   Helper function for the callback for control mode
        */
        void control_mode_callback(std_msgs::msg::Bool msg) {
            most_recent_mode = msg;
        }

        /*
        *   Helper function to subscribe to mux_heartbeat topic and get the heartbeats from mux
        */
        void subscribe_mux_heartbeat() {
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mux_heartbeat_subscriber = mux->create_subscription<std_msgs::msg::Bool>("mux_heartbeat", 10, std::bind(&TestSoftMuxInterface::mux_heartbeat_callback, this, std::placeholders::_1));
        }
        
        /*
        *   Helper function for the callback for mux heartbeat
        */
        void mux_heartbeat_callback(std_msgs::msg::Bool msg) {
            most_recent_mux_hb.data = true;
            (void) msg;
        }
};

/**
 * @brief Test mux construction
 */
TEST_F(TestSoftMuxInterface, MuxConstruction) {
    ASSERT_NO_THROW({
        createMux();
    });
    ASSERT_NE(mux, nullptr);
    EXPECT_EQ(mux->get_name(), std::string("SoftMux"));
}

/**
 * @brief Test mux sending ctrl pwms
 */
TEST_F(TestSoftMuxInterface, MuxSendCtrlPwms) {
    createMux();
    /* generate sample data
       expected1: sent through pwm_ctrl
       expected2: sent through pwm_cli
    */
    auto msg1 = std::make_unique<custom_interfaces::msg::Pwms>();
    auto msg2 = std::make_unique<custom_interfaces::msg::Pwms>();
    std::array<int, 8> expected1;
    std::array<int, 8> expected2;
    for (int i = 0; i < 8; i++) {
         msg1->pwms[i] = 2 * (i + 1);
         expected1[i] = msg1->pwms[i];
    }
    for (int i = 0; i < 8; i++) {
         msg2->pwms[i] = 8 * (i + 7);
         expected2[i] = msg2->pwms[i];
    }
    // subscribe to pwm_cmd and get the pwms
    subscribe();
   
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(mux);

    mux->is_matlab_mode = true;
    mux->no_ctrl_heartbeat = false;
    ASSERT_NE(mux, nullptr);

    // send through the messages
    mux->pwm_ctrl_callback(std::move(msg1));
    mux->pwm_cli_callback(std::move(msg2));
    exec.spin_some();
   
    for (int i = 0; i < 8; i++) {
        EXPECT_EQ(most_recent_msg.pwms[i], expected1[i]);
    }
}

/**
 * @brief Test mux sending cli pwms
 */
TEST_F(TestSoftMuxInterface, MuxSendCliPwms) {
    createMux();
    /* generate sample data
       expected1: sent through pwm_ctrl
       expected2: sent through pwm_cli
    */
    auto msg1 = std::make_unique<custom_interfaces::msg::Pwms>();
    auto msg2 = std::make_unique<custom_interfaces::msg::Pwms>();
    std::array<int, 8> expected1;
    std::array<int, 8> expected2;
    for (int i = 0; i < 8; i++) {
         msg1->pwms[i] = 2 * (i + 1);
         expected1[i] = msg1->pwms[i];
    }
    for (int i = 0; i < 8; i++) {
         msg2->pwms[i] = 8 * (i + 7);
         expected2[i] = msg2->pwms[i];
    }
    // subscribe to pwm_cmd and get the pwms
    subscribe();
   
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(mux);

    mux->is_matlab_mode = false;
    mux->no_cli_heartbeat = false;
    ASSERT_NE(mux, nullptr);

    // send through the messages
    mux->pwm_ctrl_callback(std::move(msg1));
    mux->pwm_cli_callback(std::move(msg2));
    exec.spin_some();
   
    for (int i = 0; i < 8; i++) {
        EXPECT_EQ(most_recent_msg.pwms[i], expected2[i]);
    }
}

/**
 * @brief Test control mode service
 */
TEST_F(TestSoftMuxInterface, MuxTestService) {
    createMux();
    auto client = mux->create_client<std_srvs::srv::SetBool>("control_mode");
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(mux);

    std::thread spin_thread([&executor]() {
        executor.spin();
    });

    std::shared_ptr<std_srvs::srv::SetBool::Request> request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    auto future = client->async_send_request(request);

    ASSERT_EQ (future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
    EXPECT_TRUE(future.get()->success); // request should be recieved by service
    EXPECT_TRUE(mux->is_matlab_mode); // true bc request->data is true
   
    request->data = false;
    auto future2 = client->async_send_request(request);

    ASSERT_EQ (future2.wait_for(std::chrono::seconds(1)), std::future_status::ready);
    EXPECT_TRUE(future2.get()->success); // request should be recieved by service
    EXPECT_FALSE(mux->is_matlab_mode); // false bc request->data is false

    subscribe_control_mode(); // since the mode changes (from true to false)
    EXPECT_FALSE(most_recent_mode.data); // resulting mode

    executor.cancel();
    spin_thread.join();
}

/**
 * @brief Test ctrl heartbeat
 */
TEST_F(TestSoftMuxInterface, MuxTestInputHB) {
    createMux();
   
    mux->is_matlab_mode = true;
    std_msgs::msg::Bool::UniquePtr test_heartbeat = std::make_unique< std_msgs::msg::Bool>();
    mux->ctrl_heartbeat_callback(std::move(test_heartbeat));
   
    subscribe();
   
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(mux);

    auto msg1 = std::make_unique<custom_interfaces::msg::Pwms>();
    for (int i = 0; i < 8; i++) {
         msg1->pwms[i] = 2 * (i + 1);
    }

    mux->pwm_ctrl_callback(std::move(msg1));
    exec.spin_some();

    for (int i = 0; i < 8; i++) {
        EXPECT_NE(most_recent_msg.pwms[i], 1500);
    }
   
}

/**
 * @brief Test no ctrl heartbeat on ctrl
 */
TEST_F(TestSoftMuxInterface, NoCtrlHeartbeatTriggersStop) {
    createMux();
    mux->is_matlab_mode = true;
   
    subscribe();
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(mux);

    auto msg = std::make_unique<custom_interfaces::msg::Pwms>();
    for (int i = 0; i < 8; i++) {
         msg->pwms[i] = 1600;
    }

    mux->pwm_ctrl_callback(std::move(msg));
    // fake bad heartbeat
    mux->recent_ctrl_heartbeat = std::chrono::steady_clock::now() - std::chrono::seconds(2);
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (std::chrono::steady_clock::now() < deadline) {
        exec.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (most_recent_msg.pwms[0] == 1500) {
            break;
        }
    }

    for (int i = 0; i < 8; i++) {
        EXPECT_EQ(most_recent_msg.pwms[i], 1500);
    }
}

/**
 * @brief Test no cli heartbeat on cli
 */
TEST_F(TestSoftMuxInterface, NoCliHeartbeatTriggersStop) {
    createMux();
    mux->is_matlab_mode = false;
   
    subscribe();
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(mux);

    auto msg = std::make_unique<custom_interfaces::msg::Pwms>();
    for (int i = 0; i < 8; i++) {
         msg->pwms[i] = 1600;
    }

    mux->pwm_cli_callback(std::move(msg));
    // fake bad heartbeat
    mux->recent_cli_heartbeat = std::chrono::steady_clock::now() - std::chrono::seconds(2);
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (std::chrono::steady_clock::now() < deadline) {
        exec.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (most_recent_msg.pwms[0] == 1500) {
            break;
        }
    }

    for (int i = 0; i < 8; i++) {
        EXPECT_EQ(most_recent_msg.pwms[i], 1500);
    }
}

/**
 * @brief Test no ctrl heartbeat on cli
 */
TEST_F(TestSoftMuxInterface, NoCtrlHeartbeatonCli) {
    createMux();
    mux->is_matlab_mode = false;
   
    std_msgs::msg::Bool::UniquePtr test_heartbeat = std::make_unique< std_msgs::msg::Bool>();
    mux->cli_heartbeat_callback(std::move(test_heartbeat));

    subscribe();
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(mux);

    auto msg = std::make_unique<custom_interfaces::msg::Pwms>();
    for (int i = 0; i < 8; i++) {
         msg->pwms[i] = 1600;
    }

    mux->pwm_cli_callback(std::move(msg));
    // fake bad heartbeat for ctrl
    mux->recent_ctrl_heartbeat = std::chrono::steady_clock::now() - std::chrono::seconds(2);
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (std::chrono::steady_clock::now() < deadline) {
        exec.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    for (int i = 0; i < 8; i++) {
        EXPECT_NE(most_recent_msg.pwms[i], 1500);
        EXPECT_EQ(most_recent_msg.pwms[i], 1600);
    }
}

/**
 * @brief Test no cli heartbeat on ctrl
 */
TEST_F(TestSoftMuxInterface, NoCliHeartbeatonCtrl) {
    createMux();
    mux->is_matlab_mode = true;
   
    std_msgs::msg::Bool::UniquePtr test_heartbeat = std::make_unique< std_msgs::msg::Bool>();
    mux->ctrl_heartbeat_callback(std::move(test_heartbeat));

    subscribe();
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(mux);

    auto msg = std::make_unique<custom_interfaces::msg::Pwms>();
    for (int i = 0; i < 8; i++) {
         msg->pwms[i] = 1600;
    }

    mux->pwm_ctrl_callback(std::move(msg));
    // fake bad heartbeat for cli
    mux->recent_cli_heartbeat = std::chrono::steady_clock::now() - std::chrono::seconds(2);
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (std::chrono::steady_clock::now() < deadline) {
        exec.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    for (int i = 0; i < 8; i++) {
        EXPECT_NE(most_recent_msg.pwms[i], 1500);
        EXPECT_EQ(most_recent_msg.pwms[i], 1600);
    }
}

/**
 * @brief Test mux not active
 */
TEST_F(TestSoftMuxInterface, NoMuxHeartbeat) {
    createMux();
    subscribe_mux_heartbeat();

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(mux);

    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
        exec.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_FALSE(most_recent_mux_hb.data);
}

/**
 * @brief Test mux active
 */
TEST_F(TestSoftMuxInterface, MuxHeartbeat) {
    createMux();
    mux->is_matlab_mode = true;
    subscribe_mux_heartbeat();

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(mux);

    auto start = std::chrono::steady_clock::now();
    auto deadline = start + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
        exec.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        int k = 0.5;
        if (std::chrono::steady_clock::now() == start + k * std::chrono::seconds(1)) {
            mux->no_ctrl_heartbeat = false;
            k = k + 1;
        }
        int i = 1;
        if (std::chrono::steady_clock::now() == start + i * std::chrono::seconds(1)) {
            EXPECT_TRUE(most_recent_mux_hb.data);
            i++;
        }
    }
}

#ifdef ENABLE_TESTING
    int main(int argc, char** argv) {
        ::testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();
    }
#endif