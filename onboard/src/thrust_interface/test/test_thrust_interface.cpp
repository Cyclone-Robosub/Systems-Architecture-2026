#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <custom_interfaces/msg/pwms.hpp>
#include "std_msgs/msg/bool.hpp"
#include "thrust_interface.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <thread>
#include <chrono>

/**
 * @brief Test fixture for Thrust_Interface node
 * 
 * This fixture creates a mock serial device using a pipe to test
 * the thrust interface without requiring actual hardware.
 */
class TestThrustInterface : public ::testing::Test {
protected:
    std::shared_ptr<Thrust_Interface> node;
    int pipe_fds[2];  // [0] = read end, [1] = write end
    std::vector<int> test_thrusters;
    
    /**
     * @brief Set up test environment before each test
     * 
     * Creates a pipe for mock serial communication and initializes ROS2
     */
    void SetUp() override {
        // Initialize ROS2
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        
        // Create pipe for mock serial communication
        // pipe_fds[0] = read end, pipe_fds[1] = write end
        if (pipe(pipe_fds) == -1) {
            FAIL() << "Failed to create pipe for mock serial";
        }
        
        // Set up test thruster configuration
        test_thrusters = {8, 9, 6, 7, 13, 11, 12, 10};
    }
    
    /**
     * @brief Clean up after each test completes
     * 
     * Shuts down the node and closes pipes
     */
    void TearDown() override {
        node.reset();
        close(pipe_fds[0]);
        close(pipe_fds[1]);
    }
    
    /**
     * @brief Helper function to read serial output from the node
     * 
     * @param timeout_ms Maximum time to wait for data
     * @return std::string The data read from the serial interface
     */
    std::string read_serial_output(int timeout_ms = 100) {
        char buffer[1024];
        fd_set readfds;
        struct timeval timeout;
        
        FD_ZERO(&readfds);
        FD_SET(pipe_fds[0], &readfds);
        
        timeout.tv_sec = timeout_ms / 1000;
        timeout.tv_usec = (timeout_ms % 1000) * 1000;
        
        int result = select(pipe_fds[0] + 1, &readfds, nullptr, nullptr, &timeout);
        
        if (result > 0) {
            ssize_t bytes_read = read(pipe_fds[0], buffer, sizeof(buffer) - 1);
            if (bytes_read > 0) {
                buffer[bytes_read] = '\0';
                return std::string(buffer);
            }
        }
        
        return "";
    }
    
    /**
     * @brief Create a node with custom parameters using test mode
     * 
     * @param min_pwm Minimum PWM value
     * @param max_pwm Maximum PWM value
     */
    void create_node_with_params(int min_pwm, int max_pwm) {
        // Use pipe_fds[1] (write end) as the file descriptor
        // The node will write to it, and we'll read from pipe_fds[0]
        node = std::make_shared<Thrust_Interface>(
            test_thrusters, 
            pipe_fds[1],  // Write end of pipe
            min_pwm, 
            max_pwm
        );
    }
    
    /**
     * @brief Helper to publish PWM message and process it
     * 
     * @param msg The PWM message to publish
     */
    void publish_and_process(custom_interfaces::msg::Pwms::SharedPtr msg) {
        auto publisher = node->create_publisher<custom_interfaces::msg::Pwms>("pwm_cmd", 10);
        
        // Give publisher time to connect
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        publisher->publish(*msg);
        
        // Process the message
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    /**
     * @brief Helper to publish the heartbeat message
     */
    void publish_heartbeat() {
        auto publisher = node->create_publisher<std_msgs::msg::Bool>("mux_heartbeat", 10);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        auto message = std_msgs::msg::Bool();
        message.data = true;

        publisher->publish(message);

        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
};

/**
 * @brief Test basic node construction
 */
TEST_F(TestThrustInterface, NodeConstruction) {
    ASSERT_NO_THROW({
        create_node_with_params(1200, 1800);
    });
    
    ASSERT_NE(node, nullptr);
    EXPECT_EQ(node->get_name(), std::string("thrust_interface"));
}

/**
 * @brief Test PWM clamping to minimum value
 */
TEST_F(TestThrustInterface, PWMClampingMinimum) {
    create_node_with_params(1200, 1800);
    
    auto msg = std::make_shared<custom_interfaces::msg::Pwms>();
    // Set all PWMs below minimum
    for (int i = 0; i < 8; i++) {
        msg->pwms[i] = 1000;  // Below min_pwm of 1200
    }
    publish_heartbeat();
    publish_and_process(msg);
    
    // Read all serial output
    std::string output = read_serial_output(200);
    
    // Verify that all commands have PWM value of 1200 (clamped)
    for (int i = 0; i < 8; i++) {
        std::string expected = "Set " + std::to_string(test_thrusters[i]) + " PWM 1200";
        EXPECT_NE(output.find(expected), std::string::npos) 
            << "Expected clamped PWM command for thruster " << test_thrusters[i];
    }
}

/**
 * @brief Test PWM clamping to maximum value
 */
TEST_F(TestThrustInterface, PWMClampingMaximum) {
    create_node_with_params(1200, 1800);
    
    auto msg = std::make_shared<custom_interfaces::msg::Pwms>();
    // Set all PWMs above maximum
    for (int i = 0; i < 8; i++) {
        msg->pwms[i] = 2000;  // Above max_pwm of 1800
    }
    
    publish_heartbeat();
    publish_and_process(msg);
    
    std::string output = read_serial_output(200);
    
    // Verify that all commands have PWM value of 1800 (clamped)
    for (int i = 0; i < 8; i++) {
        std::string expected = "Set " + std::to_string(test_thrusters[i]) + " PWM 1800";
        EXPECT_NE(output.find(expected), std::string::npos)
            << "Expected clamped PWM command for thruster " << test_thrusters[i];
    }
}

/**
 * @brief Test valid PWM values within range
 */
TEST_F(TestThrustInterface, ValidPWMValues) {
    create_node_with_params(1200, 1800);
    
    auto msg = std::make_shared<custom_interfaces::msg::Pwms>();
    // Set valid PWM values
    for (int i = 0; i < 8; i++) {
        msg->pwms[i] = 1500;  // Valid value within range
    }
    
    publish_heartbeat();
    publish_and_process(msg);
    
    std::string output = read_serial_output(200);
    
    // Verify that all commands have PWM value of 1500 (unchanged)
    for (int i = 0; i < 8; i++) {
        std::string expected = "Set " + std::to_string(test_thrusters[i]) + " PWM 1500";
        EXPECT_NE(output.find(expected), std::string::npos)
            << "Expected unchanged PWM command for thruster " << test_thrusters[i];
    }
}

/**
 * @brief Test mixed PWM values (some need clamping, some don't)
 */
TEST_F(TestThrustInterface, MixedPWMValues) {
    create_node_with_params(1200, 1800);
    
    auto msg = std::make_shared<custom_interfaces::msg::Pwms>();
    msg->pwms[0] = 1000;  // Below min -> should clamp to 1200
    msg->pwms[1] = 1300;  // Valid
    msg->pwms[2] = 1500;  // Valid
    msg->pwms[3] = 1700;  // Valid
    msg->pwms[4] = 2000;  // Above max -> should clamp to 1800
    msg->pwms[5] = 1200;  // At min boundary
    msg->pwms[6] = 1800;  // At max boundary
    msg->pwms[7] = 1400;  // Valid
    
    publish_heartbeat();
    publish_and_process(msg);
    
    std::string output = read_serial_output(200);
    
    // Expected PWM values after clamping
    std::vector<int> expected_pwms = {1200, 1300, 1500, 1700, 1800, 1200, 1800, 1400};
    
    for (int i = 0; i < 8; i++) {
        std::string expected = "Set " + std::to_string(test_thrusters[i]) + 
                              " PWM " + std::to_string(expected_pwms[i]);
        EXPECT_NE(output.find(expected), std::string::npos)
            << "Expected PWM " << expected_pwms[i] << " for thruster " << test_thrusters[i];
    }
}

/**
 * @brief Test subscription exists and is properly configured
 */
TEST_F(TestThrustInterface, SubscriptionExists) {
    create_node_with_params(1200, 1800);
    
    // Give node time to fully initialize
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    auto subscriptions = node->get_subscriptions_info_by_topic("pwm_cmd");
    
    EXPECT_GE(subscriptions.size(), 1);
}

/**
 * @brief Test custom PWM limits
 */
TEST_F(TestThrustInterface, CustomPWMLimits) {
    create_node_with_params(1000, 2000);
    
    auto msg = std::make_shared<custom_interfaces::msg::Pwms>();
    msg->pwms[0] = 900;   // Below custom min -> clamp to 1000
    msg->pwms[1] = 2100;  // Above custom max -> clamp to 2000
    msg->pwms[2] = 1500;  // Valid
    
    for (int i = 3; i < 8; i++) {
        msg->pwms[i] = 1500;
    }
    
    publish_heartbeat();
    publish_and_process(msg);
    
    std::string output = read_serial_output(200);
    
    // Check first thruster clamped to 1000
    std::string expected0 = "Set " + std::to_string(test_thrusters[0]) + " PWM 1000";
    EXPECT_NE(output.find(expected0), std::string::npos);
    
    // Check second thruster clamped to 2000
    std::string expected1 = "Set " + std::to_string(test_thrusters[1]) + " PWM 2000";
    EXPECT_NE(output.find(expected1), std::string::npos);
    
    // Check third thruster unchanged at 1500
    std::string expected2 = "Set " + std::to_string(test_thrusters[2]) + " PWM 1500";
    EXPECT_NE(output.find(expected2), std::string::npos);
}

/**
 * @brief Test boundary values exactly at limits
 */
TEST_F(TestThrustInterface, BoundaryValues) {
    create_node_with_params(1200, 1800);
    
    auto msg = std::make_shared<custom_interfaces::msg::Pwms>();
    // Set all to exactly minimum
    for (int i = 0; i < 4; i++) {
        msg->pwms[i] = 1200;
    }
    // Set rest to exactly maximum
    for (int i = 4; i < 8; i++) {
        msg->pwms[i] = 1800;
    }
    
    publish_heartbeat();
    publish_and_process(msg);
    
    std::string output = read_serial_output(200);
    
    // Verify boundary values are not clamped
    for (int i = 0; i < 4; i++) {
        std::string expected = "Set " + std::to_string(test_thrusters[i]) + " PWM 1200";
        EXPECT_NE(output.find(expected), std::string::npos);
    }
    
    for (int i = 4; i < 8; i++) {
        std::string expected = "Set " + std::to_string(test_thrusters[i]) + " PWM 1800";
        EXPECT_NE(output.find(expected), std::string::npos);
    }
}

/**
 * @brief Test don't accept commands when no heartbeat
 */
TEST_F(TestThrustInterface, NoHeartbeat) {
    create_node_with_params(1200, 1800);

    auto msg = std::make_shared<custom_interfaces::msg::Pwms>();
    // Set valid PWM values
    for (int i = 0; i < 8; i++) {
        msg->pwms[i] = 1800;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    publish_and_process(msg);

    std::string output = read_serial_output(200);

    // Verify that all commands have PWM value of 1500 instead of 1800
    for (int i = 0; i < 8; i++) {
        std::string expected = "Set " + std::to_string(test_thrusters[i]) + " PWM 1500";
        EXPECT_NE(output.find(expected), std::string::npos)
            << "Expected unchanged PWM command for thruster " << test_thrusters[i];

        std::string not_expected = "Set " + std::to_string(test_thrusters[i]) + " PWM 1800";
        EXPECT_EQ(output.find(not_expected), std::string::npos)
            << "Expected to not send our original command " << test_thrusters[i];
    }
}


#ifdef ENABLE_TESTING
/**
 * @brief Main test runner
 */
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
#endif