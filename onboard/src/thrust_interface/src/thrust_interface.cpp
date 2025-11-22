#include "thrust_interface.hpp"
#include <vector>
#include <string>
/* For serial */
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
/* For serial */

using namespace std::chrono_literals;
using namespace rclcpp;

// Production constructor
Thrust_Interface::Thrust_Interface(std::vector<int> thrusters, char* pico_path, int min_pwm, int max_pwm) : 
    Node("thrust_interface"), thrusters(thrusters), min_pwm(min_pwm), max_pwm(max_pwm),owns_fd(true), no_heartbeat(true)  {
        pwm_received_subscription = this->create_subscription<custom_interfaces::msg::Pwms>("pwm_cmd", 10, 
            std::bind(&Thrust_Interface::pwm_received_subscription_callback, this, std::placeholders::_1));
        heartbeat_subscription = this->create_subscription<std_msgs::msg::Bool>("mux_heartbeat", 10, 
            std::bind(&Thrust_Interface::mux_heartbeat_received_callback, this, std::placeholders::_1));
        pico_fd = open_pico_serial(pico_path);
        if (pico_fd < 0) {
            RCLCPP_INFO(this->get_logger(), "Couldn't connect to Pico over serial!");
            exit(42);
        }
        RCLCPP_INFO(this->get_logger(), "Connected to Pico on %s (fd=%d)", pico_path, pico_fd);
        heartbeat_timer = this->create_wall_timer(500ms, 
            std::bind(&Thrust_Interface::heartbeat_check_callback, this)); // heartbeat timer
}

// Testing constructor
Thrust_Interface::Thrust_Interface(std::vector<int> thrusters, int pico_fd, 
                                   int min_pwm, int max_pwm, bool is_test_mode) : 
    Node("thrust_interface"), 
    thrusters(thrusters), 
    pico_fd(pico_fd),
    min_pwm(min_pwm), 
    max_pwm(max_pwm),
    owns_fd(false),
    no_heartbeat(true)
     {
    
    pwm_received_subscription = this->create_subscription<custom_interfaces::msg::Pwms>(
        "pwm_cmd", 10, 
        std::bind(&Thrust_Interface::pwm_received_subscription_callback, this, std::placeholders::_1));

    heartbeat_subscription = this->create_subscription<std_msgs::msg::Bool>("mux_heartbeat", 10, 
            std::bind(&Thrust_Interface::mux_heartbeat_received_callback, this, std::placeholders::_1));
    
    if (pico_fd < 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid file descriptor provided: %d", pico_fd);
        exit(42);
    }
    
    if (is_test_mode) {
        RCLCPP_INFO(this->get_logger(), "Running in test mode with fd=%d", pico_fd);
    }

    heartbeat_timer = this->create_wall_timer(500ms, 
            std::bind(&Thrust_Interface::heartbeat_check_callback, this)); // heartbeat timer
}

Thrust_Interface::~Thrust_Interface() {
    if (owns_fd && pico_fd >= 0) {
        RCLCPP_INFO(this->get_logger(), "Closing serial connection (fd=%d)", pico_fd);
        close(pico_fd);
    }
}
    
void Thrust_Interface::pwm_received_subscription_callback(custom_interfaces::msg::Pwms::UniquePtr pwms_msg) {
    std::array<int, 8> pwms = pwms_msg->pwms;
    
    for (int i = 0; i < 8; i++) {
        if (pwms[i] < min_pwm) {
            pwms[i] = min_pwm;
        }
        if (pwms[i] > max_pwm) {
            pwms[i] = max_pwm;
        }
        send_to_pico(thrusters[i], pwms[i]);
    }
}

void Thrust_Interface::mux_heartbeat_received_callback(std_msgs::msg::Bool::UniquePtr heartbeat) {
    most_recent_heartbeat = std::chrono::steady_clock::now();
    (void)heartbeat; // stop compiler complaining
}

void Thrust_Interface::heartbeat_check_callback() {
    auto current_time = std::chrono::steady_clock::now();
    if (current_time - most_recent_heartbeat > 1s) {
        RCLCPP_INFO(this->get_logger(), "Didn't get heartbeat from mux. Sending stop command.");
        no_heartbeat = true;
        for (int i = 0; i < 8; i++) {
            send_to_pico(thrusters[i], 1500);
        }        
    }
    else {
        no_heartbeat = false;
    }
}

void Thrust_Interface::send_to_pico(int thruster, int pwm) {
    std::string serial_message = "Set " + std::to_string(thruster) + " PWM " + std::to_string(pwm) + "\n";
    int length = serial_message.size();
    
    ssize_t bytes_written = write(pico_fd, serial_message.c_str(), length);
    
    if (bytes_written != length) {
        RCLCPP_WARN(this->get_logger(), 
                    "Failed to write complete message for thruster %d (wrote %zd/%d bytes)", 
                    thruster, bytes_written, length);
    }
}

int Thrust_Interface::open_pico_serial(char* pico_path) {
    struct termios options;
    speed_t baud = 115200;
    int status, fd;
    
    if ((fd = open(pico_path, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1) {
        return -1;
    }
    
    fcntl(fd, F_SETFL, O_RDWR);
    
    // Get and modify current options:
    tcgetattr(fd, &options);
    cfmakeraw(&options);
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);
    
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    
    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 1;  // 1/10 (1 decisecond)
    
    tcsetattr(fd, TCSANOW, &options);
    
    ioctl(fd, TIOCMGET, &status);
    status |= TIOCM_DTR;
    status |= TIOCM_RTS;
    ioctl(fd, TIOCMSET, &status);
    
    usleep(100000);  // 100ms
    
    return fd;
}

#ifndef ENABLE_TESTING

int main(int argc, char* argv[]) {
    std::vector<int> thrusters = {8, 9, 6, 7, 13, 11, 12, 10};
    char pico_path[] = "/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e66130100f198434-if00";

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Thrust_Interface>(thrusters, pico_path, 1200, 1800));
    rclcpp::shutdown();
    
    return 0;
}
#endif

