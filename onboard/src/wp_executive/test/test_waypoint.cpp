#include "WaypointExecutive.hpp"
#include <gtest/gtest.h>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <queue>
#include <chrono>
#include <thread>

class MockPublisher {
public:
  template <typename T>
  void publish(const T& msg) {}
};

class MockPosition : public Position {
public:
    MockPosition(double x = 0, double y = 0, double z = 0, double roll = 0, double pitch = 0, double yaw = 0)
        : Position(x, y, z, roll, pitch, yaw) {}
};

class WaypointExecutiveTest : public ::testing::Test {
protected:
    void TearDown() override {
        rclcpp::shutdown();
    }
};

void publishMockPosition(const std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32MultiArray>>& positionPub, MockPosition position) {
    auto message = std_msgs::msg::Float32MultiArray();
    message.data = {position.get_x(), position.get_y(), position.get_z(), position.get_roll(), position.get_pitch(), position.get_yaw()};
    positionPub->publish(message);
}

TEST_F(WaypointExecutiveTest, StartorStopCamerasTest) {
    rclcpp::init(0, nullptr);
    std::filesystem::path MissionPath = std::filesystem::current_path().parent_path().parent_path()/ "JSON_Parser" / "MissionPathTestVision.json";
    std::cout << MissionPath << std::endl;
    auto WaypointExecutiveNode = std::make_shared<WaypointExecutive>(MissionPath);
    auto testNode = std::make_shared<rclcpp::Node>("test_node");
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(WaypointExecutiveNode);
    executor->add_node(testNode);
    std::cout << "ROS2 Waypoint running" << std::endl;

    bool camera2_activated = false;

    auto camera2_callback = [&camera2_activated](const std_msgs::msg::Bool::SharedPtr msg) {
        camera2_activated = msg->data;
    };

    // Subscribe to only the camera2 topic
    auto camera2_sub = testNode->create_subscription<std_msgs::msg::Bool>(
        "camera2_command_topic", 10, camera2_callback);

    // Mock position publisher
    auto positionPub = testNode->create_publisher<std_msgs::msg::Float32MultiArray>("position_topic", 10);
    std::jthread spin_thread([executor]() { executor->spin(); });
    publishMockPosition(positionPub, MockPosition(5, 5, 5, 0, 0, 0)); // Update to match the first waypoint
    WaypointExecutiveNode->MissionQueue.parseJSONForMission();
    WaypointExecutiveNode->getNewMissionTask(); // Load the first task
    WaypointExecutiveNode->getNewMissionStep(); // Load the first step of the task
    WaypointExecutiveNode->StartorStopCameras(); // Test camera start/stop logic
    std::this_thread::sleep_for(std::chrono::milliseconds(800)); // Allow some time for processing

    EXPECT_TRUE(camera2_activated); // Check if camera2 was activated

    executor->cancel();
}

TEST_F(WaypointExecutiveTest, SendCurrentWaypointTest) {
    rclcpp::init(0, nullptr);
    std::filesystem::path MissionPath = std::filesystem::current_path().parent_path().parent_path() / "JSON_Parser" / "MissionPathTest1.JSON";
    auto WaypointExecutiveNode = std::make_shared<WaypointExecutive>(MissionPath);
    auto testNode = std::make_shared<rclcpp::Node>("test_node");
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(WaypointExecutiveNode);
    executor->add_node(testNode);
    std::cout << "ROS2 Waypoint running" << std::endl;

    bool message_received = false;

    auto callback = [&message_received](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        ASSERT_EQ(msg->data.size(), 6);
        message_received = true;
    };
    auto waypoint_sub = testNode->create_subscription<std_msgs::msg::Float32MultiArray>(
        "waypoint_topic", 10, callback);

    WaypointExecutiveNode->CurrentWaypointPtr = std::make_shared<MockPosition>(1, 2, 3, 4, 5, 6);
    std::jthread spin_thread([executor]() { executor->spin(); });
    WaypointExecutiveNode->SendCurrentWaypoint();
    std::this_thread::sleep_for(std::chrono::milliseconds(200)); // Small delay for message processing

    EXPECT_TRUE(message_received);

    executor->cancel();
}

TEST_F(WaypointExecutiveTest, isCurrentStepCompletedTest) {
    rclcpp::init(0, nullptr);
    std::filesystem::path MissionPath = std::filesystem::current_path().parent_path().parent_path() / "JSON_Parser" / "MissionPathTest1.JSON";
    auto executive = std::make_shared<WaypointExecutive>(MissionPath);

    executive->CurrentWaypointPtr = std::make_shared<MockPosition>(5, 5, 5, 0, 0, 0);
    executive->CurrentPositionPtr = std::make_shared<MockPosition>(5, 5, 5, 0, 0, 0);
    ASSERT_TRUE(executive->isCurrentStepCompleted());
}

TEST_F(WaypointExecutiveTest, getNewMissionTaskTest) {
    rclcpp::init(0, nullptr);
    std::filesystem::path MissionPath = std::filesystem::current_path().parent_path().parent_path() / "JSON_Parser" / "MissionPathTest1.JSON";
    auto executive = std::make_shared<WaypointExecutive>(MissionPath);
    executive->MissionQueue.parseJSONForMission();
    executive->getNewMissionTask();
    ASSERT_EQ(executive->CurrentTask.name, "Navigate to pickup zone");
}

TEST_F(WaypointExecutiveTest, getNewMissionStepTest) {
    rclcpp::init(0, nullptr);
    std::filesystem::path MissionPath = std::filesystem::current_path().parent_path().parent_path() / "JSON_Parser" / "MissionPathTest1.JSON";
    auto executive = std::make_shared<WaypointExecutive>(MissionPath);
    executive->MissionQueue.parseJSONForMission();
    executive->getNewMissionTask();
    executive->getNewMissionStep();
    ASSERT_TRUE(executive->CurrentStep.WaypointPointer != nullptr);
}

TEST_F(WaypointExecutiveTest, MetPositionandTimeReqTest) {
    rclcpp::init(0, nullptr);
    std::filesystem::path MissionPath = std::filesystem::current_path().parent_path().parent_path() / "JSON_Parser" / "MissionPathTest1.JSON";
    auto executive = std::make_shared<WaypointExecutive>(MissionPath);
    executive->CurrentWaypointPtr = std::make_shared<MockPosition>(5, 5, 5, 0, 0, 0);
    executive->CurrentPositionPtr = std::make_shared<MockPosition>(5, 5, 5, 0, 0, 0);
    ASSERT_TRUE(executive->MetPositionandTimeReq());
}

TEST_F(WaypointExecutiveTest, EndReportTest) {
    rclcpp::init(0, nullptr);
    std::filesystem::path MissionPath = std::filesystem::current_path().parent_path().parent_path() / "JSON_Parser" / "MissionPathTest1.JSON";
    auto executive = std::make_shared<WaypointExecutive>(MissionPath);
    std::ofstream ofs("End_Report.txt", std::ofstream::trunc);
    ofs.close();

    executive->EndReport();
    std::ifstream ifs("End_Report.txt");
    std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

    executive->EndReport({.SOCDANGER=true});
    std::remove("End_Report.txt");
}

TEST_F(WaypointExecutiveTest, DidWeSeeObjectTest) {
    rclcpp::init(0, nullptr);
    std::filesystem::path MissionPath = std::filesystem::current_path().parent_path().parent_path() / "JSON_Parser" / "MissionPathTest1.JSON";
    auto executive = std::make_shared<WaypointExecutive>(MissionPath);
    auto testNode = std::make_shared<rclcpp::Node>("test_node");
   auto execExecutor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto testExecutor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    execExecutor->add_node(executive);
    testExecutor->add_node(testNode);
    std::cout << "ROS2 Waypoint running" << std::endl;

    // Create a JSON object for vision detection
    nlohmann::json visionData = {{{"class_name", "Object1"}}}; // array of one object

    auto visionMessage = std_msgs::msg::String();
    visionMessage.data = visionData.dump();

    auto visionPub = testNode->create_publisher<std_msgs::msg::String>("detections", 10);

    // Spin each executor in its own thread
    std::jthread execSpinThread([execExecutor]() { execExecutor->spin(); });
    std::jthread testSpinThread([testExecutor]() { testExecutor->spin(); });

    visionPub->publish(visionMessage);
    std::this_thread::sleep_for(std::chrono::milliseconds(800)); // Allow some time for processing

    ASSERT_TRUE(executive->DidWeSeeObject("Object1"));
    ASSERT_FALSE(executive->DidWeSeeObject("Object2"));

    execExecutor->cancel();
    testExecutor->cancel();
}

TEST_F(WaypointExecutiveTest, CheckINTofStep) {
    rclcpp::init(0, nullptr);
    std::filesystem::path MissionPath = std::filesystem::current_path() / "JSON_Parser" / "MissionPathTest1.JSON";
    auto executive = std::make_shared<WaypointExecutive>(MissionPath);

    auto soc_msg = std::make_shared<std_msgs::msg::Bool>();
    soc_msg->data = true;
    executive->SOCIntCallback(soc_msg);

    executive->CheckINTofStep();
    ASSERT_FALSE(executive->Current_Interrupts.empty());
    ASSERT_TRUE(executive->Current_Interrupts.front().SOCDANGER);
}

TEST_F(WaypointExecutiveTest, ServiceINTofStep) {
    rclcpp::init(0, nullptr);
    std::filesystem::path MissionPath = std::filesystem::current_path().parent_path().parent_path() / "JSON_Parser" / "MissionPathTest1.JSON";
    auto executive = std::make_shared<WaypointExecutive>(MissionPath);
    Interrupts interrupt;
    interrupt.SOCDANGER = true;
    executive->Current_Interrupts.push(interrupt);

    executive->ServiceINTofStep();
    ASSERT_TRUE(executive->StopWorking);
}

TEST_F(WaypointExecutiveTest, VisionControlModeTest) {
    rclcpp::init(0, nullptr);
    std::filesystem::path MissionPath = std::filesystem::current_path().parent_path().parent_path() / "JSON_Parser" / "MissionPathTestVision.json";
    auto executive = std::make_shared<WaypointExecutive>(MissionPath);
    executive->MissionQueue.parseJSONForMission();
    executive->getNewMissionTask(); // Load the first task

    // Assuming the first task is a vision task
    executive->getNewMissionStep(); // Load the first step of the task
    Interrupts generateVisionINT;
    generateVisionINT.BINS_SPOTTED = true;
    executive->Current_Interrupts.push(generateVisionINT);
    // Check if the vision command is properly set
    executive->ServiceINTofStep();
    ASSERT_TRUE(executive->CurrentStep.VisionINTCommand_Serviced.has_value());
    ASSERT_EQ(executive->CurrentStep.VisionINTCommand_Serviced->first, "BINS_SPOTTED");
    ASSERT_TRUE(executive->CurrentStep.VisionINTCommand_Serviced->second); // Ensure it's not yet serviced
}
TEST_F(WaypointExecutiveTest, ControllerRunsWithValidMission) {
    rclcpp::init(0, nullptr);

    std::filesystem::path missionPath = std::filesystem::current_path().parent_path().parent_path()/ "JSON_Parser" / "MissionPathTest8.json";

    WaypointExecutive exec(missionPath);

    int result = exec.Controller();

    EXPECT_EQ(result, 1);

    // Shutdown after test
    rclcpp::shutdown();
}/*
TEST_F(WaypointExecutiveTest, ResetTimer){
    rclcpp::init(0, nullptr);
     std::filesystem::path MissionPath = std::filesystem::current_path().parent_path().parent_path() / "JSON_Parser" / "MissionPathTest1.JSON";
    auto executive = std::make_shared<WaypointExecutive>(MissionPath);

    executive->CurrentWaypointPtr = std::make_shared<MockPosition>(5, 5, 5, 0, 0, 0);
    executive->CurrentPositionPtr = std::make_shared<MockPosition>(1, 1, 1, 0, 0, 0);
    EXPECT_FALSE(executive->MetPositionandTimeReq());
    executive->CurrentPositionPtr = std::make_shared<MockPosition>(5, 5, 5, 0, 0, 0);
    EXPECT_TRUE(executive->MetPositionandTimeReq());
    executive->CurrentPositionPtr = std::make_shared<MockPosition>(5, 3, 1, 0, 0, 0);
    EXPECT_FALSE(executive->MetPositionandTimeReq());
    //incoprate time Step.


}*/
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
