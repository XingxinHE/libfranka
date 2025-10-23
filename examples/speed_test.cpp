// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace Robot {
std::unique_ptr<franka::Robot> make_robot(const std::string& ip) {
  std::unique_ptr<franka::Robot> robot = std::make_unique<franka::Robot>(ip);
  setDefaultBehavior(*robot);
  return robot;
}

bool handle_reflex(const std::array<double, 7>& goal, const std::string& robot_ip) {
  constexpr int MAX_RETRIES = 10;
  constexpr auto RETRY_INTERVAL = std::chrono::milliseconds(500);

  for (int retry = 0; retry < MAX_RETRIES; ++retry) {
    std::unique_ptr<franka::Robot> recovery_robot = Robot::make_robot(robot_ip);
    const franka::RobotState state = recovery_robot->readOnce();
    const franka::RobotMode mode = state.robot_mode;

    try {
      // Check if robot mode allows movement (not in reflex)
      if (franka::RobotMode::kIdle == mode) {
        std::cout << "Robot ready, continue moving with low speed 0.3..." << std::endl;
        MotionGenerator motion_generator(static_cast<double>(0.3f), goal);
        recovery_robot->control(motion_generator);
        std::cout << "Robot succeed to continue!" << std::endl;
        return true;
      }
    } catch (const franka::Exception& e) {
      std::cout << "Retry " << (retry + 1) << ": " << e.what() << std::endl;
    }
    if (retry < MAX_RETRIES - 1) {  // Don't sleep after the last retry
      std::cout << "Robot not ready, current mode is " << mode << std::endl;
      std::this_thread::sleep_for(RETRY_INTERVAL);
    }
  }

  return false;
}
}  // namespace Robot

int move_to_joint(const std::vector<double>& target_state,
                  const std::string& robot_ip,
                  float speed_factor) {
  if (target_state.size() != 7) {
    std::cerr << "Move Joint should contain 7 values." << std::endl;
    return -1;
  }

  std::unique_ptr<franka::Robot> robot = Robot::make_robot(robot_ip);

  // First move the robot to a suitable joint configuration
  std::array<double, 7> q_goal;
  std::copy(target_state.begin(), target_state.end(), q_goal.begin());

  try {
    MotionGenerator motion_generator(static_cast<double>(speed_factor), q_goal);
    robot->control(motion_generator);
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    std::cout << "Try to handle reflex." << std::endl;

    robot->automaticErrorRecovery();
    // Why do we need to reset here?
    // This is to destroy the smart pointer and automatically invoke the destructor to release the
    // resource. When exception throw, the mutex inside Robot class may still be locked, so we need
    // to release it before creating a new Robot instance. See
    // https://github.com/frankarobotics/libfranka/blob/faaefaa0ff17a812b8a9bc4ac1ed7c353c46064d/include/franka/robot.h#L800-L801
    robot.reset();

    const bool isSuccess = Robot::handle_reflex(q_goal, robot_ip);
    return isSuccess ? 0 : -1;
  }
  return 0;
}

int main() {
  const std::vector<double> HOME_POSITION_0 = {-0.08136433, -1.39358484, -0.90367155, -2.27544727,
                                               -0.75240544, 1.40939064,  -2.60557454};
  const std::vector<double> HOME_POSITION_1 = {2.16699953,  -0.19979069, -0.21875428, -2.31549747,
                                               -0.43570103, 1.94172327,  -0.52261656};

  // Robot configurations
  struct RobotConfig {
    std::string ip;
    std::string realtime_data_dir;
    std::string precomputed_data_file;
    std::vector<double> home_position;
    int robot_id;
  };

  std::vector<RobotConfig> robots = {
      {"172.16.0.3",
       "/home/hex/Documents/github/fork/libfranka/realtime_data/0/1",
       "/home/hex/Documents/github/fork/libfranka/precomputed_data/0/traj_1_robot0_precompute.json",
       HOME_POSITION_0,
       0},
      {"172.16.0.5",
       "/home/hex/Documents/github/fork/libfranka/realtime_data/1",
       "/home/hex/Documents/github/fork/libfranka/precomputed_data/1/traj_9_robot1_precompute.json",
       HOME_POSITION_1,
       1}
  };

  const float TEST_SPEED_FACTOR = 1.0f;

  // Lambda function to test a single robot
  auto test_robot = [TEST_SPEED_FACTOR](const RobotConfig& config) {
    std::cout << "[Robot " << config.robot_id << "] Starting test (IP: " << config.ip << ")" << std::endl;

    // Look up all JSON files in the robot's realtime data directory
    std::vector<std::string> real_time_json;
    for (const auto& entry : std::filesystem::directory_iterator(config.realtime_data_dir)) {
      if (entry.path().extension() == ".json") {
        real_time_json.push_back(entry.path().string());
      }
    }

    // Go to home position first
    std::cout << "[Robot " << config.robot_id << "] Moving to home position..." << std::endl;
    move_to_joint(config.home_position, config.ip, 0.3);

    int success_count = 0;

    std::cout << "[Robot " << config.robot_id << "] Testing with Speed Factor: " << TEST_SPEED_FACTOR << std::endl;
    for (const auto& json_file : real_time_json) {
      std::cout << "[Robot " << config.robot_id << "] Testing json file: " << json_file << std::endl;

      // Parse realtime data
      std::ifstream file_stream(json_file);
      json realtime_data = json::parse(file_stream);

      // Find corresponding precomputed file
      std::ifstream pre_file_stream(config.precomputed_data_file);
      json precomputed_data = json::parse(pre_file_stream);

      const std::vector<double> pick_0 = realtime_data["pick"][0].template get<std::vector<double>>();
      const std::vector<double> real_0 = realtime_data["real"][0].template get<std::vector<double>>();

      const std::vector<std::vector<double>> place_traj =
          precomputed_data["place"].template get<std::vector<std::vector<double>>>();
      const std::vector<double> place_start = place_traj.front();
      const std::vector<double> place_end = place_traj.back();

      const std::vector<std::vector<double>> traj = {pick_0, real_0, place_start, place_end};

      bool skip_to_next_file = false;

      // Forward trajectory
      for (size_t i = 0; i < traj.size(); ++i) {
        const std::vector<double>& target_state = traj[i];
        const int result = move_to_joint(target_state, config.ip, TEST_SPEED_FACTOR);
        if (result != 0) {
          std::cout << "[Robot " << config.robot_id << "] Failed at " << i << std::endl;
          skip_to_next_file = true;
          break;
        }
      }

      if (skip_to_next_file) {
        continue;
      }

      // Reverse trajectory
      for (size_t i = traj.size(); i-- > 0;) {
        const std::vector<double>& target_state = traj[i];
        const int result = move_to_joint(target_state, config.ip, TEST_SPEED_FACTOR);
        if (result != 0) {
          std::cout << "[Robot " << config.robot_id << "] Failed at " << i << std::endl;
          break;
        }
      }
      success_count++;
    }

    std::cout << "[Robot " << config.robot_id << "] Testing completed with speed factor: " << TEST_SPEED_FACTOR << std::endl;
    std::cout << "[Robot " << config.robot_id << "] Successfully completed " << success_count << " / " << real_time_json.size()
              << " JSON files." << std::endl;
  };

  // Launch parallel threads for each robot
  std::vector<std::thread> threads;
  for (const auto& config : robots) {
    threads.emplace_back(test_robot, config);
  }

  // Wait for all threads to complete
  for (auto& thread : threads) {
    thread.join();
  }

  std::cout << "All robots testing completed!" << std::endl;
  return 0;
}