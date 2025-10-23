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

bool handle_reflex(const std::array<double, 7>& goal, std::unique_ptr<franka::Robot>& robot) {
  constexpr int MAX_RETRIES = 10;
  constexpr auto RETRY_INTERVAL = std::chrono::seconds(2);

  for (int retry = 0; retry < MAX_RETRIES; ++retry) {
    const franka::RobotState state = robot->readOnce();
    const franka::RobotMode mode = state.robot_mode;

    try {
      // Check if robot mode allows movement (not in reflex)
      if (franka::RobotMode::kIdle == mode || franka::RobotMode::kMove == mode) {
        std::cout << "Robot ready, continue moving with low speed 0.3..." << std::endl;
        MotionGenerator motion_generator(static_cast<double>(0.3f), goal);
        robot->control(motion_generator);
        std::cout << "Success!" << std::endl;
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
    const bool isSuccess = Robot::handle_reflex(q_goal, robot);
    return isSuccess ? 0 : -1;
  }
  return 0;
}

int main() {
  const std::vector<double> HOME_POSITION = {-0.08136433, -1.39358484, -0.90367155, -2.27544727,
                                             -0.75240544, 1.40939064,  -2.60557454};

  //  (1)
  //  look up all the json files in the data directory:
  //  /home/hex/Documents/github/fork/libfranka/realtime_data
  std::vector<std::string> real_time_json;
  const std::string data_dir = "/home/hex/Documents/github/fork/libfranka/realtime_data";
  for (const auto& entry : std::filesystem::directory_iterator(data_dir)) {
    if (entry.path().extension() == ".json") {
      real_time_json.push_back(entry.path().string());
    }
  }

  const std::string precomputed_json =
      "/home/hex/Documents/github/fork/libfranka/precomputed_data/traj_0_robot0_precompute.json";

  //  (3) start testing: go to home position first
  const std::string ip = "172.16.0.3";
  move_to_joint(HOME_POSITION, ip, 0.3);
  const float TEST_SPEED_FACTOR = 1.0;

  int success_count = 0;

  //  (2)
  std::cout << "Testing with Speed Factor: " << TEST_SPEED_FACTOR << std::endl;
  for (const auto& json_file : real_time_json) {
    std::cout << "Testing json file: " << json_file << std::endl;
    // use nlohmann json to open the file

    std::ifstream file_stream(json_file);
    json realtime_data = json::parse(file_stream);

    std::ifstream pre_file_stream(precomputed_json);
    json precomputed_data = json::parse(pre_file_stream);

    const std::vector<double> pick_0 = realtime_data["pick"][0].template get<std::vector<double>>();
    const std::vector<double> real_0 = realtime_data["real"][0].template get<std::vector<double>>();

    const std::vector<std::vector<double>> place_traj =
        precomputed_data["place"].template get<std::vector<std::vector<double>>>();
    const std::vector<double> place_start = place_traj.front();
    const std::vector<double> place_end = place_traj.back();

    const std::vector<std::vector<double>> traj = {pick_0, real_0, place_start, place_end};

    bool skip_to_next_file = false;
    for (size_t i = 0; i < traj.size(); ++i) {
      const std::vector<double>& target_state = traj[i];
      const int result = move_to_joint(target_state, ip, TEST_SPEED_FACTOR);
      if (result != 0) {
        std::cout << "Failed at " << i << std::endl;

        skip_to_next_file = true;
        break;
      }
    }

    if (skip_to_next_file) {
      continue;  // Skip to next JSON file
    }

    // reverse the traj and move back
    for (size_t i = traj.size(); i-- > 0;) {
      const std::vector<double>& target_state = traj[i];
      const int result = move_to_joint(target_state, ip, TEST_SPEED_FACTOR);
      if (result != 0) {
        std::cout << "Failed at " << i << std::endl;
        break;
      }
    }
    success_count++;
  }
  std::cout << "Testing completed with speed factor: " << TEST_SPEED_FACTOR << std::endl;
  std::cout << "Successfully completed " << success_count << " / " << real_time_json.size()
            << " JSON files." << std::endl;
}
