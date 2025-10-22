// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

int move_to_joint_v1(const std::vector<double>& target_state, float speed_factor) {
  try {
    const std::string robot_ip = "172.16.0.3";
    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);

    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    robot.setJointImpedance({{3000, 3000, 3000, 3000, 3000, 3000, 3000}});

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal;
    for (size_t i = 0; i < 7; i++) {
      q_goal[i] = target_state[i];
    }
    MotionGenerator motion_generator(speed_factor, q_goal);
    robot.control(motion_generator);
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
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
  move_to_joint_v1(HOME_POSITION, 0.1);
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
      const int result = move_to_joint_v1(target_state, TEST_SPEED_FACTOR);
      if (result != 0) {
        std::cout << "Failed at " << i << std::endl;
        std::cout << "Waiting for robot to exit reflex mode..." << std::endl;

        // Wait for robot to exit reflex mode
        bool moved_home = false;
        for (int retry = 0; retry < 10; ++retry) {
          try {
            franka::Robot recovery_robot("172.16.0.3");
            franka::RobotState state = recovery_robot.readOnce();

            // Check if robot mode allows movement (not in reflex)
            if (state.robot_mode == franka::RobotMode::kIdle ||
                state.robot_mode == franka::RobotMode::kMove) {
              std::cout << "Robot ready, moving to home..." << std::endl;
              if (move_to_joint_v1(HOME_POSITION, 0.1) == 0) {
                moved_home = true;
                break;
              }
            }
          } catch (const franka::Exception& e) {
            std::cout << "Retry " << (retry + 1) << ": " << e.what() << std::endl;
          }
          std::this_thread::sleep_for(std::chrono::seconds(2));
        }

        if (!moved_home) {
          std::cout << "Failed to return home. Please manually recover the robot." << std::endl;
        }

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
      const int result = move_to_joint_v1(target_state, TEST_SPEED_FACTOR);
      if (result != 0) {
        std::cout << "Failed at " << i << std::endl;
        std::cout << "Waiting for robot to exit reflex mode..." << std::endl;

        // Wait for robot to exit reflex mode
        bool moved_home = false;
        for (int retry = 0; retry < 10; ++retry) {
          try {
            franka::Robot recovery_robot("172.16.0.3");
            franka::RobotState state = recovery_robot.readOnce();

            // Check if robot mode allows movement (not in reflex)
            if (state.robot_mode == franka::RobotMode::kIdle ||
                state.robot_mode == franka::RobotMode::kMove) {
              std::cout << "Robot ready, moving to home..." << std::endl;
              if (move_to_joint_v1(HOME_POSITION, 0.1) == 0) {
                moved_home = true;
                break;
              }
            }
          } catch (const franka::Exception& e) {
            std::cout << "Retry " << (retry + 1) << ": " << e.what() << std::endl;
          }
          std::this_thread::sleep_for(std::chrono::seconds(2));
        }

        if (!moved_home) {
          std::cout << "Failed to return home. Please manually recover the robot." << std::endl;
        }

        break;
      }
    }
    success_count++;
  }
  std::cout << "Testing completed with speed factor: " << TEST_SPEED_FACTOR << std::endl;
  std::cout << "Successfully completed " << success_count << " / " << real_time_json.size()
            << " JSON files." << std::endl;
}
