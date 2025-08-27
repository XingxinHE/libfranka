// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>
#include <thread>

#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/robot.h>

#include <franka/gripper.h>
#include "examples_common.h"

/**
 * @example generate_joint_position_motion.cpp
 * An example showing how to generate a joint position motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

#include <boost/iostreams/categories.hpp>
#include <fstream>
#include <nlohmann/json.hpp>
#include <vector>
using json = nlohmann::json;

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  try {
    std::ifstream f("/home/hex/franka/catkin_ws/libfranka/examples/pick_and_place.json");
    // std::ifstream f("/home/hex/franka/catkin_ws/libfranka/examples/pick.json");
    json data = json::parse(f);

    auto traj = data[7]["joint_states"].template get<std::vector<std::vector<double>>>();

    std::vector<std::vector<double>> new_traj;
    for (size_t i = 0; i < traj.size(); i++) {
      // new_traj.push_back(traj[traj.size() - 1 - i]);
      new_traj.push_back(traj[i]);
    }

    std::cout << new_traj.size() << std::endl;

    franka::Robot robot(argv[1]);
    // franka::Robot robot("172.16.1.3");
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal;
    for (int i = 0; i < 7; i++) {
      q_goal[i] = new_traj[0][i];
    }

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    robot.setJointImpedance({{3000, 3000, 3000, 3000, 3000, 3000, 3000}});

    MotionGenerator motion_generator(0.1, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    franka::ControllerMode controller_mode = franka::ControllerMode::kJointImpedance;

    // robot.control(motion_generator,controller_mode,false);
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    std::array<double, 7> initial_position;
    double time = 0.0;

    robot.control(
        [&initial_position, &time, &new_traj](const franka::RobotState& robot_state,
                                              franka::Duration period) -> franka::JointPositions {
          time += period.toMSec();

          if (time == 0.0) {
            initial_position = robot_state.q;
          }
          size_t itraj = time / 50;
          itraj = (itraj >= new_traj.size() ? new_traj.size() - 1 : itraj);
          size_t itraj_next = itraj + 1;
          itraj_next = (itraj_next >= new_traj.size() ? new_traj.size() - 1 : itraj_next);
          double ratio = (time / 50 - itraj);
          // franka::JointPositionMotion<true, false> motionWithRateLimit;
          franka::JointPositions output = {{
              new_traj[itraj][0] * (1 - ratio) + new_traj[itraj_next][0] * ratio,
              new_traj[itraj][1] * (1 - ratio) + new_traj[itraj_next][1] * ratio,
              new_traj[itraj][2] * (1 - ratio) + new_traj[itraj_next][2] * ratio,

              new_traj[itraj][3] * (1 - ratio) + new_traj[itraj_next][3] * ratio,
              new_traj[itraj][4] * (1 - ratio) + new_traj[itraj_next][4] * ratio,
              new_traj[itraj][5] * (1 - ratio) + new_traj[itraj_next][5] * ratio,
              new_traj[itraj][6] * (1 - ratio) + new_traj[itraj_next][6] * ratio,
          }};
          std::cout << itraj << std::endl;
          if (time >= 50 * new_traj.size()) {
            std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
            return franka::MotionFinished(output);
          }
          return output;
        },
        controller_mode, true);

    franka::Gripper gripper(argv[1]);
    // double grasping_width = std::stod(argv[3]);
    double grasping_width = 0.04;

    // std::stringstream ss(argv[2]);
    std::stringstream ss("0");
    bool homing;
    if (!(ss >> homing)) {
      std::cerr << "<homing> can be 0 or 1." << std::endl;
      return -1;
    }

    if (homing) {
      // Do a homing in order to estimate the maximum grasping width with the current fingers.
      gripper.homing();
    }

    // Check for the maximum grasping width.
    franka::GripperState gripper_state = gripper.readOnce();
    std::cout << gripper_state.max_width << std::endl;

    if (gripper_state.max_width < grasping_width) {
      std::cout << "Object is too large for the current fingers on the gripper." << std::endl;
      return -1;
    }

    // Grasp the object.
    if (!gripper.grasp(grasping_width, 0.1, 60)) {
      std::cout << "Failed to grasp object." << std::endl;
      return -1;
    }

    // Wait 3s and check afterwards, if the object is still grasped.
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(3000));

    gripper_state = gripper.readOnce();
    if (!gripper_state.is_grasped) {
      std::cout << "Object lost." << std::endl;
      return -1;
    }

    std::cout << "Grasped object, will release it now." << std::endl;
    gripper.stop();
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
