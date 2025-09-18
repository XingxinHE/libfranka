//
// Created by wenjun on 4/9/25.
//
#include <fmt/core.h>
#include <nanobind/nanobind.h>

#include <cmath>
#include <iostream>
#include <thread>

#include <../include/franka/control_types.h>
#include <../include/franka/exception.h>
#include <../include/franka/robot.h>

#include <../include/franka/gripper.h>
#include <franka/model.h>

#include "../build/_deps/json-src/include/nlohmann/json.hpp"
#include <fstream>
#include "examples_common.h"

#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <Eigen/Dense>


namespace nb = nanobind;
using json = nlohmann::json;

using namespace nb::literals;


template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}

/**
 * 将位姿数组转换为Eigen::Affine3d
 */
Eigen::Affine3d pose_array_to_affine(const std::array<double, 16>& pose) {
  Eigen::Matrix4d matrix;
  // std::cout << "pose_array_to_affine" << std::endl;
  // std::cout << "input pose" << pose << std::endl;
  matrix << pose[0], pose[4], pose[8], pose[12],
            pose[1], pose[5], pose[9], pose[13],
            pose[2], pose[6], pose[10], pose[14],
            pose[3], pose[7], pose[11], pose[15];
  // std::cout << "matrix" << matrix << std::endl;

  return Eigen::Affine3d(matrix);
}

/**
 * 将Eigen::Affine3d转换为位姿数组
 */
std::array<double, 16> affine_to_pose_array(const Eigen::Affine3d& affine) {
  Eigen::Matrix4d matrix = affine.matrix();

  // std::cout << "affine_to_pose_array" << std::endl;
  // std::cout << "matrix" << matrix << std::endl;

  std::array<double, 16> pose;
  // 按列主序填充数组
  for (int col = 0; col < 4; col++) {
    for (int row = 0; row < 4; row++) {
      pose[col * 4 + row] = matrix(row, col);
    }
  }

    // std::cout << "output pose" << pose << std::endl;

  return pose;
}

/**
 * 验证变换矩阵是否有效
 */
bool is_valid_transformation_matrix(const std::array<double, 16>& pose) {
  // 检查齐次坐标的最后一行是否为 [0, 0, 0, 1]
  if (std::abs(pose[3]) > 1e-6 || std::abs(pose[7]) > 1e-6 ||
      std::abs(pose[11]) > 1e-6 || std::abs(pose[15] - 1.0) > 1e-6) {
    return false;
      }

  // 检查旋转矩阵部分是否正交
  Eigen::Matrix3d rotation;
  rotation << pose[0], pose[4], pose[8],
              pose[1], pose[5], pose[9],
              pose[2], pose[6], pose[10];

  Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d product = rotation * rotation.transpose();

  if ((product - identity).norm() > 1e-3) {
    return false;
  }

  return true;
}


int run(const std::vector<std::vector<double>>& traj,
        std::string robot_ip,
        int time_factor,
        float sleep_time,
        bool safe = true,
        float frequency = 100.0)
{

  try {
    std::vector<std::vector<double>> new_traj;
    for (size_t i = 0; i < traj.size(); i++) {
      // new_traj.push_back(traj[traj.size() - 1 - i]);
      new_traj.push_back(traj[i]);
    }
    if (safe == true) {
      frequency = 100.0;
    }

    // franka::Robot robot(argv[1]);
    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal;
    //std::cout << "traj 0 ";
    for (int i = 0; i < 7; i++) {
      q_goal[i] = new_traj[0][i];
      //std::cout << q_goal[i] << " ";
    }
    std::cout << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    robot.setJointImpedance({{3000, 3000, 3000, 3000, 3000, 3000, 3000}});

    // MotionGenerator motion_generator(speed_factor, q_goal);

    // std::cout << "WARNING: This example will move the robot! "
    //           << "Please make sure to have the user stop button at hand!" << std::endl
    //           << "Press Enter to continue..." << std::endl;
    // std::cin.ignore();

    franka::ControllerMode controller_mode = franka::ControllerMode::kJointImpedance;


    std::array<double, 7> initial_position;
    double time = 0.0;
    int timefactor = 0;
    timefactor = time_factor;
    robot.control(
        [&initial_position, &time, &new_traj, &timefactor, &sleep_time](
            const franka::RobotState& robot_state,
            franka::Duration period) -> franka::JointPositions {
          time += period.toMSec();

          if (time == 0.0) {
            initial_position = robot_state.q;
            // std::cout << "run start " << initial_position[0] << " " << initial_position[1] << " "
            //           << initial_position[2] << " " << initial_position[3] << " "
            //           << initial_position[4] << " " << initial_position[5] << " "
            //           << initial_position[6] << std::endl;
          }

          size_t itraj = time / timefactor;
          itraj = (itraj >= new_traj.size() ? new_traj.size() - 1 : itraj);
          size_t itraj_next = itraj + 1;
          itraj_next = (itraj_next >= new_traj.size() ? new_traj.size() - 1 : itraj_next);
          double ratio = (time / timefactor - itraj);
          if (time >= timefactor * new_traj.size()) {
            ratio = 1.0;
          }
          franka::JointPositions output = {{
              new_traj[itraj][0] * (1 - ratio) + new_traj[itraj_next][0] * ratio,
              new_traj[itraj][1] * (1 - ratio) + new_traj[itraj_next][1] * ratio,
              new_traj[itraj][2] * (1 - ratio) + new_traj[itraj_next][2] * ratio,

              new_traj[itraj][3] * (1 - ratio) + new_traj[itraj_next][3] * ratio,
              new_traj[itraj][4] * (1 - ratio) + new_traj[itraj_next][4] * ratio,
              new_traj[itraj][5] * (1 - ratio) + new_traj[itraj_next][5] * ratio,
              new_traj[itraj][6] * (1 - ratio) + new_traj[itraj_next][6] * ratio,
          }};

          if (time >= timefactor * new_traj.size() + sleep_time) {
            std::cout << std::endl << "Finished motion, shutting down" << std::endl;
            return franka::MotionFinished(output);
          }
          return output;
        },
        controller_mode, safe, frequency);

  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  } catch (...) {
    std::cout << "unknown error" << std::endl;
    return -1;
  }

  return 0;
}


int move_to_joint(const std::vector<double>& target_state, std::string robot_ip, float speed_factor = 0.5)  {

  try {
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
    // std::array<double, 7> q_goal = {{target_state[0], target_state[1], target_state[2], target_state[3], target_state[4], target_state[5], target_state[6]}};
    MotionGenerator motion_generator(speed_factor, q_goal);
    robot.control(motion_generator);
  }
  catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;
}

std::vector<double> get_current_joints(std::string robot_ip) {
  try {
    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);

    franka::RobotState state = robot.readOnce();
    std::array<double, 7> joints = state.q;

    std::vector<double> result;
    for (int i = 0; i < 7; i++) {
      result.push_back(joints[i]);
    }

    return result;
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    std::vector<double> empty;
    return empty;
  } catch (...) {
    std::cout << "unknown error" << std::endl;
    std::vector<double> empty;
    return empty;
  }
}

std::vector<std::vector<double>> get_current_ee_pose(std::string robot_ip) {
  try {
    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);

    franka::RobotState state = robot.readOnce();
    std::array<double, 16> pose = state.O_T_EE;

    std::vector<std::vector<double>> result(4, std::vector<double>(4));
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        result[i][j] = pose[i * 4 + j];
      }
    }

    return result;
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    std::vector<std::vector<double>> empty;
    return empty;
  } catch (...) {
    std::cout << "unknown error" << std::endl;
    std::vector<std::vector<double>> empty;
    return empty;
  }
}

std::vector<double> fk_current(
        std::string robot_ip
        ) {
  try{
    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);

    franka::RobotState state = robot.readOnce();

    franka::Model model(robot.loadModel());

    franka::Frame frame = franka::Frame::kEndEffector;
    std::array<double, 16> endpose = model.pose(frame, state);
    std::vector<double> fk_vec;
    std::cout << endpose[0] << std::endl;
    for (int i = 0; i < 16; i++) {
      fk_vec.push_back(endpose[i]);
    }
    return fk_vec;
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    std::vector<double> m;
    return m;
  } catch (...) {
    std::cout << "unknown error" << std::endl;
    std::vector<double> m;
    return m;
  }
}

std::vector<double> fk(const std::vector<double>& traj,
        std::string robot_ip
        ) {

    std::vector<double> new_traj;
    for (size_t i = 0; i < traj.size(); i++) {
      // new_traj.push_back(traj[traj.size() - 1 - i]);
      new_traj.push_back(traj[i]);
    }

    // franka::Robot robot(argv[1]);
    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal;
    std::cout << "traj 0 ";
    for (int i = 0; i < 7; i++) {
      q_goal[i] = new_traj[i];
      std::cout << q_goal[i] << " ";
    }
    std::cout << std::endl;

    franka::RobotState state = robot.readOnce();

    franka::Model model(robot.loadModel());
    // for (franka::Frame frame = franka::Frame::kJoint1; frame <= franka::Frame::kEndEffector;
         // frame++) {
        // std::cout << model.pose(frame, state) << std::endl;}

    std::array<double, 16> fk = model.pose(franka::Frame::kEndEffector, q_goal, state.F_T_EE,state.EE_T_K);

    std::vector<double> fk_vec;
    for (int i = 0; i < 16; i++) {
      fk_vec.push_back(fk[i]);
    }
  return fk_vec;
}

int cartesian_move_to_target(const std::vector<std::vector<double>>& target_matrix,
                            std::string robot_ip,
                            double speed_factor = 0.3,
                            float sleep_time = 200.0) {
  try {

    if (target_matrix.size() != 4 || target_matrix[0].size() != 4) {
      std::cout << "Error: target_matrix must be 4x4" << std::endl;
      return -1;
    }


    if (speed_factor <= 0.0 || speed_factor > 1.0) {
      std::cout << "Error: speed_factor must be in range (0.0, 1.0]" << std::endl;
      return -1;
    }

    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);

    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    // robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});


    std::array<double, 16> target_pose;
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        target_pose[i * 4 + j] = target_matrix[j][i];
      }
    }

    franka::ControllerMode controller_mode = franka::ControllerMode::kCartesianImpedance;

    franka::RobotState current_state = robot.readOnce();
    std::array<double, 16> current_pose = current_state.O_T_EE;

    double distance = 0.0;
    for (int i = 0; i < 3; i++) {
      double pos_diff = target_pose[3 * 4 + i] - current_pose[3 * 4 + i];  // 位置差
      distance += pos_diff * pos_diff;
    }
    distance = std::sqrt(distance);
    std::cout << "distance "<< distance << std::endl;

    const double max_linear_velocity = 1.7 * 0.8;  // m/s
    const double max_angular_velocity = 2.5 * 0.8;  // rad/s

    double angular_distance = 0.0;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        double rot_diff = target_pose[i * 4 + j] - current_pose[i * 4 + j];
        angular_distance += rot_diff * rot_diff;
      }
    }
    angular_distance = std::sqrt(angular_distance);
    std::cout << "angular_distance " << angular_distance << std::endl;

    // 计算所需时间 (考虑速度因子)
    double time_for_translation = distance / (max_linear_velocity * speed_factor);
    double time_for_rotation = angular_distance / (max_angular_velocity * speed_factor);
    double motion_time = std::max(time_for_translation, time_for_rotation);
    std::cout << "time_for_translation " << time_for_translation << std::endl;
    std::cout << "time_for_rotation " << time_for_rotation << std::endl;
    std::cout << "motion_time " << motion_time << std::endl;

    motion_time = std::max(motion_time, 0.5);
    motion_time *= 1.0;
    double time = 0.0;
    std::array<double, 16> initial_pose;

    std::cout << "initial_pose "<< current_pose << std::endl;
    std::cout << "target_pose "<< target_pose << std::endl;

    robot.control(
        [&time, &initial_pose, &target_pose, &motion_time, &sleep_time](
            const franka::RobotState& robot_state,
            franka::Duration period) -> franka::CartesianPose {
          time += period.toSec();

          if (time == 0.0) {
            initial_pose = robot_state.O_T_EE;
          }

          // 计算插值比例 (使用平滑的S形曲线)
          double progress = time / motion_time;
          if (progress > 1.0) progress = 1.0;

          // S形插值函数，提供平滑的加速和减速
          double s_curve = 3.0 * progress * progress - 2.0 * progress * progress * progress;
          double interpolation_factor = s_curve;

          // 使用Eigen::Affine3d进行位姿插值
          Eigen::Affine3d initial_affine = pose_array_to_affine(initial_pose);
          Eigen::Affine3d target_affine = pose_array_to_affine(target_pose);

          // 插值平移
          Eigen::Vector3d init_trans = initial_affine.translation();
          Eigen::Vector3d target_trans = target_affine.translation();
          Eigen::Vector3d interp_trans = init_trans + interpolation_factor * (target_trans - init_trans);
          // std::cout << "init_trans "<< init_trans << std::endl;
          // std::cout << "target_trans "<< target_trans << std::endl;
          // std::cout << "interp_trans "<< interp_trans << std::endl;

          // 插值旋转 (使用四元数slerp)
          Eigen::Quaterniond q_start(initial_affine.rotation());
          Eigen::Quaterniond q_end(target_affine.rotation());
          Eigen::Quaterniond q_interp = q_start.slerp(interpolation_factor, q_end);
          // std::cout << "q_start "<< initial_affine.rotation() << std::endl;
          // std::cout << "q_end "<< target_affine.rotation() << std::endl;

          // 构建插值后的位姿
          Eigen::Affine3d intermediate_goal;
          intermediate_goal.fromPositionOrientationScale(interp_trans, q_interp, Eigen::Vector3d::Ones());

          // 转换为位姿数组
          std::array<double, 16> current_pose = affine_to_pose_array(intermediate_goal);
          std::cout << "interpolation_factor "<< interpolation_factor << std::endl;
          std::cout << "current_pose "<< current_pose << std::endl;


          if (time >= motion_time + 2.0) {
            std::cout << std::endl << "Finished movel, shutting down" << std::endl;
            return franka::MotionFinished(current_pose);
          }

          return current_pose;
        },
         controller_mode, true, 100.0);

  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  } catch (const std::exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}



int cartesian_test(std::string robot_ip) {
  try {
    franka::Robot robot(robot_ip);
    setDefaultBehavior(robot);


    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    // franka::ControllerMode controller_mode = franka::ControllerMode::kJointImpedance;
    std::array<double, 16> initial_pose;
    franka::ControllerMode controller_mode = franka::ControllerMode::kCartesianImpedance;

    // franka::RobotState state = robot.readOnce();
    // initial_pose = state.O_T_EE;
    // std::array<double, 16> new_pose = initial_pose;
    // new_pose[14] += 0.01;
    // franka::CartesianPose cartesian_pose(new_pose);
    // franka::CartesianPose
    // robot.control(cartesian_pose);
    // std::cout << "Finished moving to grasp pose." << std::endl;
    // robot.writeOnce(initial_pose);
    // return 0;


    double time = 0.0;
    robot.control([&time, &initial_pose](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianPose {
      time += period.toSec();

      if (time == 0.0) {
        initial_pose = robot_state.O_T_EE;
      }
      //constexpr double kRadius = 0.3;
      //double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * time));
      //double delta_x = kRadius * std::sin(angle);
      double delta_z = 0.007 / (14.0/time);

      if (delta_z > 0.007/14.0){delta_z = 0.07/14.0;}

      if (time > 13.0) {
        delta_z = 0.07 / (14.0/(14.0-(time-0.0001)));
      }
      std::cout << delta_z << std::endl;
      std::array<double, 16> new_pose = initial_pose;
      new_pose[14] -= delta_z * time;
      std::cout << new_pose << std::endl;


      if (time >= 14.0) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(new_pose);
      }
      return new_pose;
    },
    controller_mode, true, 100.0);
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  } catch (const std::exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
//
//
//
//
//
//
// // Copyright (c) 2023 Franka Robotics GmbH
// // Use of this source code is governed by the Apache-2.0 license, see LICENSE
// #include <cmath>
// #include <iostream>
// #include <array>
// #include <vector>
// #include <algorithm>
//
// #include <franka/exception.h>
// #include <franka/robot.h>
//
// #include "examples_common.h"
//
// // æ·»åŠ  Eigen æ”¯æŒ
// #define EIGEN_DONT_ALIGN_STATICALLY
// #include <Eigen/Dense>
// #include <Eigen/Geometry>
//
// /**
//  * @example generate_cartesian_pose_motion.cpp
//  * An example showing how to generate a Cartesian motion between two poses with speed control.
//  *
//  * @warning Before executing this example, make sure there is enough space in front of the robot.
//  */
//
// // ç¬›å¡å°”ç©ºé—´é™åˆ¶ï¼ˆæ ¹æ®Franka Research 3æ–‡æ¡£ï¼‰
// constexpr double MAX_TRANSLATIONAL_VELOCITY = 1.7;    // m/s
// constexpr double MAX_TRANSLATIONAL_ACCELERATION = 13.0; // m/sÂ²
// constexpr double MAX_TRANSLATIONAL_JERK = 6500.0;     // m/sÂ³
// constexpr double MAX_ROTATIONAL_VELOCITY = 2.5;       // rad/s
// constexpr double MAX_ROTATIONAL_ACCELERATION = 25.0;  // rad/sÂ²
// constexpr double MAX_ROTATIONAL_JERK = 12500.0;       // rad/sÂ³
//
// // å°†ä½å§¿è½¬æ¢ä¸º Eigen å˜æ¢çŸ©é˜µ
// Eigen::Affine3d arrayToAffine3d(const std::array<double, 16>& pose) {
//   Eigen::Matrix4d matrix;
//   for (int i = 0; i < 4; ++i) {
//     for (int j = 0; j < 4; ++j) {
//       matrix(i, j) = pose[i * 4 + j];
//     }
//   }
//   return Eigen::Affine3d(matrix);
// }
//
// // å°† Eigen å˜æ¢çŸ©é˜µè½¬æ¢ä¸ºä½å§¿æ•°ç»„
// std::array<double, 16> affine3dToArray(const Eigen::Affine3d& transform) {
//   std::array<double, 16> pose{};
//   Eigen::Matrix4d matrix = transform.matrix();
//   for (int i = 0; i < 4; ++i) {
//     for (int j = 0; j < 4; ++j) {
//       pose[i * 4 + j] = matrix(i, j);
//     }
//   }
//   return pose;
// }
//
// // è®¡ç®—ä¸¤ç‚¹ä¹‹é—´çš„æœ€çŸ­è§’åº¦å·®ï¼ˆç”¨äºŽæ—‹è½¬ï¼‰
// double shortestAngleDistance(double from, double to) {
//   double diff = to - from;
//   while (diff > M_PI) diff -= 2 * M_PI;
//   while (diff < -M_PI) diff += 2 * M_PI;
//   return diff;
// }
//
// // è®¡ç®—æ‰€éœ€çš„æœ€å°è¿åŠ¨æ—¶é—´
// double calculateMinimumTime(const Eigen::Affine3d& start_pose,
//                            const Eigen::Affine3d& target_pose,
//                            double speed_factor) {
//   // è®¡ç®—ä½ç½®è·ç¦»
//   Eigen::Vector3d start_pos = start_pose.translation();
//   Eigen::Vector3d target_pos = target_pose.translation();
//   double position_distance = (target_pos - start_pos).norm();
//
//   // è®¡ç®—æ—‹è½¬è·ç¦»ï¼ˆä½¿ç”¨å››å…ƒæ•°çš„è§’åº¦å·®ï¼‰
//   Eigen::Quaterniond start_quat(start_pose.rotation());
//   Eigen::Quaterniond target_quat(target_pose.rotation());
//   double rotation_distance = 2.0 * std::acos(std::abs(start_quat.dot(target_quat)));
//
//   // åº”ç”¨é€Ÿåº¦å› å­
//   double max_trans_vel = MAX_TRANSLATIONAL_VELOCITY * speed_factor;
//   double max_rot_vel = MAX_ROTATIONAL_VELOCITY * speed_factor;
//
//   // è®¡ç®—åŸºäºŽä½ç½®å’Œæ—‹è½¬çš„æ—¶é—´éœ€æ±‚
//   double time_for_translation = position_distance / max_trans_vel;
//   double time_for_rotation = rotation_distance / max_rot_vel;
//
//   // è¿”å›žè¾ƒå¤§çš„æ—¶é—´éœ€æ±‚
//   return std::max(time_for_translation, time_for_rotation) * 1.2; // æ·»åŠ 20%çš„å®‰å…¨ä½™é‡
// }
//
// class CartesianMotionGenerator {
// public:
//   CartesianMotionGenerator(const std::array<double, 16>& target_pose,
//                           double speed_factor = 1.0)
//       : speed_factor_(std::clamp(speed_factor, 0.01, 1.0)),
//         time_(0.0),
//         motion_duration_(0.0),
//         initialized_(false) {
//     target_pose_ = arrayToAffine3d(target_pose);
//   }
//
//   franka::CartesianPose operator()(const franka::RobotState& robot_state,
//                                   franka::Duration period) {
//     time_ += period.toSec();
//
//     if (!initialized_) {
//       initial_pose_ = arrayToAffine3d(robot_state.O_T_EE);
//       motion_duration_ = calculateMinimumTime(initial_pose_, target_pose_, speed_factor_);
//       initialized_ = true;
//
//       std::cout << "Motion parameters:" << std::endl;
//       std::cout << "  Speed factor: " << speed_factor_ << std::endl;
//       std::cout << "  Estimated duration: " << motion_duration_ << " seconds" << std::endl;
//       std::cout << "  Max translational velocity: " << MAX_TRANSLATIONAL_VELOCITY * speed_factor_ << " m/s" << std::endl;
//       std::cout << "  Max rotational velocity: " << MAX_ROTATIONAL_VELOCITY * speed_factor_ << " rad/s" << std::endl;
//     }
//
//     double progress = std::min(time_ / motion_duration_, 1.0);
//
//     try {
//       // ä½ç½®çº¿æ€§æ’å€¼
//       Eigen::Vector3d start_position = initial_pose_.translation();
//       Eigen::Vector3d end_position = target_pose_.translation();
//       Eigen::Vector3d interpolated_position = start_position + progress * (end_position - start_position);
//
//       // å§¿æ€å››å…ƒæ•°çƒé¢çº¿æ€§æ’å€¼ (SLERP)
//       Eigen::Quaterniond start_quat(initial_pose_.rotation());
//       Eigen::Quaterniond end_quat(target_pose_.rotation());
//
//       // ç¡®ä¿å››å…ƒæ•°æ–¹å‘ä¸€è‡´
//       if (start_quat.dot(end_quat) < 0.0) {
//         end_quat = Eigen::Quaterniond(-end_quat.coeffs());
//       }
//
//       Eigen::Quaterniond interpolated_quat = start_quat.slerp(progress, end_quat);
//
//       // æž„å»ºæ–°çš„å˜æ¢çŸ©é˜µ
//       Eigen::Affine3d new_pose = Eigen::Affine3d::Identity();
//       new_pose.translation() = interpolated_position;
//       new_pose.linear() = interpolated_quat.toRotationMatrix();
//
//       if (progress >= 1.0) {
//         std::cout << std::endl << "Finished motion in " << time_ << " seconds" << std::endl;
//         return franka::MotionFinished(affine3dToArray(new_pose));
//       }
//
//       return affine3dToArray(new_pose);
//
//     } catch (const std::exception& e) {
//       std::cerr << "Error in motion generation: " << e.what() << std::endl;
//       return franka::MotionFinished(robot_state.O_T_EE);
//     }
//   }
//
// private:
//   Eigen::Affine3d initial_pose_;
//   Eigen::Affine3d target_pose_;
//   double speed_factor_;
//   double time_;
//   double motion_duration_;
//   bool initialized_;
// };
//
// int move_to_cartesian(std::string robot_ip, const std::vector<double>& init_state, std::vector<double>& target_mat,float speed_factor = 0.5) {
//
//   try {
//     franka::Robot robot(robot_ip);
//     setDefaultBehavior(robot);
//
//     // First move the robot to a suitable joint configuration
//     std::array<double, 7> q_goal = {{init_state[0], init_state[1], init_state[2], init_state[3], init_state[4], init_state[5], init_state[6]}};
//     MotionGenerator motion_generator(0.5, q_goal);
//     std::cout << "WARNING: This example will move the robot! "
//               << "Please make sure to have the user stop button at hand!" << std::endl
//               << "Press Enter to continue..." << std::endl;
//     std::cin.ignore();
//     robot.control(motion_generator);
//     std::cout << "Finished moving to initial joint configuration." << std::endl;
//
//     // Set collision behavior
//     robot.setCollisionBehavior(
//         {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
//         {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
//         {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
//         {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
//
//
//     std::cout << "Starting Cartesian motion with speed factor: " << speed_factor << std::endl;
//
//     std::array<double, 16> target_pose = {
//       // æ—‹è½¬çŸ©é˜µ (ç»•Zè½´45åº¦)
//       target_mat[0], target_mat[4], target_mat[8], target_mat[12],
//       target_mat[1],  target_mat[5], target_mat[9], target_mat[13],
//       target_mat[2],  target_mat[6], target_mat[10], target_mat[14],
//       // ä½ç½® (x, y, z)
//       target_mat[3], target_mat[7], target_mat[11], target_mat[15]
//   };
//
//     // åˆ›å»ºè¿åŠ¨ç”Ÿæˆå™¨
//     CartesianMotionGenerator motion_generator_cartesian(target_pose, speed_factor);
//
//     // æ‰§è¡Œç¬›å¡å°”è¿åŠ¨
//     robot.control(motion_generator_cartesian);
//
//   } catch (const franka::Exception& e) {
//     std::cout << "Franka exception: " << e.what() << std::endl;
//     return -1;
//   } catch (const std::exception& e) {
//     std::cout << "Standard exception: " << e.what() << std::endl;
//     return -1;
//   }
//
//   return 0;
// }

NB_MODULE(frankz, m) {
  m.doc() = "Frankz plugin";
  m.def("run", &run, nb::call_guard<nb::gil_scoped_release>());
  m.def("move_to_joint", &move_to_joint, nb::call_guard<nb::gil_scoped_release>());
  m.def("move_to_cartesian", &cartesian_move_to_target, nb::call_guard<nb::gil_scoped_release>());
  m.def("fk_current", &fk_current);
  m.def("fk", &fk);
  m.def("get_current_ee_pose", &get_current_ee_pose);
  m.def("get_current_joints", &get_current_joints);
  m.def("cart_test", &cartesian_test);
}