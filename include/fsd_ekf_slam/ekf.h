// Copyright 2023 Maciej Stępień

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef EKF_H
#define EKF_H

#include <vector>

#include <Eigen/Dense>

#include <fsd_ekf_slam/util.h>

class Ekf
{
public:
  Ekf(Eigen::Vector3f initial_position, Eigen::Vector3f odometry_noise, Eigen::Vector2f observation_noise);

  void predict(const Eigen::Vector3f& odometry_measurement);
  void update(const std::vector<std::pair<Observation, int>>& observations);

  std::pair<int, float> findClosestObservation(const Observation& observation) const;
  void addNewObservation(const Observation& cone);

  Eigen::Vector3f getCurrentPositionEstimate() const;
  Eigen::VectorXf getCurrentObservationsEstimate() const;

private:
  static const int kMaxNumObservations = 1000;
  static const int kObservationSize = 2;
  static const int kRobotStateSize = 3;

  Eigen::Matrix3f process_noise_;
  Eigen::Matrix2f observation_noise_;

  Eigen::MatrixXf means_;
  Eigen::MatrixXf covariances_;

  int observations_count_;

  void updatePositionFromObservation(const Observation& cone, int matched_index);
};

#endif