#ifndef EKF_H
#define EKF_H

#include <vector>

#include <Eigen/Dense>

#include <fsd_ekf_slam/util.h>


class Ekf
{
public:
  Ekf(Eigen::Vector3f initial_position, Eigen::Vector3f odometry_noise, Eigen::Vector2f observation_noise);

  void predict(Eigen::Vector3f odometry_measurement);
  void update(const std::vector<std::pair<Observation, int>>& observations);

  std::pair<int, float> findClosestObservation(const Observation& observation);
  void addNewObservation(const Observation& cone);

  Eigen::Vector3f getCurrentPositionEstimate() const
  {
    return means_.block<kRobotStateSize, 1>(0, 0);
  }

  Eigen::VectorXf getCurrentObservationsEstimate() const
  {
    return means_.block(kRobotStateSize, 0, observations_count_ * kObservationSize, 1);
  }
  void updatePositionFromObservation(const Observation& cone, int matchedIndex);

private:
  static const int kMaxNumObservations = 1000;
  static const int kObservationSize = 2;
  static const int kRobotStateSize = 3;

  Eigen::Matrix3f process_noise_;
  Eigen::Matrix2f observation_noise_;

  Eigen::MatrixXf means_;
  Eigen::MatrixXf covariances_;

  int observations_count_;

};

#endif