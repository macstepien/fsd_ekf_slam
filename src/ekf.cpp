#include <ekf_slam/ekf.h>

#include <cmath>

#include <string>
#include <limits>
#include <cmath>

#include <ekf_slam/util.h>

#include <ros/ros.h>

Ekf::Ekf(Eigen::Vector3f initial_position, Eigen::Vector3f odometry_noise, Eigen::Vector2f observation_noise)
{
  process_noise_.setZero();
  observation_noise_.setZero();

  process_noise_(0, 0) = odometry_noise(0);
  process_noise_(1, 1) = odometry_noise(1);
  process_noise_(2, 2) = odometry_noise(2);

  observation_noise_(0, 0) = observation_noise(0);
  observation_noise_(1, 1) = observation_noise(1);

  int max_state_size = kRobotStateSize + kMaxNumObservations * kObservationSize;

  means_ = Eigen::MatrixXf::Zero(max_state_size, 1);
  covariances_ = Eigen::MatrixXf::Zero(max_state_size, max_state_size);

  observations_count_ = 0;

  //! Allow initial position from odometry - useful for testing
  means_.block<kRobotStateSize, 1>(0, 0) = initial_position;
}

void Ekf::predict(Eigen::Vector3f odometry_measurement)
{
  // Predict state estimate

  // make sure that angle stays in the (-pi, pi) range

  double r = sqrt(pow(odometry_measurement(0), 2) + pow(odometry_measurement(1), 2));
  means_(0) += r*cos(means_(2));
  means_(1) += r*sin(means_(2));


  // Predicted covariance estimate
  Eigen::Matrix3f G = Eigen::Matrix3f::Identity();
  G(0, 2) -= r*sin(means_(2));
  G(1, 2) += r*cos(means_(2));
  
  means_(2) = BoundToMinusPiPi(means_(2) + odometry_measurement(2));

  covariances_.block<kRobotStateSize, kRobotStateSize>(0, 0) =
      G * covariances_.block<kRobotStateSize, kRobotStateSize>(0, 0) * G.transpose() + process_noise_;
  covariances_.block(0, kRobotStateSize, kRobotStateSize, observations_count_ * kObservationSize) =
      G * covariances_.block(0, kRobotStateSize, kRobotStateSize, observations_count_ * kObservationSize);
}

void Ekf::update(const std::vector<std::pair<Observation, int>>& observations)
{
  for (auto obs : observations)
  {
    updatePositionFromObservation(obs.first, obs.second);
  }
}

void Ekf::addNewObservation(const Observation& observation)
{
  int new_observation_index_x = kRobotStateSize + observations_count_ * kObservationSize;
  int new_observation_index_y = kRobotStateSize + observations_count_ * kObservationSize + 1;

  // Observation positions are stored in global coordinate frame (and first 3 values of means_
  // is current position of the robot).
  means_(new_observation_index_x, 0) = means_(0) + observation.r * cos(means_(2) + observation.angle);
  means_(new_observation_index_y, 0) = means_(1) + observation.r * sin(means_(2) + observation.angle);

  ROS_DEBUG_STREAM("Found new observation, coordinates x: " << means_(new_observation_index_x, 0)
                                                            << "y: " << means_(new_observation_index_y, 0));

  covariances_(new_observation_index_x, new_observation_index_x) = 1;
  covariances_(new_observation_index_y, new_observation_index_y) = 1;

  ++observations_count_;
}

void Ekf::updatePositionFromObservation(const Observation& observation, int matched_index)
{
  // Calculate Expected Measurment
  float x_expected = means_(kRobotStateSize + matched_index * kObservationSize) - means_(0);
  float y_expected = means_(kRobotStateSize + matched_index * kObservationSize + 1) - means_(1);

  float r_squared_expected = pow(x_expected, 2) + pow(y_expected, 2);
  float r_expected = sqrt(r_squared_expected);
  float angle_expected = BoundToMinusPiPi(atan2(y_expected, x_expected) - means_(2));

  Eigen::Vector2f z_expected = Eigen::Vector2f(r_expected, angle_expected);
  Eigen::Vector2f z_real = Eigen::Vector2f(observation.r, observation.angle);

  // If update distance is too small, then there are problems with division in observation_jacobian_robot_state_part and
  // observation_jacobian_observation_part matrices

  // TODO: move to constant
  if (fabs(r_expected - observation.r) > 0.001)
  {
    ROS_DEBUG_STREAM("Correcting position");
    int current_size = kRobotStateSize + observations_count_ * kObservationSize;

    // Observation Jacobian
    Eigen::MatrixXf observation_jacobian = Eigen::MatrixXf::Zero(kObservationSize, current_size);

    observation_jacobian(0, 0) = -x_expected * r_expected / r_squared_expected;
    observation_jacobian(0, 1) = -y_expected * r_expected / r_squared_expected;
    observation_jacobian(0, 2) = 0;

    observation_jacobian(1, 0) = y_expected / r_squared_expected;
    observation_jacobian(1, 1) = -x_expected / r_squared_expected;
    observation_jacobian(1, 2) = -1;

    observation_jacobian(0, kRobotStateSize + matched_index * kObservationSize) =
        x_expected * r_expected / r_squared_expected;
    observation_jacobian(0, kRobotStateSize + matched_index * kObservationSize + 1) =
        y_expected * r_expected / r_squared_expected;
    observation_jacobian(1, kRobotStateSize + matched_index * kObservationSize) = -y_expected / r_squared_expected;
    observation_jacobian(1, kRobotStateSize + matched_index * kObservationSize + 1) = x_expected / r_squared_expected;

    Eigen::MatrixXf covariances_block = covariances_.block(0, 0, current_size, current_size);

    Eigen::MatrixXf kalman_gain =
        covariances_block * observation_jacobian.transpose() *
        (observation_jacobian * covariances_block * observation_jacobian.transpose() + observation_noise_).inverse();

    Eigen::Vector2f dz = z_real - z_expected;
    dz(1) = BoundToMinusPiPi(dz(1));
    
    ROS_DEBUG_STREAM("Observation jacobian: " << observation_jacobian);
    ROS_DEBUG_STREAM("Kalman gain: " << kalman_gain);
    ROS_DEBUG_STREAM("Covariances block: " << covariances_block);
    means_.block(0, 0, current_size, 1) += kalman_gain * dz;
    covariances_.block(0, 0, current_size, current_size) =
        (Eigen::MatrixXf::Identity(current_size, current_size) - kalman_gain * observation_jacobian) *
        covariances_block;
  }
}

std::pair<int, float> Ekf::findClosestObservation(const Observation& observation)
{
  if (observations_count_ == 0)
  {
    throw std::runtime_error("No observations");
  }

  float x_mean, y_mean, r_mean, angle_mean, distance;
  float x_detected = observation.r * cos(observation.angle);
  float y_detected = observation.r * sin(observation.angle);

  float min_distance = std::numeric_limits<float>::max();
  int closest_observation_ind = 0;

  for (int index = 0; index < observations_count_; ++index)
  {
    x_mean = means_(kRobotStateSize + index * kObservationSize) - means_(0);
    y_mean = means_(kRobotStateSize + index * kObservationSize + 1) - means_(1);

    r_mean = sqrt(pow(x_mean, 2) + pow(y_mean, 2));
    angle_mean = BoundToMinusPiPi(atan2(y_mean, x_mean) - means_(2));

    x_mean = r_mean * cos(angle_mean);
    y_mean = r_mean * sin(angle_mean);

    distance = sqrt(pow(x_detected - x_mean, 2) + pow(y_detected - y_mean, 2));

    if (distance < min_distance)
    {
      min_distance = distance;
      closest_observation_ind = index;
    }
  }

  return std::make_pair(closest_observation_ind, min_distance);
}
