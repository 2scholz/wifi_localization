//
// Created by 2scholz on 26.07.16.
//

#ifndef PROJECT_GAUSSIAN_PROCESS_H
#define PROJECT_GAUSSIAN_PROCESS_H
#include "kernel.h"
//#include <eigen3/Eigen/Dense>
#include <Eigen/Dense>

using namespace Eigen;

template <typename T> int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

class Process
{
  friend class Optimizer;
public:
  Process(Matrix<double, Dynamic, 2> &training_coords, Matrix<double, Dynamic, 1> &training_observs,
          double signal_noise = 0.0, double signal_var = 0.0, double lengthscale = 0.0);
  void train_params(Matrix<double, Dynamic, 1> starting_point);
  double probability(double x, double y, double z);
  void set_training_values(Matrix<double, Dynamic, 2> &training_coords, Matrix<double, Dynamic, 1> &training_observs);
  void set_params(const Matrix<double, Dynamic, 1> &params);
  void set_params(double signal_noise, double signal_var, double lengthscale);
  Matrix<double, Dynamic, 1> log_likelihood_gradient();
  double log_likelihood();
  Vector3d get_params();

private:
  void compute_cov_vector(double x, double y);
  void update_covariance_matrix();

  Matrix<double, Dynamic, 2> training_coords_;
  Matrix<double, Dynamic, 1> training_observs_;
  Kernel kernel_;
  Matrix<double, Dynamic, Dynamic> K_;
  Matrix<double, Dynamic, Dynamic> K_inv_;
  Matrix<double, Dynamic, 1> cov_vector;
  int n;
  double min;
  double max;
  double x_mean_;
  double y_mean_;
  double obs_mean_;
  double x_std_;
  double y_std_;
  double obs_std_;
};


#endif //PROJECT_GAUSSIAN_PROCESS_H

