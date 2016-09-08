//
// Created by 2scholz on 26.07.16.
//

#ifndef PROJECT_KERNEL_H
#define PROJECT_KERNEL_H
// #include <eigen3/Eigen/Dense>
#include <Eigen/Dense>

using namespace Eigen;

class Kernel
{
public:
  Kernel(double signal_noise, double signal_var, double lengthscale);
  double covariance(Vector2d& pos1, Vector2d& pos2);
  Matrix<double, 3, 1> gradient(Vector2d& pos1, Vector2d& pos2);
  void set_parameters(double signal_noise, double signal_var, double lengthscale);
  Vector3d get_parameters();

private:
  double signal_noise_;
  double signal_var_;
  double lengthscale_;
  double orig_signal_noise_;
  double orig_signal_var_;
  double orig_lengthscale_;
};


#endif //PROJECT_KERNEL_H
