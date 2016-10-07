//
// Created by 2scholz on 26.07.16.
//

#ifndef PROJECT_KERNEL_H
#define PROJECT_KERNEL_H
// #include <eigen3/Eigen/Dense>
#include <Eigen/Dense>

using namespace Eigen;

/**
 * Kernel class
 * This is the radial basis function kernel. It is used in the Gaussian process.
 */
class Kernel
{
public:
  /**
   * Constructor
   * @param signal_noise
   * @param signal_var
   * @param lengthscale
   */
  Kernel(double signal_noise, double signal_var, double lengthscale);

  /**
   * Compute the covariance for two provided positions
   * @param pos1 first position
   * @param pos2 second position
   * @return Covariance of the two positions
   */
  double covariance(Vector2d& pos1, Vector2d& pos2);

  /**
   * Computes the gradient for two given positions.
   * @param pos1 first position
   * @param pos2 second position
   * @return The gradient as Eigen matrix
   */
  Matrix<double, 3, 1> gradient(Vector2d& pos1, Vector2d& pos2);

  /**
   * Set the hyper-parameters of the kernel.
   * @param signal_noise
   * @param signal_var
   * @param lengthscale
   */
  void set_parameters(double signal_noise, double signal_var, double lengthscale);

  /**
   * Get the hyper-parameters of the kernel.
   * @return
   */
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
