//
// Created by 2scholz on 12.08.16.
//

#ifndef PROJECT_OPTIMIZER_H
#define PROJECT_OPTIMIZER_H
#include "gaussian_process.h"

/**
 * Optimizer class
 * This class can be used to optimize the hyper-parameters of a Gaussian process
 */
class Optimizer
{
public:
  /**
   * Constructor
   * @param p Gaussian process, that is supposed to be optimized
   */
  Optimizer(Process &p) : p_(p)
  {}

  /**
   * Optimization algorithm.
   * @param starting_point Starting point of the algorithm
   * @param n Number of iterations, until the algorithm stops
   * @param delta0 Starting value for delta
   * @param delta_min Minimum value for delta
   * @param delta_max Maximum value for delta
   * @param eta_minus Start value for eta_minus
   * @param eta_plus Start value for eta_plus
   * @param eps_stop Used to determine when the algorithm is supposed to stop
   */
  void rprop(Matrix<double, Dynamic, 1> &starting_point, int n = 100, double delta0 = 0.1, double delta_min = 1e-6,
             double delta_max = 50.0, double eta_minus = 0.5, double eta_plus = 1.2, double eps_stop = 0.0);
  void conjugate_gradient(size_t n);
private:
  Process& p_;
};

#endif //PROJECT_OPTIMIZER_H
