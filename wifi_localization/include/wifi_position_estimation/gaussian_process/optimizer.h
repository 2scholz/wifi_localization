//
// Created by 2scholz on 12.08.16.
//

#ifndef PROJECT_OPTIMIZER_H
#define PROJECT_OPTIMIZER_H
#include "gaussian_process.h"

class Optimizer
{
public:
  Optimizer(Process &p) : p_(p)
  {}
  void rprop(Matrix<double, Dynamic, 1> &starting_point, int n = 100, double delta0 = 0.1, double delta_min = 1e-6,
             double delta_max = 50.0, double eta_minus = 0.5, double eta_plus = 1.2, double eps_stop = 0.0);
private:
  Process& p_;
};

#endif //PROJECT_OPTIMIZER_H
