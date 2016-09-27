//
// Created by 2scholz on 12.08.16.
//

#include <iostream>
#include "wifi_position_estimation/gaussian_process/optimizer.h"

void Optimizer::rprop(Matrix<double, Dynamic, 1> &starting_point, int n, double delta0, double delta_min, double delta_max,
                      double eta_minus, double eta_plus, double eps_stop)
{
  Matrix<double, Dynamic, 1> delta(3,1);
  delta << delta0, delta0, delta0;
  Matrix<double, Dynamic, 1> grad_old(3,1);
  grad_old << 0.0, 0.0, 0.0;
  Matrix<double, Dynamic, 1> params(3,1);
  params = starting_point;
  Matrix<double, Dynamic, 1> best_params(3,1);
  best_params = params;
  double best = log(0);
  p_.set_params(params);

  for(int i = 0; i < n; ++i)
  {
    Matrix<double, Dynamic, 1> grad;
    grad = p_.log_likelihood_gradient();

    grad_old = grad_old.cwiseProduct(grad);

    for(int j = 0; j < grad_old.size();++j)
    {
      if(grad_old(j,0)>0.0)
      {
        delta(j,0) = std::min(delta(j,0)*eta_plus, delta_max);
      }
      else if(grad_old(j,0)<0.0)
      {
        delta(j,0) = std::max(delta(j,0)*eta_minus, delta_min);
        grad(j,0) = 0.0;
      }
      params(j,0) += (-sgn(grad(j,0))) * delta(j,0);
    }
    grad_old = grad;
    if(grad_old.norm() < eps_stop)
    {
      break;
    }
    p_.set_params(params);
    double lik = -p_.log_likelihood();

    if(lik > best)
    {
      best = lik;
      best_params = params;
    }
    std::cout << "likelihood: " << lik << std::endl;
  }
  std::cout << "best likelihood: " << best << std::endl;
  std::cout << "gradient: " << grad_old << std::endl;
  p_.set_params(best_params);
}