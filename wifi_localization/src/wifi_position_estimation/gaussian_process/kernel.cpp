//
// Created by 2scholz on 26.07.16.
//

#include "wifi_position_estimation/gaussian_process/kernel.h"

Kernel::Kernel(double signal_noise, double signal_var, double lengthscale)
{
  set_parameters(signal_noise, signal_var, lengthscale);
}

double Kernel::covariance(Vector2d &pos1, Vector2d &pos2)
{
  int comp = 0;
  if(pos1 == pos2)
    comp = 1;

  return signal_var_ * exp(-(double((pos1 - pos2).transpose() * (pos1 - pos2)) / lengthscale_)) + comp*signal_noise_;
}

Matrix<double, 3, 1> Kernel::gradient(Vector2d &pos1, Vector2d &pos2)
{
  int comp = 0;
  if(pos1 == pos2)
    comp = 1;

  Matrix<double, 3, 1> gradient;
  double d = (-(double((pos1 - pos2).transpose() * (pos1 - pos2))/lengthscale_));
  double expd = exp(d);
  gradient(0) = signal_noise_ * comp;
  gradient(1) = 2.0 * signal_var_ * expd;
  gradient(2) = -signal_var_ * expd * d;
  return gradient;
}

void Kernel::set_parameters(double signal_noise, double signal_var, double lengthscale)
{
  signal_noise_ = exp(signal_noise);
  signal_var_ = exp(2 * signal_var);
  lengthscale_ = exp(lengthscale);
  orig_signal_noise_ = signal_noise;
  orig_signal_var_ = signal_var;
  orig_lengthscale_ = lengthscale;
}

Vector3d Kernel::get_parameters()
{
  Vector3d parameters = {orig_signal_noise_, orig_signal_var_, orig_lengthscale_};
  return parameters;
}
