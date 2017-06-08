#include "wifi_position_estimation/gaussian_process/ard_se_kernel.h"
#include <cmath>

ARD_SE_Kernel::ARD_SE_Kernel(double signal_noise, double signal_var, Vector2d lengthscale)
{
  set_parameters(signal_noise, signal_var, lengthscale);
}

ARD_SE_Kernel::ARD_SE_Kernel(double signal_noise, double signal_var, double lengthscale, double lengthscale2)
{
  set_parameters(signal_noise, signal_var, {lengthscale, lengthscale2});
}

double ARD_SE_Kernel::covariance(Vector2d &pos1, Vector2d &pos2)
{
  int comp = 0;
  if(pos1 == pos2)
    comp = 1;

  double z = (pos1-pos2).cwiseQuotient(lengthscale_).squaredNorm();
  return signal_var_*exp(-0.5*z)+comp*signal_noise_;
}

Matrix<double, 4, 1> ARD_SE_Kernel::gradient(Vector2d &pos1, Vector2d &pos2)
{
  int comp = 0;
  if(pos1 == pos2)
    comp = 1;

  Vector4d gradient;

  Eigen::Vector2d z = (pos1-pos2).cwiseQuotient(lengthscale_).array().square();
  double k = signal_var_*exp(-0.5*z.sum());

  gradient(0) = signal_noise_ * comp;
  gradient(1) = 2.0 * k;
  gradient.tail(2) = z*k;

  return gradient;
}

void ARD_SE_Kernel::set_parameters(double signal_noise, double signal_var, Vector2d lengthscale)
{
  signal_noise_ = exp(signal_noise);
  signal_var_ = exp(2 * signal_var);
  lengthscale_(0) = exp(lengthscale(0));
  lengthscale_(1) = exp(lengthscale(1));

  orig_signal_noise_ = signal_noise;
  orig_signal_var_ = signal_var;
  orig_lengthscale_ = lengthscale;
}

void ARD_SE_Kernel::set_parameters(double signal_noise, double signal_var, double lengthscale, double lengthscale2)
{
  signal_noise_ = exp(signal_noise);
  signal_var_ = exp(2 * signal_var);
  lengthscale_(0) = exp(lengthscale);
  lengthscale_(1) = exp(lengthscale2);

  orig_signal_noise_ = signal_noise;
  orig_signal_var_ = signal_var;
  orig_lengthscale_(0) = lengthscale;
  orig_lengthscale_(1) = lengthscale2;
}

Vector4d ARD_SE_Kernel::get_parameters()
{
  Vector4d parameters = {orig_signal_noise_, orig_signal_var_, orig_lengthscale_(0), orig_lengthscale_(1)};
  return parameters;
}