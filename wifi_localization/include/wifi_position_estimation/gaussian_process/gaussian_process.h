#ifndef PROJECT_GAUSSIAN_PROCESS_H
#define PROJECT_GAUSSIAN_PROCESS_H
#include "kernel.h"
#include <Eigen/Dense>
#include <map>
#include <wifi_position_estimation/precomputedDataPoint.h>

using namespace Eigen;

/**
 * Determines the sign of a given value.
 * @param val Value the sign is to be determined of
 * @return Sign as integer, so either 1 or -1
 */
template <typename T> int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

/**
 * Process class
 * This is a Gaussian process. Given a set of training coordinates and training observations, it can be used to predict
 * the probability for new given coordinates and observations.
 */
class Process
{
public:
  /**
   * Constructor
   * @param training_coords Coordinates, that usually have been recorded beforehand
   * @param training_observs Corresponding signal strengths to the coordinates
   * @param signal_noise hyperparameter of the kernel
   * @param signal_var hyperparameter of the kernel
   * @param lengthscale hyperparameter of the kernel
   */
  Process(Matrix<double, Dynamic, 2> &training_coords, Matrix<double, Dynamic, 1> &training_observs,
          double signal_noise = 0.0, double signal_var = 0.0, double lengthscale = 0.0);

  /**
   * Trains the parameters, using the optimization class and the provided rprop algorithm
   * @param starting_point Starting point of the optimization algorithm
   */
  void train_params(Matrix<double, Dynamic, 1> starting_point);

  /**
   * Returns the probability that at the given coordinates, x and y, the observation z was made.
   * @param x x coordinate
   * @param y y coordinate
   * @param z observation
   * @return probability
   */
  double probability(double x, double y, double z);

  /**
   * Returns the probability, given the observation z and a precomputed mean and variance.
   * @param mean
   * @param variance
   * @param z
   * @return probability that the observation was made at the coordinate the mean and variance were computed with
   */
  double probability_precomputed(double mean, double variance, double z);

  /**
   * Sets the training sets to new values. The values are normalized, before they are saved.
   * @param training_coords
   * @param training_observs
   */
  void set_training_values(Matrix<double, Dynamic, 2> &training_coords, Matrix<double, Dynamic, 1> &training_observs);

  /**
   * Set the hyperparameters to new values
   * @param params
   */
  void set_params(const Matrix<double, Dynamic, 1> &params);
  void set_params(double signal_noise, double signal_var, double lengthscale);

  /**
   * Log likelihood of the Gaussian process. This can be used to determine how well the process fits to the training
   * sets. Depending on the given hyperparameters this will change.
   * @return log likelihood
   */
  double log_likelihood();

  /**
   * Gradient of the log likelihood. This is used in optimization algorithms.
   * @return
   */
  Matrix<double, Dynamic, 1> log_likelihood_gradient();

  /**
   * Get the hyperparameters
   * @return hyperparameters as Vector
   */
  Vector3d get_params();

  /**
   * Precomputes the mean and variance for the given position. This can be used later for the position estimation
   * @param data Will be modified with the computed mean and variance
   * @param position The position the mean and variance are supposed to be computed for
   */
  void precompute_data(PrecomputedDataPoint& data, Eigen::Vector2d position);

private:
  /**
   * Computes the covariance vector between the provided coordinates and the training_coordinates
   * @param x x coordinate
   * @param y y coordinate
   */
  void compute_cov_vector(double x, double y);

  /**
   * This updates the covariance matrix K and the inverse of it K_inv.
   */
  void update_covariance_matrix();

  Matrix<double, Dynamic, 2> training_coords_;
  Matrix<double, Dynamic, 1> training_observs_;
  Kernel kernel_;
  Matrix<double, Dynamic, Dynamic> K_;
  Matrix<double, Dynamic, Dynamic> K_inv_;
  Matrix<double, Dynamic, 1> cov_vector_;

  /// Number of training coordinates and training observations
  int n;

  /// Values used for the normalization of the training data sets
  double x_mean_;
  double y_mean_;
  double x_std_;
  double y_std_;
};


#endif //PROJECT_GAUSSIAN_PROCESS_H

