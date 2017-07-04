#include <iostream>
#include "wifi_position_estimation/gaussian_process/optimizer.h"
#include <ros/ros.h>
#include <limits>

void Optimizer::rprop(Matrix<double, Dynamic, 1> &starting_point, int n, double delta0, double delta_min, double delta_max,
                      double eta_minus, double eta_plus, double eps_stop)
{
  Matrix<double, Dynamic, 1> delta(4,1);
  delta << delta0, delta0, delta0, delta0;
  Matrix<double, Dynamic, 1> grad_old(4,1);
  grad_old << 0.0, 0.0, 0.0, 0.0;
  Matrix<double, Dynamic, 1> params(4,1);
  params = starting_point;
  Matrix<double, Dynamic, 1> best_params(4,1);
  best_params = params;
  double best = log(0);
  if(!std::isfinite(best))
    best = -std::numeric_limits<double>::max();
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
    p_.set_params(params);
    double lik = -p_.log_likelihood();

    if(std::isnan(grad(0)))
      std::cout << "Gradient is nan with params: " << params(0) << " " << params(1) << " " << params(2) << " " << params(3) << std::endl;

    if(lik > best && std::isfinite(lik))
    {
      best = lik;
      best_params = params;
    }
    //std::cout << "likelihood: " << lik << std::endl;
    ROS_INFO("Iteration %d of %d", i+1, n);
    ROS_INFO("Current parameters: %f, %f, %f, %f \n With likelihood: %f \n With gradient: %f, %f, %f, %f", params(0), params(1), params(2), params(3), lik, grad_old(0), grad_old(1), grad_old(2), grad_old(3));

    if(grad_old.norm() <= eps_stop)
    {
      break;
    }
  }
  //std::cout << "best likelihood: " << best << std::endl;
  ROS_INFO("Found parameters: %f, %f, %f, %f \n With likelihood: %f \n With gradient: %f, %f, %f, %f", best_params(0), best_params(1), best_params(2), best_params(3), best, grad_old(0), grad_old(1), grad_old(2), grad_old(3));
  if(best == -std::numeric_limits<double>::max())
  {
    best_params(0) = 0.0;
    best_params(1) = 0.0;
    best_params(2) = 0.0;
    best_params(3) = 0.0;
  }

  p_.set_params(best_params);
}

void Optimizer::conjugate_gradient(size_t n = 4)
{
	const double INT = 0.1; // don't reevaluate within 0.1 of the limit of the current bracket
	const double EXT = 3.0; // extrapolate maximum 3 times the current step-size
	const int MAX = 20;		// max 20 function evaluations per line search
	const double RATIO = 10;	// maximum allowed slope ratio
	const double SIG = 0.1, RHO = SIG/2;
	/* SIG and RHO are the constants controlling the Wolfe-
	   Powell conditions. SIG is the maximum allowed absolute ratio between
	   previous and new slopes (derivatives in the search direction), thus setting
	   SIG to low (positive) values forces higher precision in the line-searches.
	   RHO is the minimum allowed fraction of the expected (from the slope at the
	   initial point in the linesearch). Constants must satisfy 0 < RHO < SIG < 1.
	   Tuning of SIG (depending on the nature of the function to be optimized) may
	  speed up the minimization; it is probably not worth playing much with RHO.
	*/

	/* The code falls naturally into 3 parts, after the initial line search is
	   started in the direction of steepest descent. 1) we first enter a while loop
	   which uses point 1 (p1) and (p2) to compute an extrapolation (p3), until we
	   have extrapolated far enough (Wolfe-Powell conditions). 2) if necessary, we
	   enter the second loop which takes p2, p3 and p4 chooses the subinterval
	   containing a (local) minimum, and interpolates it, unil an acceptable point
	   is found (Wolfe-Powell conditions). Note, that points are always maintained
	   in order p0 <= p1 <= p2 < p3 < p4. 3) compute a new search direction using
	   conjugate gradients (Polack-Ribiere flavour), or revert to steepest if there
	   was a problem in the previous line-search. Return the best value so far, if
	   two consecutive line-searches fail, or whenever we run out of function
	   evaluations or line-searches. During extrapolation, the "f" function may fail
	   either with an error or returning Nan or Inf, and maxmize should handle this
	   gracefully.
	*/


	bool ls_failed = false;									//prev line-search failed
	double f0 = -p_.log_likelihood();						//initial negative marginal log likelihood
	Eigen::VectorXd df0 = -p_.log_likelihood_gradient();	//initial gradient
	Eigen::VectorXd X = p_.get_params();			//hyper parameters

	Eigen::VectorXd s = -df0;								//initial search direction
	double d0 = -s.dot(s);									//initial slope
	double x3 = 1/(1-d0);

	double f3 = 0;
	double d3 = 0;
	Eigen::VectorXd df3 = df0;

	double x2 = 0, x4 = 0;
	double f2 = 0, f4 = 0;
	double d2 = 0, d4 = 0;

	for (unsigned int i = 0; i < n; ++i)
	{
		//copy current values
		Eigen::VectorXd X0 = X;
		double F0 = f0;
		Eigen::VectorXd dF0 = df0;
		unsigned int M = std::min(MAX, (int)(n-i));

		while(1)											//keep extrapolating until necessary
		{
			x2 = 0;
			f2 = f0;
			d2 = d0;
			f3 = f0;
			df3 = df0;
			double success = false;

			while( !success && M>0)
			{
				M --;
				i++;
				p_.set_params(X+s*x3);
				f3 = -p_.log_likelihood();
				df3 = -p_.log_likelihood_gradient();

				bool nanFound = false;
				//test NaN and Inf's
				for (int j = 0; j < df3.rows(); ++j)
				{
					if(isnan(df3(j)))
					{
						nanFound = true;
						break;
					}
				}
				if(!isnan(f3) && !isinf(f3) && !nanFound)
					success = true;
				else
				{
					x3 = (x2+x3)/2; 						// if fail, bissect and try again
				}
			}
			//keep best values
			if(f3 < F0)
			{
				X0 = X+s*x3;
				F0 = f3;
				dF0 = df3;
			}

			d3 = df3.dot(s);								// new slope

			if( (d3 > SIG*d0) || (f3 >  f0+x3*RHO*d0) || M == 0) // are we done extrapolating?
			{
				break;
			}

			double x1 = x2; double f1 = f2; double d1 = d2;	// move point 2 to point 1
			x2 = x3; f2 = f3; d2 = d3;						// move point 3 to point 2
			double A = 6*(f1-f2) + 3*(d2+d1)*(x2-x1);				// make cubic extrapolation
			double B = 3*(f2-f1) - (2*d1+d2)*(x2-x1);
			x3 = x1-d1*(x2-x1)*(x2-x1)/(B+sqrt(B*B -A*d1*(x2-x1)));
			if(isnan(x3) || x3 < 0 || x3 > x2*EXT)			// num prob | wrong sign | beyond extrapolation limit
				x3 = EXT*x2;
			else if(x3 < x2+INT*(x2-x1))					// too close to previous point
				x3 = x2+INT*(x2-x1);
		}

		while( ( (abs(d3) > -SIG*d0) || (f3 > f0+x3*RHO*d0) ) && (M > 0))	// keep interpolating
		{
			if( (d3 > 0) || (f3 > f0+x3*RHO*d0) )			// choose subinterval
			{												// move point 3 to point 4
				x4 = x3;
				f4 = f3;
				d4 = d3;
			}
			else
			{
				x2 = x3;									//move point 3 to point 2
				f2 = f3;
				d2 = d3;
			}

			if(f4 > f0)
				x3 = x2 - (0.5*d2*(x4-x2)*(x4-x2))/(f4-f2-d2*(x4-x2));	// quadratic interpolation
			else
			{
				double A = 6*(f2-f4)/(x4-x2)+3*(d4+d2);
				double B = 3*(f4-f2)-(2*d2+d4)*(x4-x2);
				x3 = x2+sqrt(B*B-A*d2*(x4-x2)*(x4-x2) -B)/A;
			}

			if(isnan(x3) || isinf(x3))
				x3 = (x2+x4)/2;

			x3 = std::max(std::min(x3, x4-INT*(x4-x2)), x2+INT*(x4-x2));

			p_.set_params(X+s*x3);
			f3 = -p_.log_likelihood();
			df3 = -p_.log_likelihood_gradient();

			if(f3 < F0)												// keep best values
			{
				X0 = X+s*x3;
				F0 = f3;
				dF0 = df3;
			}

			M--;
			i++;
			d3 = df3.dot(s);										// new slope
		}

		if( (abs(d3) < -SIG*d0) && (f3 < f0+x3*RHO*d0))
		{
			X = X+s*x3;
			f0 = f3;
			s = (df3.dot(df3)-df0.dot(df3)) / (df0.dot(df0))*s - df3;	// Polack-Ribiere CG direction
			df0 = df3;													// swap derivatives
			d3 = d0; d0 = df0.dot(s);
			if(d0 > 0)													// new slope must be negative
			{															// otherwise use steepest direction
				s = -df0;
				d0 = -s.dot(s);
			}

			x3 = x3 * std::min(RATIO, d3/(d0-std::numeric_limits< double >::min()));	// slope ratio but max RATIO
			ls_failed = false;																// this line search did not fail
		}
		else
		{														// restore best point so far
			X = X0;
			f0 = F0;
			df0 = dF0;

			if(ls_failed || i >= n)								// line search failed twice in a row
				break;											// or we ran out of time, so we give up

			s = -df0;
			d0 = -s.dot(s);										// try steepest
			x3 = 1/(1-d0);
			ls_failed = true;									// this line search failed
		}


	}
	p_.set_params(X);
}