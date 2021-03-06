/** Exponential Gating Function class
 *  Implements an exponential gating function, u=f(x), x:[0 1]->u:[u0 u_end],
 *  where u0 is the initial and u_end the final value.
 * The output of the gating function is:
 *    u = u0*exp(-a_u*x);
 *   du = -a_u*u0*exp(-a_u*x);
 */

#ifndef EXPONENTIAL_GATING_FUNCTION_H
#define EXPONENTIAL_GATING_FUNCTION_H

#include <dmp_lib/GatingFunction/GatingFunction.h>

namespace as64_
{

class ExpGatingFunction: public GatingFunction
{
public:
  ExpGatingFunction(double u0 = 1.0, double u_end = 0.005);

  virtual void init(double u0, double u_end);

  virtual double getOutput(double x) const;
  virtual arma::rowvec getOutput(const arma::rowvec &x) const;

  virtual double getOutputDot(double x) const;
  virtual arma::rowvec getOutputDot(const arma::rowvec &x) const;

  virtual double getPartDev_1oTau(double t, double x) const;

private:
  double u0; ///< initial value of the gating function
  double a_u; ///< the rate of evolution of the gating function

}; // class ExpGatingFunction

} // namespace as64_

#endif // EXPONENTIAL_GATING_FUNCTION_H
