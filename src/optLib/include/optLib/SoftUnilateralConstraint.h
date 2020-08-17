#pragma once

#include <math.h>
#include <optLib/ObjectiveFunction.h>

/*!
	class used to model unilateral constraints of the type x > l using a C2 penalty energy f(x).
		- l is the lower limit that x needs to be greater than
		- if x > l, then the energy of the constraint, its gradient and hessian are all 0 (i.e. inactive)
		- epsilon is the value away from the limit (how much larger should x be compared to l) after which f(x) = 0
		- stiffness controls the rate at which f(x) increases if x < l
*/
class SoftUnilateralConstraint {
private:
	double a1, b1, c1, a2, b2, c2, d2, epsilon;
public:
	double limit = 0;
public:

	SoftUnilateralConstraint(double l, double stiffness, double epsilon);

	virtual ~SoftUnilateralConstraint();

	void setLimit(double l);
	void setEpsilon(double eps);

	//comptue f(x)
	double evaluate(double x);

	//compute df/dx
	double computeDerivative(double x);

	//compute ddf/dxdx
	double computeSecondDerivative(double x);
};

typedef SoftUnilateralConstraint SoftLowerLimitConstraint;

/*!
class used to model unilateral constraints of the type x < u using a C2 penalty energy f(x).
	- u is the upper limit that x needs to be less than
	- if x < u, then the energy of the constraint, its gradient and hessian are all 0 (i.e. inactive)
	- epsilon is the value away from the limit (how much smaller should x be compared to u) after which f(x) = 0
	- stiffness controls the rate at which f(x) increases if x > u
*/
class SoftUpperLimitConstraint {
private:
	double a1, b1, c1, a2, b2, c2, d2, epsilon;
	double limit = 0;
public:
	SoftUpperLimitConstraint(double l, double stiffness, double epsilon);

	virtual ~SoftUpperLimitConstraint();

	void setLimit(double l);
	void setEpsilon(double eps);

	//comptue f(x)
	double evaluate(double x);

	//compute df/dx
	double computeDerivative(double x);

	//compute ddf/dxdx
	double computeSecondDerivative(double x);
};

class SoftBoundConstraint {
protected:
	SoftLowerLimitConstraint l;
	SoftUpperLimitConstraint u;
public:
	SoftBoundConstraint(double lLimit, double uLimit, double relEps = 0.1, double stiffness = 1.0);
	virtual ~SoftBoundConstraint();

	//comptue f(x)
	double evaluate(double x);

	//compute df/dx
	double computeDerivative(double x);

	//compute ddf/dxdx
	double computeSecondDerivative(double x);
};

class SoftSymmetricBarrierConstraint : public SoftBoundConstraint {
public:
	SoftSymmetricBarrierConstraint(double limit);

	virtual ~SoftSymmetricBarrierConstraint() {}

	void setLimit(double limit) {
		l = SoftLowerLimitConstraint(-fabs(limit), 1, fabs(limit) * 0.1);
		u = SoftUpperLimitConstraint( fabs(limit), 1, fabs(limit) * 0.1);
	}

};

