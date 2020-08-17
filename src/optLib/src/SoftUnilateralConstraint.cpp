#include <optLib/SoftUnilateralConstraint.h>

SoftUnilateralConstraint::SoftUnilateralConstraint(double l, double stiffness, double epsilon) {
	this->limit = l;
	this->epsilon = epsilon;
	a1 = stiffness;
	b1 = -0.5 * a1 * epsilon;
	c1 = -1.0 / 3 * (-b1 - a1 * epsilon) * epsilon - 1.0 / 2 * a1 * epsilon * epsilon - b1 * epsilon;

	a2 = (-b1 - a1 * epsilon) / (epsilon * epsilon);
	b2 = a1;
	c2 = b1;
	d2 = c1;
}

SoftUnilateralConstraint::~SoftUnilateralConstraint() {
}

void SoftUnilateralConstraint::setLimit(double l)
{
	limit = l;
}

void SoftUnilateralConstraint::setEpsilon(double eps)
{
	epsilon = eps;
	b1 = -0.5 * a1 * epsilon;
	c1 = -1.0 / 3 * (-b1 - a1 * epsilon) * epsilon - 1.0 / 2 * a1 * epsilon * epsilon - b1 * epsilon;

	a2 = (-b1 - a1 * epsilon) / (epsilon * epsilon);
	b2 = a1;
	c2 = b1;
	d2 = c1;
}

// returns 1/2 C'C, where C is the current set of equality constraint values
double SoftUnilateralConstraint::evaluate(double x) {
	x = x - limit;
	if (x < 0)
		return 0.5 * a1 * x * x + b1 * x + c1;
	if (x < epsilon) 
		return 1.0 / 3 * a2 * x * x * x + 0.5 * b2 * x * x + c2 * x + d2;
	return 0;
}

double SoftUnilateralConstraint::computeDerivative(double x) {
	x = x - limit;
	if (x < 0)
		return a1 * x + b1;
	if (x < epsilon)
		return a2 * x * x + b2 * x + c2;
	return 0;
}

double SoftUnilateralConstraint::computeSecondDerivative(double x) {
	x = x - limit;
	if (x < 0)
		return a1;
	if (x < epsilon)
		return 2 * a2 * x + b2;
	return 0;
}



SoftUpperLimitConstraint::SoftUpperLimitConstraint(double u, double stiffness, double epsilon){
	this->limit = u;
	this->epsilon = epsilon;
	a1 = stiffness;

	b1 = 0.5*a1*epsilon;
	c1 = 1./6. * a1*epsilon*epsilon;
	a2 = 1./(2.*epsilon)*a1;
	b2 = a1;
	c2 = 0.5*a1*epsilon;
	d2 = 1./6.*a1*epsilon*epsilon;
}

SoftUpperLimitConstraint::~SoftUpperLimitConstraint()
{

}

void SoftUpperLimitConstraint::setLimit(double l)
{
	limit = l;
}

void SoftUpperLimitConstraint::setEpsilon(double eps)
{
	epsilon = eps;
}

// returns 1/2 C'C, where C is the current set of equality constraint values
double SoftUpperLimitConstraint::evaluate(double x) {
	x = x - limit;
	if (x > 0)
		return 0.5 * a1 * x * x + b1 * x + c1;
	if (x > -epsilon)
		return 1.0 / 3 * a2 * x * x * x + 0.5 * b2 * x * x + c2 * x + d2;
	return 0;
}

double SoftUpperLimitConstraint::computeDerivative(double x) {
	x = x - limit;
	if (x > 0)
		return a1 * x + b1;
	if (x > -epsilon)
		return a2 * x * x + b2 * x + c2;
	return 0;
}

double SoftUpperLimitConstraint::computeSecondDerivative(double x) {
	x = x - limit;
	if (x > 0)
		return a1;
	if (x > -epsilon)
		return 2 * a2 * x + b2;
	return 0;
}

SoftBoundConstraint::SoftBoundConstraint(double lLimit, double uLimit, double relEps, double stiffness) : l(lLimit, stiffness, fabs(uLimit - lLimit) * relEps), u(uLimit, stiffness, fabs(uLimit - lLimit) * relEps){

}

SoftBoundConstraint::~SoftBoundConstraint() {}

double SoftBoundConstraint::evaluate(double x)
{
	return l.evaluate(x) + u.evaluate(x);
}

double SoftBoundConstraint::computeDerivative(double x)
{
	return l.computeDerivative(x) + u.computeDerivative(x);
}

double SoftBoundConstraint::computeSecondDerivative(double x){
	return l.computeSecondDerivative(x) + u.computeSecondDerivative(x);
}

SoftSymmetricBarrierConstraint::SoftSymmetricBarrierConstraint(double limit) : SoftBoundConstraint(-fabs(limit), fabs(limit)) {

}

