#pragma once

#include "optLib/ObjectiveFunction.h"
#include "optLib/ObjectiveFunctionMinimizer.h"
#include <LBFGS.h>

class LBFGSObjective
{
public:
	LBFGSObjective(const ObjectiveFunction *function) : function(function) {}

	double operator()(const dVector& x, dVector& grad)
	{
		grad.setZero(x.size());
		function->addGradientTo(x, grad);
		return function->evaluate(x);
	}

public:
	const ObjectiveFunction *function;
};

class LBFGSMinimizer : public ObjectiveFunctionMinimizer {
public:
	LBFGSMinimizer(double epsilon = 1e-6)
		: epsilon(epsilon){
	}

	bool minimize(const ObjectiveFunction *function, dVector &x, int maxIterations = 100) const override {
		// Set up parameters
		LBFGSpp::LBFGSParam<double> param;
		param.epsilon = 1e-6;
		param.max_iterations = maxIterations;

		// Create solver and function object
		LBFGSpp::LBFGSSolver<double> solver(param);
		LBFGSObjective fun(function);

		function->prepareForOptimizationStep(x);
		// x will be overwritten to be the best point found
		double fx;
		int niter = solver.minimize(fun, x, fx);

		return false;
	}
public:
	double epsilon = 1e-6;
};
