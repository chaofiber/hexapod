#pragma once

#include "optLib/ObjectiveFunction.h"
#include "optLib/ObjectiveFunctionMinimizer.h"

#include <utils/mathUtils.h>

class GradientBasedMinimizer : public ObjectiveFunctionMinimizer {
public:
	GradientBasedMinimizer(double solveResidual = 1e-5, double lineSearchStartingStepSize = 1.0, int maxLineSearchSteps = 10) {
		this->solveResidual = solveResidual;
		this->lineSearchStartingStepSize = lineSearchStartingStepSize;
		this->maxLineSearchSteps = maxLineSearchSteps;
	}

	bool minimize(const ObjectiveFunction *function, dVector &x, int maxIterations = 100) const override {
		//this will be the search direction
		dVector dx(x.size());

		if (printOutput)
			std::cout << "Initial objective function value: " << function->evaluate(x) << std::endl;

		for (int i = 0; i < maxIterations; i++) {
			function->prepareForOptimizationStep(x);
			dx.setZero();
			computeSearchDirection(function, x, dx);

			if (dx.norm() < solveResidual)
				return true;

			doLineSearch(function, dx, x);

			if (printOutput)
				std::cout << "Objective function value after optimization step: " << function->evaluate(x) << std::endl;
		}
		return false;
	}

protected:
	//this function will compute a search (i.e. descent!) direction dx evaluated at the current solution x.
	virtual void computeSearchDirection(const ObjectiveFunction *function, const dVector &x, dVector& dx) const {
	    function->getGradient(x,dx);
        dx *= -1;
	};

	// given the objective `function` and the search direction `dx`, update the candidate `x` via a bisection line search
	virtual void doLineSearch(const ObjectiveFunction *function, const dVector& dx, dVector& x) const {
		// line search
		double alpha = lineSearchStartingStepSize; // initial step size
		// these will be candidate solutions that we try out
		dVector xStart(x);
		double initialFunctionValue = function->evaluate(xStart);
		int lSteps = MAX(maxLineSearchSteps, 0);

		for (int j = 0; j <= lSteps; j++) {
			// try a step of size `alpha` in the search (descent!) direction
			x = xStart + dx * alpha;

			// if the new function value is greater than the initial one, we've gone uphill. Reduce alpha and try again...
			double f = function->evaluate(x);
			if (!std::isfinite(f) || f > initialFunctionValue)
				alpha /= 2.0;
			else
				return;
		}
	}

public:
	double solveResidual = 1e-5;
	double lineSearchStartingStepSize = 1.0;
	int maxLineSearchSteps = 10;
};

class GradientDescentMinimizer : public GradientBasedMinimizer {
public:
	GradientDescentMinimizer(double solveResidual = 1e-5, double lineSearchStartingStepSize = 1.0, int maxLineSearchSteps = 10) : GradientBasedMinimizer(solveResidual, lineSearchStartingStepSize, maxLineSearchSteps) {
	}

protected:
	virtual void computeSearchDirection(const ObjectiveFunction *function, const dVector &x, dVector& dx) const {
		function->getGradient(x, dx);
		dx *= -1;
	}
};

class GradientDescentMomentum : public GradientBasedMinimizer {
public:
	GradientDescentMomentum(double solveResidual = 1e-5, double lineSearchStartingStepSize = 1.0, int maxLineSearchSteps = 10) : GradientBasedMinimizer(solveResidual, lineSearchStartingStepSize, maxLineSearchSteps) {
	}


protected:
	void computeSearchDirection(const ObjectiveFunction *function, const dVector &x, dVector& dx) const override {
		// if gradient hasn't been set yet, set it to zero
		if(gradient.size() == 0){
			gradient.resize(x.size());
			gradient.setZero();
		}
		// compute new gradient
		dVector newGradient;
		function->getGradient(x, newGradient);
		// search direction is augmented with old gradient
		dx = -(newGradient + alpha*gradient);
		// save gradient for next step
		gradient = newGradient;
	}

public:
	double alpha = 0.5;
	mutable dVector gradient;
};



