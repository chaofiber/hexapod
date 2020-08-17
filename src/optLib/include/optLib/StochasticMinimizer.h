#pragma once

#include "optLib/ObjectiveFunctionMinimizer.h"

#include <random>

class StochasticMinimizer : public ObjectiveFunctionMinimizer
{
public:
	StochasticMinimizer(const dVector &upperLimit = dVector(), const dVector &lowerLimit = dVector(), double fBest = HUGE_VAL)
		: searchDomainMax(upperLimit), searchDomainMin(lowerLimit), fBest(fBest) {
		fBest = HUGE_VAL;

		// initial random device and set uniform distribution to [0, 1]
		rng.seed(std::random_device()());
		dist = std::uniform_real_distribution<>(0.0,1.0);
	}

	virtual ~StochasticMinimizer() {}

	bool minimize(const ObjectiveFunction *function, dVector &x, int maxIterations = 100) const override {
		for (int i = 0; i < maxIterations; ++i) {
			function->prepareForOptimizationStep();
			// for each element of `x`, generate a random variable in the search region
			dVector xr(x.size());
			for (int i = 0; i < x.size(); ++i) {
				xr[i] = dist(rng) * (searchDomainMax[i] - searchDomainMin[i]) + searchDomainMin[i];
			}

			// if function value at new `x` is smaller, let's keep it
			double f = function->evaluate(xr);
			if(f < fBest){
				x = xr;
				fBest = f;
			}
		}
		return false;
	}

public:
	int iterations = 1;
	dVector searchDomainMax, searchDomainMin;

	mutable double fBest;
	mutable std::uniform_real_distribution<double> dist;
	mutable std::mt19937 rng;
};
