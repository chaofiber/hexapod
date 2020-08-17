#pragma once

#include "optLib/ObjectiveFunction.h"

class ObjectiveFunctionMinimizer{
public:
	// Returns true if a minimum of the objective `function` has been found.
	// `x` is the initial/current candidate, and will also store the next
	// candidate once the method has returned. The method will perform at 
	// most maxIterations steps of the iterative optimization procedure.
	virtual bool minimize(const ObjectiveFunction *function, dVector &x, int maxIterations = 100) const = 0;

	bool printOutput = false;
};
