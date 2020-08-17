#pragma once

#include <vector>
#include <Eigen/Sparse>

#include <utils/mathDefs.h>

class ObjectiveFunction {
public:
	std::string description = "";

	ObjectiveFunction(std::string descr = "") {
		description = descr;
	}

	//this function is called before each and every optimization step starts -- can be used to precompute various quantities, set regularizer parameters, etc...
	virtual void prepareForOptimizationStep(const dVector& x) const {
	}

	//this should always return the current value of the objective function
	virtual double evaluate(const dVector& x) const = 0;

	virtual void getGradient(const dVector& x, dVector& grad) const;

	dVector computeGradient(const dVector& x) const {
		dVector grad;
		getGradient(x, grad);
		return grad;
	}

    virtual void getHessian(const dVector &x, SparseMatrix &hessian) const;
    
	void addFiniteDifferenceGradientTo(const dVector& x, dVector& grad) const;

	void addFiniteDifferenceHessianEntriesTo(const dVector& x, std::vector<MTriplet>& hessianEntries) const;

	void testGradientWithFD(const dVector& p);

	void testHessianWithFD(const dVector& p);

	void testHessianPSD(const dVector& p);

	virtual void printObjectiveDetails(const dVector& p) {}
};


/*
	The subobjective class need only implement the addGradientTo and addHessianEntriesTo method. If they don't, it will defaut to FD.
	But note: it is assumed that the evaluate/addGradientTo implementations of the sub objectives do set parameters themeselves for FD to work, rather than relaying on a parent function to do it once for all sub objectives...
*/
class SubObjective : public ObjectiveFunction {
public: 	
	double weight = 1.0;

	SubObjective(std::string descr = "", double w = 1.0) : ObjectiveFunction(descr) {
		weight = w;
	}

	virtual ~SubObjective() {}

	virtual void addGradientTo(const dVector& x, dVector& grad) const {
		addFiniteDifferenceGradientTo(x, grad);
	}
	virtual void addHessianEntriesTo(const dVector& x, std::vector<MTriplet>& hessianEntries) const {
		addFiniteDifferenceHessianEntriesTo(x, hessianEntries);
	}

	virtual void getGradient(const dVector& x, dVector& grad) const final override {
		resize(grad, (int)x.size());
		addGradientTo(x, grad);
	}

	virtual void getHessian(const dVector& x, SparseMatrix& hessian) const final override {
		hessian.resize((int)x.size(), (int)x.size());
		hessian.setZero();
		std::vector<MTriplet> triplets;
		triplets.clear();
		addHessianEntriesTo(x, triplets);
		hessian.setFromTriplets(triplets.begin(), triplets.end());
	}
};

