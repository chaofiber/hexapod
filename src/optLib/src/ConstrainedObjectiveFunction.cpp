#include <iostream>

#include "optLib/ConstrainedObjectiveFunction.h"
#include "utils/logger.h"
#include "utils/mathUtils.h"

void ConstrainedObjectiveFunction::addEstimatedEqualityConstraintsJacobianEntriesTo(DynamicArray<MTriplet>& jacobianEntries, const dVector& params) {
	dVector pSet = params;

	int nConstraints = getEqualityConstraintCount();
	int p = (int)pSet.size();
	//the jacobian should have dimensions nConstraints x p

	if (nConstraints > 0) {
		double dp = 10e-6;

		dVector C_P(nConstraints), C_M(nConstraints), J_i_col(nConstraints);
		//this is a very slow method that evaluates the jacobian of the objective function through FD...
		for (int i = 0; i<p; i++) {
			double tmpVal = pSet(i);
			pSet(i) = tmpVal + dp;

			C_P = getEqualityConstraintValues(pSet);

			pSet(i) = tmpVal - dp;
			C_M = getEqualityConstraintValues(pSet);

			//now reset the ith param value
			pSet(i) = tmpVal;
			J_i_col = (C_P - C_M) / (2.0 * dp);

			//each vector is a column vector of the hessian, so copy it in place...
			for (int j = 0;j<nConstraints;j++)
				if (!IS_ZERO(J_i_col(j)))
					jacobianEntries.push_back(MTriplet(j, i, J_i_col[j]));
		}
	}
}

void ConstrainedObjectiveFunction::addEstimatedInequalityConstraintsJacobianEntriesTo(DynamicArray<MTriplet>& jacobianEntries, const dVector& params) {
	dVector pSet = params;

	int nConstraints = getInequalityConstraintCount();
	int p = (int)pSet.size();
	//the jacobian should have dimensions nConstraints x p

	if (nConstraints > 0) {
		double dp = 10e-6;

		dVector C_P(nConstraints), C_M(nConstraints), J_i_col(nConstraints);
		//this is a very slow method that evaluates the jacobian of the objective function through FD...
		for (int i = 0;i<p;i++) {
			double tmpVal = pSet(i);
			pSet(i) = tmpVal + dp;
			C_P = getInequalityConstraintValues(pSet);

			pSet(i) = tmpVal - dp;
			C_M = getInequalityConstraintValues(pSet);

			//now reset the ith param value
			pSet(i) = tmpVal;
			J_i_col = 1.0 / (2.0*dp)*C_P + -1.0 / (2.0*dp)*C_M;

			//each vector is a column vector of the hessian, so copy it in place...
			for (int j = 0;j<nConstraints;j++)
				if (!IS_ZERO(J_i_col(j)))
					jacobianEntries.push_back(MTriplet(j, i, J_i_col[j]));
		}
	}
}

void ConstrainedObjectiveFunction::testJacobiansWithFD(const dVector& p) {
    Logger::print("Constrained Objective Function: testing Jacobians...\n");


	DynamicArray<MTriplet> jacobianEntries;

	SparseMatrix FDJacobian;
	addEstimatedEqualityConstraintsJacobianEntriesTo(jacobianEntries, p);
	resize(FDJacobian, getEqualityConstraintCount(), (int)p.size());
	FDJacobian.setFromTriplets(jacobianEntries.begin(), jacobianEntries.end());

	jacobianEntries.clear();

	SparseMatrix analyticJacobian;
	addEqualityConstraintsJacobianEntriesTo(jacobianEntries, p);
	resize(analyticJacobian, getEqualityConstraintCount(), (int)p.size());
	analyticJacobian.setFromTriplets(jacobianEntries.begin(), jacobianEntries.end());

	jacobianEntries.clear();

	Logger::logPrint("Function Constraints: testing equality constraints jacobian...\n");
	for (int i = 0;i<FDJacobian.rows();i++) {
		for (int j = 0;j<(int)p.size();j++) {
			double err = FDJacobian.coeff(i, j) - analyticJacobian.coeff(i, j);
			if (fabs(err) > 0.001)
				Logger::logPrint("Mismatch element %d,%d: Analytic val: %lf, FD val: %lf. Error: %lf\n", i, j, analyticJacobian.coeff(i, j), FDJacobian.coeff(i, j), err);
		}
	}

	addEstimatedInequalityConstraintsJacobianEntriesTo(jacobianEntries, p);
	resize(FDJacobian, getInequalityConstraintCount(), (int)p.size());
	FDJacobian.setFromTriplets(jacobianEntries.begin(), jacobianEntries.end());

	jacobianEntries.clear();

	addInequalityConstraintsJacobianEntriesTo(jacobianEntries, p);
	resize(analyticJacobian, getInequalityConstraintCount(), (int)p.size());
	analyticJacobian.setFromTriplets(jacobianEntries.begin(), jacobianEntries.end());

	jacobianEntries.clear();

	Logger::logPrint("Function Constraints: testing inequality constraints jacobian...\n");
	for (int i = 0;i<FDJacobian.rows();i++) {
		for (int j = 0;j<(int)p.size();j++) {
			double err = FDJacobian.coeff(i, j) - analyticJacobian.coeff(i, j);
			if (fabs(err) > 0.001)
				Logger::logPrint("Mismatch element %d,%d: Analytic val: %lf, FD val: %lf. Error: %lf\n", i, j, analyticJacobian.coeff(i, j), FDJacobian.coeff(i, j), err);
		}
	}
}

void ConstrainedObjectiveFunction::printConstraintErrors(const dVector& p, double eqTol, double iqTol){
	dVector de = getEqualityConstraintValues(p);
	if (de.size() > 0) {
		dVector dtargets = getEqualityConstraintsTargetValues();
		dVector::Index maxIndex;
		double maxError = (de - dtargets).cwiseAbs().maxCoeff(&maxIndex);
		if (maxError > eqTol) {
			Logger::logPrint("-----> Max equality constraint error: %10.10lf at index %d\n", maxError, maxIndex);
		}
		else {
			Logger::logPrint("   Equality constraints are within the tolerance.\n");
		}
	}

	de = getInequalityConstraintValues(p);
	if (de.size() > 0) {
		dVector::Index maxIndexMin;
		dVector::Index maxIndexMax;
		double maxErrorMin = (-de + getInequalityConstraintsMinValues()).maxCoeff(&maxIndexMin);
		double maxErrorMax = (de - getInequalityConstraintsMaxValues()).maxCoeff(&maxIndexMax);
		dVector::Index maxIndex;
		double maxError = 0;
		if (maxErrorMin > maxErrorMax) {
			maxError = maxErrorMin;
			maxIndex = maxIndexMin;
		}
		else {
			maxError = maxErrorMax;
			maxIndex = maxIndexMax;
		}
		if (maxError > iqTol) {
			Logger::logPrint("------> Max inequality constraint error: %10.10lf at index %d (%lf < %lf < %lf) \n", maxError, maxIndex, getInequalityConstraintsMinValues()(maxIndex),de(maxIndex), getInequalityConstraintsMaxValues()(maxIndex));
		}
		else {
			Logger::logPrint("   Inequality constraints are within the tolerance.\n");
		}
	}


	dVector minVals = getBoundConstraintsMinValues();
	dVector maxVals = getBoundConstraintsMaxValues();

	if (minVals.size() == maxVals.size() && minVals.size() == p.size()) {
		for (int i = 0;i<p.size();i++) {
			if (minVals(i) != maxVals(i) && (minVals(i)>p(i) || p[i]>maxVals(i))) {
				Logger::logPrint("-------> Error: Bound %d: %lf < %lf < %lf\n", i, minVals(i), p(i), maxVals(i));
			}
		}
	}

	Logger::logPrint("\n");
}

