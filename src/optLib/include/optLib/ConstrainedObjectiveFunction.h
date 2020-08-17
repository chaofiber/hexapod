#pragma once

#include <math.h>

#include "optLib/ObjectiveFunction.h"


/**
	Objective function with simple bound constraints on its parameters.
*/
class BoundedObjectiveFunction : public ObjectiveFunction {
protected:
	dVector l;
	dVector u;
public:
	BoundedObjectiveFunction() {}
	virtual ~BoundedObjectiveFunction() {}

	/*! @returns min of constraint min <= p <= max
	*/
	virtual const dVector& getBoundConstraintsMinValues() { return l; };

	/*! @returns max of constraint min <= p <= max
	*/
	virtual const dVector& getBoundConstraintsMaxValues() { return u; };
};

/**
	Objective function with simple bound constraints on its parameters.
*/
class ConstrainedObjectiveFunction : public BoundedObjectiveFunction {
public:
	ConstrainedObjectiveFunction() {}
	virtual ~ConstrainedObjectiveFunction() {}

	virtual int getEqualityConstraintCount() { return (int)b.size(); }
	virtual int getInequalityConstraintCount() { return (int)d.size(); }

	/*! Implement the analytic Jacobian of the equality constraints,
	* otherwise it is estimated through finite-difference.
	*
	*  Equality constraints of the form A(p) = b (if they were really linear, A(p) = A*p)
	*  Assume the constraints are either linear, or they should be linearized at p.
	*/
	virtual void getEqualityConstraintsJacobian(const dVector& p, SparseMatrix& J) {
		resize(J, getEqualityConstraintCount(), (int)p.size());
		JEntries.clear();
		addEqualityConstraintsJacobianEntriesTo(JEntries, p);
		J.setFromTriplets(JEntries.begin(), JEntries.end());
	}

	/*! inequality constraints of the form d <= C(p) <= f
	* @returns dC/dp
	*/
	virtual void getInequalityConstraintsJacobian(const dVector& p, SparseMatrix& J) {
		resize(J, getInequalityConstraintCount(), (int)p.size());
		JEntries.clear();
		addInequalityConstraintsJacobianEntriesTo(JEntries, p);
		J.setFromTriplets(JEntries.begin(), JEntries.end());
	}

	/*! @returns b of A(p) = b
	 */
	virtual const dVector& getEqualityConstraintsTargetValues() { return b; }

	/*!
	 * Get the current value of the equality constraints.
	 * @returns A(p) of A(p) = b
	 */
	virtual const dVector& getEqualityConstraintValues(const dVector& p) = 0;

    /*! @returns d of d <= C(p) <= f
    */
	virtual const dVector& getInequalityConstraintsMinValues() { return d; }

    /*! @returns f of d <= C(p) <= f
    */
	virtual const dVector& getInequalityConstraintsMaxValues() { return f; }

	/*! 
	 * get the current value of the equality constraints...
	 * @returns C(p)  of d <= C(p) <= f
	 */
	virtual const dVector& getInequalityConstraintValues(const dVector& p) = 0;

public:
  //!  Estimates the Jacobian of the equality constraints through finite-difference (FD)
  void addEstimatedEqualityConstraintsJacobianEntriesTo(DynamicArray<MTriplet>& jacobianEntries, const dVector& p);

	//!  Estimates the Jacobian of the inequality constraints through finite-difference (FD)
  void addEstimatedInequalityConstraintsJacobianEntriesTo(DynamicArray<MTriplet>& jacobianEntries, const dVector& p);

	//! Tests Jacobians of equality and inequality constraints
  void testJacobiansWithFD(const dVector& p);

  void printConstraintErrors(const dVector& p, double eqTol = 0.00001, double iqTol = 0.00001);

protected:
  dVector b;
  dVector d;
  dVector f;

  /*! Implement the analytic Jacobian of the equality constraints,
  * otherwise it is estimated through finite-difference.
  *
  *  Equality constraints of the form A(p) = b (if they were really linear, A(p) = A*p)
  *  Assume the constraints are either linear, or they should be linearized at p.
  */
  virtual void addEqualityConstraintsJacobianEntriesTo(DynamicArray<MTriplet>& jacobianEntries, const dVector& p) {
	  addEstimatedEqualityConstraintsJacobianEntriesTo(jacobianEntries, p);
  }

  /*! inequality constraints of the form d <= C(p) <= f
  * @returns dC/dp
  */
  virtual void addInequalityConstraintsJacobianEntriesTo(DynamicArray<MTriplet>& jacobianEntries, const dVector& p) {
	  addEstimatedInequalityConstraintsJacobianEntriesTo(jacobianEntries, p);
  }

private:
  DynamicArray<MTriplet> JEntries;

};




