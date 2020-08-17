#pragma once

#include <float.h>

#include "optLib/ConstrainedObjectiveFunction.h"
#include "optLib/ObjectiveFunction.h"
#include "utils/logger.h"
#include "utils/timer.h"

/**
	Use the Sequential Quadratic Programming method to optimize a function, subject to constraints.
	
	Task: Find p that minimize f(p), such that Ap = b and d <= Cp <= f
	This means df/dp(p) = 0. SQP works similar as Newton iteration, i.e. approximates the environment of f as
	f(p + dp) ~ f(p) + df/dp(p)*dp + 1/2 dp' * d/dp(df/dp)(p) * dp
	Which is minimized when df/dp is closest to zero within the constraints.

	SQP hence solves a sequence of QP problems of the form
		min d/dp(df/dp)*dp + df/dp, s.t. A(p + dp) = b and C(p + dp) <= f
	which gives us the stepping direction dp within the constraint manifold. Iterating the above will
	hopefully yield p that minimizes f.

	A warning: Convergence can be improved in some cases by doing line search in the direction of dp.
	However, this can give us intermediate points outside the constraint manifold, which is bad.
	If you're unsure whether your constraints are non-convex, set 'maxLineSearchIterations' to 0.
*/
class SQPMinimizer {
public:
    enum QpSolverType {
        EI_QUAD_PROG = 0,
        OOQP = 1,
    };

    /*!
   *
   * @param maxIterations               maximum number of iterations
   * @param solverResidual               abortion criterium
   * @param regularizer                 not used
   * @param maxLineSearchIterations     maximum number of line search iterations
   * @param solveFunctionValue          abortion criterium (if function value is lower than this value)
   * @param printOutput
   * @param checkConstraints
   */
    SQPMinimizer(QpSolverType qpSolverType = EI_QUAD_PROG,
                 int maxIterations = 2,
                 double solverResidual = 0.0001,
                 double regularizer = 0.0,
                 int maxLineSearchIterations = 0,
                 double solveFunctionValue = -DBL_MAX,
                 bool printOutput = false,
                 bool checkConstraints = false);
    virtual ~SQPMinimizer();

    /**
		min f(p) subject to the constraints...
	*/
    bool minimize(ConstrainedObjectiveFunction *function, dVector &p);

    void setPrintOutput(bool isPrinting);

protected:
    virtual void computeGradient(ConstrainedObjectiveFunction *function, const dVector &pi);
    virtual void computeHessian(ConstrainedObjectiveFunction *function, const dVector &pi);
    virtual void computeConstraintsAndJacobians(ConstrainedObjectiveFunction *function, const dVector &pi);

private:
    /**
   * We search for dp which solves the system of equations
   *   H*dp + grad == 0
   * in a least squares sense while maintaining the constraints A*p = b and d <= C*p <= f.
   * This means we want to solve
   *  min |H*dp + grad|^2,
   *   s.t. A*(p + dp) = b,
   *        d <= C*(p + dp) <= f
   */
    void computeSearchDirection(const SparseMatrix &hessian,
                                const dVector &gradient,
                                const dVector &p,
                                dVector &dp,
                                const SparseMatrix &A,
                                const dVector &b,
                                const SparseMatrix &C,
                                const dVector &d,
                                const dVector &f,
                                const dVector &minBounds,
                                const dVector &maxBounds);

    /*! @returns alpha for computing next parameter set:  p(i+1) = p(i) + alpha*dp.
	 *
	 * @param function  objective function
	 * @param p         current parameter set
	 * @param dp        parameter variation
	 * @param maxSteps  maximum number of steps the line search should do in worst case.
	 *
	 */
    double doLineSearch(ObjectiveFunction *function, dVector &p, const dVector &dp, int maxSteps);

    void checkConstraintValues(ConstrainedObjectiveFunction *function, const dVector &p, const dVector &dp);

public:
    int maxLineSearchIterations_;
    int maxIterations_;
    double solverResidual_;
    double regularizer_;
    bool printOutput_;

    Timer sqpTimer;

    SparseMatrix H, A, C;
    dVector gradient;

private:
    QpSolverType qpSolverType = QpSolverType::EI_QUAD_PROG;
};
