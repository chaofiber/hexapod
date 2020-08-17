//TODO: all jacobians and constraints should be represented in terms of the triplets...
#include <fstream>
#include <iostream>
#include <utils/mathUtils.h>

#include "optLib/SQPMinimizer.h"

#ifdef CRL_USE_OOQP
#include "ooqp/OoqpEigenInterface.hpp"
#include "ooqp/ooqpei_assert_macros.hpp"
#endif

#include "eiquadprog/eiquadprog.hpp"

SQPMinimizer::SQPMinimizer(QpSolverType qpSolver, int maxIterations, double solverResidual, double regularizer, int maxLineSearchIterations, double solveFunctionValue, bool printOutput, bool checkConstraints)
    : qpSolverType(qpSolver),
      maxIterations_(maxIterations),
      solverResidual_(solverResidual),
      regularizer_(regularizer),
      maxLineSearchIterations_(maxLineSearchIterations),
      printOutput_(printOutput)
{
#ifndef CRL_USE_OOQP
    if (qpSolver == OOQP)
        throw std::runtime_error("OOQP is disabled. Configure project with -DCRL_BUILD_OOQP=ON option.");
#endif
}

SQPMinimizer::~SQPMinimizer()
{
}

void SQPMinimizer::setPrintOutput(bool isPrinting)
{
    printOutput_ = isPrinting;
}

void SQPMinimizer::computeGradient(ConstrainedObjectiveFunction *function, const dVector &pi)
{
    function->getGradient(pi, gradient);
    //	print("..\\out\\gradient.m", gradient);
}

void SQPMinimizer::computeHessian(ConstrainedObjectiveFunction *function, const dVector &pi)
{
    function->getHessian(pi, H);
}

void SQPMinimizer::computeConstraintsAndJacobians(ConstrainedObjectiveFunction *function, const dVector &pi)
{
    function->getEqualityConstraintsJacobian(pi, A);
    function->getInequalityConstraintsJacobian(pi, C);
    //	print("C2.m", C);
}

/**
	min f(p) subject to the constraints...
*/
bool SQPMinimizer::minimize(ConstrainedObjectiveFunction *function, dVector &p)
{
    if (printOutput_) {
        Logger::logPrint("Starting SQP...\n");
    }

    const int nParameters = (int)p.size();
    dVector dp = dVector::Zero(nParameters);
    dVector pi = p;

    // Iterate - like Newton
    bool optimizationConverged = false;
    int i;
    for (i = 0; i < maxIterations_; i++) {
        function->prepareForOptimizationStep(pi);
        sqpTimer.restart();
        computeGradient(function, pi);
        if (printOutput_)
            Logger::logPrint("\t\t>>>>>>>> TIMERINFO: Time to compute gradient: %lf\n", sqpTimer.timeEllapsed());

        sqpTimer.restart();
        computeHessian(function, pi);
        if (printOutput_)
            Logger::logPrint("\t\t>>>>>>>> TIMERINFO: Time to compute hessian: %lf\n", sqpTimer.timeEllapsed());

        sqpTimer.restart();
        computeConstraintsAndJacobians(function, pi);
        if (printOutput_)
            Logger::logPrint("\t\t>>>>>>>> TIMERINFO: Time to compute constraints and their jacobians: %lf\n", sqpTimer.timeEllapsed());

        sqpTimer.restart();
        // Find the direction to step at
        computeSearchDirection(H,
                               gradient,
                               pi,
                               dp,
                               A,
                               function->getEqualityConstraintsTargetValues(),
                               C,
                               function->getInequalityConstraintsMinValues(),
                               function->getInequalityConstraintsMaxValues(),
                               function->getBoundConstraintsMinValues(),
                               function->getBoundConstraintsMaxValues());

        if (printOutput_)
            Logger::logPrint("\t\t>>>>>>>> TIMERINFO: Time to compute search direction: %lf\n", sqpTimer.timeEllapsed());

        if (printOutput_)
            checkConstraintValues(function, pi, dp);

        /*
		Eigen::SimplicialLDLT<SparseMatrix, Eigen::Lower> solver;
		//	Eigen::SparseLU<SparseMatrix> solver;
		solver.compute(H);
		dp = solver.solve(gradient);
*/
        if (dp.norm() < solverResidual_) {
            // We're done
            if (printOutput_) {
                Logger::logPrint("\tSQP Done!\n");
                Logger::logPrint("\tResdiual: %lf < %lf \n", dp.norm(), solverResidual_);
                //	   Logger::logPrint("\tFunction value: %lf < %lf \n", currentFunctionValue, solveFunctionValue_);
            }
            optimizationConverged = true;
            break;
        }

        sqpTimer.restart();
        // Do a line search
        double alpha = doLineSearch(function, pi, dp, maxLineSearchIterations_);
        if (printOutput_)
            Logger::logPrint("\t\t>>>>>>>> TIMERINFO: Time to do line search: %lf\n", sqpTimer.timeEllapsed());

        if (alpha <= 0) {
            if (printOutput_) {
                Logger::logPrint("\tCancelling iteration: No better function value found\n");
            }
            optimizationConverged = true;
            break;
        }

        // p(i+1) = p(i) + alpha*dp
        pi += alpha * dp;
        // function->postOptimizationStep(pi);
    }

    if (printOutput_) {
        Logger::logPrint("===== Done SQP optimization =====\n");
        if (optimizationConverged) {
            Logger::logPrint("   Converged in %d iterations!\n", i);
        }
        else {
            Logger::logPrint("   Did NOT converge!\n");
        }

        Logger::logPrint("   Final function value: %10.10lf\n", function->evaluate(pi));
        Logger::logPrint("   Final Gradient norm: %10.10lf\n", gradient.norm());
        Logger::logPrint("   Final search direction norm: %10.10lf\n", dp.norm());
        Logger::logPrint("   Final constraints status: \n");
        function->printConstraintErrors(pi);
    }

    p = pi;
    return optimizationConverged;
}

void SQPMinimizer::checkConstraintValues(ConstrainedObjectiveFunction *function, const dVector &p, const dVector &dp)
{
    // check to see if the solution is within the constraints.
    dVector pnext = p + dp;

    // p must satisfy A*p = b after the solve...
    if (A.cols() == p.size()) {
        dVector b = function->getEqualityConstraintsTargetValues();
        dVector ap = A * p;
        dVector apComp = function->getEqualityConstraintValues(p);
        dVector apNext = A * pnext;
        dVector apNextComp = function->getEqualityConstraintValues(pnext);
        for (int i = 0; i < ap.size(); i++) {
            //			if (!IS_EQUAL(ap(i), apComp(i)) || !IS_EQUAL(apNext(i), apNextComp(i)))
            //				Logger::logPrint("++++++++ SQP: equality constraint %d: A*p vs A(p): (%lf vs %lf), A*(p+dp) vs A(p+dp): (%lf vs %lf)\n", i, ap(i), apNext(i), apComp(i), apNextComp(i));

            //			if (std::abs(ap(i) - b(i)) >= 0.0000001 || std::abs(apNext(i) - b(i)) >= 0.0000001)
            if (std::abs(apNext(i) - b(i)) >= 0.001)
                Logger::logPrint("++++++++ SQP: equality constraint %d: A(p): %lf, A(p+dp): %lf, constraint value: %lf\n", i, ap(i), apNext(i), b(i));
        }
    }

    //	and the 10000 vs inf issue...

    // p must satisfy d <= C*p <= f
    if (C.cols() == p.size()) {
        dVector d = function->getInequalityConstraintsMinValues();
        dVector f = function->getInequalityConstraintsMaxValues();
        dVector cpComp = function->getInequalityConstraintValues(p);
        dVector cp = C * p;
        dVector cpNext = C * pnext;
        dVector cpNextComp = function->getInequalityConstraintValues(pnext);

        for (int i = 0; i < cp.size(); i++) {
            if (!IS_EQUAL(cp(i), cpComp(i)) || !IS_EQUAL(cpNext(i), cpNextComp(i)))
                Logger::logPrint("++++++++ SQP: inequality constraint %d: C*p vs C(p): (%lf vs %lf), C*(p+dp) vs C(p+dp): (%lf vs %lf)\n", i, cp(i), cpNext(i), cpComp(i), cpNextComp(i));

            if (cp(i) > f(i) + 0.0000001 || cpNext[i] > f(i) + 0.0000001 || cp(i) < d(i) - 0.0000001 || cpNext(i) < d(i) - 0.0000001)
                Logger::logPrint("++++++++ SQP: inequality constraint %d: Before: %lf < %lf < %lf, after: %lf < %lf < %lf\n", i, d(i), cp(i), f(i), d(i), cpNext(i), f(i));
        }
    }

    dVector l = function->getBoundConstraintsMinValues();
    dVector u = function->getBoundConstraintsMaxValues();
    // p must satisfy the bound constraints lo <= p <= hi
    if (l.size() == p.size()) {
        for (int i = 0; i < p.size(); i++) {
            if (l(i) == u(i))
                continue;
            //				assert("p(t) within inequality constraints" && cp[i] <= f->at(i) + 0.0000001);
            if (p(i) < l(i) - 0.0000001 || pnext(i) < l(i) - 0.0000001)
                Logger::logPrint("++++++++ SQP: min bound constraint %d: p: %.10lf, p+dp: %.10lf, lo: %.10lf\n", i, p(i), pnext(i), l(i));

            if (p(i) > u(i) + 0.0000001 || pnext(i) > u(i) + 0.0000001)
                Logger::logPrint("++++++++ SQP: max bound constraint %d: p: %.10lf, p+dp: %.10lf, hi: %.10lf\n", i, p(i), pnext(i), u(i));
        }
    }
}

void SQPMinimizer::computeSearchDirection(const SparseMatrix &hessian,
                                          const dVector &gradient,
                                          const dVector &p,
                                          dVector &dp,
                                          const SparseMatrix &A,
                                          const dVector &b,
                                          const SparseMatrix &C,
                                          const dVector &d,
                                          const dVector &f,
                                          const dVector &l,
                                          const dVector &u)
{

    /** We want dp to minimize: F(p+dp) ~ F(p) + dp' grad + 1/2 dp' H dp
	 * while maintaining the constraints A*(p+dp) = b and d <= C*(p+dp) <= f and l <= p+dp <= u.
	 * re-writing the constraints as
	 * A*dp = b - A*p
	 * d-C*p <= C*dp <= f-C*p
	 * l-p <= dp <= u-p
	 * we can get a canonical QP form:
	 * min 1/2 x' Q x + c' x s. t. A x = b, d <= Cx <= f, and l <= x <= u
	 * where:
	 * x = dp
	 * Q = hessian
	 * c = gradient
	 * A = A
	 * b = bMinusAp
	 * C = C
	 * d = dMinusCp
	 * f = fMinusCp
	 * l = minMinusp
	 * u = maxMinusp
	 */

    int np = (int)p.size();

    dVector fMinusCp;
    dVector dMinusCp;

    if (C.size() > 0) {
        const dVector Cp = C * p;
        fMinusCp = f - Cp;
        dMinusCp = d - Cp;
    }

    dVector bMinusAp;
    if (A.size() > 0) {
        bMinusAp = b - A * p;
    }

    dVector lMinusdp = l - p;
    dVector uMinusdp = u - p;

    if (qpSolverType == EI_QUAD_PROG) {
        // hessian and gradient
        Matrix G = Matrix(hessian);
        dVector g0 = gradient;

        // equality
        Matrix CE = Matrix(A).transpose();
        dVector ce0 = -bMinusAp;

        // inequality
        Matrix CIT(C.rows() * 2 + np * 2, np);
        CIT << Matrix(C), -Matrix(C), Matrix::Identity(np, np), -Matrix::Identity(np, np);
        dVector ci0(C.rows() * 2 + np * 2);
        ci0 << -dMinusCp, fMinusCp, -lMinusdp, uMinusdp;

        // solve
        bool success = !(solve_quadprog(G, g0, CE, ce0, CIT.transpose(), ci0, dp) == std::numeric_limits<double>::infinity());

        //if the hessian is not positive definite, we might be going uphill (truth is, this could also happen because of constraints). Nevertheless, if this is the case, try just gradient projection instead - slower but safer, hopefully...
        double gradSearchDirDotProduct = -(gradient.dot(dp));
        if (success && printOutput_) {
            Logger::logPrint("	==> Search direction: dot product between gradient and search direction: %lf\n", gradSearchDirDotProduct);
        }
        if (!success && gradSearchDirDotProduct <= 0) {
            Logger::consolePrint("Failed QP solve... attempting a gradient projection step...\n");
            Logger::logPrint("Failed QP solve... attempting a gradient projection step...\n");

            Matrix tmpHes(hessian.rows(), hessian.cols());
            tmpHes.setIdentity();
            tmpHes *= 10000;
            success = !(solve_quadprog(tmpHes, g0, CE, ce0, CIT.transpose(), ci0, dp) == std::numeric_limits<double>::infinity());
            if (!success) {
                Logger::consolePrint("Gradient projection step failed. Constraints are likely wrong...\n");
                assert("Failed QP solve in SQP - printing system and returning zero stepping direction" && false);
                dp.setZero();
            }
        }
    }
    else if (qpSolverType == OOQP) {
#ifdef CRL_USE_OOQP
        bool success = ooqpei::OoqpEigenInterface::solve(hessian,
                                                         gradient,
                                                         A,
                                                         bMinusAp,
                                                         C,
                                                         dMinusCp,
                                                         fMinusCp,
                                                         lMinusdp,
                                                         uMinusdp,
                                                         dp);

        double gradSearchDirDotProduct = -(gradient.dot(dp));
        if (success && printOutput_) {
            Logger::logPrint("	==> Search direction: dot product between gradient and search direction: %lf\n", gradSearchDirDotProduct);
        }
        if (!success && gradSearchDirDotProduct <= 0) {
            Logger::consolePrint("Failed QP solve... attempting a gradient projection step...\n");
            Logger::logPrint("Failed QP solve... attempting a gradient projection step...\n");

            SparseMatrix tmpHes(hessian.rows(), hessian.cols());
            tmpHes.setIdentity();
            tmpHes *= 10000;
            success = ooqpei::OoqpEigenInterface::solve(tmpHes, gradient, A, bMinusAp, C, dMinusCp, fMinusCp, lMinusdp, uMinusdp, dp);
            if (!success) {
                Logger::consolePrint("Gradient projection step failed. Constraints are likely wrong...\n");
                assert("Failed QP solve in SQP - printing system and returning zero stepping direction" && false);
                dp.setZero();
            }
        }
#endif
    }
}

double SQPMinimizer::doLineSearch(ObjectiveFunction *function, dVector &p, const dVector &dp, int maxSteps)
{
    double alpha = 1.0;

    if (maxSteps <= 1) {
        return alpha;
    }

    const double initialValue = function->evaluate(p);
    dVector pc;

    for (int j = 0; j < maxSteps; j++) {
        pc = p + alpha * dp;

        const double newLineSearchValue = function->evaluate(pc);

        if (printOutput_) {
            Logger::logPrint("\t--> LINE SEARCH iteration %d: alpha is %10.10lf, function value is: %10.10lf\n", j, alpha, newLineSearchValue);
        }

        if (newLineSearchValue >= initialValue) {
            alpha /= 2.0; // restore and try again...
        }
        else {
            //			Logger::consolePrint("Line search done after %d steps...\n", j);
            return alpha; // found a better solution!
        }
    }

    return alpha;
}
