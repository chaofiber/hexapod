#include <stdexcept>
#include "ooqp/OoqpEigenInterface.hpp"
#include "ooqp/ooqpei_assert_macros.hpp"
#include "ooqp/ooqpei_numerical_comparisons.hpp"

// OOQP
#include <QpGenData.h>
#include <QpGenData.h>
#include <QpGenVars.h>
#include <QpGenResiduals.h>
#include <GondzioSolver.h>
#include <QpGenSparseMa27.h>
#include <Status.h>
#include <cQpGenSparse.h>

#include "utils/logger.h"

using namespace std;
using namespace ooqpei;

namespace ooqpei {

bool OoqpEigenInterface::isInDebugMode_ = false;

bool OoqpEigenInterface::solve(const SparseMatrix& Q,
                          const dVector& c,
                          const SparseMatrix& A,
                          const dVector& b,
                          const SparseMatrix& C,
                          const dVector& d, const dVector& f,
                          const dVector& l, const dVector& u,
                          dVector& x)
{
  // Initialize.
  int nx = Q.rows();  // nx is the number of primal variables (x).
  OOQPEI_ASSERT_GT(range_error, nx, 0, "Matrix Q has size 0.");
  x.setZero(nx);

  // Check matrices.
  OOQPEI_ASSERT_EQ(range_error, c.size(), nx, "Vector c has wrong size.");
  OOQPEI_ASSERT_EQ(range_error, l.size(), nx, "Vector l has wrong size.");
  OOQPEI_ASSERT_EQ(range_error, u.size(), nx, "Vector u has wrong size.");
  OOQPEI_ASSERT_EQ(range_error, b.size(), A.rows(), "Vector b has wrong size.");
  if (A.size() > 0) OOQPEI_ASSERT_EQ(range_error, A.cols(), nx, "Matrix A has wrong size.");
  OOQPEI_ASSERT_EQ(range_error, d.size(), C.rows(), "Vector d has wrong size.");
  OOQPEI_ASSERT_EQ(range_error, f.size(), C.rows(), "Vector f has wrong size.");
  if (C.size() > 0) OOQPEI_ASSERT_EQ(range_error, C.cols(), nx, "Matrix C has wrong size.");


  // Make sure Q is in lower triangular form (Q is symmetric).
  // Refer to OOQP user guide section 2.2 (p. 11).
  // TODO Check if Q is really symmetric.
  Eigen::SparseMatrix<double, Eigen::RowMajor> Q_triangular = Q.triangularView<Eigen::Lower>();

  if (isInDebugMode()) printProblemFormulation(Q_triangular, c, A, b, C, d, f, l, u);

  // Compress sparse Eigen matrices (refer to Eigen Sparse Matrix user manual).
  Q_triangular.makeCompressed();
  // TODO: not sure how good/ efficient it is to keep making copies of these matrices here...
  Eigen::SparseMatrix<double, Eigen::RowMajor> A_copy = A; A_copy.makeCompressed();
  Eigen::SparseMatrix<double, Eigen::RowMajor> C_copy = C; C_copy.makeCompressed();

  // Determine which limits are active and which are not.
  // Refer to OOQP user guide section 2.2 (p. 10).
  Eigen::Matrix<char, Eigen::Dynamic, 1> useLowerLimitForX;
  Eigen::Matrix<char, Eigen::Dynamic, 1> useUpperLimitForX;
  dVector lowerLimitForX;
  dVector upperLimitForX;
  Eigen::Matrix<char, Eigen::Dynamic, 1> useLowerLimitForInequalityConstraints;
  Eigen::Matrix<char, Eigen::Dynamic, 1> useUpperLimitForInequalityConstraints;
  dVector lowerLimitForInequalityConstraints;
  dVector upperLimitForInequalityConstraints;

  generateLimits(l, u, useLowerLimitForX, useUpperLimitForX,
                 lowerLimitForX, upperLimitForX, true);
  generateLimits(d, f, useLowerLimitForInequalityConstraints,
                 useUpperLimitForInequalityConstraints,
                 lowerLimitForInequalityConstraints,
                 upperLimitForInequalityConstraints, false);

  if (isInDebugMode()){
	  Logger::logPrint("-------------------------------\n");
	  Logger::logPrint("LIMITS FOR X\n");
	  printLimits(useLowerLimitForX, useUpperLimitForX, lowerLimitForX, upperLimitForX);
	  Logger::logPrint("-------------------------------\n");
	  Logger::logPrint("LIMITS FOR Inequality Constraints\n");
	  printLimits(useLowerLimitForInequalityConstraints,
		  useUpperLimitForInequalityConstraints,
		  lowerLimitForInequalityConstraints,
		  upperLimitForInequalityConstraints);
  }

  // Setting up OOQP solver
  // Refer to OOQP user guide section 2.3 (p. 14).

  // Initialize new problem formulation.
  int my = b.size();
  int mz = lowerLimitForInequalityConstraints.size();
  int nnzQ = Q_triangular.nonZeros();
  int nnzA = A_copy.nonZeros();
  int nnzC = C_copy.nonZeros();

  // Fill in problem data.
  double* cp   = (c.size()>0) ? (double*)&c.coeffRef(0) : NULL;
  int* krowQ   =  Q_triangular.outerIndexPtr();
  int* jcolQ   =  Q_triangular.innerIndexPtr();
  double* dQ   =  Q_triangular.valuePtr();
  double* xlow = (lowerLimitForX.size()>0) ? &lowerLimitForX.coeffRef(0) : NULL;
  char* ixlow  = (useLowerLimitForX.size()>0) ? &useLowerLimitForX.coeffRef(0) : NULL;
  double* xupp = (upperLimitForX.size()>0) ? &upperLimitForX.coeffRef(0) : NULL;
  char* ixupp  = (useUpperLimitForX.size()>0) ? &useUpperLimitForX.coeffRef(0) : NULL;
  int* krowA   =  A_copy.outerIndexPtr();
  int* jcolA   =  A_copy.innerIndexPtr();
  double* dA   =  A_copy.valuePtr();
  double* bA   = (b.size()>0) ? (double*)&b.coeffRef(0) : NULL;
  int* krowC   =  C_copy.outerIndexPtr();
  int* jcolC   =  C_copy.innerIndexPtr();
  double* dC   =  C_copy.valuePtr();
  double* clow = (lowerLimitForInequalityConstraints.size()>0) ? &lowerLimitForInequalityConstraints.coeffRef(0) : NULL;
  char* iclow  = (useLowerLimitForInequalityConstraints.size()>0) ? &useLowerLimitForInequalityConstraints.coeffRef(0) : NULL;
  double* cupp = (upperLimitForInequalityConstraints.size()>0) ? &upperLimitForInequalityConstraints.coeffRef(0) : NULL;
  char* icupp  = (useUpperLimitForInequalityConstraints.size()>0) ? &useUpperLimitForInequalityConstraints.coeffRef(0) : NULL;

  /*
  //allocate space for lagrange multipliers for the various equality and inequality constraints
  dVector gamma(nx), phi(nx);		//x>=l and x<=u
  gamma.setConstant(0); phi.setConstant(0);
  dVector y(my);				//Ax=b
  y.setConstant(0);
  dVector lambda(mz), pi(mz), z(mz);	//Cx>=d, Cx<=f, and z = lambda - pi
  lambda.setConstant(0); pi.setConstant(0); z.setConstant(0);
  double* pz = z.size() > 0 ? &z.coeffRef(0) : NULL;
  double* plambda = lambda.size() > 0 ? &lambda.coeffRef(0) : NULL;
  double* ppi = pi.size() > 0 ? &pi.coeffRef(0) : NULL;
  double* pgamma = gamma.size() > 0 ? &gamma.coeffRef(0) : NULL;
  double* pphi = phi.size() > 0 ? &phi.coeffRef(0) : NULL;
  double* py = y.size() > 0 ? &y.coeffRef(0) : NULL;
  double objValue;
  int status;


  // this function is for sparse matrices that store the data in triplet form
//  qpsolvesp(cp, nx, 	  krowQ, nnzQ, jcolQ, dQ,	  xlow, ixlow, xupp, ixupp,	  krowA, nnzA, jcolA, dA, bA, my,	  krowC, nnzC, jcolC, dC, clow, mz, iclow, cupp, icupp,	  &x.coeffRef(0),	  pgamma, pphi, py,	  pz, plambda, ppi, &objValue, 0, &status);
  //while this one is for sparce matrices in reduced Harwell-Boeing format
  qpsolvehb(cp, nx, 	  krowQ, jcolQ, dQ,	  xlow, ixlow, xupp, ixupp,	  krowA, my, jcolA, dA, bA,	  krowC, mz, jcolC, dC, clow, iclow, cupp, icupp,	  &x.coeffRef(0),	  pgamma, pphi, py,	  pz, plambda, ppi, &objValue, 0, &status);
*/

  QpGenSparseMa27 * qp = new QpGenSparseMa27(nx, my, mz, nnzQ, nnzA, nnzC);
  QpGenData * prob = (QpGenData *) qp->makeData(cp, krowQ, jcolQ, dQ,
                                                xlow, ixlow, xupp, ixupp,
                                                krowA, jcolA, dA, bA,
                                                krowC, jcolC, dC,
                                                clow, iclow, cupp, icupp);

  // Create object to store problem variables.
  QpGenVars* vars = (QpGenVars*) qp->makeVariables(prob);
//  if (isInDebugMode()) prob->print(); // Matrices are printed as [index_x, index_y, value]
  // Create object to store problem residual data.
  QpGenResiduals* resid = (QpGenResiduals*) qp->makeResiduals(prob);
  // Create solver object.
  GondzioSolver* s = new GondzioSolver(qp, prob);
  if (isInDebugMode()) s->monitorSelf();
  // Solve.
  int status = s->solve(prob, vars, resid);
//  if (status == SUCCESSFUL_TERMINATION)
  vars->x->copyIntoArray(&x.coeffRef(0));
  if(isInDebugMode()) printSolution(status, x);
  //vars->x->writefToStream( cout, "x[%{index}] = %{value}" );

  delete s;
  delete resid;
  delete vars;
  delete prob;
  delete qp;

  if (status != SUCCESSFUL_TERMINATION) {
	  Logger::logPrint("QP Solver failed with code: %d\n", status);
  }

  return (status == SUCCESSFUL_TERMINATION);
}

bool OoqpEigenInterface::solve(const SparseMatrix& Q,
                          dVector& c,
                          const SparseMatrix& A,
                          dVector& b,
                          const SparseMatrix& C,
                          dVector& d, dVector& f,
                          dVector& x)
{
  int nx = Q.rows();
  dVector u = std::numeric_limits<double>::max() * dVector::Ones(nx);
  dVector l = (-u.array()).matrix();
  return solve(Q, c, A, b, C, d, f, l, u, x);
}

bool OoqpEigenInterface::solve(const SparseMatrix& Q,
                          dVector& c,
                          const SparseMatrix& A,
                          dVector& b,
                          dVector& l, dVector& u,
                          dVector& x)
{
  SparseMatrix C;
  dVector d, f;
  return solve(Q, c, A, b, C, d, f, l, u, x);
}

bool OoqpEigenInterface::solve(const SparseMatrix& Q,
                               dVector& c,
                               const SparseMatrix& C,
                               dVector& f,
                               dVector& x)
{
  SparseMatrix A;
  dVector b;
  dVector d = -std::numeric_limits<double>::max() * dVector::Ones(C.rows());
  return solve(Q, c, A, b, C, d, f, x);
}

bool OoqpEigenInterface::solve(const SparseMatrix& Q,
                          dVector& c, dVector& x)
{
  SparseMatrix A, C;
  dVector b, d, f;
  return solve(Q, c, A, b, C, d, f, x);
}

void OoqpEigenInterface::generateLimits(
    const dVector& l, const dVector& u,
    Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimit,
    Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimit,
    dVector& lowerLimit, dVector& upperLimit,
	bool ignoreEqualLowerAndUpperLimits)
{
	int n = l.size();
	useLowerLimit.setConstant(n, 1);
	useUpperLimit.setConstant(n, 1);
	lowerLimit = l;
	upperLimit = u;

	for (int i = 0; i < n; i++) {
		if (ignoreEqualLowerAndUpperLimits && ooqpei::approximatelyEqual(l(i), u[i])) {
			//Logger::logPrint("Skipping min and max limit: %d, %lf %lf\n", i, l(i), u(i));
			useLowerLimit(i) = 0; useUpperLimit(i) = 0;
			lowerLimit(i) = 0; upperLimit(i) = 0;
		}

		if (ooqpei::approximatelyEqual(l(i), -std::numeric_limits<double>::max())) {
			//Logger::logPrint("Skipping min limit: %d, %lf \n", i, l(i));
			useLowerLimit(i) = 0;
			lowerLimit(i) = 0.0;
		}

		if (ooqpei::approximatelyEqual(u(i), std::numeric_limits<double>::max())) {
			//Logger::logPrint("Skipping max limit: %d, %lf \n", i, u(i));
			useUpperLimit(i) = 0;
			upperLimit(i) = 0.0;
		}
	}
}

void OoqpEigenInterface::printProblemFormulation(
    const SparseMatrix& Q, const dVector& c,
    const SparseMatrix& A, const dVector& b,
    const SparseMatrix& C, const dVector& d, const dVector& f,
	const dVector& l, const dVector& u)
{
  cout << "-------------------------------" << endl;
  cout << "Find x: min 1/2 x' Q x + c' x such that A x = b, d <= Cx <= f, and l <= x <= u" << endl << endl;
  cout << "Q (triangular) << " << endl << Matrix(Q) << endl;
  cout << "c << " << c.transpose() << endl;
  cout << "A << " << endl << Matrix(A) << endl;
  cout << "b << " << b.transpose() << endl;
  cout << "C << " << endl << Matrix(C) << endl;
  cout << "d << " << d.transpose() << endl;
  cout << "f << " << f.transpose() << endl;
  cout << "l << " << l.transpose() << endl;
  cout << "u << " << u.transpose() << endl;
}

void OoqpEigenInterface::printLimits(
    Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimit,
    Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimit,
    dVector& lowerLimit, dVector& upperLimit)
{
	Logger::logPrint("lower limits:\n-------------------------\n");
	for (int i = 0; i < lowerLimit.size(); i++) {
		if (useLowerLimit(i))
			Logger::logPrint("%lf\t", lowerLimit(i));
		else
			Logger::logPrint("-\t");
	}
	Logger::logPrint("\n");

	Logger::logPrint("upper limits:\n-------------------------\n");
	for (int i = 0; i < upperLimit.size(); i++) {
		if (useUpperLimit(i))
			Logger::logPrint("%lf\t", upperLimit(i));
		else
			Logger::logPrint("-\t");
	}
	Logger::logPrint("\n");


//  cout << "useLowerLimit << " << std::boolalpha << useLowerLimit.cast<bool>().transpose() << endl;
//  cout << "lowerLimit << " << lowerLimit.transpose() << endl;
//  cout << "useUpperLimit << " << std::boolalpha << useUpperLimit.cast<bool>().transpose() << endl;
//  cout << "upperLimit << " << upperLimit.transpose() << endl;
}

void OoqpEigenInterface::printSolution(int& status, dVector& x)
{
  if (status == 0)
  {
    cout << "-------------------------------" << endl;
    cout << "SOLUTION" << endl;
    cout << "Ok, ended with status " << status << "."<< endl;
    cout << "x << " << x.transpose() << endl;
  }
  else
  {
    cout << "-------------------------------" << endl;
    cout << "SOLUTION" << endl;
    cout << "Error, ended with status " << status << "." << endl;
  }
}

} /* namespace ooqpei */

