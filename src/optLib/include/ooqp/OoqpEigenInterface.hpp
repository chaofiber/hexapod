#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Core>
#include <Eigen/SparseCore>

#include <utils/mathDefs.h>

namespace ooqpei {

class OoqpEigenInterface{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   * Solve min 1/2 x' Q x + c' x, such that A x = b, d <= Cx <= f, and l <= x <= u.
   * @param [in] Q a symmetric positive semidefinite matrix (nxn)
   * @param [in] c a vector (nx1)
   * @param [in] A a (possibly null) matrices (m_axn)
   * @param [in] b a vector (m_ax1)
   * @param [in] C a (possibly null) matrices (m_cxn)
   * @param [in] d a vector (m_cx1)
   * @param [in] f a vector (m_cx1)
   * @param [in] l a vector (nx1)
   * @param [in] u a vector (nx1)
   * @param [out] x a vector of variables (nx1)
   * @return true if successful
   */
  static bool solve(const SparseMatrix& Q,
                    const dVector& c,
                    const SparseMatrix& A,
                    const dVector& b,
                    const SparseMatrix& C,
                    const dVector& d, const dVector& f,
                    const dVector& l, const dVector& u,
                    dVector& x);

  /*!
   * Solve min 1/2 x' Q x + c' x, such that A x = b, and d <= Cx <= f
   * @param [in] Q a symmetric positive semidefinite matrix (nxn)
   * @param [in] c a vector (nx1)
   * @param [in] A a (possibly null) matrices (m_axn)
   * @param [in] b a vector (m_ax1)
   * @param [in] C a (possibly null) matrices (m_cxn)
   * @param [in] d a vector (m_cx1)
   * @param [in] f a vector (m_cx1)
   * @param [out] x a vector of variables (nx1)
   * @return true if successful
   */
  static bool solve(const SparseMatrix& Q,
                    dVector& c,
                    const SparseMatrix& A,
                    dVector& b,
                    const SparseMatrix& C,
                    dVector& d, dVector& f,
                    dVector& x);

  /*!
   * Solve min 1/2 x' Q x + c' x, such that A x = b, and l <= x <= u.
   * @param [in] Q a symmetric positive semidefinite matrix (nxn)
   * @param [in] c a vector (nx1)
   * @param [in] A a (possibly null) matrices (m_axn)
   * @param [in] b a vector (m_ax1)
   * @param [in] l a vector (nx1)
   * @param [in] u a vector (nx1)
   * @param [out] x a vector of variables (nx1)
   * @return true if successful
   */
  static bool solve(const SparseMatrix& Q,
                    dVector& c,
                    const SparseMatrix& A,
                    dVector& b,
                    dVector& l, dVector& u,
                    dVector& x);

  /*!
   * Solve min 1/2 x' Q x + c' x, such that Cx <= f
   * @param [in] Q a symmetric positive semidefinite matrix (nxn)
   * @param [in] c a vector (nx1)
   * @param [in] C a (possibly null) matrices (m_cxn)
   * @param [in] f a vector (m_cx1)
   * @param [out] x a vector of variables (nx1)
   * @return true if successful
   */
  static bool solve(const SparseMatrix& Q,
                    dVector& c,
                    const SparseMatrix& C,
                    dVector& f,
                    dVector& x);

  /*!
   * Solve min 1/2 x' Q x + c' x
   * @param [in] Q a symmetric positive semidefinite matrix (nxn)
   * @param [in] c a vector (nx1)
   * @param [out] x a vector of variables (nx1)
   * @return true if successful
   */
  static bool solve(const SparseMatrix& Q,
                    dVector& c,
                    dVector& x);

  /*!
   * Change to true to print debug information.
   * @return true if in debug mode
   */
  static bool isInDebugMode() { return isInDebugMode_; };
  static void setIsInDebugMode(bool isInDebugMode) {
    isInDebugMode_ = isInDebugMode;
  }

 private:
  /*!
   * Determine which limits are active and which are not.
   * @param [in]  l
   * @param [in]  u
   * @param [out] useLowerLimit
   * @param [out] useUpperLimit
   * @param [out] lowerLimit
   * @param [out] upperLimit
   */
  static void generateLimits(const dVector& l, const dVector& u,
                      Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimit,
                      Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimit,
                      dVector& lowerLimit, dVector& upperLimit, bool ignoreEqualLowerAndUpperLimits);

  static void printProblemFormulation(
      const SparseMatrix& Q, const dVector& c,
      const SparseMatrix& A, const dVector& b,
      const SparseMatrix& C, const dVector& d, const dVector& f,
	  const dVector& l, const dVector& u);

  static void printLimits(Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimit,
                          Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimit,
                          dVector& lowerLimit,
                          dVector& upperLimit);

  static void printSolution(int& status, dVector& x);

 private:
  static bool isInDebugMode_;
};

} /* namespace ooqpei */
