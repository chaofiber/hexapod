#pragma once

#define DynamicArray std::vector
typedef unsigned int uint;

#include <Eigen/Dense>
#include <Eigen/Sparse>

typedef Eigen::AngleAxisd AngleAxisd;
typedef Eigen::VectorXd dVector;
typedef Eigen::Vector3d Vector3d;
typedef Eigen::Vector3d Triplet;
typedef Eigen::MatrixXd Matrix;
typedef Eigen::Matrix3d Matrix3x3;
typedef Eigen::SparseMatrix<double> SparseMatrix;
typedef Eigen::Triplet<double> MTriplet;

//typedef Vector3d P3D;
//typedef Vector3d V3D;
typedef Eigen::Quaternion<double> Quaternion;

inline void resize(SparseMatrix& sm, int rows, int cols) {
	if (sm.rows() != rows || sm.cols() != cols)
		sm.resize(rows, cols);
	sm.setZero();
}

inline void resize(dVector& v, int n) {
	if (v.size() != n)
		v.resize(n);
	v.setZero();
}

inline void resize(Matrix& m, int rows, int cols) {
	if (m.rows() != rows || m.cols() != cols)
		m.resize(rows, cols);
	m.setZero();
}

