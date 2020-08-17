#pragma once

#include <utils/mathUtils.h>

class RigidTransformation {
public:
	Quaternion R = Quaternion::Identity();
	P3D T = P3D(0, 0, 0);

public:
	RigidTransformation(const Quaternion& _R = Quaternion::Identity(), const P3D& _T = P3D()) : R(_R), T(_T) {}
	~RigidTransformation() {}

	P3D transform(const P3D& p) {
		return T + R * V3D(p);
	}

	V3D transform(const V3D& v) {
		return R * v;
	}

	RigidTransformation inverse() {
		RigidTransformation trans;
		trans.R = R.inverse();
		trans.T = P3D() - (trans.R * V3D(P3D(), T));
		return trans;
	}

	RigidTransformation operator*(const RigidTransformation& other) {
		RigidTransformation trans;
		// use rotation matrix based multiplication to avoid singularity.
		trans.R = R * other.R;
		trans.T = T + R * V3D(other.T);
		return trans;
	}

	RigidTransformation& operator*=(const RigidTransformation& other) {
		RigidTransformation trans;
		// use rotation matrix based multiplication to avoid singularity.
		trans.R = R * other.R;
		trans.T = T + R * V3D(other.T);
		*this = trans;
		return *this;
	}
};

