#pragma once

#include <utils/mathUtils.h>

class Plane{
public:
	//a plane is defined by its normal, and a point that lies on it
	V3D n = V3D(0,1,0);
	P3D p = P3D(0,0,0);
public:
	Plane(void);
	Plane(const P3D& p, const V3D& n){
		this->n = n.normalized();
		this->p = p;
	}
	Plane(const P3D& p1, const P3D& p2, const P3D& p3){
		this->p = p1;
		this->n = (V3D(p1, p2).cross(V3D(p2, p3))).normalized();
	}
	~Plane(void){}

	double getSignedDistanceToPoint(const P3D &pt) const{
		return V3D(p,pt).dot(n);
	}

	Plane& operator = (const Plane& other);

	//get the coefficients that define the cartesian equation of the plane: ax + by + cz + d = 0
	void getCartesianEquationCoefficients(double& a, double& b, double& c, double& d) {
		a = n[0];
		b = n[1];
		c = n[2];
		d = -(n[0]*p.x + n[1]*p.y + n[2]*p.z);
	}

};
