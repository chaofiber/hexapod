#pragma once

#include <utils/mathUtils.h>
#include <utils/transformation.h>
#include <utils/ray.h>

#include <robot/RBState.h>
#include <robot/RBProperties.h>

class RBJoint;

class RobotRB  {
public:
	// state of the rigid body
	RBState state;
	// a list of properties for the rigid body
	RBProperties rbProps;
	//we will keep a list of the joints that have this rigid body as a parent (child joints).
	std::vector<RBJoint*> cJoints;
	//and the parent joint.
	RBJoint* pJoint = NULL;
	// name of the rigid body
	std::string name;
public:
	/**
		Default constructor
	*/
	RobotRB(void) {}

	/**
		Default destructor
	*/
	virtual ~RobotRB(void) {}

	// returns the world coords moment of inertia of the rigid body
	Matrix3x3 getWorldMOI();

	Matrix3x3 getMOI(const Matrix3x3& RToWorld);

	/* returns true if it is hit, false otherwise. */
	bool getRayIntersectionPoint(const Ray& ray, P3D& intersectionPoint, bool checkMeshes, bool checkSkeleton);
};

