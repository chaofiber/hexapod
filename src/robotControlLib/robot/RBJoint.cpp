#include <robot/RBJoint.h>
#include <utils/mathUtils.h>

#include <robot/RBEngine.h>
#include <utils/utils.h>
#include <robot/RBUtils.h>
#include <utils/logger.h>

/**
	Returns the rbEngine position of the joint
*/
P3D RBJoint::getWorldPosition(){
	return (child->state.getWorldCoordinates(cJPos) + parent->state.getWorldCoordinates(pJPos))/2.0;
}

/**
	This method is used to compute the relative orientation between the parent and the child rigid bodies, expressed in 
	the frame coordinate of the parent.
*/
Quaternion RBJoint::computeRelativeOrientation(){
	//if qp is the quaternion that gives the orientation of the parent, and qc gives the orientation of the child, then  qp^-1 * qc gives the relative
	//orientation between the child and the parent (child to parent)
	return (parent->state.orientation.inverse() * child->state.orientation).normalized();
}


/**
	This method is used to fix the errors in the joints (i.e. project state of child such that joint configuration is consistent). The state
	of the parent does not change.
*/
void RBJoint::fixJointConstraints(bool fixPositions, bool fixOrientations, bool fixLinVelocities, bool fixAngularVelocities) {
	assert(child && parent);

	//first fix the relative orientation
	if (fixOrientations) {
		Quaternion qRel = computeRelativeOrientation();
		//make sure that the relative rotation between the child and the parent is around the a axis
		V3D axis = qRel.vec().normalized();
		//this is the rotation angle around the axis above, which may not be the rotation axis
		double rotAngle = getRotationAngle(qRel, axis);
		//get the rotation angle around the correct axis now (we are not in the world frame now)
		double ang = axis.dot(rotationAxis) * rotAngle;
		//compute the correct child orientation
		child->state.orientation = parent->state.orientation * getRotationQuaternion(ang, rotationAxis);
	}

	//now worry about the joint positions
	if (fixPositions) {
		//compute the vector rc from the child's joint position to the child's center of mass (in rbEngine coordinates)
		V3D rc = child->state.getWorldCoordinates(V3D(cJPos, P3D(0, 0, 0)));
		//and the vector rp that represents the same quanity but for the parent
		V3D rp = parent->state.getWorldCoordinates(V3D(pJPos, P3D(0, 0, 0)));

		//the location of the child's CM is now: pCM - rp + rc
		child->state.pos = parent->state.pos + (rc - rp);
	}

	if (fixAngularVelocities) {
		V3D wRel = child->state.angularVelocity - parent->state.angularVelocity;
		V3D worldRotAxis = parent->state.getWorldCoordinates(rotationAxis);
		//only keep the part that is aligned with the rotation axis...
		child->state.angularVelocity = parent->state.angularVelocity + worldRotAxis * wRel.dot(worldRotAxis);
	}

	//fix the velocities, if need be
	if (fixLinVelocities) {
		//we want to get the relative velocity at the joint to be 0. This can be accomplished in many different ways, but this approach
		//only changes the linear velocity of the child
		V3D pJPosVel = parent->state.getVelocityForPoint_local(pJPos);
		V3D cJPosVel = child->state.getVelocityForPoint_local(cJPos);
		child->state.velocity -= cJPosVel - pJPosVel;
		V3D testVel = parent->state.getVelocityForPoint_local(pJPos) - child->state.getVelocityForPoint_local(cJPos);
		assert(IS_ZERO(testVel.norm()));
	}
}


