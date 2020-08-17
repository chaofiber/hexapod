#pragma once

#include <gui/model.h>
#include <gui/shader.h>

#include "robot/RBJoint.h"

namespace crl
{
namespace robot
{

class RBRenderer
{
private:
    /* data */
public:

	/**
		methods used to draw different types of views of the rigid body...
	*/
	static void drawSkeletonView(const RobotRB *rb, const Shader& shader, bool showJointAxes, bool showJointLimits);
	static void drawMeshes(const RobotRB *rb, const Shader& shader);
	static void drawCoordFrame(const RobotRB *rb, const Shader& shader);
	static void drawCollisionSpheres(const RobotRB *rb, const Shader& shader);
	static void drawMOI(const RobotRB *rb, const Shader& shader);
    static void drawEndEffectors(const RobotRB *rb, const Shader& shader);

    /**
		draws the axes of rotation
	*/
	static void drawAxis(RBJoint *j, const Shader& shader);

	/**
		draw joint limits
	*/
	static void drawJointLimits(RBJoint *j, const Shader& shader);
};

} // namespace robot
} // namespace crl