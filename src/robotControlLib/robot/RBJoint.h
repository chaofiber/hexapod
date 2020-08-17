#pragma once

#include <string>

#include <robot/RobotRB.h>
#include <utils/mathUtils.h>

/*==============================================================================*
 * This class is used to implements hinge joints that allow relative rotation
 * between the parent and the child only about a given axis
 *==============================================================================*/
class RBEngine;

enum RBJointControlMode {
    PASSIVE_MODE = 0,
    POSITION_MODE,
    VELOCITY_MODE,
    FORCE_MODE				// force or torque control
};

class RBJoint {
public:
    enum JointType {
        REVOLUTE 	= 0,
        FIXED 		= 1,
    };

    //parent rigid body
    RobotRB* parent = nullptr;
    //this is the location of the joint on the parent - expressed in the parent's local coordinates
    P3D pJPos = P3D(0, 0, 0);
    //this is the child link
    RobotRB* child = nullptr;
    //this is the location of the joint on the child - expressed in the child's local coordinates
    P3D cJPos = P3D(0, 0, 0);
    //selected in GUI
    bool selected = false;
    //the name of the joint
    std::string name;
    //unique index of the joint
    int jIndex = -1;
    //local coordinates of the rotation axis. Since the child and parent only rotate relative to each other about this joint, the local coordinates for the rotation axis are the same in parent and child frame
    V3D rotationAxis = V3D(0, 0, 1);
    //joint limits
    double minAngle = 0, maxAngle = 0;
    double maxSpeed = 40;	// rad/s
    double maxTorque = 100;	// N.m
    bool jointLimitsActive = false;
    //keep a 'default' angle value... useful for applications that need a regularizer...
    double defaultJointAngle = 0;
    //joint type
    JointType type = JointType::REVOLUTE;
    //joint control mode
    RBJointControlMode controlMode = RBJointControlMode::PASSIVE_MODE;

    //angle in Position mode
    //velocity in Velocity mode
    //torque in Force mode
    double desiredControlSignal = 0;

    //for position mode
    double motorKp = 12000.0;
    double motorKd = 0.1;
    double motorFeedback = 0;

public:
    /**
        Default constructor
    */
    RBJoint(void) {}

    /**
        Default destructor
    */
    ~RBJoint(void) {}

    /**
        Returns the world position of the joint
    */
    P3D getWorldPosition();

    /**
        computes the relative orientation between the parent and the child rigid bodies
    */
    Quaternion computeRelativeOrientation();

    /**
        This method is used to fix the errors in the joints (i.e. project state of child such that joint configuration is consistent). The state
        of the parent does not change.
    */
    void fixJointConstraints(bool fixPositions, bool fixOrientations, bool fixLinVelocities, bool fixAngularVelocities);

    /* this value ranges from -pi to pi*/
    inline double getCurrentJointAngle() {
        return getRotationAngle(computeRelativeOrientation(), rotationAxis);
    }
};
