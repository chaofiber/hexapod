#pragma once

#include "robot/RobotRB.h"
#include "robot/RBJoint.h"
#include "robot/RBEngine.h"
#include "robot/RBUtils.h"

#include "robot/RBRenderer.h"

#include <utils/utils.h>

class RobotState;

/**
    Robots are articulated figures (i.e. tree structures starting at a root)
*/
class Robot {
    friend class RobotState;
    friend class GeneralizedCoordinatesRobotRepresentation;

public:
    //root configuration
    RobotRB* root = nullptr;
    //keep lists of all the joints and all the RBs of the robot, for easy access
    std::vector<RBJoint*> jointList;
    std::vector<RobotRB*> rbList;
    //it is often useful to talk about things relative to the robot's forward direction, so keep track of it, as well as the vector that points to the robot's right...
    V3D forward = V3D(0, 0, 1);
    V3D right = V3D(1, 0, 0);
    //drawing flags
    bool showMeshes = true;
    bool showJointLimits = false;
    bool showCollisionSpheres = false;
    bool showEndEffectors = false;
    bool showJointAxes = false;
    bool showSkeleton = false;
    bool showMOI = false;

public:
    /**
        the constructor
    */
    Robot(const char* filePath, const char* statePath = nullptr);

    Robot(RBEngine *rbEngine, const char* filePath, const char* statePath = nullptr);

    /**
        the destructor
    */
    ~Robot(void);

    /**
        This method computes the relative orientation of the parent and child bodies of joint i.
    */
    inline Quaternion getRelativeOrientationForJoint(RBJoint* joint) {
        return joint->computeRelativeOrientation();
    }

    /**
        This method is used to get the relative angular velocities of the parent and child bodies of joint i,
        expressed in parent's local coordinates.
    */
    inline V3D getRelativeLocalCoordsAngularVelocityForJoint(RBJoint* joint) {
        //we will store wRel in the parent's coordinates, to get an orientation invariant expression for it
        return joint->parent->state.getLocalCoordinates(V3D(joint->child->state.angularVelocity - joint->parent->state.angularVelocity));
    }

    inline void setRelativeOrientationForJoint(RBJoint* joint, const Quaternion& qRel) {
        joint->child->state.orientation = joint->parent->state.orientation * qRel;
    }

    inline void setRelativeLocalCoordsAngularVelocityForJoint(RBJoint* joint, const V3D& relAngVel) {
        //assume relAngVel is stored in the parent's coordinate frame, to get an orientation invariant expression for it
        joint->child->state.angularVelocity = V3D(joint->parent->state.angularVelocity + joint->parent->state.getWorldCoordinates(relAngVel));
    }

    inline int getJointCount() {
        return (int)jointList.size();
    }

    inline RBJoint* getJoint(int i) const{
        if (i < 0 || i > (int)jointList.size()-1)
            return nullptr;
        return jointList[i];
    }

    inline int getRigidBodyCount() {
        return (int)jointList.size() + 1;
    }

    /**
        returns a pointer to the ith rigid body of the virtual robot, where the root is at 0, and the rest follow afterwards...
    */
    inline RobotRB* getRigidBody(int i) {
        if (i == 0)
            return root;
        if (i <= (int)jointList.size())
            return jointList[i - 1]->child;
        return nullptr;
    }

    /**
        this method is used to return the current heading of the robot
    */
    inline Quaternion getHeading() {
        return computeHeading(root->state.orientation, RBGlobals::worldUp);
    }

    /**
        this method is used to return the current heading of the robot, specified as an angle measured in radians
    */
    inline double getHeadingAngle() {
        return getRotationAngle(computeHeading(root->state.orientation, RBGlobals::worldUp), RBGlobals::worldUp);
    }

    /**
        this method is used to return a reference to the joint whose name is passed as a parameter, or nullptr
        if it is not found.
    */
    inline RBJoint* getJointByName(const char* jName) {
        for (uint i = 0; i < jointList.size(); i++)
            if (strcmp(jointList[i]->name.c_str(), jName) == 0)
                return jointList[i];
        return nullptr;
    }

    /**
        this method is used to return the index of the joint (whose name is passed as a parameter) in the articulated figure hierarchy.
    */
    inline int getJointIndex(const char* jName) {
        for (uint i = 0; i < jointList.size(); i++)
            if (strcmp(jointList[i]->name.c_str(), jName) == 0)
                return i;
        return -1;
    }

    /**
        returns the root of the current articulated figure.
    */
    inline RobotRB* getRoot() {
        return root;
    }

    /**
        This method is used to compute the center of mass of the robot.
    */
    P3D computeCOM();

    double getMass();

    /**
        This method is used to compute the velocity of the center of mass of the articulated figure.
    */
    V3D computeCOMVelocity();

    /**
        this method is used to read the reduced state of the robot from the file
    */
    void loadReducedStateFromFile(const char* fName);

    /**
        this method is used to write the reduced state of the robot to a file
    */
    void saveReducedStateToFile(const char* fName);

    /**
        uses the state of the robot to populate the input
    */
    void populateState(RobotState* state, bool useDefaultAngles = false);

    /**
        sets the state of the robot using the input
    */
    void setState(RobotState* state);

    void setDefaultState();

    /**
        makes sure the state of the robot is consistent with all the joint types...
    */
    void fixJointConstraints();

    /**
        this method updates the robot's root state and all connected rigid bodies
    */
    void setRootState(const P3D& position, const Quaternion& orientation);

    /**
        this method is used to return a reference to the articulated figure's rigid body whose name is passed in as a parameter,
        or nullptr if it is not found.
    */
    RobotRB* getRBByName(const char* jName);

    //returns NULL if no RBs are hit by the ray...
    RobotRB* getFirstRBHitByRay(const Ray& ray, P3D& intersectionPoint, bool checkMeshes, bool checkSkeleton) {
        RobotRB* selectedRB = nullptr;
        double t = DBL_MAX;
        P3D tmpIntersectionPoint = P3D(0, 0, 0);

        for (uint i = 0; i < rbList.size(); i++) {
            if (rbList[i]->getRayIntersectionPoint(ray, tmpIntersectionPoint, checkMeshes, checkSkeleton)) {
                double tTmp = ray.getRayParameterFor(tmpIntersectionPoint);
                if (tTmp < t) {
                    selectedRB = rbList[i];
                    t = tTmp;
                    intersectionPoint = tmpIntersectionPoint;
                }
            }
        }

        return selectedRB;
    }

    #ifdef CRL_USE_GUI
    /**
        draws the robot at its current state
    */
    inline void draw(const Shader& rbShader) {
        //Draw abstract view first
        if (showSkeleton)
            for (uint i = 0; i < rbList.size(); i++)
                crl::robot::RBRenderer::drawSkeletonView(rbList[i], rbShader, showJointAxes, showJointLimits);

        //Then draw meshes (because of blending)
        if (showMeshes)
            for (uint i = 0; i < rbList.size(); i++)
                crl::robot::RBRenderer::drawMeshes(rbList[i], rbShader);

        //Then draw collsion spheres
        if (showCollisionSpheres)
            for (uint i = 0; i < rbList.size(); i++)
                crl::robot::RBRenderer::drawCollisionSpheres(rbList[i], rbShader);

        //Then draw end effectors
        if (showEndEffectors)
            for (uint i = 0; i < rbList.size(); i++)
                crl::robot::RBRenderer::drawEndEffectors(rbList[i], rbShader);

        //and now MOIs
        if (showMOI)
            for (uint i = 0; i < rbList.size(); i++)
                crl::robot::RBRenderer::drawMOI(rbList[i], rbShader);
    }
#endif
};
