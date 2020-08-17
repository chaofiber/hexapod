#pragma once

#include <utils/mathUtils.h>

/*==============================================================================*
 * This class acts as a container for the state information (position, orientation,
 * velocity and angular velocity - all of them stored in world coordinates) of a rigid body.
 *==============================================================================*/

class RBState{
public:
    // the position of the center of mass of the rigid body, in world coords. In local coordinates this corresponds to the point (0,0,0) aka origin.
    P3D pos = P3D(0,0,0);
    // its orientation - rotates from local coordinate frame to world coordinate frame
    Quaternion orientation = Quaternion::Identity();
    // the velocity of the center of mass, in world coords
    V3D velocity = V3D(0,0,0);
    // and finally, the angular velocity in world coords
    V3D angularVelocity = V3D(0,0,0);

public:
    /**
        Default constructor - populate the data members using safe values..
    */
    RBState(void){
    }

    /**
        A copy constructor.
    */
    RBState(const RBState& other){
        this->pos = other.pos;
        this->orientation = other.orientation;
        this->velocity = other.velocity;
        this->angularVelocity = other.angularVelocity;
    }

    /**
        and a copy operator
    */
    RBState&operator = (const RBState& other){
        this->pos = other.pos;
        this->orientation = other.orientation;
        this->velocity = other.velocity;
        this->angularVelocity = other.angularVelocity;
        return *this;
    }

    /**
        Default destructor.
    */
    ~RBState(void) {}

    /**
        This method returns the coordinates of the point that is passed in as a parameter(expressed in local coordinates), in world coordinates.
    */
    inline P3D getWorldCoordinates(const P3D& pLocal) const {
        //pWorld = pos + R * V3D(origin, pLocal)
        return pos + orientation * V3D(pLocal);
    }

    /**
        This method returns the vector that is passed in as a parameter(expressed in local coordinates), in world coordinates.
    */
    inline V3D getWorldCoordinates(const V3D& vLocal) const {
        //the rigid body's orientation is a unit quaternion. Using this, we can obtain the global coordinates of a local vector
        return orientation * vLocal;
    }

    /**
        This method is used to return the local coordinates of the point that is passed in as a parameter (expressed in global coordinates)
    */
    inline P3D getLocalCoordinates(const P3D& pWorld){
        return P3D() + orientation.inverse() * (V3D(pos, pWorld));
    }

    /**
        This method is used to return the local coordinates of the vector that is passed in as a parameter (expressed in global coordinates)
    */
    inline V3D getLocalCoordinates(const V3D& vWorld){
        //the rigid body's orientation is a unit quaternion. Using this, we can obtain the global coordinates of a local vector
        return orientation.inverse() * vWorld;
    }

    /**
        This method returns the world coordinates velocity of a point that is passed in as a parameter. The point is expressed in local coordinates, and the resulting velocity will be expressed in world coordinates.
    */
    inline V3D getVelocityForPoint_local(const P3D& pLocal){
        //we need to compute the vector r, from the origin of the body to the point of interest
        V3D r = V3D(pLocal);
        //the velocity is given by omega x r + v. omega and v are already expressed in world coordinates, so we need to express r in world coordinates first.
        return angularVelocity.cross(getWorldCoordinates(r)) + velocity;
    }

    /**
        This method returns the world coordinates velocity of a point that is passed in as a parameter. The point is expressed in world coordinates, and the resulting velocity will be expressed in world coordinates.
    */
    inline V3D getVelocityForPoint_global(const P3D& pWorld){
        //we need to compute the vector r, from the origin of the body to the point of interest
        V3D r(pos, pWorld);
        //the velocity is given by omega x r + v. omega and v are already expressed in world coordinates, so we need to express r in world coordinates first.
        return angularVelocity.cross(r) + velocity;
    }
};

