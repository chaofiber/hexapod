#pragma once

#include <memory>

#include <utils/mathUtils.h>
#include <utils/transformation.h>

#ifdef CRL_USE_GUI
#include <gui/model.h>
#endif

namespace crl
{
namespace robot
{

class RRBCollsionShape {
public:
    //this is pure abstract class
    virtual ~RRBCollsionShape() = 0;
};

class RRBCollisionSphere : public RRBCollsionShape {
public:
    RRBCollisionSphere() : radius(0.01), localCoordinates(P3D(0,0,0)) {}
    RRBCollisionSphere(P3D localCoordinates, double radius) : radius(radius), localCoordinates(localCoordinates) {}
    ~RRBCollisionSphere() {}

public:
    double radius;
    P3D localCoordinates = P3D(0, 0, 0);
};

class RRBCollisionPlane : public RRBCollsionShape {
public:
    RRBCollisionPlane() : p(P3D(0,0,0)), n(0, 1, 0) {}
    RRBCollisionPlane(P3D p, V3D n) : p(p), n(n) {}
    ~RRBCollisionPlane() {}

public:
    V3D n = V3D(0, 1, 0);
    P3D p = P3D(0, 0, 0);
};

class RRBCollisionBox : public RRBCollsionShape {
public:
    RRBCollisionBox() : dim(P3D(1,1,1)), localCoordinates(P3D(0,0.5,0)) {}
    RRBCollisionBox(P3D dim, P3D localCoordinates) : dim(dim), localCoordinates(localCoordinates) {}
    ~RRBCollisionBox() {}

public:
    P3D dim = P3D(1,1,1);
    P3D localCoordinates = P3D(0,0.5,0);
};

} // namespace robot
} // namespace crl

class RB3DModel {
public:
#ifdef CRL_USE_GUI
    Model* model = NULL;
#endif
    RigidTransformation transform;
    std::string description;
    std::string path;
    V3D color = V3D(0.9, 0.9, 0.9);
};

/*================================================================================================================*
 * This class represents a container for the various properties of a rigid body                                   *
 *================================================================================================================*/
class RBProperties{
public:
    //the mass
    double mass = 1.0;
    //we'll store the moment of inertia of the rigid body, in the local coordinate frame
    Matrix3x3 MOI_local = Matrix3x3::Identity();

    // collision primitives that are relevant for this rigid body
    std::vector<std::shared_ptr<crl::robot::RRBCollsionShape>> collisionShapes;

    // meshes that are used to visualize the rigid body
    std::vector<RB3DModel> meshes;

    // end effector points
    std::vector<P3D> endEffectorPoints;

    // id of the rigid body
    int id = -1;

    // for selection via GUI
    bool selected = false;

    //for drawing abstract view
    double abstractViewCylRadius = 0.01;
    V3D jointDrawColor = V3D(0.0, 0.0, 0.9);
    V3D highlightColor = V3D(1.0, 0.5, 0.5);
    V3D colSphereDrawColor = V3D(0.75, 0.0, 0.0);
    V3D endEffectorDrawColor = V3D(0.0, 1.0, 0.0);
    //draw color for rb primitive
    V3D color = V3D(0.5, 0.5, 0.5);

    //is this body fixed to world
    bool fixed = false;

    //physics related coefficients
    double restitutionCoeff = 0;
    double frictionCoeff = 0.8;


public:
    /**
        default constructor.
    */
    RBProperties() {}

    /**
        default destructor.
    */
    ~RBProperties() {}

    /**
        set the moment of inertia of the rigid body - symmetric 3x3 matrix, so we need the six values for it.
    */
    inline void setMOI(double moi00, double moi11, double moi22, double moi01, double moi02, double moi12) {
        MOI_local <<
            moi00, moi01, moi02,
            moi01, moi11, moi12,
            moi02, moi12, moi22;
    }

    /**
        parallel axis theorem (translation)
    */
    inline void offsetMOI(double x, double y, double z) {
        setMOI(
            MOI_local(0, 0) + mass * (y*y + z*z),
            MOI_local(1, 1) + mass * (x*x + z*z),
            MOI_local(2, 2) + mass * (x*x + y*y),
            MOI_local(0, 1) - mass * x * y,
            MOI_local(0, 2) - mass * x * z,
            MOI_local(1, 2) - mass * y * z
        );
    }

    /**
        similarity transformation (rotation)
    */
    inline void rotateMOI(double q, double x, double y, double z) {
        Quaternion quat(q, x, y, z);
        MOI_local = quat * MOI_local * quat.inverse();
    }
};
